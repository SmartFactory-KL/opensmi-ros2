# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Subscription handle: one topic's lifetime and its signals, in one object.

Messages are awaited one at a time or iterated. The handle also reports type-hash mismatches, the
publisher count, and the QoS profile with its provenance. It
stays the same across a QoS adaptation: the ``rclpy`` subscription underneath is re-created while
this object, its buffer and its consumers are untouched. The publisher count comes from DDS match
events, so the graph poll is not needed for subscribed topics.

The write side is a separate object, :class:`SubscriptionSink`, held only by the Runtime, so a
Component author cannot inject messages. Nothing here imports ``rclpy``.
"""

from __future__ import annotations

import asyncio
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING, final

from opensmi.ros2.runtime.errors import LoopAffinityError, SubscriptionClosedError
from opensmi.ros2.runtime.qos import MOST_PERMISSIVE, Provenance, QoSIncompatibility, QoSProfile

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Awaitable, Callable

    from opensmi.ros2.runtime.messages import Message

__all__ = ["Subscription", "SubscriptionSink", "TypeHashMismatch", "TypeHashMismatchWarning"]


class TypeHashMismatchWarning(UserWarning):
    """A publisher's type hash differs from the one this Runtime built its dataclass against.

    A warning, not an error: the message is probably still decodable.
    """


@dataclass(frozen=True, slots=True, kw_only=True)
class TypeHashMismatch:
    """A publisher whose type hash differs from the one this Runtime built its dataclass against.

    :ivar topic: The ROS2 topic both endpoints are on.
    :ivar node_name: The fully qualified name of the publishing node.
    :ivar topic_type: The type name that publisher advertises.
    :ivar published_hash: The publisher's type hash, as ROS2 spells it.
    :ivar expected_hash: The type hash of this Runtime's own subscription on that topic.
    """

    topic: str
    node_name: str
    topic_type: str
    published_hash: str
    expected_hash: str


@final
class SubscriptionSink:
    """The write side of a :class:`Subscription`; every method runs on the Runtime's event loop."""

    def __init__(
        self,
        *,
        depth: int,
        qos: QoSProfile = MOST_PERMISSIVE,
        provenance: Provenance = Provenance.DISCOVERED,
    ) -> None:
        """Open a sink buffering at most ``depth`` messages, affine to the running loop.

        :param depth: How many messages may wait for a consumer. When the buffer is full the
            oldest is dropped, since a Monitoring variable wants the newest value.
        :param qos: The profile the subscription opened with.
        :param provenance: Where that profile came from.
        :raises RuntimeError: When constructed with no running event loop.
        :raises ValueError: When ``depth`` is below 1.
        """
        if depth < 1:
            message = f"a subscription depth must be at least 1, not {depth}"
            raise ValueError(message)
        # Captured because `asyncio.Event` binds to the first loop that awaits it; another loop
        # would hang without an error.
        self._loop = asyncio.get_running_loop()
        self._messages: deque[Message] = deque(maxlen=depth)
        self._arrival = asyncio.Event()
        self._mismatches: list[TypeHashMismatch] = []
        self._publishers = 0
        self._matched = asyncio.Event()
        self._qos = qos
        self._provenance = provenance
        self._incompatibilities: list[QoSIncompatibility] = []
        self._incompatible = asyncio.Event()
        self._closed = False

    @property
    def closed(self) -> bool:
        """Return whether the subscription's lifetime has ended."""
        return self._closed

    @property
    def type_hash_mismatches(self) -> tuple[TypeHashMismatch, ...]:
        """Return every type-hash disagreement seen on this topic so far."""
        return tuple(self._mismatches)

    @property
    def publisher_count(self) -> int:
        """Return how many publishers this subscription is currently matched with."""
        return self._publishers

    @property
    def qos(self) -> QoSProfile:
        """Return the QoS profile the subscription is currently asking for."""
        return self._qos

    @property
    def qos_provenance(self) -> Provenance:
        """Return where that profile came from, and therefore whether it may adapt."""
        return self._provenance

    @property
    def qos_incompatibilities(self) -> tuple[QoSIncompatibility, ...]:
        """Return every incompatibility the middleware has reported on this topic so far."""
        return tuple(self._incompatibilities)

    def note_qos(self, profile: QoSProfile) -> None:
        """Record the profile this subscription asks for, after a re-subscription made it true."""
        self._qos = profile

    def note_incompatibility(self, incompatibility: QoSIncompatibility) -> bool:
        """Record one incompatibility the middleware reported, unless it is one already recorded.

        Deduplicated by policy and request, so a flapping publisher cannot produce an unbounded
        list or an event storm. A clash after an adaptation counts as new, since the request moved.

        :return: Whether this was new. The caller logs only then.
        """
        seen = (incompatibility.policy, incompatibility.request)
        if any((each.policy, each.request) == seen for each in self._incompatibilities):
            return False
        self._incompatibilities.append(incompatibility)
        self._incompatible.set()
        return True

    async def await_incompatibilities(self, seen: int) -> int:
        """Await more incompatibilities than have been seen, and return how many there now are.

        :raises SubscriptionClosedError: When the subscription closes while waiting.
        :raises LoopAffinityError: When awaited on a loop other than this sink's.
        """
        self.require_own_loop()
        while len(self._incompatibilities) <= seen:
            if self._closed:
                message = "this subscription is closed and will report no further incompatibilities"
                raise SubscriptionClosedError(message)
            # No await between clear and wait, so no report can slip into the gap (same loop).
            self._incompatible.clear()
            await self._incompatible.wait()
        return len(self._incompatibilities)

    def note_publishers(self, count: int) -> None:
        """Accept the publisher count a match event reported (a missed event costs nothing)."""
        if count == self._publishers:
            return
        self._publishers = count
        self._matched.set()

    async def await_publishers(self, seen: int) -> int:
        """Await a publisher count other than one already seen, and return it.

        :raises SubscriptionClosedError: When the subscription closes while waiting.
        :raises LoopAffinityError: When awaited on a loop other than this sink's.
        """
        self.require_own_loop()
        while self._publishers == seen:
            if self._closed:
                message = "this subscription is closed and will match no further publishers"
                raise SubscriptionClosedError(message)
            # No await between clear and wait, so no match can slip into the gap (same loop).
            self._matched.clear()
            await self._matched.wait()
        return self._publishers

    def deliver(self, message: Message) -> None:
        """Accept one decoded message, dropping the oldest buffered one if the buffer is full."""
        self._messages.append(message)
        self._arrival.set()

    def note_type_hash_mismatch(self, mismatch: TypeHashMismatch) -> None:
        """Record a type-hash disagreement seen on this topic."""
        self._mismatches.append(mismatch)

    def close(self) -> None:
        """End the stream, waking every consumer waiting on it."""
        self.require_own_loop()
        self._closed = True
        self._arrival.set()
        self._matched.set()
        self._incompatible.set()

    def require_own_loop(self) -> None:
        """Refuse an operation running on a loop other than the one this sink was opened on.

        :raises LoopAffinityError: When the running loop is not this sink's.
        """
        if asyncio.get_running_loop() is not self._loop:
            message = (
                "this subscription is affine to the event loop it was opened on; this call is on "
                "another one, which would wait on messages that are being delivered elsewhere"
            )
            raise LoopAffinityError(message)

    async def take(self) -> Message:
        """Await the next buffered message.

        :raises SubscriptionClosedError: When the subscription is closed and the buffer is empty.
        :raises LoopAffinityError: When awaited on a loop other than this sink's.
        """
        self.require_own_loop()
        while not self._messages:
            if self._closed:
                message = "this subscription is closed and has no buffered messages left"
                raise SubscriptionClosedError(message)
            # No await between clear and wait, so no message can slip into the gap (same loop).
            self._arrival.clear()
            await self._arrival.wait()
        return self._messages.popleft()


@final
class Subscription:
    """A live subscription to one ROS2 topic, from :meth:`RuntimeInterface.subscribe`.

    Iterate it (``async for message in subscription``) or await :meth:`receive`; end it with
    :meth:`close`.
    """

    def __init__(
        self,
        *,
        topic: str,
        message_type: type[Message],
        sink: SubscriptionSink,
        release: Callable[[], Awaitable[None]],
    ) -> None:
        """Bind a handle to the sink and the teardown the Runtime prepared for it."""
        self._topic = topic
        self._message_type = message_type
        self._sink = sink
        self._release = release

    def __repr__(self) -> str:
        """Return a representation naming the topic and the ROS2 type."""
        state = "closed" if self._sink.closed else "open"
        return f"<Subscription {self._topic} {self._message_type.ros2.type_name} ({state})>"

    @property
    def topic(self) -> str:
        """Return the ROS2 topic this subscription is on."""
        return self._topic

    @property
    def message_type(self) -> type[Message]:
        """Return the message dataclass every payload from this subscription is an instance of."""
        return self._message_type

    @property
    def type_hash_mismatches(self) -> tuple[TypeHashMismatch, ...]:
        """Return every publisher seen whose type hash is not the one built against (stale definition)."""
        return self._sink.type_hash_mismatches

    @property
    def publisher_count(self) -> int:
        """Return how many publishers this subscription is matched with right now.

        Zero means nobody is on the other end, which differs from a matched topic that is silent;
        a Monitoring variable's status code distinguishes the two.
        """
        return self._sink.publisher_count

    @property
    def qos(self) -> QoSProfile:
        """Return the QoS profile in force; it moves only when a discovered profile adapts."""
        return self._sink.qos

    @property
    def qos_provenance(self) -> Provenance:
        """Return where the profile came from: ``DISCOVERED`` (may adapt) or ``DECLARED`` (never moves)."""
        return self._sink.qos_provenance

    @property
    def qos_incompatibilities(self) -> tuple[QoSIncompatibility, ...]:
        """Return every QoS incompatibility the middleware has reported on this topic."""
        return self._sink.qos_incompatibilities

    async def follow_qos_incompatibilities(self) -> AsyncIterator[QoSIncompatibility]:
        """Yield every QoS incompatibility reported on this topic, from the first one onwards.

        Recorded entries arrive without waiting, and any adaptation an entry caused has already
        happened. The iteration ends when the subscription closes.
        """
        seen = 0
        while True:
            try:
                total = await self._sink.await_incompatibilities(seen)
            except SubscriptionClosedError:
                return
            recorded = self._sink.qos_incompatibilities
            for each in recorded[seen:total]:
                yield each
            seen = total

    async def publisher_counts(self) -> AsyncIterator[int]:
        """Yield the publisher count now, and again each time it changes.

        The first value arrives without waiting. The iteration ends when the subscription closes.
        """
        seen = -1  # no real count is -1, so the first iteration yields the current one
        while True:
            try:
                seen = await self._sink.await_publishers(seen)
            except SubscriptionClosedError:
                return
            yield seen

    async def receive(self) -> Message:
        """Await the next message.

        :raises SubscriptionClosedError: When the subscription is closed and nothing is buffered.
        """
        return await self._sink.take()

    def __aiter__(self) -> AsyncIterator[Message]:
        """Return an iterator yielding messages until the subscription is closed."""
        return self._stream()

    async def _stream(self) -> AsyncIterator[Message]:
        """Yield messages until the subscription is closed, then stop cleanly."""
        while True:
            try:
                yield await self._sink.take()
            except SubscriptionClosedError:
                return

    async def close(self) -> None:
        """End this subscription, releasing its ROS2 resources.

        Idempotent.
        """
        if self._sink.closed:
            return
        self._sink.close()
        await self._release()
