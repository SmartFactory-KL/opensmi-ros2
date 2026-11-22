# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The TF facet: both transform topics, held by us, in both directions.

TF is ROS2's frame network: coordinate frames, each naming its parent, carried on ``/tf`` (moving)
and ``/tf_static`` (never moving). Inward, the facet drains both topics through two ordinary
subscriptions into a topology record and a ``tf2_ros.Buffer``. It does not use
``tf2_ros.TransformListener``, which would start a second executor and break the rule that this
package owns one Node and one spinning thread. The topology is read from the messages (parent
``header.frame_id``, child ``child_frame_id``), geometry is looked up synchronously with
``Buffer.lookup_transform`` and no timeout (its ``_async`` variant busy-spins on the Runtime's
loop), and nothing here imports ``rclpy``.

Outward, :meth:`TFFacet.broadcast` publishes on the same topics. The publishers are opened by a
watch that declares it broadcasts, so a Machine without outward frames never advertises ``/tf``. Our
own publisher is discounted from :attr:`TFFacet.satisfied`, since a subscription's matched count
includes it (checked on Jazzy). The stamp is one reading of the Runtime's clock per call
(:mod:`opensmi.ros2.runtime.stamp`). ``tf2_ros.StaticTransformBroadcaster`` is not used, because it
silently drops an update for a frame it already sent. The static publisher is transient-local and
the moving one volatile (:data:`TF_STATIC_QOS`, :data:`TF_QOS`); both are declared constants, while
the subscriptions are discovered.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Final, TypeAlias, final

import structlog

from opensmi.ros2.runtime.errors import (
    ROS2RuntimeError,
    UnusableMessageTypeError,
)
from opensmi.ros2.runtime.messages import repeated_message
from opensmi.ros2.runtime.qos import DEFAULT_PROFILE, LATCHED
from opensmi.ros2.runtime.stamp import NANOSECONDS_FIELD, SECONDS_FIELD, STAMP_FIELD

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable, Iterable, Mapping, Sequence

    from opensmi.ros2.runtime.messages import Message
    from opensmi.ros2.runtime.publisher import Publisher
    from opensmi.ros2.runtime.subscription import Subscription

__all__ = [
    "CHILD_FRAME_FIELD",
    "FRAME_FIELD",
    "HEADER_FIELD",
    "ROTATION_FIELD",
    "TF_DEPTH",
    "TF_MESSAGE_TYPE",
    "TF_QOS",
    "TF_STATIC_QOS",
    "TF_STATIC_TOPIC",
    "TF_TOPIC",
    "TRANSFORMS_FIELD",
    "TRANSFORM_CACHE",
    "TRANSFORM_FIELD",
    "TRANSLATION_FIELD",
    "AdvertiseFunction",
    "ClockFunction",
    "FeedFunction",
    "FrameLink",
    "LookupFunction",
    "SubscribeFunction",
    "TFFacet",
    "TFWatch",
    "Transform",
    "links",
    "roots_of",
]

TF_TOPIC: Final = "/tf"
"""The topic every moving transform in a ROS2 system is published on."""

TF_STATIC_TOPIC: Final = "/tf_static"
"""The topic a transform that never moves is published on, once, with transient-local QoS."""

TF_MESSAGE_TYPE: Final = "tf2_msgs/msg/TFMessage"
"""The one type both topics carry."""

TF_QOS: Final = DEFAULT_PROFILE
"""What this Runtime offers on the moving transform topic: reliable and volatile.

Replaying an old pose to a subscriber that has just joined would report a robot where it used to be.
"""

TF_STATIC_QOS: Final = LATCHED
"""What this Runtime offers on the static transform topic: reliable and transient-local.

This is the ROS2 convention. A static frame is published once, so a volatile publisher would leave
every later subscriber with no frame. It is a constant, because ``tf2_ros``' own static broadcaster
hard-codes the same profile.
"""

TRANSFORMS_FIELD: Final = "transforms"
"""The repeated field of a ``TFMessage``, which is what keys the frame network."""

CHILD_FRAME_FIELD: Final = "child_frame_id"
"""The field of one element naming the frame it describes. The parent is in its header."""

HEADER_FIELD: Final = "header"
"""The field of one element carrying the parent frame and the stamp. Read inward, written outward."""

TRANSFORM_FIELD: Final = "transform"
"""The field of one element carrying the geometry: a translation and a rotation."""

TRANSLATION_FIELD: Final = "translation"
"""The field of one transform holding metres along three axes."""

ROTATION_FIELD: Final = "rotation"
"""The field of one transform holding the unit quaternion."""

FRAME_FIELD: Final = "frame_id"
"""The field of one header naming the parent frame. Read inward, written outward."""

TRANSFORM_CACHE: Final = 10.0
"""How many seconds of transform history the buffer keeps (tf2's default). Composed poses are answered from it."""

TF_DEPTH: Final = 100
"""How many messages each transform subscription buffers.

Larger than the Binding default, because ``/tf`` carries one message per publisher per control
cycle. A drop here is a transform the buffer never saw.
"""

_LOGGER = structlog.get_logger("opensmi.ros2.runtime.tf")

SubscribeFunction: TypeAlias = "Callable[..., Awaitable[Subscription]]"
"""How the facet opens a topic: the Runtime's own ``subscribe``."""

AdvertiseFunction: TypeAlias = "Callable[..., Awaitable[Publisher]]"
"""How the facet opens a topic outward: the Runtime's own ``advertise``."""

ClockFunction: TypeAlias = "Callable[[], tuple[int, int]]"
"""How a broadcast learns what time it is: whole seconds and nanoseconds off the ROS2 Node's own clock.

It sits behind the seam because a Node on simulated time is the case a wall clock gets wrong.
"""

FeedFunction: TypeAlias = "Callable[[Message, bool], None]"
"""How one message reaches the transform buffer: the message, and whether it arrived static."""

LookupFunction: TypeAlias = "Callable[[str, str], Transform | None]"
"""How a composed pose is asked for: a target frame and a source frame.

The answer comes from the cache or not at all. Synchronous and non-blocking.
"""


@dataclass(frozen=True, slots=True, kw_only=True)
class Transform:
    """One rigid-body transform, as ROS2 spells it: a translation and a unit quaternion.

    It is the ROS2 shape because this is behind the seam. Turning the quaternion into angles is the
    server half's job.

    :ivar translation: Metres along x, y and z.
    :ivar rotation: The quaternion as x, y, z, w (ROS2's own field order).
    """

    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True, kw_only=True)
class FrameLink:
    """One link of the frame network, whichever direction it is travelling.

    Inward it is what the last message to carry a frame left it as. Outward it is what
    :meth:`TFFacet.broadcast` is asked to put on the wire.

    :ivar child: The frame this link describes.
    :ivar parent: The frame it is relative to. tf2 permits one parent per frame, so the network is a
        tree.
    :ivar transform: Where the child sits in its parent.
    :ivar static: Inward, whether it arrived on ``/tf_static``. Outward, which topic it is published
        to.
    """

    child: str
    parent: str
    transform: Transform
    static: bool


def links(message: Message, *, static: bool) -> tuple[FrameLink, ...]:
    """Return the frame links one ``TFMessage`` carries, read off its own structure.

    An element missing either name, or naming itself as its parent, is skipped: one malformed
    element must not end the stream.

    :param message: One message from ``/tf`` or ``/tf_static``, as it crossed the seam.
    :param static: Whether it arrived on the static topic.
    """
    found: list[FrameLink] = []
    for element in getattr(message, TRANSFORMS_FIELD, ()):
        child = _named(getattr(element, CHILD_FRAME_FIELD, None))
        parent = _named(getattr(getattr(element, HEADER_FIELD, None), FRAME_FIELD, None))
        transform = _transform_of(element)
        if not child or not parent or child == parent or transform is None:
            continue
        found.append(FrameLink(child=child, parent=parent, transform=transform, static=static))
    return tuple(found)


def roots_of(frames: Mapping[str, FrameLink]) -> tuple[str, ...]:
    """Return every frame that is somebody's parent and nobody's child, in sorted order.

    The root of a TF tree is what OPC 10000-210 calls the World Frame. A well-formed tree has one;
    two means two trees are published into one graph.
    """
    return tuple(sorted({link.parent for link in frames.values()} - set(frames)))


@final
class TFWatch:
    """One consumer's interest in TF, and its lifetime.

    Handed out by :meth:`TFFacet.watch`. Closing it lets the subscriptions close once no watch is
    left.
    """

    def __init__(self, *, facet: TFFacet, release: Callable[[TFWatch], None]) -> None:
        """Bind a watch to the facet it reads and the deregistration the facet prepared."""
        self._facet = facet
        self._release = release
        self._closed = False

    def __repr__(self) -> str:
        """Return a representation naming the watch's state."""
        return f"<TFWatch ({'closed' if self._closed else 'open'})>"

    @property
    def closed(self) -> bool:
        """Return whether this watch's lifetime has ended."""
        return self._closed

    async def close(self) -> None:
        """End this watch, and with it the subscriptions if it was the last one. Idempotent."""
        if self._closed:
            return
        self._closed = True
        self._release(self)
        await self._facet.settle()


@final
class TFFacet:
    """The frame network, for as long as anything is watching it.

    Construct one through :class:`opensmi.ros2.runtime.runtime.Runtime`. The facet is self-disabling
    like the graph and parameter facets: a Machine with no TF Binding never subscribes to ``/tf``,
    the most expensive topic on a busy robot.
    """

    def __init__(
        self,
        *,
        subscribe: SubscribeFunction,
        advertise: AdvertiseFunction,
        feed: FeedFunction,
        lookup: LookupFunction,
        clock: ClockFunction,
    ) -> None:
        """Describe a facet without subscribing to or advertising anything.

        The topics, and whether it publishes, are the first watch's: they come from a declaration.

        :param subscribe: How to open a topic inward. The Runtime's own.
        :param advertise: How to open a topic outward. The Runtime's own.
        :param feed: How one message reaches the transform buffer.
        :param lookup: How a composed pose is asked of that buffer.
        :param clock: What time it is, off the ROS2 Node's own clock.
        """
        self._subscribe = subscribe
        self._advertise = advertise
        self._feed = feed
        self._lookup = lookup
        self._clock = clock
        self._reading: _Sources | None = None
        self._frames: dict[str, FrameLink] = {}
        self._seen: dict[str, float] = {}
        self._clock_started = False
        self._watches: list[TFWatch] = []
        self._sources: list[_Source] = []
        self._outward: dict[bool, Publisher] = {}

    def __repr__(self) -> str:
        """Return a representation naming the state and the frame count."""
        state = "reading" if self.reading else "resting"
        return f"<TFFacet {state} over {len(self._frames)} frames>"

    @property
    def reading(self) -> bool:
        """Return whether the two subscriptions are open, which they are while a watch is."""
        return bool(self._sources)

    @property
    def broadcasting(self) -> bool:
        """Return whether the two publishers are open, which they are while a publishing watch is.

        ``False`` on a Machine that declares no frame of its own.
        """
        return bool(self._outward)

    @property
    def frames(self) -> Mapping[str, FrameLink]:
        """Return the frame network as the messages so far describe it, keyed by child frame.

        This is a live view of the facet's record, not a copy.
        """
        return self._frames

    @property
    def seen(self) -> Mapping[str, float]:
        """Return when each frame was last in a message, on this Runtime's own clock.

        A link stays in :attr:`frames` once it has arrived, so this is what tells a frame that is
        published and has not moved from one whose publisher stopped. It is separate from
        :class:`FrameLink` because an outgoing link has no arrival time. Like :attr:`frames`, it is
        a live view.
        """
        return self._seen

    @property
    def now(self) -> float:
        """Return what time it is in seconds, off the same clock a broadcast is stamped from.

        This is the ROS2 Node's clock and not :func:`time.monotonic`, because replayed data under
        ``use_sim_time`` advances the Node's clock and not the wall's. An unset clock reads zero,
        which makes every frame equally old and none stale. When the clock starts, :meth:`_record`
        re-bases earlier stamps so that the robot does not appear to go silent at once.
        """
        seconds, nanoseconds = self._clock()
        return seconds + nanoseconds / 1e9

    @property
    def roots(self) -> tuple[str, ...]:
        """Return every frame that is nobody's child, which is where the tree terminates."""
        return roots_of(self._frames)

    @property
    def satisfied(self) -> bool:
        """Return whether anything else is publishing transforms right now.

        Presence, not data: a publisher that has sent nothing yet still means the robot is on.
        Either topic counts. Our own publisher is discounted, because a subscription's matched count
        includes it (checked on Jazzy) and a required TF Binding would otherwise reach ``Running``
        against a robot that is off.
        """
        return any(source.subscription.publisher_count > self._ours_on(source.static) for source in self._sources)

    def _ours_on(self, static: bool) -> int:
        """Return how many publishers this facet holds on one of the two topics (one or none)."""
        return 1 if static in self._outward else 0

    def lookup(self, *, target: str, source: str) -> Transform | None:
        """Return where ``source`` sits in ``target``, or ``None`` when the buffer cannot say.

        Synchronous and non-blocking: a chain the buffer cannot complete answers ``None`` instead of
        waiting.
        """
        return self._lookup(target, source)

    async def watch(
        self,
        *,
        topic: str = TF_TOPIC,
        static_topic: str = TF_STATIC_TOPIC,
        depth: int = TF_DEPTH,
        broadcasting: bool = False,
    ) -> TFWatch:
        """Register interest in TF, opening both subscriptions if they were not open.

        The first watch decides where TF is read from and every later one must agree. One Runtime is
        one Domain ID and one frame network, so a second watch naming other topics is refused at
        init. Whether the Runtime publishes is a union instead: one declaration that wants to
        publish is enough.

        :param topic: The moving-transform topic, which a deployment may have remapped.
        :param static_topic: The static-transform topic.
        :param depth: How many messages each of the two subscriptions buffers.
        :param broadcasting: Whether what opened this watch also publishes a frame of its own, which
            advertises the two topics. It is asked here, at init, because :attr:`satisfied`
            discounts our own publisher and the count must not change meaning at the first tick.
        :return: A handle carrying its own lifetime. Close it when the Binding behind it is torn
            down. The subscriptions and publishers close with the last watch.
        :raises ROS2RuntimeError: When an earlier watch opened different sources.
        """
        wanted = _Sources(topic=topic, static_topic=static_topic, depth=depth)
        self._require_one_frame_network(wanted)
        watch = TFWatch(facet=self, release=self._watches.remove)
        self._watches.append(watch)
        try:
            await self._open(wanted)
            if broadcasting:
                await self._open_outward(wanted)
        except BaseException:
            self._watches.remove(watch)
            await self.settle()
            raise
        return watch

    async def broadcast(self, links: Iterable[FrameLink]) -> None:
        """Put every link on whichever of the two transform topics it belongs on.

        One message per topic rather than per link, so a subscriber wakes once. An empty call sends
        nothing. The stamp is one reading of the Runtime's clock for the whole call.

        :param links: The links to publish. Each goes on ``/tf_static`` when it says it is static
            and on ``/tf`` otherwise.
        :raises ROS2RuntimeError: When no watch on this facet declared that it
            broadcasts. This is checked before the payload, so an empty call is refused on the same
            Machine a full one would be.
        :raises PublishFailedError: When ``rclpy`` refused to send one of the two messages.
        """
        # Checked before the empty case, so the refusal depends on the declaration and not on the payload.
        self._require_advertised()
        grouped: dict[bool, list[FrameLink]] = {}
        for link in links:
            grouped.setdefault(link.static, []).append(link)
        seconds, nanoseconds = self._clock()
        # Static first, matching the order the subscriptions open in.
        for static in (True, False):
            held = grouped.get(static)
            if not held:
                continue
            publisher = self._outward[static]
            await publisher.publish(_tf_message(publisher.message_type, held, seconds=seconds, nanoseconds=nanoseconds))

    def _require_advertised(self) -> None:
        """Refuse a broadcast on a facet nothing declared an outward frame on.

        :raises ROS2RuntimeError: Always, when the publishers are not open.
        """
        if self._outward:
            return
        message = (
            "something asked this Runtime to publish a transform and it declares no frame of its "
            "own; the two transform publishers are opened by a watch that says it broadcasts, "
            "because a publisher on /tf is what every TF Binding in the system reads as a "
            "transform source being up"
        )
        raise ROS2RuntimeError(message)

    def _require_one_frame_network(self, wanted: _Sources) -> None:
        """Refuse a second watch that reads TF from somewhere else.

        :raises ROS2RuntimeError: When they do not agree.
        """
        held = self._reading
        if held is None or held == wanted:
            return
        message = (
            f"TF is already being read from {held.topic!r} and {held.static_topic!r} at depth "
            f"{held.depth}, and something asks for {wanted.topic!r} and {wanted.static_topic!r} "
            f"at depth {wanted.depth}; one Runtime is one Domain ID and one frame network, so "
            f"every declaration on this Machine reads it from the same two topics"
        )
        raise ROS2RuntimeError(message)

    async def settle(self) -> None:
        """Close both subscriptions and both publishers if nothing is watching. Called by a closing watch."""
        if self._watches or not (self._sources or self._outward):
            return
        self._reading = None
        sources, self._sources = self._sources, []
        for source in sources:
            source.task.cancel()
        for source in sources:
            await asyncio.gather(source.task, return_exceptions=True)
            await source.subscription.close()
        outward, self._outward = self._outward, {}
        for publisher in outward.values():
            await publisher.close()

    async def close(self) -> None:
        """End every watch and close every endpoint, which is what stopping the Runtime does."""
        for watch in tuple(self._watches):
            await watch.close()
        await self.settle()

    async def _open_outward(self, wanted: _Sources) -> None:
        """Advertise both transform topics, unless they are already advertised.

        They are the pair the subscriptions use, so a remapped ``/tf`` is published where it is
        read. What we publish comes back through our own subscription.
        """
        if self._outward:
            return
        opened: dict[bool, Publisher] = {}
        try:
            for topic, static in ((wanted.static_topic, True), (wanted.topic, False)):
                opened[static] = await self._advertise(
                    topic=topic,
                    message_type=TF_MESSAGE_TYPE,
                    depth=wanted.depth,
                    # Declared, and different per topic. See TF_STATIC_QOS.
                    qos=TF_STATIC_QOS if static else TF_QOS,
                )
        except BaseException:
            for publisher in opened.values():
                await publisher.close()
            raise
        self._outward = opened

    async def _open(self, wanted: _Sources) -> None:
        """Open both transform topics, unless they are already open.

        Static first, so a transient-local frame is in the record before the first moving one.
        Neither subscription declares a profile, so both are discovered: the publishers decide what
        is on somebody else's topic. That also recovers a transient-local ``/tf_static`` history
        whether its publisher was up before us or comes up later.
        """
        if self._sources:
            return
        opened: list[_Source] = []
        try:
            for topic, static in ((wanted.static_topic, True), (wanted.topic, False)):
                subscription = await self._subscribe(
                    topic=topic,
                    message_type=TF_MESSAGE_TYPE,
                    depth=wanted.depth,
                )
                task = asyncio.create_task(self._follow(subscription, static=static), name=f"ros2-tf{topic}")
                opened.append(_Source(subscription=subscription, task=task, static=static))
        except BaseException:
            for source in opened:
                source.task.cancel()
                await asyncio.gather(source.task, return_exceptions=True)
                await source.subscription.close()
            raise
        self._sources = opened
        self._reading = wanted

    async def _follow(self, subscription: Subscription, *, static: bool) -> None:
        """Record every message of one transform topic, until the subscription closes."""
        async for message in subscription:
            try:
                self._record(message, static=static)
            except Exception:
                # A malformed message must not end the stream.
                _LOGGER.exception("Could not read a transform message", topic=subscription.topic)

    def _record(self, message: Message, *, static: bool) -> None:
        """Put one message into the topology record and into the transform buffer.

        The record feeds the partition and the buffer answers composed poses. Feeding the buffer may
        raise on a transform tf2 refuses (an infinite value, a leading slash), which loses one
        transform. Arrival is stamped per frame, because one message does not come from one
        publisher (an arm driver dying silences only its frames), and from our own clock, not the
        message's stamp, which a drifting clock or a replayed bag can put anywhere.
        """
        arrived = self.now
        if arrived and not self._clock_started:
            # The clock has just started. Earlier stamps read zero and would all look as old as the clock's
            # new reading, so every frame would go BadNoCommunication at once. Their true age is unknown, so
            # now is the honest stamp.
            self._seen = dict.fromkeys(self._seen, arrived)
            self._clock_started = True
        for link in links(message, static=static):
            self._frames[link.child] = link
            self._seen[link.child] = arrived
        self._feed(message, static)


@dataclass(frozen=True, slots=True, kw_only=True)
class _Sources:
    """Where one declaration reads TF from, which every declaration on a Machine must share."""

    topic: str
    static_topic: str
    depth: int


@dataclass(frozen=True, slots=True, kw_only=True)
class _Source:
    """One of the two transform topics, live: its subscription and the task draining it.

    :ivar static: Which of the two it is, so a matched count knows whether one of its publishers is
        ours.
    """

    subscription: Subscription
    task: asyncio.Task[None]
    static: bool


def _tf_message(
    message_type: type[Message],
    links: Sequence[FrameLink],
    *,
    seconds: int,
    nanoseconds: int,
) -> Message:
    """Return the one ``TFMessage`` a list of links becomes.

    Built rather than stamped: :func:`~opensmi.ros2.runtime.stamp.stamped` does not descend into
    repeated message fields and every stamp in a ``TFMessage`` is inside one, so each stamp is
    written as its element is constructed. Nested types are read off the dataclass, since nothing
    here imports ``rclpy``.
    """
    element_type = _element_type(message_type)
    header_type = _nested_type(element_type, HEADER_FIELD)
    time_type = _nested_type(header_type, STAMP_FIELD)
    transform_type = _nested_type(element_type, TRANSFORM_FIELD)
    vector_type = _nested_type(transform_type, TRANSLATION_FIELD)
    rotation_type = _nested_type(transform_type, ROTATION_FIELD)
    stamp = _built(time_type, **{SECONDS_FIELD: seconds, NANOSECONDS_FIELD: nanoseconds})
    elements = [
        _built(
            element_type,
            **{
                HEADER_FIELD: _built(header_type, **{STAMP_FIELD: stamp, FRAME_FIELD: link.parent}),
                CHILD_FRAME_FIELD: link.child,
                TRANSFORM_FIELD: _built(
                    transform_type,
                    **{
                        TRANSLATION_FIELD: _built(
                            vector_type,
                            x=link.transform.translation[0],
                            y=link.transform.translation[1],
                            z=link.transform.translation[2],
                        ),
                        ROTATION_FIELD: _built(
                            rotation_type,
                            x=link.transform.rotation[0],
                            y=link.transform.rotation[1],
                            z=link.transform.rotation[2],
                            w=link.transform.rotation[3],
                        ),
                    },
                ),
            },
        )
        for link in links
    ]
    return _built(message_type, **{TRANSFORMS_FIELD: tuple(elements)})


def _built(message_type: type[Message], **values: Any) -> Message:
    """Return one message dataclass instance, built by field name.

    A message dataclass has no statically known fields, so type checkers reject naming them at a
    call site and accept a keyword splat.
    """
    return message_type(**values)


def _element_type(message_type: type[Message]) -> type[Message]:
    """Return the type one element of a ``TFMessage``'s repeated field is.

    :raises UnusableMessageTypeError: When the type advertised for the transform topics does not
        repeat a message at all, which is a robot whose ``tf2_msgs`` is not ``tf2_msgs``.
    """
    element = repeated_message(message_type, TRANSFORMS_FIELD)
    if element is None:
        message = (
            f"{message_type.ros2.type_name} is what this Runtime advertised for the transform "
            f"topics and it has no repeated {TRANSFORMS_FIELD!r} field of messages; a TFMessage "
            f"is a list of stamped transforms and there is nothing to publish into"
        )
        raise UnusableMessageTypeError(message)
    return element


def _nested_type(message_type: type[Message], field: str) -> type[Message]:
    """Return the message type one field of another holds.

    :raises UnusableMessageTypeError: When the field is not a nested message, which is the same
        condition as above one level down and says which level it was.
    """
    nested = message_type.ros2_nested.get(field)
    if nested is None:
        message = (
            f"{message_type.ros2.type_name} has no nested message field {field!r}, so a transform "
            f"cannot be built out of it; the transform topics carry {TF_MESSAGE_TYPE}"
        )
        raise UnusableMessageTypeError(message)
    return nested


def _named(value: object) -> str:
    """Return a frame id as the network keys it: text, without tf2's optional leading slash.

    tf2 treats ``/base_link`` and ``base_link`` as one frame, so do we.
    """
    return value.lstrip("/") if isinstance(value, str) else ""


def _transform_of(element: object) -> Transform | None:
    """Return one element's transform, or ``None`` when it does not carry the shape of one."""
    transform = getattr(element, TRANSFORM_FIELD, None)
    translation = _triple(getattr(transform, TRANSLATION_FIELD, None), ("x", "y", "z"))
    rotation = _triple(getattr(transform, ROTATION_FIELD, None), ("x", "y", "z", "w"))
    if translation is None or rotation is None:
        return None
    return Transform(translation=translation, rotation=rotation)  # type: ignore[arg-type]


def _triple(held: object, names: tuple[str, ...]) -> tuple[float, ...] | None:
    """Return the named float fields of one nested message, or ``None`` if any is missing."""
    values: list[float] = []
    for name in names:
        value = getattr(held, name, None)
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            return None
        values.append(float(value))
    return tuple(values)
