# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The graph facet: what is on the ROS2 graph, learned by polling because ``rclpy`` has no change event.

The facet reads only while a watch is registered and stops when the last one closes. A Machine whose
Bindings all declare their types never polls: their publisher presence comes from DDS match events
(:attr:`~opensmi.ros2.runtime.subscription.Subscription.publisher_count`). The poll serves Bindings
with no declared type, which cannot create a subscription.

A watch receives a delta against what it last saw, so a consumer that skipped polls gets one delta
covering all of them. Nothing here imports ``rclpy``; the Runtime hands the facet its read.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Final, final

import structlog

from opensmi.ros2.runtime.errors import SubscriptionClosedError

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Awaitable, Callable, Mapping

__all__ = [
    "EMPTY_GRAPH",
    "POLL_INTERVAL",
    "GraphDelta",
    "GraphFacet",
    "GraphSnapshot",
    "GraphWatch",
    "NameDelta",
    "diff",
]

POLL_INTERVAL: Final = 5.0
"""Seconds between graph reads while anything is watching (each read walks every endpoint)."""

_LOGGER = structlog.get_logger("opensmi.ros2.runtime.graph")

_NAMES = ("topics", "services", "actions")
"""The three graph vocabularies, which diff identically and are listed once here."""


@dataclass(frozen=True, slots=True, kw_only=True)
class GraphSnapshot:
    """Everything on the ROS2 graph at one moment, by name and by the types on that name.

    One ROS2 name may carry more than one type (publishers can disagree about a topic), so each
    value is a tuple.

    :ivar topics: Every topic name, mapped to the types advertised on it.
    :ivar services: Every service name, mapped to its types.
    :ivar actions: Every action name, mapped to its types.
    """

    topics: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    services: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    actions: Mapping[str, tuple[str, ...]] = field(default_factory=dict)


EMPTY_GRAPH: Final = GraphSnapshot()
"""The graph before anything has been read: what a new watch diffs its first delta against."""


@dataclass(frozen=True, slots=True, kw_only=True)
class NameDelta:
    """What changed in one of the graph's three vocabularies.

    :ivar appeared: Names that were not there before, with the types they carry.
    :ivar vanished: Names that are no longer there, in sorted order.
    :ivar retyped: Names that stayed and whose types changed, with the types they carry now.
        Separate from ``appeared`` because a Binding that already took its nodes from a type
        cannot take them again.
    """

    appeared: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    vanished: tuple[str, ...] = ()
    retyped: Mapping[str, tuple[str, ...]] = field(default_factory=dict)

    @property
    def empty(self) -> bool:
        """Return whether nothing changed in this vocabulary."""
        return not (self.appeared or self.vanished or self.retyped)

    def types_now_on(self, name: str) -> tuple[str, ...] | None:
        """Return the types this delta leaves on one name, or ``None`` if it carries no news of it.

        Appearing and being retyped count the same here, for a consumer still waiting to learn a
        type. A name that vanished answers ``None``.
        """
        if name in self.appeared:
            return self.appeared[name]
        return self.retyped.get(name)


@dataclass(frozen=True, slots=True, kw_only=True)
class GraphDelta:
    """What changed on the graph between two snapshots, one entry per vocabulary."""

    topics: NameDelta = NameDelta()
    services: NameDelta = NameDelta()
    actions: NameDelta = NameDelta()

    @property
    def empty(self) -> bool:
        """Return whether the graph did not move at all."""
        return all(getattr(self, name).empty for name in _NAMES)


def diff(previous: GraphSnapshot, current: GraphSnapshot) -> GraphDelta:
    """Return what changed between two snapshots (pure, so a watch can compute its delta late)."""
    return GraphDelta(**{name: _name_delta(getattr(previous, name), getattr(current, name)) for name in _NAMES})


@final
class GraphWatch:
    """One consumer's view of the graph, from :meth:`GraphFacet.watch`. Close it so the poll can stop."""

    def __init__(self, *, facet: GraphFacet, release: Callable[[GraphWatch], None]) -> None:
        """Bind a watch to the facet it reads and the deregistration the facet prepared."""
        self._facet = facet
        self._release = release
        self._seen = EMPTY_GRAPH
        self._arrival = asyncio.Event()
        self._closed = False
        self._arrival.set()

    def __repr__(self) -> str:
        """Return a representation naming the watch's state."""
        return f"<GraphWatch ({'closed' if self._closed else 'open'})>"

    @property
    def closed(self) -> bool:
        """Return whether this watch's lifetime has ended."""
        return self._closed

    def wake(self) -> None:
        """Tell this watch the facet has read the graph again."""
        self._arrival.set()

    async def receive(self) -> GraphDelta:
        """Await the next change to the graph, as a delta against what this watch last received.

        :raises SubscriptionClosedError: When the watch has been closed.
        """
        while True:
            if self._closed:
                message = "this graph watch is closed; the graph is no longer being read for it"
                raise SubscriptionClosedError(message)
            delta = diff(self._seen, self._facet.snapshot)
            self._seen = self._facet.snapshot
            if not delta.empty:
                return delta
            # Nothing awaits between the clear and the wait, so no poll can land in between.
            self._arrival.clear()
            await self._arrival.wait()

    def __aiter__(self) -> AsyncIterator[GraphDelta]:
        """Return an iterator yielding deltas until the watch is closed."""
        return self._stream()

    async def _stream(self) -> AsyncIterator[GraphDelta]:
        """Yield deltas until the watch is closed, then stop cleanly."""
        while True:
            try:
                yield await self.receive()
            except SubscriptionClosedError:
                return

    async def close(self) -> None:
        """End this watch, and with it the poll if it was the last one. Idempotent."""
        if self._closed:
            return
        self._closed = True
        self._arrival.set()
        self._release(self)
        await self._facet.settle()


@final
class GraphFacet:
    """What is on the ROS2 graph, read on a timer while anything is watching (see ``Runtime.graph``)."""

    def __init__(
        self,
        *,
        read: Callable[[], Awaitable[GraphSnapshot]],
        interval: float = POLL_INTERVAL,
    ) -> None:
        """Describe a facet without reading anything.

        :param read: How to read the graph once.
        :param interval: Seconds between reads while anything is watching.
        """
        self._read = read
        self._interval = interval
        self._snapshot = EMPTY_GRAPH
        self._watches: list[GraphWatch] = []
        self._task: asyncio.Task[None] | None = None
        self._polls = 0

    def __repr__(self) -> str:
        """Return a representation naming the poll state and how often it has read."""
        state = "polling" if self.polling else "resting"
        return f"<GraphFacet {state} after {self._polls} polls>"

    @property
    def interval(self) -> float:
        """Return the seconds between reads while anything is watching."""
        return self._interval

    @property
    def polls(self) -> int:
        """Return how often the graph has been read (0 for a fully declared Machine)."""
        return self._polls

    @property
    def polling(self) -> bool:
        """Return whether the poll is running, which is exactly while a watch is open."""
        return self._task is not None

    @property
    def snapshot(self) -> GraphSnapshot:
        """Return the graph as the last read left it (empty before the first one)."""
        return self._snapshot

    def watch(self) -> GraphWatch:
        """Register interest in the graph, starting the poll if it was not running.

        :return: A handle carrying the deltas. The poll stops when the last watch closes.
        """
        watch = GraphWatch(facet=self, release=self._watches.remove)
        self._watches.append(watch)
        self._start()
        return watch

    async def refresh(self) -> GraphSnapshot:
        """Read the graph once, now, and hold the result. Counts as a poll."""
        self._polls += 1
        self._snapshot = await self._read()
        return self._snapshot

    async def settle(self) -> None:
        """Stop the poll if nothing is watching any more. Called by a watch as it closes."""
        if self._watches or self._task is None:
            return
        task, self._task = self._task, None
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)

    async def close(self) -> None:
        """End every watch and stop the poll, which is what stopping the Runtime does to it."""
        for watch in tuple(self._watches):
            await watch.close()
        await self.settle()

    def _start(self) -> None:
        """Start the poll loop, unless it is already running or nothing is watching."""
        if self._task is not None or not self._watches:
            return
        self._task = asyncio.create_task(self._poll(), name="ros2-graph-poll")

    async def _poll(self) -> None:
        """Read the graph on the interval and wake every watch, until cancelled."""
        while True:
            await asyncio.sleep(self._interval)
            try:
                await self.refresh()
            except Exception:
                # A background read has no caller to raise at, and the next read will probably work.
                _LOGGER.exception("Could not read the ROS2 graph; the poll continues")
                continue
            for watch in tuple(self._watches):
                watch.wake()


def _name_delta(previous: Mapping[str, tuple[str, ...]], current: Mapping[str, tuple[str, ...]]) -> NameDelta:
    """Return what changed in one vocabulary between two snapshots."""
    return NameDelta(
        appeared={name: types for name, types in current.items() if name not in previous},
        vanished=tuple(sorted(name for name in previous if name not in current)),
        retyped={name: types for name, types in current.items() if name in previous and previous[name] != types},
    )
