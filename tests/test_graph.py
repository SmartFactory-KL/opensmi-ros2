# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The graph poll's diffing and self-disable rule, against a reader written in the test."""

from __future__ import annotations

import asyncio

import pytest

from opensmi.ros2.runtime.errors import SubscriptionClosedError
from opensmi.ros2.runtime.graph import (
    EMPTY_GRAPH,
    POLL_INTERVAL,
    GraphFacet,
    GraphSnapshot,
    diff,
)


def snapshot(**topics: tuple[str, ...]) -> GraphSnapshot:
    """Return a snapshot of topics alone, which is what the graph-driven case reads."""
    return GraphSnapshot(topics={f"/{name}": types for name, types in topics.items()})


class Reader:
    """A graph the test moves by hand, and a count of how often it was read."""

    def __init__(self, *snapshots: GraphSnapshot) -> None:
        self.current = snapshots[0] if snapshots else EMPTY_GRAPH
        self.reads = 0

    async def __call__(self) -> GraphSnapshot:
        self.reads += 1
        return self.current


# -- the diff ------------------------------------------------------------------------------


def test_a_topic_that_was_not_there_before_appeared() -> None:
    delta = diff(EMPTY_GRAPH, snapshot(scan=("sensor_msgs/msg/LaserScan",)))

    assert delta.topics.appeared == {"/scan": ("sensor_msgs/msg/LaserScan",)}
    assert delta.topics.vanished == ()
    assert not delta.empty


def test_a_topic_that_is_gone_vanished() -> None:
    delta = diff(snapshot(scan=("sensor_msgs/msg/LaserScan",)), EMPTY_GRAPH)

    assert delta.topics.vanished == ("/scan",)
    assert delta.topics.appeared == {}


def test_a_topic_whose_type_changed_is_retyped_rather_than_appearing_twice() -> None:
    delta = diff(snapshot(scan=("a/msg/A",)), snapshot(scan=("b/msg/B",)))

    assert delta.topics.retyped == {"/scan": ("b/msg/B",)}
    assert delta.topics.appeared == {}
    assert delta.topics.vanished == ()


def test_an_unchanged_graph_is_an_empty_delta() -> None:
    held = snapshot(scan=("a/msg/A",))

    assert diff(held, held).empty


def test_a_name_that_appeared_and_one_that_was_retyped_both_carry_their_types_now() -> None:
    delta = diff(snapshot(scan=("a/msg/A", "b/msg/B")), snapshot(scan=("a/msg/A",), odom=("c/msg/C",)))

    assert delta.topics.types_now_on("/scan") == ("a/msg/A",)
    assert delta.topics.types_now_on("/odom") == ("c/msg/C",)


def test_a_name_this_delta_says_nothing_about_carries_no_types_now() -> None:
    delta = diff(snapshot(scan=("a/msg/A",)), EMPTY_GRAPH)

    assert delta.topics.types_now_on("/scan") is None  # it vanished; nothing on it can be read
    assert delta.topics.types_now_on("/never_seen") is None


def test_services_and_actions_diff_the_same_way_as_topics() -> None:
    before = GraphSnapshot(services={"/set": ("a/srv/Set",)}, actions={"/go": ("a/action/Go",)})
    after = GraphSnapshot(services={}, actions={"/go": ("a/action/Go",), "/stop": ("a/action/Stop",)})

    delta = diff(before, after)

    assert delta.services.vanished == ("/set",)
    assert delta.actions.appeared == {"/stop": ("a/action/Stop",)}
    assert not delta.empty


# -- the self-disable rule -----------------------------------------------------------------


async def test_a_facet_nobody_watches_never_reads_the_graph() -> None:
    reader = Reader(snapshot(scan=("a/msg/A",)))

    facet = GraphFacet(read=reader, interval=0.01)
    await asyncio.sleep(0.05)

    assert facet.polls == 0
    assert reader.reads == 0
    assert not facet.polling


async def test_the_poll_starts_on_the_first_watch_and_stops_after_the_last() -> None:
    reader = Reader(snapshot(scan=("a/msg/A",)))
    facet = GraphFacet(read=reader, interval=0.01)

    first = facet.watch()
    second = facet.watch()
    await asyncio.sleep(0.05)
    assert facet.polling
    assert facet.polls > 0

    await first.close()
    assert facet.polling

    await second.close()
    assert not facet.polling
    resting = facet.polls
    await asyncio.sleep(0.05)
    assert facet.polls == resting


async def test_the_default_interval_is_five_seconds() -> None:
    assert POLL_INTERVAL == 5.0
    assert GraphFacet(read=Reader()).interval == POLL_INTERVAL


# -- what a watch carries ------------------------------------------------------------------


async def test_a_watch_receives_the_topic_that_appeared() -> None:
    reader = Reader(EMPTY_GRAPH)
    facet = GraphFacet(read=reader, interval=0.01)
    watch = facet.watch()

    reader.current = snapshot(scan=("a/msg/A",))
    delta = await asyncio.wait_for(watch.receive(), timeout=2.0)

    assert delta.topics.appeared == {"/scan": ("a/msg/A",)}
    await watch.close()


async def test_a_watch_opened_after_the_topic_appeared_still_sees_it() -> None:
    reader = Reader(snapshot(scan=("a/msg/A",)))
    facet = GraphFacet(read=reader, interval=0.01)
    opener = facet.watch()
    await asyncio.wait_for(opener.receive(), timeout=2.0)

    late = facet.watch()
    delta = await asyncio.wait_for(late.receive(), timeout=2.0)

    assert delta.topics.appeared == {"/scan": ("a/msg/A",)}
    await opener.close()
    await late.close()


async def test_a_watch_that_read_nothing_for_two_polls_gets_one_delta_covering_both() -> None:
    reader = Reader(EMPTY_GRAPH)
    facet = GraphFacet(read=reader, interval=0.01)
    watch = facet.watch()

    reader.current = snapshot(scan=("a/msg/A",))
    await asyncio.sleep(0.05)
    reader.current = snapshot(scan=("a/msg/A",), odom=("a/msg/B",))
    await asyncio.sleep(0.05)
    delta = await asyncio.wait_for(watch.receive(), timeout=2.0)

    assert delta.topics.appeared == {"/scan": ("a/msg/A",), "/odom": ("a/msg/B",)}
    await watch.close()


async def test_iterating_a_watch_ends_when_it_is_closed() -> None:
    reader = Reader(EMPTY_GRAPH)
    facet = GraphFacet(read=reader, interval=0.01)
    watch = facet.watch()
    seen: list[str] = []
    arrived = asyncio.Event()

    async def follow() -> None:
        async for delta in watch:
            seen.extend(delta.topics.appeared)
            arrived.set()

    task = asyncio.create_task(follow())
    reader.current = snapshot(scan=("a/msg/A",))
    await asyncio.wait_for(arrived.wait(), timeout=5.0)
    await watch.close()
    await asyncio.wait_for(task, timeout=2.0)

    assert seen == ["/scan"]


async def test_receiving_from_a_closed_watch_says_so() -> None:
    facet = GraphFacet(read=Reader(), interval=0.01)
    watch = facet.watch()
    await watch.close()

    with pytest.raises(SubscriptionClosedError):
        await watch.receive()


async def test_closing_the_facet_ends_every_watch_it_handed_out() -> None:
    facet = GraphFacet(read=Reader(), interval=0.01)
    watch = facet.watch()

    await facet.close()

    assert watch.closed
    assert not facet.polling


async def test_a_graph_read_that_fails_does_not_end_the_poll() -> None:
    reads = 0

    async def unreliable() -> GraphSnapshot:
        nonlocal reads
        reads += 1
        if reads == 1:
            message = "the node went away mid-poll"
            raise RuntimeError(message)
        return snapshot(scan=("a/msg/A",))

    facet = GraphFacet(read=unreliable, interval=0.01)
    watch = facet.watch()

    delta = await asyncio.wait_for(watch.receive(), timeout=2.0)

    assert delta.topics.appeared == {"/scan": ("a/msg/A",)}
    await watch.close()
