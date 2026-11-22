# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Subscription handle's lifetime and signals, without ROS2 or a server."""

from __future__ import annotations

import asyncio
from typing import Any

import pytest

from opensmi.ros2.runtime.errors import LoopAffinityError, SubscriptionClosedError
from opensmi.ros2.runtime.messages import (
    FieldDescription,
    MessageDescription,
    PrimitiveType,
    build_message_dataclass,
)
from opensmi.ros2.runtime.subscription import Subscription, SubscriptionSink, TypeHashMismatch

# Dynamic dataclass: the generated stubs do not cover a fixture's fields.
COUNT: Any = build_message_dataclass(
    MessageDescription(
        type_name="fixture_msgs/msg/Count",
        fields=(FieldDescription(name="value", field_type=PrimitiveType(name="int32"), ros2_text="int32"),),
    )
)


def open_handle(depth=10):
    """Return a handle, its sink, and a list recording each release."""
    sink = SubscriptionSink(depth=depth)
    releases = []

    async def release():
        releases.append(True)

    handle = Subscription(topic="/count", message_type=COUNT, sink=sink, release=release)
    return handle, sink, releases


async def test_a_depth_below_one_is_refused() -> None:
    with pytest.raises(ValueError, match="at least 1"):
        SubscriptionSink(depth=0)


async def test_a_delivered_message_is_received() -> None:
    handle, sink, _ = open_handle()

    sink.deliver(COUNT(value=1))

    assert await handle.receive() == COUNT(value=1)
    assert handle.topic == "/count"
    assert handle.message_type is COUNT


async def test_receive_waits_for_a_message_that_has_not_arrived_yet() -> None:
    handle, sink, _ = open_handle()
    waiting = asyncio.ensure_future(handle.receive())

    sink.deliver(COUNT(value=7))

    assert await waiting == COUNT(value=7)


async def test_iteration_yields_every_message_and_ends_on_close() -> None:
    handle, sink, _ = open_handle()
    for value in (1, 2, 3):
        sink.deliver(COUNT(value=value))
    await handle.close()

    received: list[Any] = [message async for message in handle]

    assert [message.value for message in received] == [1, 2, 3]


async def test_a_full_buffer_drops_the_oldest() -> None:
    # Monitoring wants the newest value; an unbounded buffer on a 100 Hz topic leaks slowly.
    handle, sink, _ = open_handle(depth=2)

    for value in (1, 2, 3, 4):
        sink.deliver(COUNT(value=value))

    await handle.close()
    remaining: list[Any] = [message async for message in handle]
    assert [message.value for message in remaining] == [3, 4]


async def test_receiving_from_a_closed_and_empty_subscription_raises() -> None:
    handle, _, _ = open_handle()
    await handle.close()

    with pytest.raises(SubscriptionClosedError):
        await handle.receive()


async def test_closing_twice_releases_once() -> None:
    handle, _, releases = open_handle()

    await handle.close()
    await handle.close()

    assert releases == [True]


async def test_a_type_hash_mismatch_is_visible_on_the_handle() -> None:
    handle, sink, _ = open_handle()
    mismatch = TypeHashMismatch(
        topic="/count",
        node_name="/talker",
        topic_type="fixture_msgs/msg/Count",
        published_hash="RIHS01_aaaa",
        expected_hash="RIHS01_bbbb",
    )

    assert handle.type_hash_mismatches == ()
    sink.note_type_hash_mismatch(mismatch)

    assert handle.type_hash_mismatches == (mismatch,)


async def test_a_handle_awaited_on_another_event_loop_is_refused() -> None:
    # asyncio.Event binds to the first loop that awaits it; a second loop would hang, not error.
    handle, _, _ = open_handle()

    def on_another_loop():
        return asyncio.run(handle.receive())

    with pytest.raises(LoopAffinityError):
        await asyncio.to_thread(on_another_loop)


async def test_closing_from_another_event_loop_is_refused() -> None:
    handle, _, _ = open_handle()

    def on_another_loop():
        return asyncio.run(handle.close())

    with pytest.raises(LoopAffinityError):
        await asyncio.to_thread(on_another_loop)


async def test_a_handle_starts_matched_with_no_publishers() -> None:
    handle, _, _ = open_handle()

    assert handle.publisher_count == 0


async def test_the_publisher_count_follows_the_match_events() -> None:
    handle, sink, _ = open_handle()
    seen = []

    async def follow():
        seen.extend([count async for count in handle.publisher_counts()])

    task = asyncio.create_task(follow())
    await asyncio.sleep(0)
    sink.note_publishers(1)
    await asyncio.sleep(0)
    sink.note_publishers(0)
    await asyncio.sleep(0)
    await handle.close()
    await asyncio.wait_for(task, timeout=2.0)

    assert seen == [0, 1, 0]
    assert handle.publisher_count == 0


async def test_a_match_event_repeating_a_count_is_not_a_change() -> None:
    handle, sink, _ = open_handle()
    sink.note_publishers(2)
    seen = []

    async def follow():
        seen.extend([count async for count in handle.publisher_counts()])

    task = asyncio.create_task(follow())
    await asyncio.sleep(0)
    sink.note_publishers(2)
    await asyncio.sleep(0)
    await handle.close()
    await asyncio.wait_for(task, timeout=2.0)

    assert seen == [2]


async def test_following_the_publisher_count_of_a_closed_handle_ends_at_once() -> None:
    handle, _, _ = open_handle()
    await handle.close()

    assert [count async for count in handle.publisher_counts()] == [0]
