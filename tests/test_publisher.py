# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Publisher handle's lifetime and the three checks before the wire."""

from __future__ import annotations

from typing import Any

import pytest

from opensmi.ros2.runtime.errors import (
    FieldTypeMismatchError,
    PublishFailedError,
    ROS2RuntimeError,
)
from opensmi.ros2.runtime.publisher import Publisher
from tests import fixture_messages

NOW = (1_764_000_000, 250_000)


class Wire:
    """Everything the Runtime hands a handle, as a test can drive it."""

    def __init__(self, *, refuse: bool = False) -> None:
        self.sent: list[Any] = []
        self.released = 0
        self.refuse = refuse

    async def send(self, message: Any) -> None:
        if self.refuse:
            refused = "rclpy refused to publish on /cmd"
            raise PublishFailedError(refused)
        self.sent.append(message)

    async def release(self) -> None:
        self.released += 1


def opened(wire: Wire, message_type: Any = fixture_messages.Point) -> Publisher:
    return Publisher(
        topic="/cmd",
        message_type=message_type,
        clock=lambda: NOW,
        send=wire.send,
        release=wire.release,
    )


async def test_a_handle_names_the_topic_and_the_type_it_carries() -> None:
    publisher = opened(Wire())

    assert publisher.topic == "/cmd"
    assert publisher.message_type is fixture_messages.Point
    assert "fixture_msgs/msg/Point" in repr(publisher)
    assert not publisher.closed


async def test_publishing_sends_the_message_and_counts_it() -> None:
    wire = Wire()
    publisher = opened(wire)

    await publisher.publish(fixture_messages.Point(x=1.0, y=2.0, z=3.0))

    assert wire.sent == [fixture_messages.Point(x=1.0, y=2.0, z=3.0)]
    assert publisher.published == 1


async def test_the_header_stamp_is_filled_on_the_way_out() -> None:
    wire = Wire()
    publisher = opened(wire, fixture_messages.Timed)
    zero = fixture_messages.Time(sec=0, nanosec=0)

    await publisher.publish(fixture_messages.Timed(stamp=zero, duration=zero, label="go"))

    (sent,) = wire.sent
    assert sent.stamp == fixture_messages.Time(sec=NOW[0], nanosec=NOW[1])


async def test_a_payload_of_the_wrong_type_never_reaches_the_wire() -> None:
    wire = Wire()
    publisher = opened(wire)

    with pytest.raises(TypeError, match="fixture_msgs/msg/Point"):
        await publisher.publish(fixture_messages.Voltage(data=1.0))

    assert wire.sent == []
    assert publisher.published == 0


async def test_a_field_of_the_wrong_python_type_is_refused_before_the_wire() -> None:
    # Must be refused here: rclpy aborts the process.
    wire = Wire()
    publisher = opened(wire)

    with pytest.raises(FieldTypeMismatchError):
        await publisher.publish(fixture_messages.Point(x=1, y=2.0, z=3.0))

    assert wire.sent == []
    assert publisher.published == 0


async def test_the_guard_runs_on_the_stamped_message_and_not_on_the_one_handed_over() -> None:
    # The stamp goes in before the guard, so it is checked too.
    wire = Wire()
    publisher = Publisher(
        topic="/cmd",
        message_type=fixture_messages.Timed,
        clock=lambda: (2**31, 0),
        send=wire.send,
        release=wire.release,
    )
    zero = fixture_messages.Time(sec=0, nanosec=0)

    with pytest.raises(Exception, match="stamp"):
        await publisher.publish(fixture_messages.Timed(stamp=zero, duration=zero, label="go"))

    assert wire.sent == []


async def test_a_refused_send_does_not_count_as_published() -> None:
    wire = Wire(refuse=True)
    publisher = opened(wire)

    with pytest.raises(PublishFailedError):
        await publisher.publish(fixture_messages.Point(x=1.0, y=2.0, z=3.0))

    assert publisher.published == 0


async def test_a_closed_publisher_sends_nothing_further() -> None:
    wire = Wire()
    publisher = opened(wire)

    await publisher.close()

    assert publisher.closed
    assert "closed" in repr(publisher)
    with pytest.raises(ROS2RuntimeError):
        await publisher.publish(fixture_messages.Point(x=1.0, y=2.0, z=3.0))
    assert wire.sent == []


async def test_closing_twice_releases_once() -> None:
    wire = Wire()
    publisher = opened(wire)

    await publisher.close()
    await publisher.close()

    assert wire.released == 1
