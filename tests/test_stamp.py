# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Which message field is a header timestamp and when it is still unset."""

from __future__ import annotations

from typing import Any, cast

from opensmi.ros2.runtime.stamp import stamp_paths, stamped
from tests import fixture_messages

NOW = (1_764_000_000, 250_000)


def test_a_stamp_is_a_time_under_that_name_and_nothing_else() -> None:
    # duration is a Time but not a stamp; Duration has the same two fields.
    assert stamp_paths(fixture_messages.Timed) == (("stamp",),)


def test_a_field_called_stamp_that_is_not_a_time_is_not_one() -> None:
    assert stamp_paths(fixture_messages.Mislabelled) == ()


def test_a_message_with_no_header_has_no_stamp() -> None:
    assert stamp_paths(fixture_messages.Point) == ()


def test_a_stamp_inside_a_header_is_found_by_its_path() -> None:
    assert stamp_paths(fixture_messages.PoseStamped) == (("header", "stamp"),)


def test_a_repeated_message_field_is_not_descended_into() -> None:
    assert stamp_paths(fixture_messages.Path) == (("header", "stamp"),)


def test_a_message_with_no_stamp_comes_back_as_the_very_same_object() -> None:
    point = fixture_messages.Point(x=1.0, y=2.0, z=3.0)

    assert stamped(point, seconds=NOW[0], nanoseconds=NOW[1]) is point


def test_a_stamp_that_is_still_zero_is_filled() -> None:
    timed = fixture_messages.Timed(
        stamp=fixture_messages.Time(sec=0, nanosec=0),
        duration=fixture_messages.Time(sec=0, nanosec=0),
        label="go",
    )

    filled = cast("Any", stamped(timed, seconds=NOW[0], nanoseconds=NOW[1]))

    assert filled.stamp == fixture_messages.Time(sec=NOW[0], nanosec=NOW[1])
    assert filled.duration == fixture_messages.Time(sec=0, nanosec=0)
    assert filled.label == "go"


def test_a_stamp_a_client_wrote_is_left_exactly_as_it_arrived() -> None:
    # A zero stamp means unset in ROS2; anything else is a client's own answer.
    written = fixture_messages.Time(sec=17, nanosec=4)
    timed = fixture_messages.Timed(stamp=written, duration=written, label="go")

    assert stamped(timed, seconds=NOW[0], nanoseconds=NOW[1]) is timed


def test_a_stamp_with_only_nanoseconds_set_counts_as_written() -> None:
    timed = fixture_messages.Timed(
        stamp=fixture_messages.Time(sec=0, nanosec=1),
        duration=fixture_messages.Time(sec=0, nanosec=0),
        label="go",
    )

    assert stamped(timed, seconds=NOW[0], nanoseconds=NOW[1]) is timed


def test_a_nested_stamp_is_filled_and_everything_around_it_is_carried_across() -> None:
    pose = fixture_messages.pose_stamped(frame_id="map", x=1.5)
    unset = fixture_messages.PoseStamped(
        header=fixture_messages.Header(stamp=fixture_messages.Time(sec=0, nanosec=0), frame_id="map"),
        pose=pose.pose,
    )

    filled = cast("Any", stamped(unset, seconds=NOW[0], nanoseconds=NOW[1]))

    assert filled.header.stamp == fixture_messages.Time(sec=NOW[0], nanosec=NOW[1])
    assert filled.header.frame_id == "map"
    assert filled.pose is unset.pose
