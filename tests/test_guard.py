# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The type-mismatch guard rejects a bad value before it reaches a send."""

from __future__ import annotations

import pytest

from opensmi.ros2.runtime.errors import FieldTypeMismatchError, FieldValueOutOfRangeError
from opensmi.ros2.runtime.guard import require_sendable
from tests import fixture_messages


def test_a_well_typed_message_is_accepted() -> None:
    require_sendable(fixture_messages.pose_stamped(frame_id="map", x=1.0))


def test_an_int_is_not_a_float64() -> None:
    # The aborting case: PyFloat_Check fails on an int and the process dies at send time.
    with pytest.raises(FieldTypeMismatchError, match="'x'"):
        require_sendable(fixture_messages.Point(x=1, y=0.0, z=0.0))


def test_a_bool_is_not_a_float64() -> None:
    with pytest.raises(FieldTypeMismatchError, match="float64"):
        require_sendable(fixture_messages.Point(x=True, y=0.0, z=0.0))


def test_a_string_is_not_a_float64() -> None:
    with pytest.raises(FieldTypeMismatchError, match="float64"):
        require_sendable(fixture_messages.Point(x="1.0", y=0.0, z=0.0))


def test_a_float_is_not_an_int32() -> None:
    with pytest.raises(FieldTypeMismatchError, match="int32"):
        require_sendable(fixture_messages.Time(sec=1.5, nanosec=0))


def test_a_bool_is_not_an_int32() -> None:
    with pytest.raises(FieldTypeMismatchError, match="int32"):
        require_sendable(fixture_messages.Time(sec=True, nanosec=0))


def test_a_number_is_not_a_string() -> None:
    with pytest.raises(FieldTypeMismatchError, match="frame_id"):
        require_sendable(fixture_messages.Header(stamp=fixture_messages.Time(sec=1, nanosec=2), frame_id=7))


def test_the_path_of_a_nested_field_is_named_in_full() -> None:
    message = fixture_messages.pose_stamped()
    broken = fixture_messages.PoseStamped(
        header=message.header,
        pose=fixture_messages.Pose(
            position=fixture_messages.Point(x=1, y=0.0, z=0.0),
            orientation=message.pose.orientation,
        ),
    )

    with pytest.raises(FieldTypeMismatchError, match=r"pose/position/x"):
        require_sendable(broken)


def test_a_nested_field_holding_the_wrong_message_is_refused() -> None:
    with pytest.raises(FieldTypeMismatchError, match="fixture_msgs/msg/Point"):
        require_sendable(fixture_messages.Pose(position=fixture_messages.Time(sec=1, nanosec=2), orientation=None))


def test_an_integer_past_its_width_is_refused_even_though_ros2_accepts_it() -> None:
    # `2**31` into an `int32` does not abort but goes onto the wire wrong: the one silent case.
    with pytest.raises(FieldValueOutOfRangeError, match="int32"):
        require_sendable(fixture_messages.Time(sec=2**31, nanosec=0))


def test_a_negative_value_in_an_unsigned_field_is_refused() -> None:
    with pytest.raises(FieldValueOutOfRangeError, match="uint32"):
        require_sendable(fixture_messages.Time(sec=0, nanosec=-1))


def test_a_sequence_is_walked_element_by_element() -> None:
    with pytest.raises(FieldTypeMismatchError, match=r"readings"):
        require_sendable(fixture_messages.Sensor(ok=True, labels=(), readings=(1.0, 2), payload=b""))


def test_a_sequence_of_the_right_element_type_is_accepted() -> None:
    require_sendable(fixture_messages.Sensor(ok=True, labels=("a",), readings=(1.0,), payload=b"\x01"))


def test_a_fixed_size_array_must_carry_exactly_its_length() -> None:
    pose = fixture_messages.pose_stamped().pose
    with pytest.raises(FieldValueOutOfRangeError, match="covariance"):
        require_sendable(fixture_messages.PoseWithCovariance(pose=pose, covariance=(0.0, 0.0, 0.0)))


def test_a_fixed_size_array_of_its_declared_length_is_accepted() -> None:
    pose = fixture_messages.pose_stamped().pose
    require_sendable(fixture_messages.PoseWithCovariance(pose=pose, covariance=(0.0, 0.0, 0.0, 0.0)))


def test_a_byte_container_wants_bytes_and_not_a_list_of_numbers() -> None:
    with pytest.raises(FieldTypeMismatchError, match="payload"):
        require_sendable(fixture_messages.Sensor(ok=True, labels=(), readings=(), payload=[1, 2]))


def test_a_scalar_where_a_sequence_belongs_is_refused() -> None:
    with pytest.raises(FieldTypeMismatchError, match="labels"):
        require_sendable(fixture_messages.Sensor(ok=True, labels="a", readings=(), payload=b""))


def test_a_bounded_string_may_not_exceed_its_bound() -> None:
    bounded = fixture_messages.build(
        fixture_messages.describe(
            "fixture_msgs/msg/Label",
            text=(fixture_messages.StringType(maximum_length=3), "string<3>"),
        )
    )

    require_sendable(bounded(text="abc"))
    with pytest.raises(FieldValueOutOfRangeError, match="text"):
        require_sendable(bounded(text="abcd"))


def test_a_bounded_sequence_may_not_exceed_its_bound() -> None:
    bounded = fixture_messages.build(
        fixture_messages.describe(
            "fixture_msgs/msg/Few",
            values=(
                fixture_messages.SequenceType(element=fixture_messages.PrimitiveType(name="int32"), maximum_length=2),
                "sequence<int32, 2>",
            ),
        )
    )

    require_sendable(bounded(values=(1, 2)))
    with pytest.raises(FieldValueOutOfRangeError, match="values"):
        require_sendable(bounded(values=(1, 2, 3)))
