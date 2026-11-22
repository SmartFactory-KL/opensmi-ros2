# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The ROS2 side of the Structure DataTypes: descriptions as dataclasses, and the checks of a write."""

from __future__ import annotations

import dataclasses
from typing import Any

import pytest
from asyncua import ua
from asyncua.common.utils import Buffer
from asyncua.ua.ua_binary import struct_from_binary, struct_to_binary

from opensmi.ros2.server.structures import structure_class, structure_elements, structure_instance
from tests.fixture_structures import CONTROLLER, DURATION, POINT, TRAJECTORY, point


def test_the_class_is_named_by_the_ros2_type_and_has_the_fields_of_the_message() -> None:
    assert POINT.__name__ == "trajectory_msgs/msg/JointTrajectoryPoint"
    assert [each.name for each in dataclasses.fields(POINT)] == [
        "positions",
        "velocities",
        "time_from_start",
        "covariance",
        "raw",
        "valid",
    ]
    assert structure_class(POINT.__name__, (("x", ua.Double, 0),)) is not POINT


def test_equal_fields_make_the_same_class() -> None:
    fields = (("sec", ua.Int32, 0), ("nanosec", ua.UInt32, 0))

    assert structure_class(DURATION.__name__, fields) is DURATION


def test_the_annotations_are_the_precise_ua_types() -> None:
    hints = POINT.__annotations__

    assert hints["positions"] == list[ua.Double]
    assert hints["raw"] is ua.ByteString
    assert hints["valid"] is ua.Boolean
    assert hints["time_from_start"] is DURATION
    assert DURATION.__annotations__ == {"sec": ua.Int32, "nanosec": ua.UInt32}
    assert TRAJECTORY.__annotations__["points"] == list[POINT]


def test_a_nested_message_is_an_instance_of_its_own_class() -> None:
    instance = structure_instance(POINT, point(7.0))

    assert instance.time_from_start == DURATION(sec=7, nanosec=500)


def test_the_maps_of_a_projection_round_trip_through_instances() -> None:
    values = {"joint_names": ["a", "b"], "points": [point(1.0), point(5.0, positions=0)]}

    instance = structure_instance(TRAJECTORY, values)

    assert structure_elements(TRAJECTORY, [instance]) == [values]


def test_instances_survive_the_binary_encoding() -> None:
    values = {"joint_names": ["a"], "points": [point(2.0, positions=3)]}

    received: Any = struct_from_binary(TRAJECTORY, Buffer(struct_to_binary(structure_instance(TRAJECTORY, values))))

    assert structure_elements(TRAJECTORY, [received]) == [values]


def test_a_missing_field_in_the_values_is_a_fault_of_the_caller() -> None:
    with pytest.raises(KeyError, match="valid"):
        structure_instance(POINT, {key: value for key, value in point().items() if key != "valid"})


def test_a_fixed_size_array_of_the_wrong_length_is_refused() -> None:
    spoiled = structure_instance(POINT, {**point(), "covariance": [1.0, 2.0]})

    with pytest.raises(ValueError, match=r"covariance.*3 elements, and 2 were given"):
        structure_elements(POINT, [spoiled])


def nulled(datatype: type, values: dict[str, object], field: str, nested: object = None) -> object:
    """Return an instance as asyncua decodes a null: the field is None."""
    instance = structure_instance(datatype, values)
    setattr(instance, field, nested)
    return instance


@pytest.mark.parametrize("field", ["positions", "raw"])
def test_a_null_is_refused_and_not_read_as_empty(field: str) -> None:
    with pytest.raises(ValueError, match=rf"{field}.*is null"):
        structure_elements(POINT, [nulled(POINT, point(), field)])


def test_a_null_string_in_an_array_is_refused() -> None:
    spoiled = structure_instance(TRAJECTORY, {"joint_names": ["a", None], "points": []})

    with pytest.raises(ValueError, match=r"joint_names.*is null"):
        structure_elements(TRAJECTORY, [spoiled])


def test_a_null_inside_a_nested_element_is_refused() -> None:
    inner = nulled(POINT, point(), "positions")
    spoiled = nulled(TRAJECTORY, {"joint_names": [], "points": []}, "points", [inner])

    with pytest.raises(ValueError, match=r"element 0: .*positions.*is null"):
        structure_elements(TRAJECTORY, [spoiled])


def test_something_that_is_not_an_array_of_the_structure_is_refused() -> None:
    controller = structure_instance(CONTROLLER, {"name": "a", "claimed_interfaces": []})

    with pytest.raises(ValueError, match="not an array"):
        structure_elements(POINT, controller)
    with pytest.raises(ValueError, match=r"element 1: .*float"):
        structure_elements(POINT, [structure_instance(POINT, point()), 2.5])
    with pytest.raises(ValueError, match=r"element 0: .*arrived as"):
        structure_elements(POINT, [controller])


def test_an_empty_array_is_a_value() -> None:
    assert structure_elements(POINT, []) == []
