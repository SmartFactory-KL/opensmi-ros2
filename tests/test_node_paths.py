# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The node path the plan computes for each rendered field, without a server or rclpy."""

from __future__ import annotations

import pytest

from opensmi.ros2.runtime.selection import FieldSelection
from opensmi.ros2.server.errors import BindingNameError, DuplicateVariableNameError
from opensmi.ros2.server.projection import (
    PlanVariable,
    ProjectionContext,
    inner_levels,
    narrowed,
    shaped,
)
from opensmi.ros2.server.recipes import GENERIC_REGISTRY, Scalar
from tests import fixture_messages as messages
from tests.fixture_messages import FLOAT64, STRING, build, describe, nested


def shape(message_type, **selection) -> tuple[PlanVariable, ...]:
    context = narrowed(
        ProjectionContext(registry=GENERIC_REGISTRY, selected=frozenset()), message_type, FieldSelection(**selection)
    )
    plan = shaped(message_type, context)
    assert all(isinstance(entry, PlanVariable) for entry in plan)
    return plan  # type: ignore[return-value]


def test_a_nested_field_is_a_path_of_pascal_case_names() -> None:
    plan = shape(messages.PoseStamped)

    assert [entry.node_path for entry in plan] == [
        ("Header", "Stamp", "Sec"),
        ("Header", "Stamp", "Nanosec"),
        ("Header", "FrameId"),
        ("Pose", "Position", "X"),
        ("Pose", "Position", "Y"),
        ("Pose", "Position", "Z"),
        ("Pose", "Orientation", "X"),
        ("Pose", "Orientation", "Y"),
        ("Pose", "Orientation", "Z"),
        ("Pose", "Orientation", "W"),
    ]


def test_every_level_keeps_the_raw_ros2_name_as_its_display_name() -> None:
    frame_id = shape(messages.PoseStamped)[2]

    assert frame_id.display_names == ("header", "frame_id")


def test_the_flat_rendering_joins_the_path_with_dots() -> None:
    assert shape(messages.PoseStamped)[2].full_name == "Header.FrameId"


def test_the_inner_levels_name_the_ros2_type_of_each_structure() -> None:
    levels = inner_levels(shape(messages.PoseStamped))

    assert {path: level.ros2_type for path, level in levels.items()} == {
        ("Header",): "fixture_msgs/msg/Header",
        ("Header", "Stamp"): "fixture_msgs/msg/Time",
        ("Pose",): "fixture_msgs/msg/Pose",
        ("Pose", "Position"): "fixture_msgs/msg/Point",
        ("Pose", "Orientation"): "fixture_msgs/msg/Quaternion",
    }
    assert levels[("Header", "Stamp")].display_name == "stamp"


def test_a_repeated_message_stays_one_leaf_with_no_inner_level() -> None:
    plan = shape(messages.Path, include=("poses",))

    assert [entry.node_path for entry in plan] == [("Poses",)]
    assert inner_levels(plan) == {}


def test_excluding_a_field_leaves_the_levels_of_the_rest() -> None:
    plan = shape(messages.PoseStamped, exclude=("header", "pose/orientation"))

    assert {entry.node_path for entry in plan} == {
        ("Pose", "Position", "X"),
        ("Pose", "Position", "Y"),
        ("Pose", "Position", "Z"),
    }
    assert set(inner_levels(plan)) == {("Pose",), ("Pose", "Position")}


def test_a_rename_replaces_the_name_and_the_display_name_of_a_leaf() -> None:
    plan = shape(messages.PoseStamped, rename={"header/frame_id": "Frame"})

    assert plan[2].node_path == ("Header", "Frame")
    assert plan[2].display_names == ("header", "Frame")


def test_a_rename_of_an_inner_level_renames_the_whole_subtree() -> None:
    plan = shape(messages.PoseStamped, include=("pose/position/*",), rename={"pose": "Where"})

    assert [entry.node_path for entry in plan] == [
        ("Where", "Position", "X"),
        ("Where", "Position", "Y"),
        ("Where", "Position", "Z"),
    ]
    assert inner_levels(plan)[("Where",)].display_name == "Where"


def test_a_message_collapsed_to_one_value_ends_the_path_at_its_field() -> None:
    registry = GENERIC_REGISTRY.extended([Scalar("fixture_msgs/msg/Voltage")])
    plan_message = build(describe("fixture_msgs/msg/Battery", voltage=nested("fixture_msgs/msg/Voltage")))
    context = narrowed(ProjectionContext(registry=registry, selected=frozenset()), plan_message, FieldSelection())

    (entry,) = shaped(plan_message, context)

    assert isinstance(entry, PlanVariable)
    assert entry.node_path == ("Voltage",)
    assert entry.display_names == ("voltage",)
    assert inner_levels((entry,)) == {}


def test_two_siblings_that_convert_to_one_name_are_refused_with_both_names() -> None:
    clash = build(describe("fixture_msgs/msg/Clash", x_1=FLOAT64, x1=FLOAT64))

    with pytest.raises(DuplicateVariableNameError, match=r"'x_1' and 'x1'.*'X1'"):
        shape(clash)


def test_two_renames_to_one_new_name_are_refused_with_both_names() -> None:
    with pytest.raises(DuplicateVariableNameError, match=r"'stamp' and 'frame_id'.*'Same'"):
        shape(messages.Header, rename={"frame_id": "Same", "stamp": "Same"})


def test_a_rename_that_collides_with_the_converted_name_of_a_sibling_is_refused() -> None:
    with pytest.raises(DuplicateVariableNameError, match=r"'stamp' and 'frame_id'.*'Stamp'"):
        shape(messages.Header, rename={"frame_id": "Stamp"})


def test_a_collision_among_the_children_of_an_inner_level_is_refused_too() -> None:
    with pytest.raises(DuplicateVariableNameError, match=r"'x' and 'y'"):
        shape(messages.Pose, rename={"position/x": "Same", "position/y": "Same"})


def test_a_field_name_that_cannot_convert_is_refused() -> None:
    odd = build(describe("fixture_msgs/msg/Odd", none=FLOAT64, ok=STRING))

    with pytest.raises(BindingNameError, match="'none' cannot be a browse name"):
        shape(odd)
