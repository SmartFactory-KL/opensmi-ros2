# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The DataType of every level of a plan, per ROS2 type or curated and qualified."""

from __future__ import annotations

import dataclasses
from typing import cast

import pytest
from asyncua import ua

from opensmi.ros2.runtime.selection import FieldSelection
from opensmi.ros2.server.errors import UnprojectableFieldError
from opensmi.ros2.server.projection import (
    PlanVariable,
    ProjectionContext,
    curated_naming,
    generic_decode,
    level_datatypes,
    level_value,
    message_class,
    narrowed,
    shaped,
)
from opensmi.ros2.server.recipes import GENERIC_REGISTRY, Scalar
from opensmi.ros2.server.structures import DuplicateStructureError, structure_class
from tests import fixture_messages as messages
from tests.fake_runtime import FakeRuntime
from tests.fixture_messages import FLOAT32, build, describe, nested
from tests.fixture_structures import node_id
from tests.served import TestComponent, TestMachine, serve

NAMING = curated_naming("Platform.Pose")


def context(message_type, registry=GENERIC_REGISTRY, **selection) -> ProjectionContext:
    return narrowed(
        ProjectionContext(registry=registry, selected=frozenset()), message_type, FieldSelection(**selection)
    )


def plan_of(message_type, registry=GENERIC_REGISTRY, **selection) -> list[PlanVariable]:
    plan = shaped(message_type, context(message_type, registry, **selection))
    assert all(isinstance(entry, PlanVariable) for entry in plan)
    return plan  # type: ignore[return-value]


def classes(message_type, registry=GENERIC_REGISTRY, **selection):
    return level_datatypes(plan_of(message_type, registry, **selection), message_type, NAMING)


def names(datatypes) -> dict[tuple[str, ...], str]:
    return {path: each.__name__ for path, each in datatypes.items()}


def test_a_per_type_class_has_the_ros2_field_names_and_nests_its_messages() -> None:
    pose = message_class(messages.Pose)

    assert pose.__name__ == "fixture_msgs/msg/Pose"
    assert [each.name for each in dataclasses.fields(pose)] == ["position", "orientation"]
    assert pose.__annotations__["position"] is message_class(messages.Point)
    assert message_class(messages.Point).__annotations__ == {"x": ua.Double, "y": ua.Double, "z": ua.Double}


def test_fixed_size_arrays_and_sequences_are_lists_and_the_length_is_kept() -> None:
    covariant = message_class(messages.PoseWithCovariance)
    sensor = message_class(messages.Sensor)

    assert covariant.__annotations__["covariance"] == list[ua.Double]
    assert covariant.__dataclass_fields__["covariance"].metadata["length"] == 4
    assert sensor.__annotations__["payload"] is ua.ByteString
    assert sensor.__annotations__["labels"] == list[ua.String]


def test_a_repeated_message_is_a_list_of_its_class() -> None:
    assert message_class(messages.Path).__annotations__["poses"] == list[message_class(messages.PoseStamped)]


def test_an_uncurated_plan_keeps_the_class_of_every_ros2_type() -> None:
    datatypes = classes(messages.PoseStamped)

    assert names(datatypes) == {
        (): "fixture_msgs/msg/PoseStamped",
        ("Header",): "fixture_msgs/msg/Header",
        ("Header", "Stamp"): "fixture_msgs/msg/Time",
        ("Pose",): "fixture_msgs/msg/Pose",
        ("Pose", "Position"): "fixture_msgs/msg/Point",
        ("Pose", "Orientation"): "fixture_msgs/msg/Quaternion",
    }
    assert datatypes[()] is message_class(messages.PoseStamped)
    assert datatypes["Pose", "Position"] is message_class(messages.Point)


def test_an_excluded_field_curates_the_path_to_it_and_nothing_else() -> None:
    datatypes = classes(messages.PoseStamped, exclude=("pose/orientation",))

    assert names(datatypes) == {
        (): "Platform.Pose",
        ("Header",): "fixture_msgs/msg/Header",
        ("Header", "Stamp"): "fixture_msgs/msg/Time",
        ("Pose",): "Platform.Pose.Pose",
        ("Pose", "Position"): "fixture_msgs/msg/Point",
    }
    assert [each.name for each in dataclasses.fields(datatypes["Pose",])] == ["position"]
    assert datatypes["Pose",].__annotations__["position"] is message_class(messages.Point)


def test_the_curation_of_a_deep_field_reaches_every_level_above_it() -> None:
    datatypes = classes(messages.PoseStamped, exclude=("pose/position/z",))

    assert names(datatypes) == {
        (): "Platform.Pose",
        ("Header",): "fixture_msgs/msg/Header",
        ("Header", "Stamp"): "fixture_msgs/msg/Time",
        ("Pose",): "Platform.Pose.Pose",
        ("Pose", "Position"): "Platform.Pose.Pose.Position",
        ("Pose", "Orientation"): "fixture_msgs/msg/Quaternion",
    }


def test_a_renamed_leaf_curates_its_level_with_the_new_name_as_the_field() -> None:
    datatypes = classes(messages.PoseStamped, rename={"header/frame_id": "Frame"})

    assert names(datatypes)["Header",] == "Platform.Pose.Header"
    assert [each.name for each in dataclasses.fields(datatypes["Header",])] == ["stamp", "Frame"]
    assert datatypes["Header", "Stamp"] is message_class(messages.Time)


def test_a_renamed_level_curates_the_level_above_it_but_not_itself() -> None:
    datatypes = classes(messages.PoseStamped, rename={"pose": "Where"})

    assert names(datatypes)[()] == "Platform.Pose"
    assert names(datatypes)["Where",] == "fixture_msgs/msg/Pose"


def test_a_projection_that_changes_the_type_of_a_leaf_curates_the_path_to_it() -> None:
    battery = build(describe("fixture_msgs/msg/Battery", voltage=nested("fixture_msgs/msg/Voltage"), cells=FLOAT32))
    registry = GENERIC_REGISTRY.extended([Scalar("fixture_msgs/msg/Voltage", variant_type=ua.VariantType.Float)])

    datatypes = classes(battery, registry)

    assert names(datatypes) == {(): "Platform.Pose"}
    assert datatypes[()].__annotations__ == {"voltage": ua.Float, "cells": ua.Float}
    assert classes(battery)[()] is message_class(battery)


def test_the_names_depend_on_the_qualifier_and_the_path_only() -> None:
    plan = plan_of(messages.PoseStamped, exclude=("pose/position/z",))
    other = level_datatypes(plan, messages.PoseStamped, curated_naming("Arm.Goal"))
    first = level_datatypes(plan, messages.PoseStamped, NAMING)
    again = level_datatypes(plan, messages.PoseStamped, NAMING)

    assert first["Pose",] is again["Pose",]
    assert other["Pose",].__name__ == "Arm.Goal.Pose"
    assert other["Pose",] is not first["Pose",]


def test_a_repeated_message_stays_one_leaf_whose_element_is_the_nested_class() -> None:
    datatypes = classes(messages.Path)

    assert set(datatypes) == {(), ("Header",), ("Header", "Stamp")}
    assert datatypes[()].__annotations__["poses"] == list[message_class(messages.PoseStamped)]


def decoded(message_type, message, **selection):
    return generic_decode(message, context(message_type, **selection))


def test_a_level_value_is_the_instance_of_its_class_from_the_decoded_message() -> None:
    plan = plan_of(messages.PoseStamped)
    datatypes = level_datatypes(plan, messages.PoseStamped, NAMING)
    values = decoded(messages.PoseStamped, messages.pose_stamped(frame_id="odom", x=1.5, y=2.5))

    whole = level_value(plan, datatypes, values)
    position = level_value(plan, datatypes, values, ("Pose", "Position"))

    assert isinstance(whole, message_class(messages.PoseStamped))
    assert whole.header.frame_id == "odom"
    assert whole.header.stamp == message_class(messages.Time)(sec=1, nanosec=2)
    assert whole.pose.position == position == message_class(messages.Point)(x=1.5, y=2.5, z=0.0)


def test_a_curated_level_value_has_the_new_fields_only() -> None:
    selection = {"exclude": ("pose/orientation", "header"), "rename": {"pose/position/x": "East"}}
    plan = plan_of(messages.PoseStamped, **selection)
    datatypes = level_datatypes(plan, messages.PoseStamped, NAMING)
    values = decoded(messages.PoseStamped, messages.pose_stamped(x=4.0), **selection)

    whole = level_value(plan, datatypes, values)

    assert type(whole).__name__ == "Platform.Pose"
    assert dataclasses.asdict(whole) == {"pose": {"position": {"East": 4.0, "y": 0.0, "z": 0.0}}}


def test_a_leaf_a_structure_cannot_carry_is_refused_by_name() -> None:
    plan = [
        dataclasses.replace(each, variant_type=ua.VariantType.DateTime) if each.name == "Sec" else each
        for each in plan_of(messages.Time)
    ]

    with pytest.raises(UnprojectableFieldError, match=r"Sec.*DateTime"):
        level_datatypes(plan, messages.Time, NAMING)


async def test_a_curated_type_and_the_per_type_ones_below_it_are_minted_side_by_side() -> None:
    machine = TestMachine(TestComponent(name="Arm"), runtime=FakeRuntime())
    server = await serve(machine)
    try:
        arm = machine.added[0]
        datatypes = classes(messages.PoseStamped, exclude=("pose/orientation",))

        curated = await arm.mint_structure(datatypes[()])
        point = await arm.mint_structure(datatypes["Pose", "Position"])

        namespace = arm.ros2_namespace_index
        assert curated.Identifier == "Platform.Pose"
        assert point == node_id("fixture_msgs/msg/Point", namespace)
        pose = server.ua_server.get_node(node_id("Platform.Pose.Pose", namespace))
        definition = cast("ua.StructureDefinition", await pose.read_data_type_definition())
        assert [each.Name for each in definition.Fields] == ["position"]
        assert definition.Fields[0].DataType == point
        with pytest.raises(DuplicateStructureError):
            await arm.mint_structure(structure_class("Platform.Pose.Pose", (("other", ua.Double, 0),)))
    finally:
        await server.shutdown()
