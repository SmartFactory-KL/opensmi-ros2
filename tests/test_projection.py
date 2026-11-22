# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Projection engine's steps (shape, decode, registry order, repeated messages),
without a server or rclpy."""

from __future__ import annotations

import dataclasses
from typing import TYPE_CHECKING, Any, cast

import pytest
from asyncua import ua

from opensmi.ros2.runtime.selection import FieldSelection, resolve_selection
from opensmi.ros2.server.errors import NotInvertibleError, UnprojectableFieldError
from opensmi.ros2.server.projection import (
    ANY_TYPE,
    PlanObject,
    PlanVariable,
    ProjectionContext,
    ProjectionRegistry,
    message_class,
    require_invertible,
)
from opensmi.ros2.server.recipes import GENERIC_REGISTRY, Functions
from tests import fixture_messages

if TYPE_CHECKING:
    from collections.abc import Mapping


def context(message_type, selection=None, registry=GENERIC_REGISTRY) -> ProjectionContext:
    return ProjectionContext(
        registry=registry,
        selected=resolve_selection(message_type, selection or FieldSelection()),
    )


def shape(message_type, selection=None, registry=GENERIC_REGISTRY):
    projection = registry.find(message_type.ros2.type_name)
    return projection.shape(message_type, context(message_type, selection, registry))


def decode(message, selection=None, registry=GENERIC_REGISTRY):
    projection = registry.find(type(message).ros2.type_name)
    return projection.decode(message, context(type(message), selection, registry))


def encode(message_type, values, selection=None, registry=GENERIC_REGISTRY):
    projection = registry.find(message_type.ros2.type_name)
    return projection.inverse_of(message_type, values, context(message_type, selection, registry))


def elements(values: Mapping[str, object], name: str) -> list[dict[str, Any]]:
    """Return the elements a decode answered for one repeated field."""
    return cast("list[dict[str, Any]]", values[name])


def variables(plan) -> tuple[PlanVariable, ...]:
    return tuple(entry for entry in plan if isinstance(entry, PlanVariable))


def test_the_generic_walk_renders_a_nested_message_as_flat_slash_separated_names() -> None:
    plan = shape(fixture_messages.Pose)

    assert [entry.full_name for entry in plan] == [
        "Position.X",
        "Position.Y",
        "Position.Z",
        "Orientation.X",
        "Orientation.Y",
        "Orientation.Z",
        "Orientation.W",
    ]
    assert all(isinstance(entry, PlanVariable) for entry in plan)


def test_a_variable_carries_the_typed_value_and_the_ros2_path_it_reads() -> None:
    (x, *_) = shape(fixture_messages.Pose)

    assert isinstance(x, PlanVariable)
    assert x.source == "position/x"
    assert x.initial == 0.0
    assert x.variant_type == ua.VariantType.Double


def test_a_fixed_size_array_keeps_its_length_in_the_value_written_at_boot() -> None:
    plan = {entry.full_name: entry for entry in shape(fixture_messages.PoseWithCovariance)}

    covariance = plan["Covariance"]
    assert isinstance(covariance, PlanVariable)
    assert covariance.initial == [0.0, 0.0, 0.0, 0.0]
    assert covariance.variant_type == ua.VariantType.Double


def test_selection_is_applied_before_projection() -> None:
    plan = shape(fixture_messages.PoseStamped, FieldSelection(exclude=("header", "pose/orientation")))

    assert [entry.full_name for entry in plan] == ["Pose.Position.X", "Pose.Position.Y", "Pose.Position.Z"]


POSE_LEAVES = [
    "header.stamp.sec",
    "header.stamp.nanosec",
    "header.frame_id",
    "pose.position.x",
    "pose.position.y",
    "pose.position.z",
    "pose.orientation.x",
    "pose.orientation.y",
    "pose.orientation.z",
    "pose.orientation.w",
]


def test_a_repeated_message_field_is_one_array_of_a_structure_variable() -> None:
    plan = {entry.full_name: entry for entry in shape(fixture_messages.Path)}

    assert list(plan) == ["Header.Stamp.Sec", "Header.Stamp.Nanosec", "Header.FrameId", "Poses"]
    poses = plan["Poses"]
    assert isinstance(poses, PlanVariable)
    assert poses.source == "poses"
    assert poses.initial == []
    assert poses.is_array
    assert poses.variant_type == ua.VariantType.ExtensionObject
    assert poses.datatype is message_class(fixture_messages.PoseStamped)


def test_the_structure_of_an_element_is_nested_like_its_message() -> None:
    poses = shape(fixture_messages.Path)[-1]
    assert isinstance(poses, PlanVariable)

    pose_stamped = poses.datatype
    fields = {each.name: each for each in dataclasses.fields(cast("type", pose_stamped))}

    assert list(fields) == ["header", "pose"]
    assert fields["pose"].type is message_class(fixture_messages.Pose)
    assert fields["pose"].type.__annotations__["position"] is message_class(fixture_messages.Point)
    assert message_class(fixture_messages.Point).__annotations__["x"] is ua.Double
    assert message_class(fixture_messages.Time).__annotations__["sec"] is ua.Int32
    assert message_class(fixture_messages.Header).__annotations__["frame_id"] is ua.String


def test_a_repeated_field_inside_the_element_of_another_is_a_field_of_its_structure() -> None:
    units = shape(fixture_messages.Units)[-1]
    assert isinstance(units, PlanVariable)

    fields = {each.name: each for each in dataclasses.fields(cast("type", units.datatype))}

    assert list(fields) == ["level", "name", "hardware_id", "message", "values"]
    assert fields["values"].type == list[message_class(fixture_messages.Pair)]
    assert [each.name for each in dataclasses.fields(message_class(fixture_messages.Pair))] == ["key", "value"]


def test_a_sequence_a_fixed_size_array_and_a_byte_array_are_told_apart_in_a_structure() -> None:
    plan = shape(fixture_messages.Waypoints)[-1]
    assert isinstance(plan, PlanVariable)

    fields = {each.name: each for each in dataclasses.fields(cast("type", plan.datatype))}

    assert (fields["positions"].type, fields["positions"].metadata["length"]) == (list[ua.Double], 0)
    assert (fields["covariance"].type, fields["covariance"].metadata["length"]) == (list[ua.Double], 3)
    assert fields["raw"].type is ua.ByteString
    assert fields["time_from_start"].type is message_class(fixture_messages.Time)


def test_the_structure_of_a_type_is_one_value_wherever_it_is_reached() -> None:
    first = shape(fixture_messages.Units)[-1]
    second = shape(fixture_messages.InventoryGoal)[-1]

    assert isinstance(first, PlanVariable)
    assert isinstance(second, PlanVariable)
    assert first.datatype is second.datatype


def test_a_curated_element_type_changes_the_fields_of_its_structure() -> None:
    def without_values(message_type, context):
        return tuple(
            entry
            for entry in GENERIC_REGISTRY.find("*").shape(message_type, context)
            if "values" not in entry.full_name.lower()
        )

    def decoded(message, context):
        return {
            key: value
            for key, value in GENERIC_REGISTRY.find("*").decode(message, context).items()
            if key.lower() != "values"
        }

    registry = curated(Functions("fixture_msgs/msg/Unit", shaped_by=without_values, decoded_by=decoded))

    plan = shape(fixture_messages.Units, registry=registry)

    units = plan[-1]
    assert isinstance(units, PlanVariable)
    assert [each.name for each in dataclasses.fields(cast("type", units.datatype))] == [
        "level",
        "name",
        "hardware_id",
        "message",
    ]
    assert cast("type", units.datatype).__name__ == "fixture_msgs/msg/Unit"


def test_an_element_whose_projection_is_an_object_is_refused_by_name() -> None:
    with pytest.raises(UnprojectableFieldError, match=r"Object.*'poses'"):
        shape(fixture_messages.Path, registry=curated(FOLDED))


def test_an_element_field_no_structure_can_carry_is_refused_by_name() -> None:
    def when_shape(_type, _context):
        return (PlanVariable(name="when", source="", initial=None, variant_type=ua.VariantType.DateTime),)

    registry = curated(Functions("fixture_msgs/msg/Time", shaped_by=when_shape, decoded_by=lambda *_: {}))

    with pytest.raises(UnprojectableFieldError, match=r"'.*when'.*DateTime"):
        shape(fixture_messages.Path, registry=registry)


def path(*poses) -> Any:
    return fixture_messages.Path(
        header=fixture_messages.Header(stamp=fixture_messages.Time(sec=1, nanosec=2), frame_id="map"),
        poses=poses,
    )


def test_decode_answers_a_repeated_message_field_with_its_elements_from_one_message() -> None:
    message = path(
        fixture_messages.pose_stamped(frame_id="a", x=1.0, y=10.0),
        fixture_messages.pose_stamped(frame_id="b", x=2.0, y=20.0),
    )

    values = decode(message)

    poses = elements(values, "Poses")
    assert [each["pose.position.x"] for each in poses] == [1.0, 2.0]
    assert [each["pose.position.y"] for each in poses] == [10.0, 20.0]
    assert [each["header.frame_id"] for each in poses] == ["a", "b"]
    assert all(list(each) == POSE_LEAVES for each in poses)
    assert values["Header.FrameId"] == "map"
    assert set(values) == {entry.full_name for entry in shape(fixture_messages.Path)}


def test_decode_answers_a_list_inside_an_element_with_a_list_of_its_own() -> None:
    message = fixture_messages.units(
        fixture_messages.unit(name="motor", values=(("a", "1"), ("b", "2"))),
        fixture_messages.unit(name="brake"),
    )

    values = decode(message)

    motor, brake = elements(values, "Units")
    assert motor["name"] == "motor"
    assert motor["values"] == [{"key": "a", "value": "1"}, {"key": "b", "value": "2"}]
    assert brake["values"] == []


def test_an_empty_sequence_decodes_to_an_empty_list() -> None:
    assert decode(path())["Poses"] == []


def test_a_list_of_none_two_and_twenty_reach_ros2_unchanged() -> None:
    for count in (0, 2, 20):
        message = path(*(fixture_messages.pose_stamped(x=float(each)) for each in range(count)))

        assert encode(fixture_messages.Path, decode(message)) == message


def test_a_message_with_a_list_inside_each_element_of_a_list_round_trips() -> None:
    message = fixture_messages.units(
        fixture_messages.unit(name="motor", level=1, values=(("a", "1"), ("b", "2"))),
        fixture_messages.unit(name="brake"),
        fixture_messages.unit(name="arm", values=(("c", "3"),)),
    )

    assert encode(fixture_messages.Units, decode(message)) == message


def test_a_message_with_byte_arrays_fixed_arrays_and_times_in_its_elements_round_trips() -> None:
    message = fixture_messages.waypoints(
        fixture_messages.waypoint(seed=1.0),
        fixture_messages.waypoint(seed=2.0, positions=0, raw=b""),
    )

    assert encode(fixture_messages.Waypoints, decode(message)) == message


def test_an_array_written_as_something_else_is_refused_by_name() -> None:
    values = dict(decode(path(fixture_messages.pose_stamped(x=1.0))))
    values["Poses"] = 1.0

    with pytest.raises(ValueError, match="'poses' was written as float rather than as an array"):
        encode(fixture_messages.Path, values)


def test_an_element_that_is_not_a_map_is_refused_by_name() -> None:
    values = dict(decode(path(fixture_messages.pose_stamped(x=1.0))))
    values["Poses"] = [1.0]

    with pytest.raises(ValueError, match="element 0 of 'poses'"):
        encode(fixture_messages.Path, values)


def test_an_element_with_a_field_missing_is_refused_rather_than_filled() -> None:
    values = dict(decode(path(fixture_messages.pose_stamped(x=1.0))))
    values["Poses"] = [{key: value for key, value in elements(values, "Poses")[0].items() if key != "pose.position.x"}]

    with pytest.raises(NotInvertibleError, match="pose/position/x"):
        encode(fixture_messages.Path, values)


def test_a_missing_array_is_refused_rather_than_filled() -> None:
    values = dict(decode(path(fixture_messages.pose_stamped(x=1.0))))
    del values["Poses"]

    with pytest.raises(NotInvertibleError, match="'poses'"):
        encode(fixture_messages.Path, values)


def test_a_repeated_element_with_no_inverse_makes_the_message_unpublishable() -> None:
    with pytest.raises(NotInvertibleError, match="fixture_msgs/msg/Quaternion"):
        require_invertible(fixture_messages.Path, context(fixture_messages.Path, registry=curated(DEGREES)))


def test_excluding_a_repeated_message_field_makes_the_rest_declarable() -> None:
    plan = shape(fixture_messages.Path, FieldSelection(exclude=("poses",)))

    assert [entry.full_name for entry in plan] == ["Header.Stamp.Sec", "Header.Stamp.Nanosec", "Header.FrameId"]


def test_decode_answers_with_the_names_the_plan_created() -> None:
    message = fixture_messages.pose_stamped(frame_id="odom", x=1.0, y=2.0, z=3.0)

    values = decode(message)

    assert values["Pose.Position.X"] == 1.0
    assert values["Header.FrameId"] == "odom"
    assert set(values) == {entry.full_name for entry in shape(fixture_messages.PoseStamped)}


def test_decode_skips_what_the_selection_removed() -> None:
    message = fixture_messages.pose_stamped(frame_id="odom")
    selection = FieldSelection(include=("pose/position",))

    values = decode(message, selection)

    assert set(values) == {"Pose.Position.X", "Pose.Position.Y", "Pose.Position.Z"}


def test_a_container_arrives_as_a_list_rather_than_the_tuple_the_payload_holds() -> None:
    message = fixture_messages.PoseWithCovariance(
        pose=fixture_messages.pose_stamped().pose,
        covariance=(1.0, 2.0, 3.0, 4.0),
    )

    values = decode(message)

    assert values["Covariance"] == [1.0, 2.0, 3.0, 4.0]


# -- the registry, and the entry that matches last -------------------------------------------


def degrees_shape(_type, _context):
    return (PlanVariable(name="yaw", source="", initial=0.0, variant_type=ua.VariantType.Double),)


def degrees_decode(message, _context):
    return {"yaw": message.w * 90.0}


def folded_shape(_type, _context):
    return (
        PlanObject(
            name="Position",
            children=(PlanVariable(name="x", source="x", initial=0.0, variant_type=ua.VariantType.Double),),
        ),
    )


def folded_decode(message, _context):
    return {"Position_x": message.x}


DEGREES = Functions("fixture_msgs/msg/Quaternion", shaped_by=degrees_shape, decoded_by=degrees_decode)

FOLDED = Functions("fixture_msgs/msg/Point", shaped_by=folded_shape, decoded_by=folded_decode)


def curated(*entries) -> ProjectionRegistry:
    return ProjectionRegistry(entries=(*entries, *GENERIC_REGISTRY.entries))


def test_the_generic_entry_is_the_one_that_matches_last() -> None:
    registry = curated(DEGREES)

    assert registry.find("fixture_msgs/msg/Quaternion") is DEGREES
    assert registry.find("fixture_msgs/msg/Pose").type_name == ANY_TYPE


def test_a_curated_projection_applies_wherever_its_type_appears_nested() -> None:
    plan = shape(fixture_messages.Pose, registry=curated(DEGREES))

    assert [entry.full_name for entry in plan] == [
        "Position.X",
        "Position.Y",
        "Position.Z",
        "Orientation.yaw",
    ]


def test_a_curated_decode_is_reached_through_the_nesting_too() -> None:
    message = fixture_messages.pose_stamped()

    values = decode(message, registry=curated(DEGREES))

    assert values["Pose.Orientation.yaw"] == 90.0


def test_the_object_verb_survives_the_walk_as_an_ordinary_entry() -> None:
    (position, *_) = shape(fixture_messages.Pose, registry=curated(FOLDED))

    assert isinstance(position, PlanObject)
    assert position.name == "Position.Position"
    assert [child.name for child in position.children] == ["x"]


# -- running the walk backwards -----------------------------------------------------------------


def test_the_walk_round_trips_a_nested_payload() -> None:
    message = fixture_messages.pose_stamped(frame_id="odom", x=1.0, y=2.0, z=3.0)

    assert encode(fixture_messages.PoseStamped, decode(message)) == message


def test_a_container_comes_back_as_the_tuple_a_payload_holds() -> None:
    message = fixture_messages.Sensor(ok=True, labels=("left",), readings=(0.5,), payload=b"")

    rebuilt: Any = encode(fixture_messages.Sensor, decode(message))

    assert rebuilt == message
    assert rebuilt.labels == ("left",)
    assert rebuilt.payload == b""


def test_the_walk_refuses_to_fill_a_field_the_selection_removed() -> None:
    # no invented fields.
    selection = FieldSelection(exclude=("z",))
    values = decode(fixture_messages.Point(x=1.0, y=2.0, z=3.0), selection)

    with pytest.raises(NotInvertibleError, match="'z'"):
        encode(fixture_messages.Point, values, selection)


def test_a_curated_entry_without_an_inverse_stops_the_walk_backwards() -> None:
    message = fixture_messages.pose_stamped()
    values = decode(message, registry=curated(DEGREES))

    with pytest.raises(NotInvertibleError, match="fixture_msgs/msg/Quaternion"):
        encode(fixture_messages.PoseStamped, values, registry=curated(DEGREES))
