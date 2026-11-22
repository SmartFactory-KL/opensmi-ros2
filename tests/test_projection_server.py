# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""What a curated Projection looks like in a real address space."""

from __future__ import annotations

import struct
from typing import TYPE_CHECKING, Any, cast

import pytest
from asyncua import ua
from asyncua.ua.attribute_ids import AttributeIds
from asyncua.ua.object_ids import ObjectIds

from opensmi.ros2.server.bindings import MonitoringBinding
from opensmi.ros2.server.errors import DuplicateProjectionError
from opensmi.ros2.server.events import SIGNED_BYTE_STRING_TYPE
from opensmi.ros2.server.limits import byte_string_ceiling
from opensmi.ros2.server.mirror import PAYLOAD_CAPPED
from opensmi.ros2.server.projection import (
    EnumDataType,
    PlanVariable,
    ProjectDataType,
    generic_decode,
    generic_shape,
)
from opensmi.ros2.server.recipes import Functions
from tests import fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.fixture_structures import node_id as structure_node_id
from tests.served import (
    TestComponent,
    TestMachine,
    connected,
    leaf,
    monitoring_nodes,
    serve,
    until_elements,
    until_value,
)

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Mapping

    from opensmi.server import Server

    from opensmi.ros2.server.projection import Projection, ProjectionContext

MESSAGE_TYPES = {
    "fixture_msgs/msg/Grid": fixture_messages.Grid,
    "fixture_msgs/msg/Mode": fixture_messages.Mode,
    "fixture_msgs/msg/Pose": fixture_messages.Pose,
    "fixture_msgs/msg/Point": fixture_messages.Point,
    "fixture_msgs/msg/Quaternion": fixture_messages.Quaternion,
    "fixture_msgs/msg/Path": fixture_messages.Path,
}

GRID = MonitoringBinding(name="Map", path="/map", message_type="fixture_msgs/msg/Grid", include=("data",))
MODE = MonitoringBinding(name="Mode", path="/mode", message_type="fixture_msgs/msg/Mode", include=("mode",))
POSE = MonitoringBinding(name="Pose", path="/pose", message_type="fixture_msgs/msg/Pose")
PLAN = MonitoringBinding(name="Plan", path="/plan", message_type="fixture_msgs/msg/Path")


def _packed(_type: Any, _context: ProjectionContext) -> tuple[PlanVariable]:
    return (
        PlanVariable(
            name="Data",
            display_name="data",
            source="data",
            initial=b"",
            variant_type=ua.VariantType.ByteString,
            datatype=ProjectDataType(identifier=SIGNED_BYTE_STRING_TYPE),
        ),
    )


def _pack(message: Any, context: ProjectionContext) -> Mapping[str, object]:
    data = struct.pack(f"<{len(message.data)}b", *message.data)
    ceiling = context.byte_string_ceiling
    return {"Data": data[:ceiling] if ceiling else data}


SIGNED_BLOB = Functions("fixture_msgs/msg/Grid", shaped_by=_packed, decoded_by=_pack)
"""A signed bulk buffer, typed with this project's own SignedByteString and capped at the ceiling."""


def _labelled(_type: Any, context: ProjectionContext) -> tuple[PlanVariable]:
    return (
        PlanVariable(
            name="Mode",
            display_name="mode",
            source="mode",
            initial=0,
            variant_type=ua.VariantType.Int32,
            datatype=EnumDataType(name=f"{context.qualifier}_Mode_mode", values=((0, "Idle"), (7, "Running"))),
        ),
    )


def _mode(message: Any) -> Mapping[str, object]:
    return {"Mode": int(message.mode)}


LABELLED = Functions(
    "fixture_msgs/msg/Mode",
    shaped_by=_labelled,
    decoded_by=lambda message, _context: _mode(message),
)
"""An enumeration whose constants start at zero and skip, minted per Component."""


def dropping(type_name: str, *paths: str, override: bool = False) -> Projection:
    """Return an entry that renders a type through the walk with some of its fields left out."""
    return Functions(
        type_name,
        shaped_by=lambda message_type, context: generic_shape(message_type, context.without(paths)),
        decoded_by=lambda message, context: generic_decode(message, context.without(paths)),
        override=override,
    )


class Curated(TestComponent):
    """What a Component author writes: Bindings, and the curation they render through."""

    bindings = (GRID, MODE, PLAN)
    projections = (SIGNED_BLOB, LABELLED)


class LeftCamera(TestComponent):
    bindings = (POSE,)
    projections = (dropping("fixture_msgs/msg/Pose", "orientation"),)


class RightCamera(TestComponent):
    bindings = (POSE,)
    projections = (dropping("fixture_msgs/msg/Pose", "position"),)


@pytest.fixture
def runtime() -> FakeRuntime:
    return FakeRuntime(message_types=MESSAGE_TYPES)


async def start(machine: TestMachine) -> tuple[Server, Any]:
    return await serve(machine), machine


@pytest.fixture
async def served(runtime: FakeRuntime) -> AsyncIterator[tuple[Server, Any]]:
    server, machine = await start(TestMachine(Curated(name="curated"), name="robot", runtime=runtime))
    try:
        yield server, machine
    finally:
        await server.shutdown()


def curated(machine: Any) -> dict[str, Any]:
    return monitoring_nodes(machine.added[0])


async def test_a_variable_of_ours_is_typed_with_the_datatype_our_nodeset_defines(served) -> None:
    server, machine = served
    node = curated(machine)["Map.Data"]

    async with connected(server, "visitor") as client:
        datatype = await client.get_node(node.nodeid).read_data_type()

    assert datatype == server.ua_get_node_id(SIGNED_BYTE_STRING_TYPE)


async def test_a_bulk_variable_carries_the_packed_buffer_a_message_brings(served, runtime: FakeRuntime) -> None:
    server, machine = served
    node = curated(machine)["Map.Data"]

    runtime.note_publishers("/map", 1)
    runtime.deliver("/map", fixture_messages.Grid(width=2, data=(-1, 100)))

    async with connected(server, "visitor") as client:
        await until_value(client.get_node(node.nodeid), b"\xff\x64")


async def test_an_enum_variable_is_typed_with_a_datatype_named_for_its_component(served) -> None:
    server, machine = served
    node = curated(machine)["Mode.Mode"]

    async with connected(server, "visitor") as client:
        datatype = client.get_node(await client.get_node(node.nodeid).read_data_type())
        browse_name = await datatype.read_browse_name()
        definition = await datatype.read_data_type_definition()

    assert browse_name.Name == "robot_curated_Mode_mode"
    assert isinstance(definition, ua.EnumDefinition)
    assert [(field.Value, field.Name) for field in definition.Fields] == [(0, "Idle"), (7, "Running")]


async def test_two_components_render_one_message_type_two_ways(runtime: FakeRuntime) -> None:
    server, machine = await start(
        TestMachine(LeftCamera(name="left"), RightCamera(name="right"), name="robot", runtime=runtime)
    )
    try:
        left = monitoring_nodes(machine.added[0])
        right = monitoring_nodes(machine.added[1])

        assert sorted(left) == ["Pose.Position.X", "Pose.Position.Y", "Pose.Position.Z"]
        assert sorted(right) == [
            "Pose.Orientation.W",
            "Pose.Orientation.X",
            "Pose.Orientation.Y",
            "Pose.Orientation.Z",
        ]
    finally:
        await server.shutdown()


class CuratedMachine(TestMachine):
    projections = (dropping("fixture_msgs/msg/Pose", "orientation"),)


async def test_each_component_derives_its_registry_from_the_machines_default() -> None:
    # A duplicate curation is fatal unless it says it replaces.
    with pytest.raises(DuplicateProjectionError, match="fixture_msgs/msg/Pose"):
        await start(CuratedMachine(LeftCamera(name="left"), runtime=FakeRuntime(message_types=MESSAGE_TYPES)))


async def test_a_machines_entry_reaches_a_component_that_curates_nothing(runtime: FakeRuntime) -> None:
    class Plain(TestComponent):
        bindings = (POSE,)

    server, machine = await start(CuratedMachine(Plain(name="plain"), runtime=runtime))
    try:
        assert sorted(monitoring_nodes(machine.added[0])) == [
            "Pose.Position.X",
            "Pose.Position.Y",
            "Pose.Position.Z",
        ]
    finally:
        await server.shutdown()


async def test_the_server_publishes_the_byte_string_ceiling_it_actually_carries(served) -> None:
    server, machine = served

    async with connected(server, "visitor") as client:
        node = client.get_node(ua.NodeId(ua.Int32(ObjectIds.Server_ServerCapabilities_MaxByteStringLength)))
        published = await node.read_value()

    expected = byte_string_ceiling(max_message_size=server.ua_server.limits.max_message_size)
    assert published == expected
    assert machine.ros2_byte_string_ceiling == expected
    assert expected > 0


async def test_a_capped_bulk_payload_is_reported_as_a_binding_event(runtime: FakeRuntime) -> None:
    class Tiny(TestMachine):
        @property
        def ros2_byte_string_ceiling(self) -> int:
            """Stand in for a server whose transport carries almost nothing."""
            return 4

    server, machine = await start(Tiny(Curated(name="curated"), runtime=runtime))
    try:
        component = machine.added[0]
        emitted: list[dict[str, object]] = []

        async def collect(**event: object) -> None:
            emitted.append(event)

        component.emit_binding_event = collect

        runtime.note_publishers("/map", 1)
        runtime.deliver("/map", fixture_messages.Grid(width=8, data=tuple(range(8))))
        runtime.deliver("/map", fixture_messages.Grid(width=8, data=tuple(range(8))))

        async with connected(server, "visitor") as client:
            await until_value(client.get_node(curated(machine)["Map.Data"].nodeid), b"\x00\x01\x02\x03")

        assert [event["code"] for event in emitted] == [PAYLOAD_CAPPED]
        assert "4 bytes" in str(emitted[0]["text"])
    finally:
        await server.shutdown()


# -- a repeated message, as an array of a structure -------------------------------------


def plan_of(*poses: Any) -> Any:
    return fixture_messages.Path(
        header=fixture_messages.Header(stamp=fixture_messages.Time(sec=1, nanosec=2), frame_id="map"),
        poses=poses,
    )


async def test_a_repeated_message_is_one_array_variable_with_the_datatype_of_its_structure(served) -> None:
    server, machine = served
    names = [name for name in curated(machine) if name.startswith("Plan.")]

    assert sorted(names) == ["Plan.Header.FrameId", "Plan.Header.Stamp.Nanosec", "Plan.Header.Stamp.Sec", "Plan.Poses"]
    namespace = machine.added[0].ros2_namespace_index
    async with connected(server, "visitor") as client:
        node = client.get_node(curated(machine)["Plan.Poses"].nodeid)
        datatype, rank, dimensions = await node.read_attributes(
            [AttributeIds.DataType, AttributeIds.ValueRank, AttributeIds.ArrayDimensions]
        )
        datatype_node = client.get_node(await node.read_data_type())
        definition = cast("ua.StructureDefinition", await datatype_node.read_data_type_definition())

    assert datatype.Value is not None
    assert rank.Value is not None
    assert dimensions.Value is not None
    assert datatype.Value.Value == structure_node_id("fixture_msgs/msg/PoseStamped", namespace)
    assert rank.Value.Value == ua.ValueRank.OneDimension
    assert dimensions.Value.Value == [0]
    assert [each.Name for each in definition.Fields] == ["header", "pose"]


async def test_a_client_reads_the_elements_one_message_carried(served, runtime: FakeRuntime) -> None:
    server, machine = served
    poses = curated(machine)["Plan.Poses"]
    variable = leaf(machine.added[0].monitoring, "Plan.Poses")

    runtime.note_publishers("/plan", 1)
    runtime.deliver(
        "/plan",
        plan_of(
            fixture_messages.pose_stamped(frame_id="a", x=1.0),
            fixture_messages.pose_stamped(frame_id="b", x=2.0),
        ),
    )

    async with connected(server, "visitor") as client:
        node = client.get_node(poses.nodeid)
        await until_elements(node, variable, [element("a", 1.0), element("b", 2.0)])

        runtime.deliver("/plan", plan_of())
        await until_elements(node, variable, [])


def element(frame_id: str, x: float) -> dict[str, object]:
    """Return the fields of the pose ``pose_stamped`` builds, as the structure holds them."""
    return {
        "header.stamp.sec": 1,
        "header.stamp.nanosec": 2,
        "header.frame_id": frame_id,
        "pose.position.x": x,
        "pose.position.y": 0.0,
        "pose.position.z": 0.0,
        "pose.orientation.x": 0.0,
        "pose.orientation.y": 0.0,
        "pose.orientation.z": 0.0,
        "pose.orientation.w": 1.0,
    }
