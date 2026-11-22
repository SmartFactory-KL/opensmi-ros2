# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Structure DataTypes minted from ROS2 message types, as a client in another process sees them."""

from __future__ import annotations

import asyncio
import dataclasses
import json
import sys
import textwrap
from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua
from asyncua.ua.object_ids import ObjectIds

from opensmi.ros2.server.structures import DuplicateStructureError, structure_class
from tests.fake_runtime import FakeRuntime
from tests.fixture_structures import POINT, TRAJECTORY, encoding_id, node_id
from tests.served import TestComponent, TestMachine, serve

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from opensmi.server import Server

LABEL = structure_class("a/msg/Label", (("text", ua.String, 0),))


@pytest.fixture
async def served() -> AsyncIterator[tuple[Server, Any, Any]]:
    machine = TestMachine(TestComponent(name="Arm"), TestComponent(name="Gripper"), runtime=FakeRuntime())
    server = await serve(machine)
    try:
        yield server, machine.added[0], machine.added[1]
    finally:
        await server.shutdown()


CLIENT = textwrap.dedent("""
    import asyncio, json, logging, sys
    from asyncua import ua
    from asyncua.client.client import Client

    logging.disable(logging.CRITICAL)

    async def main(url, nodeid):
        async with Client(url) as client:
            datatype = client.get_node(nodeid)
            name = await datatype.read_browse_name()
            definition = await datatype.read_data_type_definition()
            encoding = client.get_node(definition.DefaultEncodingId)
            supertypes = [parent.NodeId.to_string() for parent in await datatype.get_references(
                ua.ObjectIds.HasSubtype, ua.BrowseDirection.Inverse)]
            print(json.dumps({
                "nodeid": datatype.nodeid.to_string(),
                "browse_name": [name.Name, name.NamespaceIndex],
                "encoding": definition.DefaultEncodingId.to_string(),
                "encoding_name": (await encoding.read_browse_name()).Name,
                "encoding_types": [
                    ref.NodeId.to_string() for ref in await encoding.get_references(
                        ua.ObjectIds.HasTypeDefinition, ua.BrowseDirection.Forward)
                ],
                "supertype_is_structure": ua.NodeId(ua.ObjectIds.Structure).to_string() in supertypes,
                "fields": [
                    [f.Name, f.DataType.to_string(), f.ValueRank, [int(d) for d in f.ArrayDimensions],
                     bool(f.IsOptional)]
                    for f in definition.Fields
                ],
            }))

    asyncio.run(main(sys.argv[1], sys.argv[2]))
""")


async def read_in_another_process(server: Server, node_id: ua.NodeId) -> dict[str, Any]:
    """Browse one DataType with a client that has never heard of the server's types."""
    bserver = server.ua_server.bserver
    assert bserver is not None
    process = await asyncio.create_subprocess_exec(
        sys.executable,
        "-c",
        CLIENT,
        f"opc.tcp://visitor:visitor@127.0.0.1:{bserver.port}/server",
        node_id.to_string(),
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    out, err = await asyncio.wait_for(process.communicate(), timeout=60)
    assert process.returncode == 0, err.decode()
    return json.loads(out)


def builtin(variant_type: ua.VariantType) -> str:
    return ua.NodeId(ua.Int32(variant_type.value)).to_string()


async def test_a_trajectory_is_created_with_the_type_of_its_points(served) -> None:
    server, arm, _ = served
    namespace = arm.ros2_namespace_index

    node_id_ = await arm.mint_structure(TRAJECTORY)

    assert node_id_ == node_id("trajectory_msgs/msg/JointTrajectory", namespace)
    trajectory = await read_in_another_process(server, node_id_)
    assert trajectory["nodeid"] == f"ns={namespace};s=trajectory_msgs/msg/JointTrajectory"
    # NodeId and BrowseName are both the ROS2 type name.
    assert trajectory["browse_name"] == ["trajectory_msgs/msg/JointTrajectory", namespace]
    assert trajectory["supertype_is_structure"]
    assert trajectory["encoding"] == f"ns={namespace};s=trajectory_msgs/msg/JointTrajectory.DefaultBinary"
    assert trajectory["encoding_name"] == "Default Binary"
    assert trajectory["encoding_types"] == [f"i={ObjectIds.DataTypeEncodingType}"]
    point_id = f"ns={namespace};s=trajectory_msgs/msg/JointTrajectoryPoint"
    assert trajectory["fields"] == [
        ["joint_names", builtin(ua.VariantType.String), 1, [0], False],
        ["points", point_id, 1, [0], False],
    ]
    point = await read_in_another_process(server, ua.NodeId.from_string(point_id))
    assert point["browse_name"] == ["trajectory_msgs/msg/JointTrajectoryPoint", namespace]
    # The length of a fixed-size array is checked on write but not published: the definition says [0].
    assert [field[:4] for field in point["fields"]] == [
        ["positions", builtin(ua.VariantType.Double), 1, [0]],
        ["velocities", builtin(ua.VariantType.Double), 1, [0]],
        ["time_from_start", f"ns={namespace};s=builtin_interfaces/msg/Duration", -1, []],
        ["covariance", builtin(ua.VariantType.Double), 1, [0]],
        ["raw", builtin(ua.VariantType.ByteString), -1, []],
        ["valid", builtin(ua.VariantType.Boolean), -1, []],
    ]


async def test_asking_twice_is_a_no_op(served) -> None:
    _, arm, gripper = served

    first = await arm.mint_structure(TRAJECTORY)
    again = await arm.mint_structure(TRAJECTORY)
    from_the_other_component = await gripper.mint_structure(POINT)

    assert again == first
    assert from_the_other_component == node_id(POINT.__name__, arm.ros2_namespace_index)


async def test_the_nodes_do_not_depend_on_the_order_the_types_are_asked_in() -> None:
    for batch in ([TRAJECTORY, LABEL], [LABEL, TRAJECTORY]):
        machine = TestMachine(TestComponent(name="Arm"), runtime=FakeRuntime())
        server = await serve(machine)
        try:
            arm = machine.added[0]
            for each in batch:
                await arm.mint_structure(each)
            for each in (LABEL, POINT, TRAJECTORY):
                encoding = server.ua_server.get_node(encoding_id(each.__name__, arm.ros2_namespace_index))
                assert (await encoding.read_browse_name()).Name == "Default Binary"
        finally:
            await server.shutdown()


async def test_names_that_differ_only_in_a_slash_are_two_types(served) -> None:
    server, arm, _ = served
    other = structure_class("a/msg_Label", (("text", ua.String, 0),))

    first = await arm.mint_structure(LABEL)
    second = await arm.mint_structure(other)

    assert first != second
    for each in (first, second):
        assert (await server.ua_server.get_node(each).read_browse_name()).Name == each.Identifier


async def test_two_components_that_curate_one_type_differently_are_refused_naming_the_second(served) -> None:
    server, arm, gripper = served
    narrower = structure_class(POINT.__name__, (("positions", list[ua.Double], 0),))
    await arm.mint_structure(POINT)

    with pytest.raises(DuplicateStructureError) as refused:
        await gripper.mint_structure(narrower)

    assert "'Gripper'" in str(refused.value)
    assert POINT.__name__ in str(refused.value)
    definition = await server.ua_server.get_node(
        node_id(POINT.__name__, arm.ros2_namespace_index)
    ).read_data_type_definition()
    assert len(definition.Fields) == len(dataclasses.fields(POINT))


async def test_a_nested_type_curated_differently_is_refused_before_anything_is_created(served) -> None:
    server, arm, gripper = served
    narrower = structure_class(POINT.__name__, (("positions", list[ua.Double], 0),))
    narrow_trajectory = structure_class("other_msgs/msg/Narrow", (("points", list[narrower], 0),))
    await arm.mint_structure(TRAJECTORY)

    with pytest.raises(DuplicateStructureError, match="'Gripper'"):
        await gripper.mint_structure(narrow_trajectory)

    with pytest.raises(ua.UaError):
        await server.ua_server.get_node(
            node_id(narrow_trajectory.__name__, arm.ros2_namespace_index)
        ).read_browse_name()
