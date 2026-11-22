# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A variable holding an array of structures, as a client sees it.

Wire-level tests use a subprocess because asyncua keeps a process-global class table.
"""

from __future__ import annotations

import struct
from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua
from asyncua.ua.attribute_ids import AttributeIds
from asyncua.ua.object_ids import ObjectIds
from asyncua.ua.status_codes import StatusCodes

from opensmi.ros2.server.errors import DeclarationError
from opensmi.ros2.server.events import root_of
from opensmi.ros2.server.mutation import add_plan_variable
from opensmi.ros2.server.projection import PlanVariable
from opensmi.ros2.server.structures import structure_elements, structure_instance
from tests.fake_runtime import FakeRuntime
from tests.fixture_structures import CONTROLLER, POINT, point
from tests.fixture_structures import encoding_id as encoding_node_id
from tests.served import TestComponent, TestMachine, connected, leaf, serve
from tests.wire_client import run_client

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from opensmi.server import Server
    from opensmi.server.ua_variable import UaVariable

EXTENSION_OBJECTS = ua.VariantType.ExtensionObject


@pytest.fixture
async def served() -> AsyncIterator[tuple[Server, Any]]:
    machine = TestMachine(TestComponent(name="Arm"), runtime=FakeRuntime())
    server = await serve(machine)
    try:
        yield server, machine.added[0]
    finally:
        await server.shutdown()


def points_entry(name: str = "points") -> PlanVariable:
    """Return the plan entry of a repeated message of points, as the projection will shape it."""
    return PlanVariable(name=name, source=name, initial=[], variant_type=EXTENSION_OBJECTS, datatype=POINT)


async def add_points(component: Any, name: str = "points", *, writable: bool = True) -> UaVariable[Any]:
    """Add a variable of points the way the chokepoint does, DataType resolved on this server."""
    entry = points_entry(name)
    datatype = await component.resolve_plan_datatype(entry.datatype)
    return await add_plan_variable(component.monitoring, name, entry, datatype, writable=writable)


def elements(*values: dict[str, object], datatype: Any = POINT) -> ua.Variant:
    """Return an array of elements as a client writes it."""
    return ua.Variant([structure_instance(datatype, each) for each in values], EXTENSION_OBJECTS)


async def lock(client: Any, component: Any) -> None:
    await client.get_node(root_of(component).lock.ua_node.nodeid).call_method("2:InitLock")


async def test_a_variable_of_points_is_typed_and_starts_empty(served) -> None:
    server, arm = served
    variable = await add_points(arm)
    namespace = arm.ros2_namespace_index

    async with connected(server, "visitor") as client:
        node = client.get_node(variable.ua_node.nodeid)
        datatype, rank, dimensions = await node.read_attributes(
            [AttributeIds.DataType, AttributeIds.ValueRank, AttributeIds.ArrayDimensions]
        )
        value = await node.read_value()

    assert datatype.Value is not None
    assert datatype.Value.Value == ua.StringNodeId(Identifier=ua.String(POINT.__name__), NamespaceIndex=namespace)
    assert rank.Value is not None
    assert rank.Value.Value == ua.ValueRank.OneDimension
    assert dimensions.Value is not None
    assert dimensions.Value.Value == [0]
    assert value == []
    assert await variable.read() == []


async def test_the_structure_is_created_once_and_shared_by_two_variables(served) -> None:
    server, arm = served

    left = await add_points(arm, "left")
    right = await add_points(arm, "right")

    assert await left.ua_node.read_data_type() == await right.ua_node.read_data_type()
    subtypes = await server.ua_server.nodes.base_structure_type.get_children(refs=ObjectIds.HasSubtype)
    names = [(await each.read_browse_name()).Name for each in subtypes]
    assert names.count(POINT.__name__) == 1


async def test_a_variable_is_empty_again_after_reset(served) -> None:
    server, arm = served
    variable = await add_points(arm)
    await variable.write(elements(point(1.0), point(2.0)))
    assert len(await variable.read()) == 2

    await variable.reset()

    async with connected(server, "visitor") as client:
        assert await client.get_node(variable.ua_node.nodeid).read_value() == []
    assert await variable.read() == []


async def test_what_the_server_wrote_reads_back_as_the_elements(served) -> None:
    server, arm = served
    variable = await add_points(arm)
    await variable.write(elements(point(1.0), point(2.0, positions=0)))

    async with connected(server, "visitor") as client:
        read = await client.get_node(variable.ua_node.nodeid).read_value()

    assert structure_elements(POINT, read) == [point(1.0), point(2.0, positions=0)]


async def test_a_failed_add_leaves_nothing_registered(served) -> None:
    _, arm = served
    first = await add_points(arm)
    before = list(arm.monitoring)

    with pytest.raises(ua.UaError):
        await add_points(arm)

    assert list(arm.monitoring) == before
    assert leaf(arm.monitoring, "points") is first
    assert [await each.read() for each in arm.monitoring if each.name == "points"] == [[]]


async def test_a_failed_add_of_a_new_name_is_not_filed(served) -> None:
    _, arm = served
    entry = points_entry("fresh")

    with pytest.raises(DeclarationError, match="resolved"):
        await add_plan_variable(arm.monitoring, "fresh", entry, None)
    scalar = PlanVariable(name="fresh", source="fresh", initial=0, variant_type=ua.VariantType.Int32, datatype=POINT)
    with pytest.raises(DeclarationError, match="must be an array"):
        await add_plan_variable(arm.monitoring, "fresh", scalar, await arm.resolve_plan_datatype(POINT))

    assert "fresh" not in arm.monitoring


async def test_a_failed_add_inside_a_group_leaves_the_group_as_it_was(served) -> None:
    _, arm = served
    member = await arm.monitoring.add_group("M")
    datatype = await arm.resolve_plan_datatype(POINT)
    first = await add_plan_variable(member, "points", points_entry(), datatype)

    with pytest.raises(ua.UaError):
        await add_plan_variable(member, "points", points_entry(), datatype)

    assert list(member) == [first]


async def test_the_write_check_of_the_caller_runs_after_the_structure_check(served) -> None:
    server, arm = served
    seen: list[object] = []

    async def check(value: object) -> None:
        seen.append(value)
        if value:
            raise ua.UaStatusCodeError(StatusCodes.BadOutOfRange)

    entry = points_entry()
    variable = await add_plan_variable(
        arm.monitoring, "points", entry, await arm.resolve_plan_datatype(POINT), writable=True, write_check=check
    )

    async with connected(server) as client:
        await lock(client, arm)
        node = client.get_node(variable.ua_node.nodeid)
        with pytest.raises(ua.uaerrors.BadOutOfRange):
            await node.write_value(ua.DataValue(elements(point())))
        seen.clear()
        with pytest.raises(ua.uaerrors.BadTypeMismatch):
            await node.write_value(ua.DataValue(ua.Variant([1.5], ua.VariantType.Double)))

    assert seen == []


async def test_a_non_admin_writer_with_the_lock_writes_and_a_read_only_variable_is_refused(served) -> None:
    server, arm = served
    writable = await add_points(arm, "writable")
    read_only = await add_points(arm, "read_only", writable=False)
    value = ua.DataValue(elements(point(3.0)))

    async with connected(server) as client:
        await lock(client, arm)
        await client.get_node(writable.ua_node.nodeid).write_value(value)
        with pytest.raises(ua.uaerrors.BadUserAccessDenied):
            await client.get_node(read_only.ua_node.nodeid).write_value(value)
        read = await client.get_node(writable.ua_node.nodeid).read_value()

    assert structure_elements(POINT, read) == [point(3.0)]
    assert await read_only.read() == []


async def test_a_reader_below_the_access_level_cannot_write(served) -> None:
    server, arm = served
    variable = await add_points(arm)
    value = ua.DataValue(elements(point()))

    async with connected(server, "visitor") as client:
        with pytest.raises(ua.uaerrors.BadUserAccessDenied):
            await client.get_node(variable.ua_node.nodeid).write_value(value)

    assert await variable.read() == []


async def test_a_writer_without_the_lock_is_refused(served) -> None:
    server, arm = served
    variable = await add_points(arm)
    value = ua.DataValue(elements(point()))

    async with connected(server) as client, connected(server, "orchestrator") as other:
        await lock(other, arm)
        with pytest.raises(ua.UaStatusCodeError):
            await client.get_node(variable.ua_node.nodeid).write_value(value)

    assert await variable.read() == []


async def test_an_element_of_another_structure_is_refused_in_process(served) -> None:
    server, arm = served
    variable = await add_points(arm)
    other = elements({"name": "a", "claimed_interfaces": []}, datatype=CONTROLLER)
    await arm.mint_structure(CONTROLLER)

    async with connected(server) as client:
        await lock(client, arm)
        with pytest.raises(ua.uaerrors.BadTypeMismatch):
            await client.get_node(variable.ua_node.nodeid).write_value(ua.DataValue(other))

    assert await variable.read() == []


def body(positions: list[float] | None, tail: bytes = b"") -> str:
    """Return the hex body of a point as a stranger writes it; None writes positions as a null array."""
    count = -1 if positions is None else len(positions)
    return (
        struct.pack("<i", count)
        + b"".join(struct.pack("<d", each) for each in positions or [])
        + struct.pack("<i", 0)  # velocities
        + struct.pack("<iI", 4, 6)  # time_from_start
        + struct.pack("<i3d", 3, 1.0, 2.0, 3.0)  # covariance
        + struct.pack("<i", 1)
        + b""  # raw
        + b""  # valid
        + tail
    ).hex()


async def test_a_client_in_another_process_writes_points_and_is_refused_what_is_not_points(served) -> None:
    server, arm = served
    variable = await add_points(arm)
    await arm.mint_structure(CONTROLLER)
    namespace = arm.ros2_namespace_index
    points = encoding_node_id(POINT.__name__, namespace).to_string()
    controller = encoding_node_id(CONTROLLER.__name__, namespace).to_string()
    good = body([0.25, 0.5])
    steps = {
        "lock": root_of(arm).lock.ua_node.nodeid.to_string(),
        "writes": [
            ["good", "elements", [[points, good], [points, body([])]]],
            ["another structure", "elements", [[controller, good]]],
            ["unknown type", "elements", [["ns=9;s=not/there.DefaultBinary", good]]],
            ["body runs out", "elements", [[points, good[:-6]]]],
            ["null array", "elements", [[points, body(None)]]],
            ["good among bad", "elements", [[points, good], [controller, good]]],
            ["scalar", "scalar", [points, good]],
            ["doubles", "doubles", [1.5]],
            ["empty", "elements", []],
            ["bytes left over", "elements", [[points, good + "00"]]],
        ],
    }

    result = await run_client(server, "operator", variable.ua_node.nodeid, steps)

    two = [[points, good], [points, body([])]]
    assert result[0] == ["initial", []]
    outcomes = {label: (status, stored) for label, status, stored in result[1:]}
    assert outcomes["good"] == ("Good", two)
    for label in (
        "another structure",
        "unknown type",
        "null array",
        "good among bad",
        "scalar",
        "doubles",
    ):
        assert outcomes[label] == ("BadTypeMismatch", two), label
    # asyncua refuses a body that ends mid-field while decoding the request, before any check of ours.
    assert outcomes["body runs out"] == ("BadInternalError", two)
    assert outcomes["empty"] == ("Good", [])
    # asyncua ignores bytes after the last field. The codec we replaced refused them.
    assert outcomes["bytes left over"] == ("Good", [[points, good]])
    assert len(await variable.read()) == 1
