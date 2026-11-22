# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""`add_plan_variable` sets DataType, ValueRank and ArrayDimensions, and writes stay behind opensmi's
access control and write check."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua
from asyncua.ua.attribute_ids import AttributeIds
from asyncua.ua.object_ids import ObjectIds
from asyncua.ua.status_codes import StatusCodes

from opensmi.ros2.server.events import root_of
from opensmi.ros2.server.mutation import add_plan_variable
from opensmi.ros2.server.projection import PlanVariable
from tests.fake_runtime import FakeRuntime
from tests.served import TestComponent, TestMachine, connected, serve

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from asyncua.common.node import Node
    from opensmi.server import Server


@pytest.fixture
async def served() -> AsyncIterator[tuple[Server, Any]]:
    machine = TestMachine(TestComponent(name="Platform"), runtime=FakeRuntime())
    server = await serve(machine)
    try:
        yield server, machine.added[0]
    finally:
        await server.shutdown()


async def attributes(node: Node) -> tuple[ua.NodeId, int, list[int] | None]:
    datatype, rank, dimensions = await node.read_attributes(
        [AttributeIds.DataType, AttributeIds.ValueRank, AttributeIds.ArrayDimensions]
    )
    assert datatype.Value is not None
    assert rank.Value is not None
    assert dimensions.Value is not None
    return datatype.Value.Value, rank.Value.Value, dimensions.Value.Value


async def test_an_empty_integer_array_is_typed_without_a_value_to_guess_from(served) -> None:
    server, component = served
    entry = PlanVariable(name="counts", source="counts", initial=[], variant_type=ua.VariantType.UInt16)

    variable = await add_plan_variable(component.monitoring, "counts", entry, None)

    async with connected(server, "visitor") as client:
        node = client.get_node(variable.ua_node.nodeid)
        assert await attributes(node) == (ua.NodeId(ua.Int32(ObjectIds.UInt16)), ua.ValueRank.OneDimension, [0])
        assert await node.read_value() == []
        assert (await node.read_description()).Text == "counts"


async def test_a_scalar_says_it_is_a_scalar_and_a_named_datatype_wins_over_the_builtin(served) -> None:
    server, component = served
    entry = PlanVariable(name="level", source="level", initial=0, variant_type=ua.VariantType.Int32)
    named = ua.NodeId(ua.Int32(ObjectIds.Enumeration))

    variable = await add_plan_variable(component.monitoring, "level", entry, named)

    async with connected(server, "visitor") as client:
        datatype, rank, dimensions = await attributes(client.get_node(variable.ua_node.nodeid))

    assert (datatype, rank) == (named, ua.ValueRank.Scalar)
    assert not dimensions


async def test_a_writable_variable_awaits_its_write_check_and_a_refusal_reaches_the_client(served) -> None:
    server, component = served
    checked: list[object] = []

    async def write_check(value: object) -> None:
        checked.append(value)
        if isinstance(value, float) and value < 0:
            raise ua.UaStatusCodeError(StatusCodes.BadOutOfRange)

    entry = PlanVariable(name="speed", source="speed", initial=0.0, variant_type=ua.VariantType.Double)
    variable = await add_plan_variable(
        component.monitoring, "speed", entry, None, writable=True, write_check=write_check
    )
    assert checked == [0.0]

    async with connected(server) as client:
        node = client.get_node(variable.ua_node.nodeid)
        await client.get_node(root_of(component).lock.ua_node.nodeid).call_method("2:InitLock")

        await node.write_value(ua.DataValue(ua.Variant(1.5, ua.VariantType.Double)))
        assert await node.read_value() == 1.5

        with pytest.raises(ua.uaerrors.BadOutOfRange):
            await node.write_value(ua.DataValue(ua.Variant(-1.0, ua.VariantType.Double)))
        assert await node.read_value() == 1.5

        with pytest.raises(ua.uaerrors.BadTypeMismatch):
            await node.write_value(ua.DataValue(ua.Variant(2, ua.VariantType.Int64)))

    assert checked == [0.0, 1.5, -1.0, 2]
