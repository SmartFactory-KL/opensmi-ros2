# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Spike: DataType, ValueRank and ArrayDimensions written after creation on a started opensmi server.

opensmi's UaVariable takes none of them, so they are written through the asyncua node; the tests
check what a client sees and that writes still pass opensmi's access control.
"""

from collections.abc import AsyncGenerator, Awaitable, Callable, Sequence
from enum import IntEnum

import pytest
from asyncua import ua
from asyncua.client.client import Client
from asyncua.common.node import Node
from asyncua.ua.attribute_ids import AttributeIds
from asyncua.ua.object_ids import ObjectIds
from asyncua.ua.status_codes import StatusCodes
from opensmi.server import BaseMachine, ParameterSet, Server, UaVariable
from opensmi.server.config import ServerConfiguration
from opensmi.server.mixins import ParameterSetMixin

_WriteCheck = Callable[[object], Awaitable[None]]


def _ns0(identifier: int) -> ua.NodeId:
    return ua.NodeId(ua.Int32(identifier))


DOUBLE = _ns0(ObjectIds.Double)
INT32 = _ns0(ObjectIds.Int32)


class _ParameterSet(ParameterSet):
    """Empty at class level; every variable in these tests is added at runtime."""


class _Machine(ParameterSetMixin[_ParameterSet], BaseMachine):
    async def _write_identification(self) -> None:
        await self.identification.SerialNumber.write("spike-24")
        await self.identification.ProductInstanceUri.write("urn:opensmi-ros2:test:spike-24")
        await self.identification.Manufacturer.write(
            ua.LocalizedText("Technologie-Initiative SmartFactory KL e.V.", "en-US")
        )


async def _create_variable(
    container: ParameterSet,
    *,
    name: str,
    initial_value: ua.Variant,
    data_type: ua.NodeId,
    is_list: bool,
    write_check_callback: _WriteCheck | None = None,
) -> UaVariable[object]:
    """Create a variable through opensmi, then write DataType, ValueRank and ArrayDimensions.

    This is the shape of the one function in the UA layer that owns the workaround.
    """
    variable: UaVariable[object] = UaVariable(
        initial_value,
        name=name,
        writable=True,
        write_check_callback=write_check_callback,
        description=f"Spike variable {name}",
    )
    await container.add(variable)

    node = variable.ua_node
    if is_list:
        await node.write_value_rank(ua.ValueRank.OneDimension)
        await node.write_attribute(
            AttributeIds.ArrayDimensions,
            ua.DataValue(ua.Variant([0], ua.VariantType.UInt32)),
        )
    else:
        await node.write_value_rank(ua.ValueRank.Scalar)
    await node.write_attribute(
        AttributeIds.DataType,
        ua.DataValue(ua.Variant(data_type, ua.VariantType.NodeId)),
    )
    return variable


@pytest.fixture
async def machine() -> AsyncGenerator[_Machine]:
    config = ServerConfiguration()
    config.ua_server.endpoint_address = "opc.tcp://0.0.0.0:0/server"
    server = Server(config=config)
    machine = _Machine()
    async with server:
        await server.add_machine(machine)
        await server.start(blocking=False)
        yield machine


def _url(machine: _Machine, user: str) -> str:
    """Client URL for one of opensmi's default users, whose password equals the name."""
    bserver = machine.server.ua_server.bserver
    assert bserver is not None
    return f"opc.tcp://{user}:{user}@localhost:{bserver.port}"


async def _read_type_attributes(node: Node) -> tuple[ua.NodeId, int, Sequence[int] | None]:
    data_type, value_rank, array_dimensions = await node.read_attributes(
        [AttributeIds.DataType, AttributeIds.ValueRank, AttributeIds.ArrayDimensions]
    )
    for data_value in (data_type, value_rank, array_dimensions):
        assert data_value.StatusCode is not None
        assert data_value.StatusCode.is_good()
    assert data_type.Value is not None
    assert value_rank.Value is not None
    assert array_dimensions.Value is not None
    return data_type.Value.Value, value_rank.Value.Value, array_dimensions.Value.Value


def _mint_enum() -> type[IntEnum]:
    """Build an IntEnum at runtime, the way the UA layer will for a ROS2 constant set."""
    return IntEnum("SpikeMode", ["IDLE", "RUNNING", "STOPPED"], start=0)


async def test_list_of_builtin_reads_one_dimension_and_datatype(machine: _Machine) -> None:
    doubles = await _create_variable(
        machine.parameter_set,
        name="Doubles",
        initial_value=ua.Variant([1.0, 2.0], ua.VariantType.Double),
        data_type=DOUBLE,
        is_list=True,
    )
    ints = await _create_variable(
        machine.parameter_set,
        name="Ints",
        initial_value=ua.Variant([], ua.VariantType.Int32),
        data_type=INT32,
        is_list=True,
    )

    async with Client(url=_url(machine, "visitor")) as client:
        assert await _read_type_attributes(client.get_node(doubles.ua_node.nodeid)) == (
            DOUBLE,
            ua.ValueRank.OneDimension,
            [0],
        )
        assert await _read_type_attributes(client.get_node(ints.ua_node.nodeid)) == (
            INT32,
            ua.ValueRank.OneDimension,
            [0],
        )
        assert await client.get_node(doubles.ua_node.nodeid).read_value() == [1.0, 2.0]


async def test_scalar_reads_scalar_rank(machine: _Machine) -> None:
    scalar = await _create_variable(
        machine.parameter_set,
        name="Scalar",
        initial_value=ua.Variant(1.5, ua.VariantType.Double),
        data_type=DOUBLE,
        is_list=False,
    )

    async with Client(url=_url(machine, "visitor")) as client:
        data_type, value_rank, array_dimensions = await _read_type_attributes(client.get_node(scalar.ua_node.nodeid))
    assert data_type == DOUBLE
    assert value_rank == ua.ValueRank.Scalar
    assert not array_dimensions


async def test_enum_minted_at_runtime_reads_as_datatype(machine: _Machine) -> None:
    enum_type = _mint_enum()
    enum_data_type = await machine.server.ensure_enum(enum_type)
    scalar = await _create_variable(
        machine.parameter_set,
        name="Mode",
        initial_value=ua.Variant(0, ua.VariantType.Int32),
        data_type=enum_data_type,
        is_list=False,
    )
    modes = await _create_variable(
        machine.parameter_set,
        name="Modes",
        initial_value=ua.Variant([0, 1], ua.VariantType.Int32),
        data_type=enum_data_type,
        is_list=True,
    )

    async with Client(url=_url(machine, "visitor")) as client:
        data_type, value_rank, _ = await _read_type_attributes(client.get_node(scalar.ua_node.nodeid))
        assert (data_type, value_rank) == (enum_data_type, ua.ValueRank.Scalar)
        assert await _read_type_attributes(client.get_node(modes.ua_node.nodeid)) == (
            enum_data_type,
            ua.ValueRank.OneDimension,
            [0],
        )
        # The DataType points at a real Enumeration subtype; its fields live in DataTypeDefinition only.
        data_type_node = client.get_node(enum_data_type)
        assert (await data_type_node.read_browse_name()).Name == "SpikeMode"
        assert (await data_type_node.read_node_class()) == ua.NodeClass.DataType
        supertypes = await data_type_node.get_referenced_nodes(
            refs=ObjectIds.HasSubtype, direction=ua.BrowseDirection.Inverse
        )
        assert [node.nodeid for node in supertypes] == [_ns0(ObjectIds.Enumeration)]
        definition = await data_type_node.read_data_type_definition()
        assert isinstance(definition, ua.EnumDefinition)
        assert [(field.Value, field.Name) for field in definition.Fields] == [
            (0, "IDLE"),
            (1, "RUNNING"),
            (2, "STOPPED"),
        ]


async def test_client_write_passes_access_control_and_write_check(machine: _Machine) -> None:
    checked: list[object] = []

    async def write_check(value: object) -> None:
        checked.append(value)
        if isinstance(value, list) and any(item < 0 for item in value):  # pyright: ignore[reportUnknownVariableType]
            raise ua.UaStatusCodeError(StatusCodes.BadOutOfRange)

    doubles = await _create_variable(
        machine.parameter_set,
        name="Doubles",
        initial_value=ua.Variant([1.0, 2.0], ua.VariantType.Double),
        data_type=DOUBLE,
        is_list=True,
        write_check_callback=write_check,
    )
    enum_data_type = await machine.server.ensure_enum(_mint_enum())
    mode = await _create_variable(
        machine.parameter_set,
        name="Mode",
        initial_value=ua.Variant(0, ua.VariantType.Int32),
        data_type=enum_data_type,
        is_list=False,
        write_check_callback=write_check,
    )

    # opensmi also runs the write check for the initial value written during add().
    assert checked == [[1.0, 2.0], 0]
    checked.clear()

    accepted = ua.DataValue(ua.Variant([3.0, 4.0, 5.0], ua.VariantType.Double))
    refused = ua.DataValue(ua.Variant([-1.0], ua.VariantType.Double))

    async with Client(url=_url(machine, "operator")) as client:
        doubles_node = client.get_node(doubles.ua_node.nodeid)
        mode_node = client.get_node(mode.ua_node.nodeid)

        with pytest.raises(ua.uaerrors.BadUserAccessDenied):
            await doubles_node.write_value(accepted)
        assert checked == []

        await client.get_node(machine.lock.ua_node.nodeid).call_method("2:InitLock")

        await doubles_node.write_value(accepted)
        assert checked == [[3.0, 4.0, 5.0]]
        assert await doubles_node.read_value() == [3.0, 4.0, 5.0]

        with pytest.raises(ua.uaerrors.BadOutOfRange):
            await doubles_node.write_value(refused)
        assert checked[-1] == [-1.0]
        assert await doubles_node.read_value() == [3.0, 4.0, 5.0]

        await mode_node.write_value(ua.DataValue(ua.Variant(2, ua.VariantType.Int32)))
        assert checked[-1] == 2
        assert await mode_node.read_value() == 2

        assert await _read_type_attributes(doubles_node) == (DOUBLE, ua.ValueRank.OneDimension, [0])

    await doubles.write([6.0])
    assert await doubles.read() == [6.0]
