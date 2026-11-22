# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A message rendered as a complex Variable: atomic reads, one write per message, status on every field.

The tests drive a real server and read it as a client does, through browse paths.
"""

from __future__ import annotations

import asyncio
from datetime import UTC, datetime
from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua
from asyncua.ua.status_codes import StatusCodes

from opensmi.ros2.server.bindings import MonitoringBinding
from opensmi.ros2.server.errors import DeclarationError
from opensmi.ros2.server.projection import PlanVariable, generic_decode, generic_shape
from opensmi.ros2.server.recipes import Functions
from opensmi.ros2.server.status import GOOD, NO_COMMUNICATION
from tests import fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.served import (
    TestComponent,
    TestMachine,
    browse,
    connected,
    leaf,
    names_of,
    serve,
    until_status,
    until_value,
    value_of,
)

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from opensmi.server import Server

MESSAGE_TYPES = {
    "fixture_msgs/msg/Pose": fixture_messages.Pose,
    "fixture_msgs/msg/Point": fixture_messages.Point,
    "fixture_msgs/msg/Quaternion": fixture_messages.Quaternion,
    "fixture_msgs/msg/PoseWithCovariance": fixture_messages.PoseWithCovariance,
}

TARGET = MonitoringBinding(name="Target", path="/target", message_type="fixture_msgs/msg/Pose")
COVARIED = MonitoringBinding(name="Covaried", path="/covaried", message_type="fixture_msgs/msg/PoseWithCovariance")


class Platform(TestComponent):
    bindings = (TARGET, COVARIED)


@pytest.fixture
def runtime() -> FakeRuntime:
    return FakeRuntime(domain_id=7, message_types=MESSAGE_TYPES)


@pytest.fixture
async def served(runtime: FakeRuntime) -> AsyncIterator[tuple[Server, Any]]:
    machine = TestMachine(Platform(name="Platform"), runtime=runtime)
    server = await serve(machine)
    try:
        yield server, machine.added[0]
    finally:
        await server.shutdown()


def pose(x: float) -> Any:
    return fixture_messages.Pose(
        position=fixture_messages.Point(x=x, y=2.0, z=3.0),
        orientation=fixture_messages.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


async def test_a_client_reads_the_whole_message_and_a_field_of_it_and_they_agree(served, runtime: FakeRuntime) -> None:
    server, platform = served
    runtime.note_publishers("/target", 1)
    runtime.deliver("/target", pose(1.5))

    async with connected(server, "visitor") as client:
        monitoring = client.get_node(platform.monitoring.ua_node.nodeid)
        message = await browse(monitoring, "Target")
        x = await browse(message, "Position", "X")
        await until_value(x, 1.5)

        whole = await value_of(message)
        assert whole.position.x == 1.5
        assert whole.orientation.w == 1.0
        assert (await x.read_data_value()).SourceTimestamp == (await message.read_data_value()).SourceTimestamp


async def test_every_field_reads_the_status_of_the_message(served, runtime: FakeRuntime) -> None:
    server, platform = served
    x = leaf(platform.monitoring, "Target.Position.X")

    async with connected(server, "visitor") as client:
        field = client.get_node(x.ua_node.nodeid)
        await until_status(field, NO_COMMUNICATION.value)
        runtime.note_publishers("/target", 1)
        runtime.deliver("/target", pose(4.0))
        await until_status(field, GOOD.value)
        runtime.note_publishers("/target", 0)
        await until_status(field, NO_COMMUNICATION.value)


async def test_a_write_to_a_field_is_checked_on_the_whole_message_and_reaches_the_parent(
    served, runtime: FakeRuntime
) -> None:
    _, platform = served
    runtime.note_publishers("/covaried", 1)
    runtime.deliver("/covaried", fixture_messages.PoseWithCovariance(pose=pose(1.0), covariance=(0.0,) * 4))
    covariance = leaf(platform.monitoring, "Covaried.Covariance")
    await until_status_of(covariance)

    await covariance.write([1.0, 2.0, 3.0, 4.0])

    assert (await leaf(platform.monitoring, "Covaried.Covariance").read()) == [1.0, 2.0, 3.0, 4.0]
    assert platform.monitoring["Covaried"] is not None
    assert (await platform.monitoring["Covaried"].read()).covariance == [1.0, 2.0, 3.0, 4.0]
    with pytest.raises(ua.UaStatusCodeError) as refused:
        await covariance.write([1.0])
    assert refused.value.code == StatusCodes.BadTypeMismatch
    assert (await covariance.read()) == [1.0, 2.0, 3.0, 4.0]


async def until_status_of(variable: Any) -> None:
    """Wait for the mirror to have written a sample, which makes the message readable and writable."""
    for _ in range(400):
        if (await variable.ua_node.read_data_value(raise_on_bad_status=False)).StatusCode.is_good():
            return
        await asyncio.sleep(0.005)
    pytest.fail("the message never arrived")


def _stamped_point(_type: Any, context: Any) -> tuple[PlanVariable, ...]:
    return (
        PlanVariable(
            name="Taken",
            source=f"{context.path}/taken",
            initial=datetime.fromtimestamp(0, UTC),
            variant_type=ua.VariantType.DateTime,
        ),
    )


def _stamp(_message: Any, _context: Any) -> dict[str, object]:
    return {"Taken": datetime.fromtimestamp(5, UTC)}


class Stamping(TestComponent):
    bindings = (TARGET,)
    projections = (Functions("fixture_msgs/msg/Point", shaped_by=_stamped_point, decoded_by=_stamp),)


async def test_a_level_with_a_leaf_no_structure_can_carry_is_groups_and_plain_variables(
    runtime: FakeRuntime,
) -> None:
    machine = TestMachine(Stamping(name="Platform"), runtime=runtime)
    server = await serve(machine)
    try:
        platform = machine.added[0]
        runtime.note_publishers("/target", 1)
        runtime.deliver("/target", pose(1.0))
        taken = leaf(platform.monitoring, "Target.Position.Taken")

        async with connected(server, "visitor") as client:
            root = await browse(client.get_node(platform.monitoring.ua_node.nodeid), "Target")
            assert await root.read_node_class() == ua.NodeClass.Object
            assert "Orientation" in await names_of(root)
            await until_value(client.get_node(taken.ua_node.nodeid), datetime.fromtimestamp(5, UTC))
    finally:
        await server.shutdown()


def _lowercase(message_type: Any, context: Any) -> tuple[Any, ...]:
    return tuple(
        PlanVariable(
            name=entry.display_names[-1], source=entry.source, initial=entry.initial, variant_type=entry.variant_type
        )
        if isinstance(entry, PlanVariable)
        else entry
        for entry in generic_shape(message_type, context)
    )


class Lowercase(TestComponent):
    bindings = (TARGET,)
    projections = (
        Functions(
            "fixture_msgs/msg/Point",
            shaped_by=_lowercase,
            decoded_by=lambda message, context: generic_decode(message, context),
        ),
    )


async def test_a_name_opensmi_would_not_derive_is_refused_and_not_renamed_silently(runtime: FakeRuntime) -> None:
    machine = TestMachine(Lowercase(name="Platform"), runtime=runtime)

    with pytest.raises(DeclarationError, match="name the entry 'X'"):
        await serve(machine)
