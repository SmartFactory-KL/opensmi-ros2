# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A declared ROS2 parameter is a Parameter a client tunes, against a real in-process opensmi server
."""

from __future__ import annotations

import asyncio
from datetime import UTC, datetime
from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua
from asyncua.ua.attribute_ids import AttributeIds
from asyncua.ua.status_codes import StatusCodes
from opensmi.server import UaVariable

from opensmi.ros2.runtime.parameters import (
    ParameterDescription,
    ParameterRange,
    ParameterReading,
    ParameterType,
    ParameterUpdate,
    ParameterValue,
)
from opensmi.ros2.server.bindings import ROS2ParameterBinding
from opensmi.ros2.server.events import BINDING_EVENT_TYPE
from opensmi.ros2.server.parameters import PARAMETER_ABSENT, PARAMETER_TYPE_MISMATCH, PARAMETER_UNREACHABLE
from opensmi.ros2.server.required import satisfaction_of
from tests.fake_runtime import FakeRuntime
from tests.served import TestComponent, TestMachine, browse, connected, leaves, serve

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Callable

    from asyncua.client.client import Client
    from asyncua.common.node import Node
    from opensmi.server import Server

SETTLED = 5.0
"""How long a test waits for a task of ours to have run before calling it a failure.

Generous on purpose: what is being waited on is a task of ours reading a node, so the assertion is
*it happened* and never *it happened quickly* -- a tight bound would make this a flake generator
on a loaded runner.
"""

DI_URI = "http://opcfoundation.org/UA/DI/"
"""The namespace the Lock's methods are defined in."""

MAX_VEL_X = "FollowPath.max_vel_x"
"""A dotted ROS2 parameter name, as a controller plugin's parameters are."""

CONTROLLER = ROS2ParameterBinding(
    name="Controller",
    path="/controller_server",
    parameters={
        MAX_VEL_X: ParameterType.DOUBLE,
        "robot_name": ParameterType.STRING,
        "use_sim_time": ParameterType.BOOL,
    },
    read_only=("use_sim_time",),
    retry_interval=0.05,
)

# Declared without a leading slash to exercise normalisation.
PLANNER = ROS2ParameterBinding(
    name="Planner",
    path="planner_server",
    parameters={"tolerance": ParameterType.DOUBLE},
    retry_interval=0.05,
)


def held() -> dict[str, ParameterValue | None]:
    """Return what the controller node holds when it is up."""
    return {
        MAX_VEL_X: ParameterValue(type=ParameterType.DOUBLE, value=0.26),
        "robot_name": ParameterValue(type=ParameterType.STRING, value="rover"),
        "use_sim_time": ParameterValue(type=ParameterType.BOOL, value=True),
    }


class Platform(TestComponent):
    bindings = (CONTROLLER, PLANNER)


@pytest.fixture
def runtime() -> FakeRuntime:
    """A Runtime whose two ROS2 nodes are up and holding what a live robot would."""
    fake = FakeRuntime(domain_id=7)
    fake.serve_parameters("/controller_server", held())
    fake.serve_parameters("/planner_server", {"tolerance": ParameterValue(type=ParameterType.DOUBLE, value=0.1)})
    return fake


@pytest.fixture
def absent_runtime() -> FakeRuntime:
    """A Runtime whose ROS2 nodes are not running, which is a robot that is off."""
    return FakeRuntime(domain_id=7)


async def start(runtime: FakeRuntime, component: type[TestComponent] = Platform) -> tuple[Server, Any]:
    """Bring up a server carrying one Machine with one Component, and hand back both."""
    machine = TestMachine(component(name="Platform"), runtime=runtime)
    return await serve(machine), machine


@pytest.fixture
async def served(runtime: FakeRuntime) -> AsyncIterator[tuple[Server, Any]]:
    server, machine = await start(runtime)
    try:
        yield server, machine
    finally:
        await server.shutdown()


@pytest.fixture
async def served_without_the_robot(absent_runtime: FakeRuntime) -> AsyncIterator[tuple[Server, Any]]:
    server, machine = await start(absent_runtime)
    try:
        yield server, machine
    finally:
        await server.shutdown()


def platform(machine: Any) -> Any:
    return machine.added[0]


def parameter_nodes(machine: Any) -> dict[str, Node]:
    """Return the server-side node of every variable in the Component's ParameterSet, by name."""
    return {name: variable.ua_node for name, variable in leaves(platform(machine).parameter_set).items()}


def parameter_node(machine: Any, name: str) -> Node:
    return parameter_nodes(machine)[name]


def remote(client: Client, machine: Any, name: str) -> Node:
    return client.get_node(parameter_node(machine, name).nodeid)


async def holding_the_lock(client: Client, machine: Any) -> None:
    """Take the Machine's Lock, which a write needs besides the access level."""
    di = await client.get_namespace_index(DI_URI)
    await client.get_node(machine.lock.ua_node.nodeid).call_method(f"{di}:InitLock")


async def refused(node: Node, value: ua.Variant) -> int:
    """Write a value a client expects to be refused, and return the status code it was answered with."""
    with pytest.raises(ua.UaStatusCodeError) as caught:
        await node.write_value(ua.DataValue(value))
    return caught.value.code


class Collector:
    """An asyncua subscription handler that keeps what it was sent."""

    def __init__(self) -> None:
        self.events: list[Any] = []

    def event_notification(self, event: Any) -> None:
        self.events.append(event)

    async def coded(self, code: str, within: float = SETTLED) -> Any:
        """Wait for the first event carrying one code, failing rather than hanging."""
        deadline = asyncio.get_running_loop().time() + within
        while True:
            matching = [each for each in self.events if getattr(each, "Code", None) == code]
            if matching:
                return matching[0]
            seen = [getattr(each, "Code", None) for each in self.events]
            assert asyncio.get_running_loop().time() < deadline, f"no {code} event arrived; got {seen}"
            await asyncio.sleep(0.005)


async def subscribed(client: Client, server: Server, machine: Any) -> Collector:
    """Subscribe to the Component's BindingEvents and hand back what they land in."""
    collector = Collector()
    subscription = await client.create_subscription(10, collector)
    await subscription.subscribe_events(
        client.get_node(platform(machine).notification.ua_node.nodeid),
        client.get_node(server.ua_get_node_id(BINDING_EVENT_TYPE)),
    )
    return collector


async def until(answered: Callable[[], bool], complaint: str = "it never happened", within: float = SETTLED) -> None:
    """Poll until something one of our tasks is doing has happened."""
    deadline = asyncio.get_running_loop().time() + within
    while not answered():
        assert asyncio.get_running_loop().time() < deadline, complaint
        await asyncio.sleep(0.005)


async def data_value(node: Node) -> ua.DataValue:
    """Read a node without raising, which a bad status otherwise makes a client do."""
    return await node.read_data_value(raise_on_bad_status=False)


async def until_reads(node: Node, answered: Callable[[ua.DataValue], bool], complaint: str) -> None:
    deadline = asyncio.get_running_loop().time() + SETTLED
    while True:
        read = await data_value(node)
        if answered(read):
            return
        shown = None if read.Value is None else read.Value.Value
        assert asyncio.get_running_loop().time() < deadline, f"{complaint}; it read {shown!r} under {read.StatusCode}"
        await asyncio.sleep(0.005)


async def until_value(node: Node, expected: object) -> None:
    """Read a node until it carries the value the mirror was given, under a good status."""
    await until_reads(
        node,
        lambda each: good(each) and each.Value is not None and each.Value.Value == expected,
        f"the value never became {expected!r}",
    )


async def until_status(node: Node, expected: int) -> None:
    """Read a node until its status code is the one expected."""
    await until_reads(
        node,
        lambda each: each.StatusCode is not None and each.StatusCode.value == expected,
        f"the status never became {expected:#x}",
    )


def good(read: ua.DataValue) -> bool:
    return read.StatusCode is None or read.StatusCode.is_good()


def controller_reads(runtime: FakeRuntime) -> int:
    return len([each for each in runtime.parameter_reads if each[0] == "/controller_server"])


def event_of(name: str, value: object, parameter_type: ParameterType = ParameterType.DOUBLE) -> ParameterUpdate:
    """Return one change the controller node announces on `/parameter_events`."""
    return ParameterUpdate(
        node="/controller_server",
        readings=(ParameterReading(name=name, present=True, value=ParameterValue(type=parameter_type, value=value)),),
    )


# -- what a client sees before anything answers ----------------------------------------------


async def test_a_declared_node_becomes_parameters_on_the_components_parameter_set(served) -> None:
    _, machine = served

    assert sorted(parameter_nodes(machine)) == [
        "Controller.FollowPath.MaxVelX",
        "Controller.RobotName",
        "Controller.UseSimTime",
        "Planner.Tolerance",
    ]


async def test_a_namespaced_parameter_is_one_group_per_level(served) -> None:
    server, machine = served
    node = parameter_node(machine, "Controller.FollowPath.MaxVelX")

    assert (await node.read_browse_name()).Name == "MaxVelX"
    assert (await node.read_display_name()).Text == "max_vel_x"
    async with connected(server) as client:
        parameter_set = client.get_node(platform(machine).parameter_set.ua_node.nodeid)
        found = await browse(parameter_set, "Controller", "FollowPath", "MaxVelX")
        group = await browse(parameter_set, "Controller", "FollowPath")
        assert found.nodeid == node.nodeid
        assert await group.read_node_class() == ua.NodeClass.Object
        assert (await group.read_display_name()).Text == "FollowPath"


async def test_a_parameter_carries_its_declared_type_rather_than_a_discovered_one(served_without_the_robot) -> None:
    _, machine = served_without_the_robot

    assert (await parameter_node(machine, "Controller.FollowPath.MaxVelX").read_data_type_as_variant_type()) == (
        ua.VariantType.Double
    )
    assert (await parameter_node(machine, "Controller.RobotName").read_data_type_as_variant_type()) == (
        ua.VariantType.String
    )
    assert (await parameter_node(machine, "Controller.UseSimTime").read_data_type_as_variant_type()) == (
        ua.VariantType.Boolean
    )


async def test_a_declared_parameter_is_writable_and_a_read_only_one_is_not(served) -> None:
    # Read-only is declared, never discovered: access levels cannot move after init.
    _, machine = served

    writable = await parameter_node(machine, "Controller.FollowPath.MaxVelX").read_attribute(AttributeIds.AccessLevel)
    read_only = await parameter_node(machine, "Controller.UseSimTime").read_attribute(AttributeIds.AccessLevel)

    assert writable.Value is not None
    assert read_only.Value is not None
    assert ua.AccessLevel.CurrentWrite in ua.AccessLevel.parse_bitfield(writable.Value.Value)
    assert ua.AccessLevel.CurrentWrite not in ua.AccessLevel.parse_bitfield(read_only.Value.Value)


async def test_a_parameter_names_the_ros2_parameter_it_reads(served) -> None:
    _, machine = served

    description = await parameter_node(machine, "Controller.FollowPath.MaxVelX").read_description()

    assert description.Text == f"/controller_server/{MAX_VEL_X}"


async def test_a_binding_whose_node_is_not_running_reads_bad_no_communication(served_without_the_robot) -> None:
    server, machine = served_without_the_robot

    async with connected(server) as client:
        await until_status(remote(client, machine, "Controller.FollowPath.MaxVelX"), StatusCodes.BadNoCommunication)


async def test_nothing_waits_for_the_robot_while_the_server_is_being_built(absent_runtime: FakeRuntime) -> None:
    server, machine = await start(absent_runtime)
    try:
        assert len(parameter_nodes(machine)) == 4
    finally:
        await server.shutdown()


# -- reading the node ------------------------------------------------------------------------


async def test_every_declared_name_is_read_and_described_one_request_at_a_time(served, runtime: FakeRuntime) -> None:
    # Batch calls return empty without error on one undeclared name, so read per name.
    _, machine = served
    names = [
        ("/controller_server", MAX_VEL_X),
        ("/controller_server", "robot_name"),
        ("/controller_server", "use_sim_time"),
        ("/planner_server", "tolerance"),
    ]

    await until(
        lambda: platform(machine).ros2_unconfirmed_parameters == {"Controller": (), "Planner": ()},
        "a declared name was never confirmed present",
    )

    assert sorted(runtime.parameter_reads) == sorted(names * 2)


async def test_a_client_reads_the_value_the_node_holds(served) -> None:
    server, machine = served

    async with connected(server) as client:
        await until_value(remote(client, machine, "Controller.FollowPath.MaxVelX"), 0.26)
        assert await (remote(client, machine, "Controller.RobotName")).read_value() == "rover"


async def test_a_node_declared_without_its_leading_slash_is_still_found(served) -> None:
    server, machine = served

    async with connected(server) as client:
        await until_value(remote(client, machine, "Planner.Tolerance"), 0.1)


async def test_a_binding_is_satisfied_when_every_declared_name_is_confirmed(served) -> None:
    _, machine = served

    await until(
        lambda: platform(machine).ros2_unconfirmed_parameters == {"Controller": (), "Planner": ()},
        "a declared name was never confirmed present",
    )


async def test_satisfied_is_asked_of_the_parameter_mirror(served) -> None:
    _, machine = served
    probe = satisfaction_of(platform(machine), CONTROLLER)

    assert probe is not None
    await until(probe, "the controller Binding never became Satisfied")


async def test_a_binding_whose_node_is_off_is_not_satisfied(served_without_the_robot) -> None:
    _, machine = served_without_the_robot
    probe = satisfaction_of(platform(machine), CONTROLLER)

    assert probe is not None
    assert probe() is False


# -- a name the robot does not have ------------------------------------------------------------


class Typo(TestComponent):
    bindings = (
        ROS2ParameterBinding(
            name="Controller",
            path="/controller_server",
            parameters={"FollowPath.max_vel_y": ParameterType.DOUBLE},
            retry_interval=0.05,
        ),
    )


async def test_a_declared_name_the_node_does_not_have_leaves_the_binding_unsatisfied(absent_runtime) -> None:
    # The robot comes up after the client listens, since an event is a moment.
    server, machine = await start(absent_runtime, Typo)
    try:
        async with connected(server) as client:
            collector = await subscribed(client, server, machine)
            node = remote(client, machine, "Controller.FollowPath.MaxVelY")

            absent_runtime.serve_parameters("/controller_server", held())

            await until_status(node, StatusCodes.BadNoCommunication)
            event = await collector.coded(PARAMETER_ABSENT)

            assert event.Binding == "Controller"
            assert event.ROS2Path == "/controller_server/FollowPath.max_vel_y"
            assert "FollowPath.max_vel_y" in event.Message.Text
        assert platform(machine).ros2_unconfirmed_parameters == {"Controller": ("FollowPath.max_vel_y",)}
    finally:
        await server.shutdown()


class Disagreeing(TestComponent):
    bindings = (
        ROS2ParameterBinding(
            name="Controller",
            path="/controller_server",
            parameters={"robot_name": ParameterType.DOUBLE},
            retry_interval=0.05,
        ),
    )


async def test_a_type_the_node_disagrees_about_is_reported_and_the_parameter_stays_bad(absent_runtime) -> None:
    server, machine = await start(absent_runtime, Disagreeing)
    try:
        async with connected(server) as client:
            collector = await subscribed(client, server, machine)
            node = remote(client, machine, "Controller.RobotName")

            absent_runtime.serve_parameters("/controller_server", held())

            await until_status(node, StatusCodes.BadNoCommunication)
            event = await collector.coded(PARAMETER_TYPE_MISMATCH)

            assert "string" in event.Message.Text
            assert "double" in event.Message.Text
    finally:
        await server.shutdown()


# -- a client's write the node takes ---------------------------------------------------------------


async def test_an_accepted_write_is_answered_good_and_stored(served, runtime: FakeRuntime) -> None:
    server, machine = served

    async with connected(server) as client:
        node = remote(client, machine, "Controller.FollowPath.MaxVelX")
        await holding_the_lock(client, machine)
        await until_value(node, 0.26)

        await node.write_value(ua.DataValue(ua.Variant(0.5, ua.VariantType.Double)))

        read = await data_value(node)
        assert good(read)
        assert read.Value is not None
        assert read.Value.Value == 0.5

    ((path, batch),) = runtime.parameter_writes
    assert path == "/controller_server"
    assert batch == {MAX_VEL_X: ParameterValue(type=ParameterType.DOUBLE, value=0.5)}
    assert type(batch[MAX_VEL_X].value) is float


async def test_several_parameters_written_in_one_request_are_each_decided_on_their_own(
    served,
    runtime: FakeRuntime,
) -> None:
    runtime.refuse_parameter("/controller_server", "robot_name", "the name is fixed at launch")
    server, machine = served

    async with connected(server) as client:
        velocity = remote(client, machine, "Controller.FollowPath.MaxVelX")
        name = remote(client, machine, "Controller.RobotName")
        await holding_the_lock(client, machine)
        await until_value(velocity, 0.26)

        results = await client.write_values(
            [velocity, name],
            [ua.DataValue(ua.Variant(0.4, ua.VariantType.Double)), ua.DataValue(ua.Variant("other"))],
            raise_on_partial_error=False,
        )

        assert [each.value for each in results] == [StatusCodes.Good, StatusCodes.BadInvalidArgument]
        assert await velocity.read_value() == 0.4
        assert await name.read_value() == "rover"

    assert len(runtime.parameter_writes) == 2


async def test_a_container_parameter_is_an_array_a_client_reads_and_writes(runtime: FakeRuntime) -> None:
    runtime.serve_parameters("/local_costmap", costmap())
    server, machine = await start(runtime, WithArrays)
    try:
        async with connected(server) as client:
            scales = remote(client, machine, "Costmap.PluginScales")
            await holding_the_lock(client, machine)

            await until_value(scales, [0.5, 1.5])
            assert await (remote(client, machine, "Costmap.PluginNames")).read_value() == [
                "inflation",
                "obstacle",
            ]
            assert await (remote(client, machine, "Costmap.Footprint")).read_value() == b""

            await scales.write_value(ua.DataValue(ua.Variant([0.25, 2.5], ua.VariantType.Double)))
            assert await scales.read_value() == [0.25, 2.5]

        ((_, batch),) = runtime.parameter_writes
        assert batch["plugin_scales"] == ParameterValue(type=ParameterType.DOUBLE_ARRAY, value=(0.25, 2.5))
        assert all(type(each) is float for each in batch["plugin_scales"].value)  # type: ignore[union-attr]
    finally:
        await server.shutdown()


async def test_the_typed_zero_is_not_taken_for_a_value_the_node_holds(runtime: FakeRuntime) -> None:
    runtime.serve_parameters("/controller_server", {**held(), MAX_VEL_X: None})
    server, machine = await start(runtime)
    try:
        async with connected(server) as client:
            node = remote(client, machine, "Controller.FollowPath.MaxVelX")
            await holding_the_lock(client, machine)
            await until_status(node, StatusCodes.BadWaitingForInitialData)

            await node.write_value(ua.DataValue(ua.Variant(0.0, ua.VariantType.Double)))

            assert good(await data_value(node))

        assert runtime.parameter_writes == (
            ("/controller_server", {MAX_VEL_X: ParameterValue(type=ParameterType.DOUBLE, value=0.0)}),
        )
    finally:
        await server.shutdown()


async def test_the_typed_zero_written_to_a_silent_node_is_not_answered_good(served_without_the_robot) -> None:
    server, machine = served_without_the_robot

    async with connected(server) as client:
        node = remote(client, machine, "Controller.FollowPath.MaxVelX")
        await holding_the_lock(client, machine)
        await until_status(node, StatusCodes.BadNoCommunication)

        assert await refused(node, ua.Variant(0.0, ua.VariantType.Double)) == StatusCodes.BadTimeout
        await until_status(node, StatusCodes.BadNoCommunication)


# -- a client's write refused without asking the node ------------------------------------------------


async def test_a_read_only_parameter_refuses_a_write_with_bad_not_writable(served, runtime: FakeRuntime) -> None:
    server, machine = served

    async with connected(server) as client:
        node = remote(client, machine, "Controller.UseSimTime")
        await holding_the_lock(client, machine)
        await until_value(node, True)

        code = await refused(node, ua.Variant(False, ua.VariantType.Boolean))

        assert code == StatusCodes.BadNotWritable
        assert await node.read_value() is True

    assert runtime.parameter_writes == ()


async def test_a_parameter_the_node_describes_as_read_only_refuses_with_bad_not_writable(runtime: FakeRuntime) -> None:
    runtime.describe_parameter(
        "/controller_server",
        "robot_name",
        ParameterDescription(name="robot_name", present=True, type=ParameterType.STRING, read_only=True),
    )
    server, machine = await start(runtime)
    try:
        async with connected(server) as client:
            node = remote(client, machine, "Controller.RobotName")
            await holding_the_lock(client, machine)
            await until_value(node, "rover")

            assert await refused(node, ua.Variant("other")) == StatusCodes.BadNotWritable
            assert await node.read_value() == "rover"

        assert runtime.parameter_writes == ()
    finally:
        await server.shutdown()


@pytest.mark.parametrize(
    "written",
    [
        ua.Variant(1, ua.VariantType.Int64),
        ua.Variant("fast"),
        ua.Variant(True, ua.VariantType.Boolean),
        ua.Variant([0.5], ua.VariantType.Double),
    ],
    ids=["integer", "string", "boolean", "array"],
)
async def test_a_value_of_another_type_is_refused_with_bad_type_mismatch(
    served,
    runtime: FakeRuntime,
    written: ua.Variant,
) -> None:
    server, machine = served

    async with connected(server) as client:
        node = remote(client, machine, "Controller.FollowPath.MaxVelX")
        await holding_the_lock(client, machine)
        await until_value(node, 0.26)

        assert await refused(node, written) == StatusCodes.BadTypeMismatch
        assert await node.read_value() == 0.26

    assert runtime.parameter_writes == ()


async def test_a_value_outside_the_declared_range_is_refused_with_bad_out_of_range(runtime: FakeRuntime) -> None:
    runtime.describe_parameter(
        "/controller_server",
        MAX_VEL_X,
        ParameterDescription(
            name=MAX_VEL_X,
            present=True,
            type=ParameterType.DOUBLE,
            range=ParameterRange(low=0.0, high=1.0),
        ),
    )
    server, machine = await start(runtime)
    try:
        async with connected(server) as client:
            node = remote(client, machine, "Controller.FollowPath.MaxVelX")
            await holding_the_lock(client, machine)
            await until_value(node, 0.26)

            assert await refused(node, ua.Variant(5000.0, ua.VariantType.Double)) == StatusCodes.BadOutOfRange
            assert await node.read_value() == 0.26
            assert runtime.parameter_writes == ()

            await node.write_value(ua.DataValue(ua.Variant(1.0, ua.VariantType.Double)))
            assert await node.read_value() == 1.0
    finally:
        await server.shutdown()


async def test_a_value_the_guard_refuses_does_not_reach_the_node(served, runtime: FakeRuntime) -> None:
    runtime.refuse_parameter_value("/controller_server", MAX_VEL_X, "a float64 wants a float")
    server, machine = served

    async with connected(server) as client:
        node = remote(client, machine, "Controller.FollowPath.MaxVelX")
        sibling = remote(client, machine, "Controller.RobotName")
        await holding_the_lock(client, machine)
        await until_value(node, 0.26)

        assert await refused(node, ua.Variant(0.5, ua.VariantType.Double)) == StatusCodes.BadTypeMismatch
        assert good(await data_value(node))
        assert good(await data_value(sibling))

    assert runtime.parameter_writes == ()


# -- a client's write the node refuses -----------------------------------------------------------------


@pytest.mark.parametrize(
    ("reason", "code"),
    [
        ("speed_limit out of range. Min: 0, Max: 100, value: 5000", StatusCodes.BadOutOfRange),
        ("the controller is active, deactivate it first", StatusCodes.BadInvalidArgument),
    ],
    ids=["range", "other"],
)
async def test_a_write_the_node_refuses_is_answered_bad_and_not_stored(
    served,
    runtime: FakeRuntime,
    reason: str,
    code: int,
) -> None:
    runtime.refuse_parameter("/controller_server", MAX_VEL_X, reason)
    server, machine = served

    async with connected(server) as client:
        node = remote(client, machine, "Controller.FollowPath.MaxVelX")
        await holding_the_lock(client, machine)
        await until_value(node, 0.26)

        assert await refused(node, ua.Variant(5000.0, ua.VariantType.Double)) == code

        read = await data_value(node)
        assert good(read)
        assert read.Value is not None
        assert read.Value.Value == 0.26

    assert len(runtime.parameter_writes) == 1


# -- a client's write the node does not answer -------------------------------------------------------


class Hurried(TestComponent):
    bindings = (
        ROS2ParameterBinding(
            name="Controller",
            path="/controller_server",
            parameters={MAX_VEL_X: ParameterType.DOUBLE, "robot_name": ParameterType.STRING},
            retry_interval=0.05,
            write_timeout=0.2,
        ),
    )


async def test_a_write_the_node_does_not_answer_in_time_is_answered_bad_timeout(runtime: FakeRuntime) -> None:
    # The node took the value but never answered: BadTimeout, then a re-read.
    runtime.hold_parameter_writes()
    server, machine = await start(runtime, Hurried)
    try:
        async with connected(server) as client:
            node = remote(client, machine, "Controller.FollowPath.MaxVelX")
            await holding_the_lock(client, machine)
            await until_value(node, 0.26)

            assert await refused(node, ua.Variant(0.5, ua.VariantType.Double)) == StatusCodes.BadTimeout

            await until_value(node, 0.5)
    finally:
        await server.shutdown()


async def test_a_write_to_a_node_that_has_gone_is_answered_bad_timeout_and_the_node_is_found_out(
    runtime: FakeRuntime,
) -> None:
    server, machine = await start(runtime, Hurried)
    try:
        async with connected(server) as client:
            node = remote(client, machine, "Controller.FollowPath.MaxVelX")
            sibling = remote(client, machine, "Controller.RobotName")
            collector = await subscribed(client, server, machine)
            await holding_the_lock(client, machine)
            await until_value(node, 0.26)

            runtime.stop_parameters("/controller_server")

            assert await refused(node, ua.Variant(0.5, ua.VariantType.Double)) == StatusCodes.BadTimeout

            event = await collector.coded(PARAMETER_UNREACHABLE)
            await until_status(node, StatusCodes.BadNoCommunication)
            await until_status(sibling, StatusCodes.BadNoCommunication)

            assert event.Binding == "Controller"
            assert event.ROS2Path == "/controller_server"
    finally:
        await server.shutdown()


# -- writes to one Parameter, one at a time -------------------------------------------------------------


async def test_writes_to_one_parameter_are_decided_one_after_the_other(served, runtime: FakeRuntime) -> None:
    server, machine = served
    gate = runtime.hold_parameter_writes()

    async with connected(server) as client:
        node = remote(client, machine, "Controller.FollowPath.MaxVelX")
        await holding_the_lock(client, machine)
        await until_value(node, 0.26)

        first = asyncio.create_task(node.write_value(ua.DataValue(ua.Variant(0.4, ua.VariantType.Double))))
        await until(lambda: len(runtime.parameter_writes) == 1, "the first write never reached the node")
        second = asyncio.create_task(node.write_value(ua.DataValue(ua.Variant(0.6, ua.VariantType.Double))))
        await asyncio.sleep(0.1)

        assert len(runtime.parameter_writes) == 1

        gate.set()
        await first
        await second

        assert [batch[MAX_VEL_X].value for _, batch in runtime.parameter_writes] == [0.4, 0.6]
        assert await node.read_value() == 0.6


# -- following the node -------------------------------------------------------------------------


async def test_a_change_the_node_made_itself_reaches_the_parameter_without_a_call(served, runtime: FakeRuntime) -> None:
    server, machine = served

    async with connected(server) as client:
        node = remote(client, machine, "Controller.FollowPath.MaxVelX")
        await until_value(node, 0.26)

        runtime.deliver_parameter_event(event_of(MAX_VEL_X, 0.42))

        await until_value(node, 0.42)

    assert runtime.parameter_writes == ()


async def test_a_read_only_parameter_still_follows_the_node(served, runtime: FakeRuntime) -> None:
    server, machine = served

    async with connected(server) as client:
        node = remote(client, machine, "Controller.UseSimTime")
        await until_value(node, True)

        runtime.deliver_parameter_event(event_of("use_sim_time", False, ParameterType.BOOL))

        await until_value(node, False)

    assert runtime.parameter_writes == ()


async def test_a_change_announced_while_a_write_is_in_flight_wins(served, runtime: FakeRuntime) -> None:
    server, machine = served
    gate = runtime.hold_parameter_writes()

    async with connected(server) as client:
        node = remote(client, machine, "Controller.FollowPath.MaxVelX")
        await holding_the_lock(client, machine)
        await until_value(node, 0.26)

        writing = asyncio.create_task(node.write_value(ua.DataValue(ua.Variant(5.0, ua.VariantType.Double))))
        await until(lambda: bool(runtime.parameter_writes), "the write never reached the node")

        runtime.deliver_parameter_event(event_of(MAX_VEL_X, 1.5))
        await asyncio.sleep(0.05)
        gate.set()
        await writing

        await until_value(node, 1.5)
        await asyncio.sleep(0.1)
        settled = await data_value(node)
        assert settled.Value is not None
        assert settled.Value.Value == 1.5
        assert good(settled)


async def test_a_mirrored_value_carries_the_server_timestamp_of_its_arrival(served, runtime: FakeRuntime) -> None:
    server, machine = served

    async with connected(server) as client:
        node = remote(client, machine, "Controller.FollowPath.MaxVelX")
        await until_value(node, 0.26)
        before = datetime.now(UTC)

        runtime.deliver_parameter_event(event_of(MAX_VEL_X, 0.42))

        await until_value(node, 0.42)
        stamped = (await data_value(node)).ServerTimestamp
        assert stamped is not None, "the mirrored value reached the client without a ServerTimestamp"
        assert stamped.replace(tzinfo=UTC) >= before.replace(microsecond=0)


async def test_a_parameter_the_node_deleted_goes_bad(served, runtime: FakeRuntime) -> None:
    server, machine = served

    async with connected(server) as client:
        node = remote(client, machine, "Controller.RobotName")
        await until_value(node, "rover")

        runtime.serve_parameters(
            "/controller_server", {name: value for name, value in held().items() if name != "robot_name"}
        )
        runtime.deliver_parameter_event(
            ParameterUpdate(node="/controller_server", readings=(ParameterReading(name="robot_name", present=False),))
        )

        await until_status(node, StatusCodes.BadNoCommunication)


async def test_an_event_for_a_name_this_binding_did_not_declare_is_ignored(served, runtime: FakeRuntime) -> None:
    server, machine = served

    async with connected(server) as client:
        await until_value(remote(client, machine, "Controller.RobotName"), "rover")

        runtime.deliver_parameter_event(event_of("something_else", "x", ParameterType.STRING))
        await asyncio.sleep(0.05)

    assert sorted(parameter_nodes(machine)) == [
        "Controller.FollowPath.MaxVelX",
        "Controller.RobotName",
        "Controller.UseSimTime",
        "Planner.Tolerance",
    ]


async def test_a_node_that_was_off_at_boot_recovers_without_anything_announcing_it(absent_runtime) -> None:
    # No parameter event fires for a node that comes up unchanged, hence the retry.
    server, machine = await start(absent_runtime)
    try:
        async with connected(server) as client:
            node = remote(client, machine, "Controller.FollowPath.MaxVelX")
            await until_status(node, StatusCodes.BadNoCommunication)

            absent_runtime.serve_parameters("/controller_server", held())

            await until_value(node, 0.26)

        await until(
            lambda: platform(machine).ros2_unconfirmed_parameters["Controller"] == (),
            "the controller was never confirmed",
        )
        reads = controller_reads(absent_runtime)
        await asyncio.sleep(0.2)

        assert controller_reads(absent_runtime) == reads, "a confirmed Binding went on reading its node"
    finally:
        await server.shutdown()


# -- the container types, which are the ones a variant type cannot be inferred for ----------------


ARRAYS = ROS2ParameterBinding(
    name="Costmap",
    path="/local_costmap",
    parameters={
        "plugin_scales": ParameterType.DOUBLE_ARRAY,
        "plugin_names": ParameterType.STRING_ARRAY,
        "footprint": ParameterType.BYTE_ARRAY,
    },
    retry_interval=0.05,
)


class WithArrays(TestComponent):
    bindings = (ARRAYS,)


def costmap() -> dict[str, ParameterValue | None]:
    return {
        "plugin_scales": ParameterValue(type=ParameterType.DOUBLE_ARRAY, value=(0.5, 1.5)),
        "plugin_names": ParameterValue(type=ParameterType.STRING_ARRAY, value=("inflation", "obstacle")),
        "footprint": ParameterValue(type=ParameterType.BYTE_ARRAY, value=b""),
    }


async def test_an_empty_container_parameter_still_carries_its_type_before_the_robot_answers(
    absent_runtime: FakeRuntime,
) -> None:
    server, machine = await start(absent_runtime, WithArrays)
    try:
        scales = parameter_node(machine, "Costmap.PluginScales")
        footprint = parameter_node(machine, "Costmap.Footprint")

        assert (await scales.read_data_type_as_variant_type()) == ua.VariantType.Double
        assert (await scales.read_value_rank()) == ua.ValueRank.OneDimension
        assert (await footprint.read_data_type_as_variant_type()) == ua.VariantType.ByteString
    finally:
        await server.shutdown()


# -- what is not a way into ROS2 -----------------------------------------------------------------


class WithAHandWrittenParameter(Platform):
    """A Component using the one sanctioned escape hatch: subclass, override, add a variable."""

    async def _init(self) -> None:
        await super()._init()
        await self.parameter_set.add(UaVariable(1.0, name="hand_written", writable=True, description="ours"))


async def test_a_parameter_nobody_declared_is_not_a_way_into_ros2(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, WithAHandWrittenParameter)
    try:
        async with connected(server) as client:
            node = remote(client, machine, "hand_written")
            await holding_the_lock(client, machine)
            await node.write_value(ua.DataValue(ua.Variant(2.0, ua.VariantType.Double)))
            await asyncio.sleep(0.05)

            assert await node.read_value() == 2.0

        assert runtime.parameter_writes == ()
    finally:
        await server.shutdown()


# -- lifetime ---------------------------------------------------------------------------------------


async def test_shutting_the_machine_down_stops_following_its_nodes(runtime: FakeRuntime) -> None:
    server, _ = await start(runtime)
    await until(lambda: runtime.parameters._watching, "the parameter events were never followed")

    await server.shutdown()

    assert runtime.parameters._watching is False
