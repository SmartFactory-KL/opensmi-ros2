# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A required Binding is the StartupSkill's contract, and a client can watch it.

The fake Runtime levers are `note_publishers`, `set_graph` and `serve_parameters`. opensmi adds no
StartupSkill itself at the pinned commit, so several tests cover what this package wires.
"""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua
from asyncua.ua.status_codes import StatusCodes
from opensmi.core import MachineryItemState, SkillState
from opensmi.core.nodesets import DiNodeIds, SmartFactoryMachineSetNodeIds, SmartFactorySkillSetNodeIds
from opensmi.server.base_startup_skill import BaseStartupSkill

from opensmi.ros2.runtime.graph import GraphSnapshot
from opensmi.ros2.runtime.parameters import ParameterType, ParameterValue
from opensmi.ros2.server.bindings import (
    ActionBinding,
    LifecycleBinding,
    MethodBinding,
    MonitoringBinding,
    ROS2ParameterBinding,
)
from opensmi.ros2.server.config import ROS2Config
from opensmi.ros2.server.errors import DeclarationError, ImplementationClassError
from opensmi.ros2.server.events import BINDING_EVENT_TYPE
from opensmi.ros2.server.introspection import ROS2IntrospectionComponent
from opensmi.ros2.server.lifecycle import LifecycleNodeState
from opensmi.ros2.server.required import (
    COMPONENT_NOT_RUNNING,
    REQUIRED_BINDING_ABSENT,
    REQUIRED_BINDING_LOST,
    ROS2StartupSkill,
)
from opensmi.ros2.server.startup import ROS2LifecycleStartupSkill
from tests import fixture_lifecycle, fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.served import TestComponent, TestMachine, alone, connected, monitoring_nodes, serve

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Callable

    from asyncua.client.client import Client
    from opensmi.server import Server

SETTLED = 5.0
"""Long enough not to flake on a loaded runner, short enough that a hang fails one test."""

DEADLINE = 0.3
"""What every required Binding here is given, so that a startup that fails takes a third of a second.

A real deployment's is thirty seconds, which is the default this file deliberately does not use:
what is under test is the shape of the wait, not how patient it is.
"""

MESSAGE_TYPES = {"fixture_msgs/msg/Pose": fixture_messages.Pose}
SERVICE_TYPES = {"fixture_msgs/srv/SetMode": fixture_messages.SetMode}
ACTION_TYPES = {"fixture_msgs/action/Navigate": fixture_messages.Navigate}

GOOD = ua.StatusCode(ua.UInt32(StatusCodes.Good))
INVALID_STATE = ua.StatusCode(ua.UInt32(StatusCodes.BadInvalidState))
DENIED = ua.StatusCode(ua.UInt32(StatusCodes.BadUserAccessDenied))

LIDAR = MonitoringBinding(
    name="Lidar",
    path="/scan",
    message_type="fixture_msgs/msg/Pose",
    required=True,
    startup_timeout=DEADLINE,
)
CAMERA = MonitoringBinding(
    name="Camera",
    path="/image",
    message_type="fixture_msgs/msg/Pose",
    required=True,
    startup_timeout=DEADLINE * 2,
)
ODOM = MonitoringBinding(name="Odom", path="/odom", message_type="fixture_msgs/msg/Pose")
SET_MODE = MethodBinding(name="SetMode", path="/set_mode", service_type="fixture_msgs/srv/SetMode")


class Platform(TestComponent):
    """One required topic, one optional one, and a Method the contract has to refuse."""

    bindings = (LIDAR, ODOM, SET_MODE)


class Quiet(Platform):
    """Nothing required: the Component that has no startup contract at all."""

    bindings = (ODOM,)


class Pair(Platform):
    """Two required topics with different deadlines, which is what fills an alarm list in order."""

    bindings = (LIDAR, CAMERA)


@pytest.fixture
def runtime() -> FakeRuntime:
    return FakeRuntime(
        domain_id=7,
        message_types={**MESSAGE_TYPES, **fixture_lifecycle.MESSAGE_TYPES},
        service_types={**SERVICE_TYPES, **fixture_lifecycle.SERVICE_TYPES},
        action_types=ACTION_TYPES,
        graph_poll_interval=0.05,
    )


async def start(
    runtime: FakeRuntime,
    platform: TestComponent | None = None,
    ros2_config: ROS2Config | None = None,
) -> tuple[Server, Any]:
    """Bring up a server carrying one Machine with one Component, and hand back both."""
    config = ros2_config or ROS2Config(startup_timeout=DEADLINE)
    machine = TestMachine(platform or Platform(name="Platform"), runtime=runtime, ros2_config=config)
    return await serve(machine), machine


@pytest.fixture
async def served(runtime: FakeRuntime) -> AsyncIterator[tuple[Server, Any]]:
    server, machine = await start(runtime)
    try:
        yield server, machine
    finally:
        await server.shutdown()


@asynccontextmanager
async def running(
    runtime: FakeRuntime,
    platform: TestComponent,
    ros2_config: ROS2Config | None = None,
) -> AsyncIterator[tuple[Server, Any]]:
    """Serve one Component for the length of a test that declares its own."""
    server, machine = await start(runtime, platform, ros2_config)
    try:
        yield server, machine
    finally:
        await server.shutdown()


async def until(answered: Callable[[], Any], complaint: str = "it never happened", within: float = SETTLED) -> None:
    """Poll until something opensmi is doing on a task of its own has happened."""
    deadline = asyncio.get_running_loop().time() + within
    while not answered():
        assert asyncio.get_running_loop().time() < deadline, complaint
        await asyncio.sleep(0.005)


async def reached(skill: Any, state: SkillState) -> None:
    """Wait until a Skill reaches a state, failing the test rather than hanging if it does not."""
    await until(lambda: skill.current_state is state, f"the Skill stayed {skill.current_state}, never {state}")


def platform_of(machine: Any) -> Any:
    return machine.added[0]


def startup_of(item: Any) -> Any:
    """Return one machinery item's StartupSkill, the way opensmi finds it."""
    return item.skill_set[BaseStartupSkill.NAME]


class Collector:
    """An asyncua subscription handler that keeps what it was sent."""

    def __init__(self) -> None:
        self.events: list[Any] = []

    def event_notification(self, event: Any) -> None:
        self.events.append(event)

    def codes(self) -> list[str]:
        return [event.Code for event in self.events]

    def error_codes(self) -> list[str]:
        """Return the codes on opensmi's own fault events, which spell the field differently."""
        return [event.ErrorCode for event in self.events]


async def subscribe_to_events(client: Client, server: Server, item: Any, collector: Collector) -> None:
    """Subscribe to the BindingEvents one machinery item fires, on the node it fires them from."""
    subscription = await client.create_subscription(20, collector)
    await subscription.subscribe_events(
        client.get_node(item.notification.ua_node.nodeid),
        client.get_node(server.ua_get_node_id(BINDING_EVENT_TYPE)),
    )


async def subscribe_to_faults(client: Client, server: Server, item: Any, collector: Collector) -> None:
    """Subscribe to opensmi's own fault events, which is where a startup verdict lands."""
    subscription = await client.create_subscription(20, collector)
    await subscription.subscribe_events(
        client.get_node(item.notification.ua_node.nodeid),
        client.get_node(server.ua_get_node_id(SmartFactoryMachineSetNodeIds.AlertType)),
    )


async def started(skill: Any) -> None:
    """Take one StartupSkill from wherever it is to a start, the way an operator does."""
    if skill.current_state is not SkillState.READY:
        await skill.reset()
        await reached(skill, SkillState.READY)
    await skill.start()


# -- nothing waits before the endpoint opens ---------------------------------------------------


async def test_the_server_comes_up_against_a_robot_that_is_not_there(served) -> None:
    # init creates nodes and returns without waiting.
    _, machine = served
    platform = platform_of(machine)

    assert "Lidar.Position.X" in monitoring_nodes(platform)
    assert platform.ros2_monitoring_presence == {"Lidar": False, "Odom": False}
    assert startup_of(platform).current_state is SkillState.HALTED


async def test_a_component_with_a_required_binding_gains_a_startup_skill(served) -> None:
    _, machine = served
    skill = startup_of(platform_of(machine))

    assert isinstance(skill, ROS2StartupSkill)
    assert not isinstance(skill, ROS2LifecycleStartupSkill)
    assert not skill.is_finite
    assert await skill.ua_state_machine.get_child("CurrentState")


async def test_a_component_with_nothing_required_gains_no_startup_skill(runtime) -> None:
    async with running(runtime, Quiet(name="Platform")) as (_, machine):
        assert BaseStartupSkill.NAME not in platform_of(machine).skill_set


async def test_a_machine_carries_a_startup_skill_of_its_own(served) -> None:
    _, machine = served
    skill = startup_of(machine)

    assert isinstance(skill, ROS2StartupSkill)
    assert skill.current_state is SkillState.HALTED
    assert machine.current_state is MachineryItemState.OUT_OF_SERVICE


async def test_a_machine_that_registers_its_own_startup_skill_keeps_it(runtime) -> None:
    class Own(BaseStartupSkill):
        async def _handle_running(self) -> None:
            pass

    class Robot(TestMachine):
        async def _init(self) -> None:
            await super()._init()
            await self.add_skill(Own(handle_status_change=False))

    machine = Robot(Platform(name="Platform"), runtime=runtime)
    server = await serve(machine)
    try:
        assert type(startup_of(machine)) is Own
    finally:
        await server.shutdown()


# -- the access level is the one the ROS2 settings configure -------------------------------------


async def test_the_startup_skills_read_the_configured_skill_level(runtime) -> None:
    raised = ROS2Config(startup_timeout=DEADLINE, skill_access_level=20)
    async with running(runtime, Platform(name="Platform"), raised) as (_, machine):
        assert startup_of(platform_of(machine)).minimum_access_level == 20
        assert startup_of(machine).minimum_access_level == 20


async def test_the_startup_skills_default_to_the_orchestrator_level(served) -> None:
    _, machine = served

    assert startup_of(platform_of(machine)).minimum_access_level == 10
    assert startup_of(machine).minimum_access_level == 10


async def press(client: Client, server: Server, skill: Any, verb: str) -> ua.StatusCode:
    browse_name = f"{server.ua_get_namespace_index(SmartFactoryMachineSetNodeIds)}:{verb}"
    try:
        await client.get_node(skill.ua_state_machine.nodeid).call_method(browse_name)
    except ua.UaStatusCodeError as answer:
        return ua.StatusCode(ua.UInt32(answer.code))
    return GOOD


async def test_an_orchestrator_may_reset_the_startup_skill_and_a_raised_level_refuses_it(runtime) -> None:
    async def reset_as(server: Server, machine: Any, user: str) -> ua.StatusCode:
        skill = startup_of(platform_of(machine))
        async with connected(server, user) as client:
            lock = client.get_node(machine.lock.ua_node.nodeid)
            await lock.call_method(f"{server.ua_get_namespace_index(DiNodeIds)}:InitLock", "the test suite")
            try:
                return await press(client, server, skill, "Reset")
            finally:
                await lock.call_method(f"{server.ua_get_namespace_index(DiNodeIds)}:ExitLock")

    async with running(runtime, Platform(name="Platform")) as (server, machine):
        assert await reset_as(server, machine, "orchestrator") == GOOD

    raised = ROS2Config(startup_timeout=DEADLINE, skill_access_level=20)
    async with running(runtime, Platform(name="Platform"), raised) as (server, machine):
        assert await reset_as(server, machine, "orchestrator") == DENIED
        assert await reset_as(server, machine, "operator") == GOOD


# -- the wait is observable, and it ends one of two ways ---------------------------------------


async def test_a_required_binding_that_is_there_is_started_through_quietly(served, runtime) -> None:
    server, machine = served
    runtime.note_publishers("/scan", 1)
    platform = platform_of(machine)
    skill = startup_of(platform)

    collector = Collector()
    async with connected(server) as client:
        await subscribe_to_events(client, server, platform, collector)
        await started(skill)
        await reached(skill, SkillState.RUNNING)
        await asyncio.sleep(0.2)

    assert collector.codes() == []


async def test_the_skill_sits_in_starting_while_it_waits_and_then_runs(served, runtime) -> None:
    # The wait lives in the starting handler, which opensmi runs as a task, so CurrentState shows Starting then Running.
    _, machine = served
    skill = startup_of(platform_of(machine))

    await started(skill)
    await reached(skill, SkillState.STARTING)
    runtime.note_publishers("/scan", 1)

    await reached(skill, SkillState.RUNNING)


async def test_a_running_startup_skill_stays_running(served, runtime) -> None:
    _, machine = served
    runtime.note_publishers("/scan", 1)
    skill = startup_of(platform_of(machine))
    await started(skill)
    await reached(skill, SkillState.RUNNING)

    await asyncio.sleep(0.3)

    assert skill.current_state is SkillState.RUNNING


async def test_a_required_binding_that_never_arrives_halts_the_startup(served) -> None:
    _, machine = served
    skill = startup_of(platform_of(machine))

    await started(skill)
    await reached(skill, SkillState.STARTING)

    await reached(skill, SkillState.HALTED)


async def test_each_missing_binding_fires_at_its_own_deadline_and_the_verdict_is_the_last(runtime) -> None:
    async with running(runtime, Pair(name="Platform")) as (server, machine):
        platform = platform_of(machine)
        skill = startup_of(platform)
        events = Collector()
        faults = Collector()
        async with connected(server) as client:
            await subscribe_to_events(client, server, platform, events)
            await subscribe_to_faults(client, server, platform, faults)
            await started(skill)
            await until(lambda: events.codes() == [REQUIRED_BINDING_ABSENT], "the first deadline said nothing")

            # The camera's deadline is twice the lidar's, so the Skill still waits for it after the lidar is reported.
            assert skill.current_state is SkillState.STARTING
            await reached(skill, SkillState.HALTED)
            await until(lambda: REQUIRED_BINDING_ABSENT in faults.error_codes(), "no verdict reached the client")
            # Wait inside the subscription: events in flight after the client is gone are lost.
            await until(lambda: len(events.events) == 2, "the camera's deadline said nothing")

        assert [event.Binding for event in events.events] == ["Lidar", "Camera"]
        assert [event.ROS2Path for event in events.events] == ["/scan", "/image"]
        verdict = next(event for event in faults.events if event.ErrorCode == REQUIRED_BINDING_ABSENT)
        assert "Lidar (/scan)" in verdict.Message.Text
        assert "Camera (/image)" in verdict.Message.Text


async def test_a_binding_that_arrives_after_its_own_deadline_is_still_a_failed_startup(runtime) -> None:
    async with running(runtime, Pair(name="Platform")) as (_, machine):
        skill = startup_of(platform_of(machine))
        await started(skill)
        await asyncio.sleep(DEADLINE * 1.5)
        runtime.note_publishers("/scan", 1)
        runtime.note_publishers("/image", 1)

        await reached(skill, SkillState.HALTED)


# -- a startup that did not happen refuses everything on that Component --------------------------


async def occupy(client: Client, server: Server, machine: Any) -> None:
    await alone(server, client)
    lock = client.get_node(machine.lock.ua_node.nodeid)
    await lock.call_method(f"{server.ua_get_namespace_index(DiNodeIds)}:InitLock", "the test suite")


async def release(client: Client, server: Server, machine: Any) -> None:
    lock = client.get_node(machine.lock.ua_node.nodeid)
    await lock.call_method(f"{server.ua_get_namespace_index(DiNodeIds)}:ExitLock")


async def called(server: Server, machine: Any, name: str = "SetMode") -> ua.StatusCode:
    """Call a Method the way a client does, and return the status code it answered with."""
    method = platform_of(machine).method_set[name]
    browse_name = f"{server.ua_get_namespace_index(SmartFactorySkillSetNodeIds)}:Call"
    async with connected(server) as client:
        await occupy(client, server, machine)
        try:
            await client.get_node(method.ua_node.nodeid).call_method(browse_name)
        except ua.UaStatusCodeError as answer:
            return ua.StatusCode(ua.UInt32(answer.code))
        finally:
            await release(client, server, machine)
    return GOOD


async def test_a_halted_startup_refuses_this_components_methods(served) -> None:
    server, machine = served
    skill = startup_of(platform_of(machine))
    await started(skill)
    await reached(skill, SkillState.HALTED)

    assert await called(server, machine) == INVALID_STATE


async def test_a_running_startup_lets_this_components_methods_through(served, runtime) -> None:
    server, machine = served
    answer = fixture_messages.SetModeResponse(success=True, message="", applied=1)
    runtime.serve_service("/set_mode", lambda _request: answer)
    runtime.note_publishers("/scan", 1)
    skill = startup_of(platform_of(machine))
    await started(skill)
    await reached(skill, SkillState.RUNNING)

    assert await called(server, machine) == GOOD


async def test_the_introspection_listings_answer_before_startup_completes(served) -> None:
    # Read-only listings do not wait for a hanging startup.
    server, machine = served
    introspection = next(each for each in machine.components if isinstance(each, ROS2IntrospectionComponent))
    assert startup_of(machine).current_state is SkillState.HALTED
    assert machine.current_state is MachineryItemState.OUT_OF_SERVICE

    async with connected(server, "visitor") as client:
        for name in ("ListTopics", "ListBindings", "ListServices"):
            method = introspection.method_set[name]
            call = f"{server.ua_get_namespace_index(SmartFactorySkillSetNodeIds)}:Call"
            await client.get_node(method.ua_node.nodeid).call_method(call)

    assert await called(server, machine) == INVALID_STATE


# -- the Machine follows its Components ----------------------------------------------------------


async def test_the_machines_startup_starts_each_components_and_waits_for_it(served, runtime) -> None:
    # opensmi's handler only resets the children and raises at the first one that halted.
    _, machine = served
    runtime.note_publishers("/scan", 1)

    await started(startup_of(machine))

    await reached(startup_of(platform_of(machine)), SkillState.RUNNING)
    await reached(startup_of(machine), SkillState.RUNNING)
    await until(
        lambda: machine.current_state is MachineryItemState.NOT_EXECUTING,
        "the Machine never left OutOfService",
    )


async def test_a_component_that_did_not_come_up_halts_the_machine_and_names_itself(served) -> None:
    server, machine = served

    faults = Collector()
    async with connected(server) as client:
        await subscribe_to_faults(client, server, machine, faults)
        await started(startup_of(machine))
        await reached(startup_of(machine), SkillState.HALTED)
        await until(lambda: COMPONENT_NOT_RUNNING in faults.error_codes(), "nothing said which Component")

    verdict = next(event for event in faults.events if event.ErrorCode == COMPONENT_NOT_RUNNING)
    assert "Platform" in verdict.Message.Text
    assert machine.current_state is MachineryItemState.OUT_OF_SERVICE


async def test_a_machine_whose_components_declare_nothing_required_starts_at_once(runtime) -> None:
    async with running(runtime, Quiet(name="Platform")) as (_, machine):
        await started(startup_of(machine))

        await reached(startup_of(machine), SkillState.RUNNING)
        await until(
            lambda: machine.current_state is MachineryItemState.NOT_EXECUTING,
            "the Machine never left OutOfService",
        )


# -- after startup: reported, never halted, and rechecked on every start ------------------------


async def test_a_binding_lost_after_startup_is_reported_and_never_halts(served, runtime) -> None:
    server, machine = served
    runtime.note_publishers("/scan", 1)
    platform = platform_of(machine)
    skill = startup_of(platform)
    await started(skill)
    await reached(skill, SkillState.RUNNING)

    collector = Collector()
    async with connected(server) as client:
        await subscribe_to_events(client, server, platform, collector)
        runtime.note_publishers("/scan", 0)
        await until(lambda: REQUIRED_BINDING_LOST in collector.codes(), "the loss was never reported")

    assert skill.current_state is SkillState.RUNNING
    assert next(event for event in collector.events if event.Code == REQUIRED_BINDING_LOST).ROS2Path == "/scan"


async def test_a_loss_is_reported_once_and_coming_back_is_not_reported_at_all(served, runtime) -> None:
    server, machine = served
    runtime.note_publishers("/scan", 1)
    platform = platform_of(machine)
    skill = startup_of(platform)
    await started(skill)
    await reached(skill, SkillState.RUNNING)

    collector = Collector()
    async with connected(server) as client:
        await subscribe_to_events(client, server, platform, collector)
        runtime.note_publishers("/scan", 0)
        await until(lambda: REQUIRED_BINDING_LOST in collector.codes(), "the loss was never reported")
        runtime.note_publishers("/scan", 1)
        await asyncio.sleep(2.5)

    assert collector.codes() == [REQUIRED_BINDING_LOST]


async def test_the_check_re_runs_on_every_start(served, runtime) -> None:
    # A start is a contract: a later start must re-check the prerequisite.
    _, machine = served
    runtime.note_publishers("/scan", 1)
    skill = startup_of(platform_of(machine))
    await started(skill)
    await reached(skill, SkillState.RUNNING)

    await skill.halt()
    await reached(skill, SkillState.HALTED)
    runtime.note_publishers("/scan", 0)

    await started(skill)

    await reached(skill, SkillState.HALTED)


async def test_recovery_is_the_operators_reset_and_start(served, runtime) -> None:
    _, machine = served
    skill = startup_of(platform_of(machine))
    await started(skill)
    await reached(skill, SkillState.HALTED)

    runtime.note_publishers("/scan", 1)
    await asyncio.sleep(DEADLINE)
    assert skill.current_state is SkillState.HALTED

    await started(skill)

    await reached(skill, SkillState.RUNNING)


# -- what Satisfied means, kind by kind ----------------------------------------------------------


async def test_a_required_service_and_action_are_waited_for_on_the_graph(runtime) -> None:
    class Discovered(Platform):
        bindings = (
            MethodBinding(
                name="SetMode",
                path="/set_mode",
                service_type="fixture_msgs/srv/SetMode",
                required=True,
                startup_timeout=DEADLINE * 8,
            ),
            ActionBinding(
                name="Navigate",
                path="/navigate",
                action_type="fixture_msgs/action/Navigate",
                required=True,
                startup_timeout=DEADLINE * 8,
            ),
        )

    async with running(runtime, Discovered(name="Platform")) as (_, machine):
        skill = startup_of(platform_of(machine))
        await started(skill)
        await reached(skill, SkillState.STARTING)
        runtime.set_graph(GraphSnapshot(services={"/set_mode": ("fixture_msgs/srv/SetMode",)}))
        await asyncio.sleep(0.2)
        assert skill.current_state is SkillState.STARTING

        runtime.set_graph(
            GraphSnapshot(
                services={"/set_mode": ("fixture_msgs/srv/SetMode",)},
                actions={"/navigate": ("fixture_msgs/action/Navigate",)},
            )
        )

        await reached(skill, SkillState.RUNNING)
        assert machine.runtime.graph.polling
        await skill.halt()
        await reached(skill, SkillState.HALTED)
        await until(lambda: not machine.runtime.graph.polling, "the graph is still being read for nobody")


async def test_a_required_ros2_parameter_binding_waits_for_every_declared_name(runtime) -> None:
    tuned = ROS2ParameterBinding(
        name="Controller",
        path="/controller_server",
        parameters={"max_vel_x": ParameterType.DOUBLE},
        retry_interval=0.05,
        required=True,
        startup_timeout=DEADLINE * 3,
    )

    class Tuned(Platform):
        bindings = (tuned,)

    async with running(runtime, Tuned(name="Platform")) as (_, machine):
        skill = startup_of(platform_of(machine))
        await started(skill)
        await reached(skill, SkillState.STARTING)
        runtime.serve_parameters(
            "/controller_server",
            {"max_vel_x": ParameterValue(type=ParameterType.DOUBLE, value=0.26)},
        )

        await reached(skill, SkillState.RUNNING)


async def test_a_required_lifecycle_binding_waits_for_its_node_to_be_there(runtime) -> None:
    # Covers a node that answered when configured and went away before the Start.
    class Managed(Platform):
        bindings = (LifecycleBinding(name="Talker", path="/lc_talker", required=True, startup_timeout=DEADLINE * 8),)

    async with running(runtime, Managed(name="Platform")) as (_, machine):
        skill = startup_of(platform_of(machine))
        assert isinstance(skill, ROS2LifecycleStartupSkill)
        states = fixture_lifecycle.serve_nodes(runtime, {"/lc_talker": int(LifecycleNodeState.UNCONFIGURED)})
        await skill.reset()
        await reached(skill, SkillState.READY)

        runtime.note_publishers("/lc_talker/transition_event", 0)
        await until(lambda: skill.node_presence == {"/lc_talker": False}, "the node still looks present")
        await skill.start()
        await reached(skill, SkillState.STARTING)
        assert states["/lc_talker"] == int(LifecycleNodeState.INACTIVE)

        runtime.note_publishers("/lc_talker/transition_event", 1)

        await reached(skill, SkillState.RUNNING)
        assert states["/lc_talker"] == int(LifecycleNodeState.ACTIVE)


# -- declaring it wrong is fatal at init ---------------------------------------------------------


async def test_a_component_that_registered_its_own_startup_skill_is_refused(runtime) -> None:
    class Own(BaseStartupSkill):
        async def _handle_running(self) -> None:
            pass

    class Twice(Platform):
        async def _init(self) -> None:
            await self.add_skill(Own(handle_status_change=False))
            await super()._init()

    with pytest.raises(DeclarationError, match="a required Binding is what the StartupSkill waits for"):
        await start(runtime, Twice(name="Platform"))


async def test_a_startup_skill_class_that_is_not_ours_is_refused(runtime) -> None:
    class Foreign(BaseStartupSkill):
        async def _handle_running(self) -> None:
            pass

    class Named(Platform):
        startup_skill_type = Foreign  # type: ignore[assignment]

    with pytest.raises(ImplementationClassError, match="not a subclass of ROS2StartupSkill"):
        await start(runtime, Named(name="Platform"))


async def test_a_startup_skill_class_that_is_a_subclass_of_ours_is_used(runtime) -> None:
    class Narrower(ROS2StartupSkill):
        pass

    class Named(Platform):
        startup_skill_type = Narrower

    async with running(runtime, Named(name="Platform")) as (_, machine):
        assert type(startup_of(platform_of(machine))) is Narrower


async def test_shutting_the_server_down_stops_the_startup_skills_tasks(runtime) -> None:
    server, machine = await start(runtime)
    skills = [startup_of(platform_of(machine)), startup_of(machine)]
    await server.shutdown()

    assert all(skill._handle_status_change_task is None for skill in skills)
