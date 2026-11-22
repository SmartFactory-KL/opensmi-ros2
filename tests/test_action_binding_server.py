# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A declared action is a finite Skill a UA client can start, watch, halt and read.

Runs against a real opensmi server and the fake Runtime. Start, Halt and Reset go through a real
client holding the Machine's Lock. The Skill starts Halted, so tests reset it first.
"""

from __future__ import annotations

import asyncio
import inspect
from contextlib import asynccontextmanager
from typing import TYPE_CHECKING, Any, cast

import pytest
from asyncua import ua
from asyncua.ua.object_ids import ObjectIds
from asyncua.ua.status_codes import StatusCodes
from opensmi.core import SkillState
from opensmi.core.errors import SkillHaltedError
from opensmi.core.nodesets import DiNodeIds, SmartFactoryMachineSetNodeIds
from opensmi.server.base_startup_skill import BaseStartupSkill

from opensmi.ros2.runtime.errors import (
    ActionServerUnavailableError,
    FieldTypeMismatchError,
    MissingInterfacePackageError,
)
from opensmi.ros2.runtime.outcomes import GoalStatus
from opensmi.ros2.runtime.selection import FieldSelection, resolve_selection
from opensmi.ros2.server.access import SKILL_ACCESS_LEVEL
from opensmi.ros2.server.bindings import ActionBinding
from opensmi.ros2.server.config import ROS2Config
from opensmi.ros2.server.errors import ImplementationClassError, NotInvertibleError, UnprojectableFieldError
from opensmi.ros2.server.projection import PlanObject, PlanVariable, ProjectionContext, join_path
from opensmi.ros2.server.recipes import Functions
from opensmi.ros2.server.skill import GOAL_UNBUILDABLE, ROS2ActionSkill
from tests import fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.fixture_structures import node_id as structure_node_id
from tests.served import TestComponent, TestMachine, alone, connected, elements_of, leaf, leaves, serve, wire_value

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Callable, Mapping

    from asyncua.client.client import Client
    from asyncua.common.node import Node
    from opensmi.server import Server

ACTION_TYPES = {
    "fixture_msgs/action/Navigate": fixture_messages.Navigate,
    "fixture_msgs/action/Dock": fixture_messages.Dock,
    "fixture_msgs/action/Plan": fixture_messages.Plan,
}

NAVIGATE = ActionBinding(
    name="Navigate",
    path="/navigate",
    action_type="fixture_msgs/action/Navigate",
)

#: An action declared with only name, path and type; its goal is one repeated message field.
PLAN = ActionBinding(
    name="Plan",
    path="/plan",
    action_type="fixture_msgs/action/Plan",
)

GOOD = ua.StatusCode(ua.UInt32(StatusCodes.Good))

PLANNING = fixture_messages.PlanFeedback(remaining=0)

# opensmi adds a variable of its own to every finite Skill's FinalResultData; it is not asserted on.
FRAMEWORKS_OWN = frozenset({"SuccessfulExecutionsCount"})

# Generous enough not to flake on a loaded runner, short enough to fail on a hang.
SETTLED = 5.0


def status(code: int) -> ua.StatusCode:
    return ua.StatusCode(ua.UInt32(code))


class Platform(TestComponent):
    bindings = (NAVIGATE,)


@pytest.fixture
def runtime() -> FakeRuntime:
    started = FakeRuntime(domain_id=7, action_types=ACTION_TYPES)
    started.serve_action("/navigate")
    return started


async def start(
    runtime: FakeRuntime,
    platform: TestComponent | None = None,
    ros2_config: ROS2Config | None = None,
) -> tuple[Server, Any]:
    """Bring up a server carrying one Machine with one Component, and hand back both."""
    machine = TestMachine(platform or Platform(name="Platform"), runtime=runtime, ros2_config=ros2_config)
    return await serve(machine), machine


@pytest.fixture
async def served(runtime: FakeRuntime) -> AsyncIterator[tuple[Server, Any]]:
    server, machine = await start(runtime)
    try:
        yield server, machine
    finally:
        await server.shutdown()


async def running_with(
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


def skill_of(machine: Any, name: str = "Navigate") -> Any:
    return machine.added[0].skill_set[name]


def names(container: Any) -> list[str]:
    return sorted(name for name in leaves(container) if name not in FRAMEWORKS_OWN)


async def results_of(skill: Any) -> dict[str, object]:
    """Return what the FinalResultData holds now, without opensmi's own variable."""
    values = {name: value for name, value in (await skill.read_results()).items() if name not in FRAMEWORKS_OWN}
    for name in ("succeeded", "aborted", "canceled"):
        if name in skill.final_result_data:
            values[name] = await skill.final_result_data[name].read()
    return values


async def until(answered: Callable[[], Any]) -> None:
    """Poll until something opensmi is doing on a task of its own has happened.

    A poll rather than an event, and deliberately so: what is being waited on is opensmi's state
    machine and the tasks it runs, which publish no event to await. The timeout is what turns a
    hang into a failure.
    """
    async with asyncio.timeout(SETTLED):
        while True:
            answer = answered()
            if inspect.isawaitable(answer):
                answer = await answer
            if answer:
                return
            await asyncio.sleep(0.01)


async def reads(node: Node, expected: object) -> bool:
    """Return whether a node already carries a value, for `until` to poll."""
    return await node.read_value() == expected


async def reached(skill: Any, state: SkillState) -> None:
    """Wait until a Skill reaches a state, failing the test rather than hanging if it does not."""
    await until(lambda: skill.current_state is state)


# -- a client, the Lock, and the three Methods of the state machine -----------------------------


async def occupy(client: Client, server: Server, machine: Any) -> None:
    """Take the Machine's Lock, which a Skill needs as well as an access level."""
    await alone(server, client)
    lock = client.get_node(machine.lock.ua_node.nodeid)
    await lock.call_method(f"{server.ua_get_namespace_index(DiNodeIds)}:InitLock", "the test suite")


async def release(client: Client, server: Server, machine: Any) -> None:
    """Give the Lock back, which closing the session does not do: opensmi keeps it until a timeout."""
    lock = client.get_node(machine.lock.ua_node.nodeid)
    await lock.call_method(f"{server.ua_get_namespace_index(DiNodeIds)}:ExitLock")


@asynccontextmanager
async def operating(server: Server, machine: Any, user: str = "operator") -> AsyncIterator[Client]:
    """Connect a client that holds the Machine's Lock, and give the Lock back before closing."""
    async with connected(server, user) as client:
        await occupy(client, server, machine)
        try:
            yield client
        finally:
            await release(client, server, machine)


async def press(client: Client, server: Server, skill: Any, verb: str) -> ua.StatusCode:
    """Call one Method of a Skill's state machine, and return the status code it answered.

    A bad code reaches a client as a raised ``UaStatusCodeError``; it is returned here as the
    code it carries, so that every test below reads as "the client got this status".
    """
    browse_name = f"{server.ua_get_namespace_index(SmartFactoryMachineSetNodeIds)}:{verb}"
    try:
        await client.get_node(skill.ua_state_machine.nodeid).call_method(browse_name)
    except ua.UaStatusCodeError as answer:
        return status(answer.code)
    return GOOD


async def ready(client: Client, server: Server, skill: Any) -> None:
    """Take a Skill from Halted or Completed to Ready, which is what a client's Reset does."""
    assert await press(client, server, skill, "Reset") == GOOD
    await reached(skill, SkillState.READY)


async def running(
    client: Client,
    server: Server,
    skill: Any,
    runtime: FakeRuntime,
    action: str = "/navigate",
    feedback: Any = None,
) -> None:
    """Press Start and wait until the goal is in flight, which is what Running means."""
    first = feedback or fixture_messages.NavigateFeedback(distance_remaining=0.0)
    assert await press(client, server, skill, "Start") == GOOD
    await reached(skill, SkillState.RUNNING)
    await until(lambda: runtime.deliver_feedback(action, first))


async def write(client: Client, variable: Any, value: object) -> None:
    """Write one variable the way a client does, with the variant type the server declared."""
    declared = await variable.ua_node.read_data_value()
    stored = wire_value(variable, value)
    await client.get_node(variable.ua_node.nodeid).write_value(ua.Variant(stored, declared.Value.VariantType))


async def write_parameters(client: Client, skill: Any, values: Mapping[str, object]) -> None:
    for name, value in values.items():
        await write(client, leaf(skill.parameter_set, name), value)


class Collector:
    """An asyncua subscription handler that keeps the events it was sent."""

    def __init__(self) -> None:
        self.events: list[Any] = []

    def event_notification(self, event: Any) -> None:
        self.events.append(event)

    async def next_event(self) -> Any:
        await until(lambda: self.events)
        return self.events[0]


async def alerts(client: Client, server: Server, item: Any) -> Collector:
    """Subscribe to the runtime errors a machinery item's Notification raises."""
    collector = Collector()
    subscription = await client.create_subscription(20, collector)
    await subscription.subscribe_events(
        client.get_node(item.notification.ua_node.nodeid),
        client.get_node(server.ua_get_node_id(SmartFactoryMachineSetNodeIds.AlertType)),
    )
    return collector


# -- what a client sees before anything runs -------------------------------------------------


async def test_an_action_is_a_finite_skill_on_the_declaring_component(served) -> None:
    _, machine = served
    skill = skill_of(machine)

    assert isinstance(skill, ROS2ActionSkill)
    assert skill.is_finite


async def test_a_client_browses_the_skill_under_the_components_skill_set(served) -> None:
    server, machine = served
    skill_set = machine.added[0].skill_set.ua_node

    async with connected(server, "visitor") as client:
        children = await client.get_node(skill_set.nodeid).get_children()
        browsed = {(await child.read_browse_name()).Name for child in children}

    assert "Navigate" in browsed


async def test_the_goal_becomes_the_skills_parameters(served) -> None:
    _, machine = served

    assert names(skill_of(machine).parameter_set) == ["Label", "Target.X", "Target.Y", "Target.Z"]


async def test_the_feedback_becomes_the_skills_monitoring(served) -> None:
    _, machine = served

    assert names(skill_of(machine).monitoring) == ["DistanceRemaining"]


async def test_a_feedback_field_called_status_is_free_in_a_skills_monitoring() -> None:
    # Machinery reserves Status only at the top of a Machine's or Component's Monitoring.
    runtime = FakeRuntime(domain_id=7, action_types={"fixture_msgs/action/Run": fixture_messages.Run})
    runtime.serve_action("/run")

    class Running(TestComponent):
        bindings = (ActionBinding(name="Run", path="/run", action_type="fixture_msgs/action/Run"),)

    server, machine = await start(runtime, Running(name="Platform"))
    try:
        assert names(skill_of(machine, "Run").monitoring) == ["Status"]
    finally:
        await server.shutdown()


async def test_the_result_becomes_the_skills_final_result_data(served) -> None:
    _, machine = served

    assert names(skill_of(machine).final_result_data) == ["TotalDistance", "Waypoints"]


async def test_a_variable_is_named_for_the_field_path_and_carries_no_binding_prefix(served) -> None:
    server, machine = served
    node = leaf(skill_of(machine).parameter_set, "Target.X").ua_node

    async with connected(server, "visitor") as client:
        browse_name = await client.get_node(node.nodeid).read_browse_name()

    assert browse_name.Name == "X"


async def test_a_parameter_carries_the_datatype_and_value_rank_of_its_field(served) -> None:
    server, machine = served
    node = leaf(skill_of(machine).parameter_set, "Target.X").ua_node

    async with connected(server, "visitor") as client:
        seen = client.get_node(node.nodeid)
        datatype = await seen.read_data_type()
        value_rank = await seen.read_value_rank()

    assert datatype == ua.NodeId(ua.Int32(ObjectIds.Double))
    assert value_rank == ua.ValueRank.Scalar


async def test_a_parameter_is_writable_and_a_result_is_not(served) -> None:
    server, machine = served
    skill = skill_of(machine)

    async with connected(server, "visitor") as client:
        parameter = await client.get_node(leaf(skill.parameter_set, "Target.X").ua_node.nodeid).get_access_level()
        result = await client.get_node(leaf(skill.final_result_data, "Waypoints").ua_node.nodeid).get_access_level()

    assert ua.AccessLevel.CurrentWrite in parameter
    assert ua.AccessLevel.CurrentWrite not in result


# -- the gates, which are opensmi's ---------------------------------------------------------------


async def test_a_skill_asks_for_the_skill_level(served) -> None:
    _, machine = served

    assert skill_of(machine).minimum_access_level == SKILL_ACCESS_LEVEL == 10


async def test_a_reset_without_the_lock_answers_bad_requires_lock(served) -> None:
    server, machine = served
    skill = skill_of(machine)

    async with connected(server) as client:
        assert await press(client, server, skill, "Reset") == status(StatusCodes.BadRequiresLock)

    assert skill.current_state is SkillState.HALTED


async def test_a_start_without_the_lock_answers_bad_requires_lock(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine)
    async with operating(server, machine) as client:
        await ready(client, server, skill)

    async with connected(server) as client:
        assert await press(client, server, skill, "Start") == status(StatusCodes.BadRequiresLock)

    assert skill.current_state is SkillState.READY
    assert runtime.deliver_feedback("/navigate", fixture_messages.NavigateFeedback(distance_remaining=0.0)) == 0


async def test_a_user_below_the_skill_level_is_refused_even_holding_the_lock(served) -> None:
    server, machine = served
    skill = skill_of(machine)

    async with operating(server, machine, "planner") as client:
        assert await press(client, server, skill, "Reset") == status(StatusCodes.BadUserAccessDenied)

    assert skill.current_state is SkillState.HALTED


async def test_a_user_at_the_skill_level_holding_the_lock_starts_the_skill(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine)

    async with operating(server, machine, "orchestrator") as client:
        await ready(client, server, skill)
        await running(client, server, skill, runtime)


async def test_the_skill_level_is_the_one_the_ros2_settings_configure(runtime) -> None:
    raised = ROS2Config(skill_access_level=20)
    async for server, machine in running_with(runtime, Platform(name="Platform"), raised):
        skill = skill_of(machine)
        assert skill.minimum_access_level == 20

        async with operating(server, machine, "orchestrator") as client:
            assert await press(client, server, skill, "Reset") == status(StatusCodes.BadUserAccessDenied)
        async with operating(server, machine, "operator") as client:
            assert await press(client, server, skill, "Reset") == GOOD


class NeverStarted(BaseStartupSkill):
    """A StartupSkill nobody starts, so the Component it is on never completes its startup."""

    async def _handle_running(self) -> None:
        pass


class Starting(Platform):
    """A Component whose StartupSkill exists and has not been started."""

    async def _init(self) -> None:
        await super()._init()
        await self.add_skill(NeverStarted(handle_status_change=False))


async def test_a_start_before_startup_completed_is_refused_by_opensmi(runtime) -> None:
    # a Machine whose startup did not complete refuses to start its Skills.
    async for server, machine in running_with(runtime, Starting(name="Platform")):
        skill = skill_of(machine)
        async with operating(server, machine) as client:
            await ready(client, server, skill)
            assert await press(client, server, skill, "Start") == status(StatusCodes.BadInvalidState)

        assert skill.current_state is SkillState.READY


# -- the state machine the outcome drives ----------------------------------------------------


async def test_start_sends_the_goal_a_client_wrote_and_reaches_running(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine)

    async with operating(server, machine) as client:
        await ready(client, server, skill)
        await write_parameters(client, skill, {"Target.X": 1.5, "Label": "kitchen"})
        await running(client, server, skill, runtime)

    assert skill.ros2_goal is not None
    assert runtime.sent_goals[-1] == (
        "/navigate",
        fixture_messages.NavigateGoal(target=fixture_messages.Point(x=1.5, y=0.0, z=0.0), label="kitchen"),
    )


async def test_feedback_reaches_monitoring_while_the_goal_runs(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine)

    async with operating(server, machine) as client:
        await ready(client, server, skill)
        await running(client, server, skill, runtime)

        runtime.deliver_feedback("/navigate", fixture_messages.NavigateFeedback(distance_remaining=4.25))

        node = client.get_node(leaf(skill.monitoring, "DistanceRemaining").ua_node.nodeid)
        await until(lambda: reads(node, 4.25))


async def test_a_succeeded_goal_completes_the_skill_and_its_fields_are_the_result_data(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine)

    async with operating(server, machine) as client:
        await ready(client, server, skill)
        await running(client, server, skill, runtime)

        runtime.settle_goal(
            "/navigate",
            status=GoalStatus.SUCCEEDED,
            result=fixture_messages.NavigateResult(total_distance=12.5, waypoints=3),
        )

        await reached(skill, SkillState.COMPLETED)
        waypoints = client.get_node(leaf(skill.final_result_data, "Waypoints").ua_node.nodeid)
        assert await waypoints.read_value() == 3

    assert await results_of(skill) == {"TotalDistance": 12.5, "Waypoints": 3}


async def test_an_aborted_goal_halts_the_skill_and_still_writes_what_it_produced(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine)

    async with operating(server, machine) as client:
        await ready(client, server, skill)
        await running(client, server, skill, runtime)

        runtime.settle_goal(
            "/navigate",
            status=GoalStatus.ABORTED,
            result=fixture_messages.NavigateResult(total_distance=4.0, waypoints=1),
        )

        await reached(skill, SkillState.HALTED)

    assert await results_of(skill) == {"TotalDistance": 4.0, "Waypoints": 1}


async def test_an_aborted_goal_arrives_as_a_runtime_error_in_the_notification(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine)

    async with operating(server, machine) as client:
        collector = await alerts(client, server, machine.added[0])
        await ready(client, server, skill)
        await running(client, server, skill, runtime)

        runtime.settle_goal("/navigate", status=GoalStatus.ABORTED, result=None)

        event = await collector.next_event()

    await reached(skill, SkillState.HALTED)
    assert event.ErrorCode == "ABORTED"


async def test_a_goal_cancelled_by_somebody_else_halts_the_skill(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine)

    async with operating(server, machine) as client:
        await ready(client, server, skill)
        await running(client, server, skill, runtime)

        runtime.settle_goal(
            "/navigate",
            status=GoalStatus.CANCELED,
            result=fixture_messages.NavigateResult(total_distance=1.0, waypoints=0),
        )

        await reached(skill, SkillState.HALTED)


async def test_a_rejected_goal_never_reaches_running(served, runtime) -> None:
    server, machine = served
    runtime.serve_action("/navigate", accepted=False)
    skill = skill_of(machine)
    seen: list[SkillState] = []

    async with operating(server, machine) as client:
        collector = await alerts(client, server, machine.added[0])
        await ready(client, server, skill)
        assert await press(client, server, skill, "Start") == GOOD
        await until(lambda: seen.append(skill.current_state) or skill.current_state is SkillState.HALTED)
        event = await collector.next_event()

    assert SkillState.RUNNING not in seen
    assert event.ErrorCode == "GoalRejected"
    assert skill.ros2_goal is None


async def test_an_action_nobody_serves_halts_rather_than_sitting_in_starting(served, runtime) -> None:
    server, machine = served
    runtime._servers.clear()
    skill = skill_of(machine)

    async with operating(server, machine) as client:
        await ready(client, server, skill)
        assert await press(client, server, skill, "Start") == GOOD

        await reached(skill, SkillState.HALTED)


async def test_halt_asks_the_server_to_cancel_the_goal(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine)

    async with operating(server, machine) as client:
        await ready(client, server, skill)
        await running(client, server, skill, runtime)

        assert await press(client, server, skill, "Halt") == GOOD

        await reached(skill, SkillState.HALTED)

    assert runtime.cancel_requests == ("/navigate",)
    assert skill.ros2_goal is None


async def test_reset_clears_what_the_last_run_produced_and_keeps_what_a_client_wrote(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine)

    async with operating(server, machine) as client:
        await ready(client, server, skill)
        await write_parameters(client, skill, {"Target.X": 2.0})
        await running(client, server, skill, runtime)
        runtime.deliver_feedback("/navigate", fixture_messages.NavigateFeedback(distance_remaining=9.0))
        runtime.settle_goal(
            "/navigate",
            status=GoalStatus.SUCCEEDED,
            result=fixture_messages.NavigateResult(total_distance=7.0, waypoints=2),
        )
        await reached(skill, SkillState.COMPLETED)
        assert await skill.read_monitoring() == {"DistanceRemaining": 9.0}

        await ready(client, server, skill)

    assert await results_of(skill) == {"TotalDistance": 0.0, "Waypoints": 0}
    assert await skill.read_monitoring() == {"DistanceRemaining": 0.0}
    assert (await skill.read_parameters())["Target.X"] == 2.0


# -- what a handler that does not return does ----------------------------------------------------


class Followed(ROS2ActionSkill):
    """A custom class that remembers the task its running handler is on."""

    handler: asyncio.Task[Any] | None = None

    async def _handle_running(self) -> None:
        self.handler = asyncio.current_task()
        await super()._handle_running()


class FollowedPlatform(TestComponent):
    bindings = (
        ActionBinding(
            name="Navigate",
            path="/navigate",
            action_type="fixture_msgs/action/Navigate",
            skill_type=Followed,
        ),
    )


async def test_a_cancelled_handler_ends_in_halted_and_cancels_its_goal(runtime) -> None:
    # Cancelled from outside, as a shutdown does: opensmi halts the Skill, which cancels the goal on the robot.
    async for server, machine in running_with(runtime, FollowedPlatform(name="Platform")):
        skill = skill_of(machine)
        async with operating(server, machine) as client:
            await ready(client, server, skill)
            await running(client, server, skill, runtime)
            await until(lambda followed=skill: followed.handler is not None)

            skill.handler.cancel()

            await reached(skill, SkillState.HALTED)

        assert runtime.cancel_requests == ("/navigate",)
        assert skill.ros2_goal is None


class CallingAnother(ROS2ActionSkill):
    """A custom class whose running handler meets a Skill it called being halted."""

    async def _handle_running(self) -> None:
        msg = "the Skill this one called was halted while it waited for it"
        raise SkillHaltedError(msg)


class CallingPlatform(TestComponent):
    bindings = (
        ActionBinding(
            name="Navigate",
            path="/navigate",
            action_type="fixture_msgs/action/Navigate",
            skill_type=CallingAnother,
        ),
    )


async def test_a_skill_halted_error_arrives_as_a_runtime_error_in_the_notification(runtime) -> None:
    async for server, machine in running_with(runtime, CallingPlatform(name="Platform")):
        skill = skill_of(machine)
        async with operating(server, machine) as client:
            collector = await alerts(client, server, machine.added[0])
            await ready(client, server, skill)
            assert await press(client, server, skill, "Start") == GOOD

            event = await collector.next_event()
            await reached(skill, SkillState.HALTED)

        assert "was halted while it waited" in event.Message.Text
        assert runtime.cancel_requests == ("/navigate",)


# -- suspend, which is absent on purpose -----------------------------------------------------


async def test_the_skill_is_never_suspendable_and_has_no_suspend_method(served) -> None:
    server, machine = served
    skill = skill_of(machine)

    async with connected(server, "visitor") as client:
        children = await client.get_node(skill.ua_state_machine.nodeid).get_children()
        browsed = {(await child.read_browse_name()).Name for child in children}

    assert "Start" in browsed
    assert "Suspend" not in browsed


# -- the custom implementation class ---------------------------------------------------------


class CountingNavigate(ROS2ActionSkill):
    """A custom class that adds behaviour without giving up the declaration."""

    built: int = 0

    async def build_goal(self, values: Mapping[str, object]) -> Any:
        self.built += 1
        return await super().build_goal(values)


class CustomPlatform(TestComponent):
    bindings = (
        ActionBinding(
            name="Navigate",
            path="/navigate",
            action_type="fixture_msgs/action/Navigate",
            skill_type=CountingNavigate,
        ),
    )


async def test_a_declaration_may_name_a_subclass_of_the_stock_skill(runtime) -> None:
    async for _server, machine in running_with(runtime, CustomPlatform(name="Platform")):
        skill = skill_of(machine)

        assert isinstance(skill, CountingNavigate)
        assert skill.built == 1


def test_a_class_that_is_not_a_subclass_is_fatal_where_it_is_declared() -> None:
    with pytest.raises(ImplementationClassError, match="ROS2ActionSkill"):
        ActionBinding(
            name="Navigate",
            path="/navigate",
            action_type="fixture_msgs/action/Navigate",
            skill_type=dict,
        )


def test_a_string_where_a_class_belongs_is_fatal_too() -> None:
    with pytest.raises(ImplementationClassError):
        ActionBinding(
            name="Navigate",
            path="/navigate",
            action_type="fixture_msgs/action/Navigate",
            skill_type=cast("Any", "tests.test_action_binding_server.CountingNavigate"),
        )


# -- the three outcome booleans, behind the configuration flag --------------------------------


async def test_the_outcome_booleans_are_absent_by_default(served) -> None:
    _, machine = served

    assert not {"succeeded", "aborted", "canceled"} & set(names(skill_of(machine).final_result_data))


async def test_the_configuration_flag_brings_the_three_booleans_back(runtime) -> None:
    flagged = ROS2Config(debug_goal_outcome_booleans=True)
    async for server, machine in running_with(runtime, Platform(name="Platform"), flagged):
        skill = skill_of(machine)
        async with operating(server, machine) as client:
            await ready(client, server, skill)
            await running(client, server, skill, runtime)
            runtime.settle_goal(
                "/navigate",
                status=GoalStatus.ABORTED,
                result=fixture_messages.NavigateResult(total_distance=0.0, waypoints=0),
            )
            await reached(skill, SkillState.HALTED)

            results = await results_of(skill)
            assert (results["succeeded"], results["aborted"], results["canceled"]) == (False, True, False)

            await ready(client, server, skill)

        results = await results_of(skill)
        assert (results["succeeded"], results["aborted"], results["canceled"]) == (False, False, False)


# -- the declaration faults that are fatal at init ---------------------------------------------


async def test_an_action_type_whose_package_is_absent_is_fatal_at_init(runtime) -> None:
    class MissingPackage(TestComponent):
        bindings = (ActionBinding(name="Navigate", path="/navigate", action_type="absent_msgs/action/Nope"),)

    with pytest.raises(MissingInterfacePackageError):
        await start(runtime, MissingPackage(name="Platform"))

    assert not runtime.started


async def test_an_action_with_an_empty_goal_and_no_feedback_is_declarable(runtime) -> None:
    class Trigger(TestComponent):
        bindings = (ActionBinding(name="Dock", path="/dock", action_type="fixture_msgs/action/Dock"),)

    runtime.serve_action("/dock")
    async for _server, machine in running_with(runtime, Trigger(name="Platform")):
        skill = skill_of(machine, "Dock")

        assert names(skill.parameter_set) == []
        assert names(skill.monitoring) == []
        assert names(skill.final_result_data) == ["Docked"]


async def test_a_type_mismatched_parameter_value_is_refused_on_send(served, runtime) -> None:
    # The send-time guard via the build path: an int in a string field would abort the process.
    _, machine = served
    skill = skill_of(machine)

    values = dict(await skill.read_parameters(), Label=7)
    with pytest.raises(FieldTypeMismatchError, match="label"):
        await runtime.send_goal(
            action="/navigate",
            action_type=fixture_messages.Navigate,
            goal=await skill.build_goal(values),
        )


async def test_a_goal_for_a_path_with_no_server_raises_rather_than_reporting(runtime) -> None:
    await runtime.start()
    try:
        with pytest.raises(ActionServerUnavailableError):
            await runtime.send_goal(
                action="/nowhere",
                action_type=fixture_messages.Navigate,
                goal=fixture_messages.NavigateGoal(target=fixture_messages.Point(x=0.0, y=0.0, z=0.0), label=""),
            )
    finally:
        await runtime.stop()


async def test_a_selection_that_leaves_the_goal_unbuildable_is_fatal_at_init(runtime) -> None:
    class PartialGoal(TestComponent):
        bindings = (
            ActionBinding(
                name="Navigate",
                path="/navigate",
                action_type="fixture_msgs/action/Navigate",
                exclude=("label",),
            ),
        )

    with pytest.raises(NotInvertibleError, match="label"):
        await start(runtime, PartialGoal(name="Platform"))


def _as_text(_message_type: Any, context: Any) -> tuple[PlanVariable, ...]:
    """Render Point's x as a String variable -- a curation that cannot be sent back."""
    return (
        PlanVariable(
            name="X",
            display_name="x",
            source=join_path(context.path, "x"),
            initial="",
            variant_type=ua.VariantType.String,
        ),
    )


def _read_as_text(message: Any, _context: Any) -> dict[str, object]:
    return {"X": str(message.x)}


def _build_from_text(message_type: Any, values: Any, _context: Any) -> Any:
    """Hand the String value straight back, which is where the declaration turns out to be wrong."""
    return message_type(x=values["X"], y=0.0, z=0.0)


async def test_a_goal_type_reaching_a_projection_that_cannot_invert_is_fatal_at_init(runtime) -> None:
    class NoWayBack(Platform):
        projections = (Functions("fixture_msgs/msg/Point", shaped_by=_as_text, decoded_by=_read_as_text),)

    with pytest.raises(NotInvertibleError, match="Functions"):
        await start(runtime, NoWayBack(name="Platform"))


async def test_a_declaration_whose_own_defaults_could_not_be_sent_is_fatal_at_init(runtime) -> None:
    # Declaration-time guard: a float64 rendered as a String would abort the process at send time.
    class TextPoint(Platform):
        projections = (
            Functions(
                "fixture_msgs/msg/Point",
                shaped_by=_as_text,
                decoded_by=_read_as_text,
                encode=_build_from_text,
            ),
        )

    with pytest.raises(FieldTypeMismatchError, match="float64"):
        await start(runtime, TextPoint(name="Platform"))

    assert not runtime.started


async def test_a_curated_projection_that_nests_is_refused_at_init(runtime) -> None:
    def nested(_type: Any, _context: Any) -> tuple[PlanObject]:
        child = PlanVariable(name="X", source="target/x", initial=0.0, variant_type=ua.VariantType.Double)
        return (PlanObject(name="target", children=(child,)),)

    class Folded(Platform):
        projections = (
            Functions(
                "fixture_msgs/msg/Point",
                shaped_by=nested,
                decoded_by=lambda _message, _context: {},
                encode=_build_from_text,
            ),
        )

    with pytest.raises(UnprojectableFieldError, match="ParameterSet of the Skill 'Navigate'"):
        await start(runtime, Folded(name="Platform"))


# -- what a reset actually writes ---------------------------------------------------------------

# `_reset_values` writes PlanVariable.initial, a typed zero only for generic Projections.


def _state_shape(_message_type: Any, context: Any) -> tuple[PlanVariable, ...]:
    """Render Navigate's feedback as one String variable declared holding `Idle`."""
    return (
        PlanVariable(
            name="state",
            source=join_path(context.path, "distance_remaining"),
            initial="Idle",
            variant_type=ua.VariantType.String,
        ),
    )


def _state_decode(message: Any, _context: Any) -> dict[str, object]:
    return {"state": "Running" if message.distance_remaining > 0.0 else "Arrived"}


class DeclaredIdle(Platform):
    projections = (
        Functions(
            "fixture_msgs/action/Navigate_Feedback",
            shaped_by=_state_shape,
            decoded_by=_state_decode,
        ),
    )


async def test_a_reset_writes_the_declared_value_rather_than_a_typed_zero(runtime) -> None:
    async for server, machine in running_with(runtime, DeclaredIdle(name="Platform")):
        skill = skill_of(machine)
        assert await skill.read_monitoring() == {"state": "Idle"}

        async with operating(server, machine) as client:
            await ready(client, server, skill)
            await running(client, server, skill, runtime)
            runtime.deliver_feedback("/navigate", fixture_messages.NavigateFeedback(distance_remaining=9.0))
            state = leaf(skill.monitoring, "state").ua_node
            await until(lambda node=state: reads(node, "Running"))
            runtime.settle_goal(
                "/navigate",
                status=GoalStatus.SUCCEEDED,
                result=fixture_messages.NavigateResult(total_distance=7.0, waypoints=2),
            )
            await reached(skill, SkillState.COMPLETED)

            await ready(client, server, skill)

        # `Idle`, not `""`: deleting the hand-written write of `Idle` would change what a client sees.
        assert await skill.read_monitoring() == {"state": "Idle"}


# -- the Parameters a client's values cannot become a goal from ---------------------------------


def _parse_text(message_type: Any, values: Any, _context: Any) -> Any:
    """Read the String back as a number, refusing what is not one -- as a codec refuses a body."""
    return message_type(x=float(values["X"]), y=0.0, z=0.0)


def _as_numeric_text(_message_type: Any, context: Any) -> tuple[PlanVariable, ...]:
    """Render Point's x as a String whose untouched default is a number, so init can send it."""
    return (
        PlanVariable(
            name="X",
            display_name="x",
            source=join_path(context.path, "x"),
            initial="0",
            variant_type=ua.VariantType.String,
        ),
    )


class ParsedPoint(Platform):
    projections = (
        Functions(
            "fixture_msgs/msg/Point",
            shaped_by=_as_numeric_text,
            decoded_by=_read_as_text,
            encode=_parse_text,
        ),
    )


async def test_parameters_that_cannot_become_a_goal_halt_start_with_an_error_code(runtime) -> None:
    async for server, machine in running_with(runtime, ParsedPoint(name="Platform")):
        skill = skill_of(machine)
        async with operating(server, machine) as client:
            collector = await alerts(client, server, machine.added[0])
            await ready(client, server, skill)
            await write_parameters(client, skill, {"Target.X": "north"})

            assert await press(client, server, skill, "Start") == GOOD
            event = await collector.next_event()
            await reached(skill, SkillState.HALTED)

        assert event.ErrorCode == GOAL_UNBUILDABLE
        assert "north" in event.Message.Text
        assert "Traceback" not in event.Message.Text
        assert skill.ros2_goal is None


# -- a goal that is a list, and a result that carries one ---------------------------------------


class Planner(TestComponent):
    bindings = (PLAN,)


@pytest.fixture
def planning() -> FakeRuntime:
    started = FakeRuntime(domain_id=7, action_types=ACTION_TYPES)
    started.serve_action("/plan")
    return started


@pytest.fixture
async def planning_served(planning: FakeRuntime) -> AsyncIterator[tuple[Server, Any]]:
    async for served in running_with(planning, Planner(name="Platform")):
        yield served


def waypoints(machine: Any, *xs: float) -> Mapping[str, object]:
    """Return the values a client writes to the Parameter a goal's pose list becomes."""
    goal = fixture_messages.PlanGoal(poses=tuple(fixture_messages.pose_stamped(x=each) for each in xs))
    context = ProjectionContext(
        registry=machine.added[0].ros2_projections,
        selected=resolve_selection(fixture_messages.PlanGoal, FieldSelection()),
    )
    return context.registry.find("fixture_msgs/action/Plan_Goal").decode(goal, context)


async def test_an_action_whose_goal_repeats_a_message_declares_with_no_skill_type(planning_served) -> None:
    server, machine = planning_served
    skill = skill_of(machine, "Plan")

    assert PLAN.skill_type is None
    assert PLAN.goal_from is None
    assert names(skill.parameter_set) == ["Poses"]
    async with connected(server, "visitor") as client:
        seen = client.get_node(leaf(skill.parameter_set, "Poses").ua_node.nodeid)
        assert await seen.read_value_rank() == ua.ValueRank.OneDimension
        namespace = machine.added[0].ros2_namespace_index
        assert await seen.read_data_type() == structure_node_id("fixture_msgs/msg/PoseStamped", namespace)
        assert await seen.read_value() == []


async def test_a_result_carrying_a_planned_path_browses_without_being_excluded(planning_served) -> None:
    _, machine = planning_served
    skill = skill_of(machine, "Plan")

    assert names(skill.final_result_data) == [
        "Path.Header.FrameId",
        "Path.Header.Stamp.Nanosec",
        "Path.Header.Stamp.Sec",
        "Path.Poses",
        "PlanningTime",
    ]


async def test_the_list_a_client_wrote_is_the_list_the_goal_carries(planning_served, planning) -> None:
    server, machine = planning_served
    skill = skill_of(machine, "Plan")

    async with operating(server, machine) as client:
        await ready(client, server, skill)
        await write_parameters(client, skill, waypoints(machine, 0.0, 1.0, 2.0))
        await running(client, server, skill, planning, "/plan", PLANNING)

    assert skill.ros2_goal is not None
    assert planning.sent_goals[-1] == (
        "/plan",
        fixture_messages.PlanGoal(poses=tuple(fixture_messages.pose_stamped(x=float(each)) for each in range(3))),
    )


async def test_the_same_parameter_takes_a_longer_list_on_the_next_goal(planning_served, planning) -> None:
    server, machine = planning_served
    skill = skill_of(machine, "Plan")

    async with operating(server, machine) as client:
        for count in (2, 20):
            await ready(client, server, skill)
            await write_parameters(client, skill, waypoints(machine, *(float(each) for each in range(count))))
            await running(client, server, skill, planning, "/plan", PLANNING)
            assert skill.ros2_goal is not None
            assert [each.pose.position.x for each in planning.sent_goals[-1][1].poses] == [
                float(each) for each in range(count)
            ]
            planning.settle_goal(
                "/plan",
                status=GoalStatus.SUCCEEDED,
                result=fixture_messages.PlanResult(path=fixture_messages.path(), planning_time=0.0),
            )
            await reached(skill, SkillState.COMPLETED)


async def test_the_planned_path_a_result_carried_reaches_the_variable_that_publishes_it(
    planning_served, planning
) -> None:
    server, machine = planning_served
    skill = skill_of(machine, "Plan")

    async with operating(server, machine) as client:
        await ready(client, server, skill)
        await running(client, server, skill, planning, "/plan", PLANNING)

        planning.settle_goal(
            "/plan",
            status=GoalStatus.SUCCEEDED,
            result=fixture_messages.PlanResult(
                path=fixture_messages.path(*(fixture_messages.pose_stamped(x=float(each)) for each in range(4))),
                planning_time=0.25,
            ),
        )

        await reached(skill, SkillState.COMPLETED)
        carried = client.get_node(leaf(skill.final_result_data, "Path.Poses").ua_node.nodeid)
        stored = elements_of(leaf(skill.final_result_data, "Path.Poses"), await carried.read_value())
        assert [each["pose.position.x"] for each in cast("list[dict[str, float]]", stored)] == [0.0, 1.0, 2.0, 3.0]


async def test_a_parameter_written_as_something_else_than_poses_is_refused_on_the_write(
    planning_served, planning
) -> None:
    server, machine = planning_served
    skill = skill_of(machine, "Plan")

    async with operating(server, machine) as client:
        await ready(client, server, skill)
        node = client.get_node(leaf(skill.parameter_set, "Poses").ua_node.nodeid)
        with pytest.raises(ua.uaerrors.BadTypeMismatch):
            await node.write_value(ua.Variant([1.0, 2.0], ua.VariantType.Double))

    assert await leaf(skill.parameter_set, "Poses").read() == []


# -- the Component ---------------------------------------------------------------------------------


async def test_two_actions_on_one_component_are_two_skills(runtime) -> None:
    class Both(TestComponent):
        bindings = (NAVIGATE, ActionBinding(name="Dock", path="/dock", action_type="fixture_msgs/action/Dock"))

    async for _server, machine in running_with(runtime, Both(name="Platform")):
        assert sorted(skill.name for skill in machine.added[0].skill_set) == ["Dock", "Navigate"]


async def test_a_component_with_no_action_declared_gains_no_skill(runtime) -> None:
    async for _server, machine in running_with(runtime, TestComponent(name="Platform")):
        assert len(machine.added[0].skill_set) == 0
