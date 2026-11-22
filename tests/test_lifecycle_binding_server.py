# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A managed node drives the StartupSkill and the StartupSkill drives it back.

The dictionary `serve_nodes` returns is the robot's state. The StartupSkill is not suspendable, so a
node deactivated elsewhere takes the Skill to Halted.
"""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from typing import TYPE_CHECKING, Any, cast

import pytest
from asyncua import ua
from asyncua.ua.object_ids import ObjectIds
from asyncua.ua.status_codes import StatusCodes
from asyncua.ua.uaerrors import BadNoMatch
from opensmi.core import SkillState
from opensmi.core.nodesets import SmartFactoryMachineSetNodeIds
from opensmi.server import BaseSkillContinuous
from opensmi.server.base_startup_skill import BaseStartupSkill
from opensmi.server.user import INTERNAL_USER

from opensmi.ros2.server.bindings import LifecycleBinding
from opensmi.ros2.server.errors import DeclarationError
from opensmi.ros2.server.events import BINDING_EVENT_TYPE
from opensmi.ros2.server.lifecycle import LifecycleNodeState
from opensmi.ros2.server.startup import (
    EXTERNALLY_MOVED,
    TERMINAL,
    TRANSITION_REFUSED,
    UNKNOWN_STATE,
    ROS2LifecycleStartupSkill,
)
from tests import fixture_lifecycle
from tests.fake_runtime import FakeRuntime
from tests.served import TestComponent, TestMachine, connected, monitoring_nodes, serve

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Callable

    from asyncua.client.client import Client
    from asyncua.common.node import Node
    from opensmi.server import Server

TALKER = "/lc_talker"
LISTENER = "/lc_listener"
MANAGER = "/lifecycle_manager"

UNCONFIGURED = int(LifecycleNodeState.UNCONFIGURED)
INACTIVE = int(LifecycleNodeState.INACTIVE)
ACTIVE = int(LifecycleNodeState.ACTIVE)
FINALIZED = int(LifecycleNodeState.FINALIZED)
CONFIGURING = int(LifecycleNodeState.CONFIGURING)
ACTIVATING = int(LifecycleNodeState.ACTIVATING)
DEACTIVATING = int(LifecycleNodeState.DEACTIVATING)
CLEANING_UP = int(LifecycleNodeState.CLEANING_UP)
SHUTTING_DOWN = int(LifecycleNodeState.SHUTTING_DOWN)

SETTLED = 5.0
"""Long enough not to flake on a loaded runner, short enough that a hang fails one test."""

TALKER_BINDING = LifecycleBinding(name="Talker", path=TALKER)
PAIR_BINDING = LifecycleBinding(name="Pair", path=TALKER, nodes=(TALKER, LISTENER))
NAV2_BINDING = LifecycleBinding(name="Navigation", path=MANAGER, nodes=(TALKER,), driver="nav2")


class Platform(TestComponent):
    bindings = (TALKER_BINDING,)


class Pair(Platform):
    bindings = (PAIR_BINDING,)


class Managed(Platform):
    bindings = (NAV2_BINDING,)


class Quiet(Platform):
    bindings = ()


@pytest.fixture
def runtime() -> FakeRuntime:
    return FakeRuntime(
        domain_id=7,
        message_types=fixture_lifecycle.MESSAGE_TYPES,
        service_types=fixture_lifecycle.SERVICE_TYPES,
    )


async def start(runtime: FakeRuntime, platform: TestComponent | None = None) -> tuple[Server, Any]:
    """Bring up a server carrying one Machine with one Component, and hand back both."""
    machine = TestMachine(platform or Platform(name="Platform"), runtime=runtime)
    return await serve(machine), machine


@pytest.fixture
async def served(runtime: FakeRuntime) -> AsyncIterator[tuple[Server, Any]]:
    server, machine = await start(runtime)
    try:
        yield server, machine
    finally:
        await server.shutdown()


@asynccontextmanager
async def running(runtime: FakeRuntime, platform: TestComponent) -> AsyncIterator[tuple[Server, Any]]:
    """Serve one Component for the length of a test that declares its own."""
    server, machine = await start(runtime, platform)
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


def startup_of(machine: Any) -> ROS2LifecycleStartupSkill:
    """Return the Component's StartupSkill, browsed for the way opensmi finds it."""
    skill: Any = platform_of(machine).skill_set[BaseStartupSkill.NAME]
    return skill


def announce(runtime: FakeRuntime, states: dict[str, int], node: str, *, start: int, goal: int) -> None:
    """Move one node and let it announce the move, as a real node does."""
    states[node] = goal
    runtime.deliver(f"{node}/transition_event", fixture_lifecycle.transition_event(start=start, goal=goal))


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
    """Subscribe to opensmi's own fault events, which is where a refused transition lands."""
    subscription = await client.create_subscription(20, collector)
    await subscription.subscribe_events(
        client.get_node(item.notification.ua_node.nodeid),
        client.get_node(server.ua_get_node_id(SmartFactoryMachineSetNodeIds.AlertType)),
    )


# -- the Binding is the StartupSkill ----------------------------------------------------------


async def test_the_binding_is_the_startup_skill_opensmi_already_looks_for(served) -> None:
    _, machine = served
    skill = startup_of(machine)

    assert skill.name == BaseStartupSkill.NAME
    assert isinstance(skill, BaseSkillContinuous)
    assert isinstance(skill, BaseStartupSkill)
    assert platform_of(machine).startup_skill is skill
    assert not skill.is_finite


async def test_the_startup_skill_is_not_suspendable_and_a_client_has_no_suspend_to_press(served) -> None:
    # a StartupSkill is never suspendable, so there is no Suspended row.
    server, machine = served
    skill = startup_of(machine)
    suspend = f"{server.ua_get_namespace_index(SmartFactoryMachineSetNodeIds)}:Suspend"

    with pytest.raises(BadNoMatch):
        await skill.ua_state_machine.get_child(suspend)


async def test_a_component_that_declares_no_lifecycle_binding_gains_no_startup_skill(runtime) -> None:
    async with running(runtime, Quiet(name="Platform")) as (_, machine):
        assert BaseStartupSkill.NAME not in platform_of(machine).skill_set


async def test_a_component_that_already_has_a_startup_skill_is_refused(runtime) -> None:
    class Own(BaseStartupSkill):
        async def _handle_running(self) -> None:
            pass

    class Twice(Platform):
        async def _init(self) -> None:
            await self.add_skill(Own(handle_status_change=False))
            await super()._init()

    with pytest.raises(DeclarationError, match="a lifecycle Binding is the StartupSkill"):
        await start(runtime, Twice(name="Platform"))


async def test_two_declarations_following_one_node_are_refused(runtime) -> None:
    class Twin(Platform):
        bindings = (
            LifecycleBinding(name="One", path=TALKER),
            LifecycleBinding(name="Two", path=MANAGER, nodes=(TALKER,), driver="nav2"),
        )

    with pytest.raises(DeclarationError, match="followed by both"):
        await start(runtime, Twin(name="Platform"))


async def test_every_declared_node_has_a_state_variable_before_the_robot_says_anything(served) -> None:
    _, machine = served
    nodes = monitoring_nodes(startup_of(machine))

    assert list(nodes) == ["Talker.LcTalker.State"]
    assert await nodes["Talker.LcTalker.State"].read_value() == UNKNOWN_STATE


async def test_a_state_variable_is_a_string_with_the_scalar_rank(served) -> None:
    _, machine = served
    node = monitoring_nodes(startup_of(machine))["Talker.LcTalker.State"]

    assert await node.read_data_type() == ua.NodeId(ua.Int32(ObjectIds.String))
    assert await node.read_value_rank() == ua.ValueRank.Scalar


# -- the triggers command the transitions -----------------------------------------------------


async def test_reset_configures_and_start_activates(served, runtime) -> None:
    _, machine = served
    states = fixture_lifecycle.serve_nodes(runtime, {TALKER: UNCONFIGURED})
    skill = startup_of(machine)

    await skill.reset()
    await reached(skill, SkillState.READY)
    assert states == {TALKER: INACTIVE}

    await skill.start()
    await reached(skill, SkillState.RUNNING)
    assert states == {TALKER: ACTIVE}


async def test_a_running_startup_skill_stays_running(served, runtime) -> None:
    _, machine = served
    fixture_lifecycle.serve_nodes(runtime, {TALKER: UNCONFIGURED})
    skill = startup_of(machine)
    await skill.reset()
    await reached(skill, SkillState.READY)
    await skill.start()
    await reached(skill, SkillState.RUNNING)

    await asyncio.sleep(0.3)

    assert skill.current_state is SkillState.RUNNING


async def test_halt_deactivates_and_cleans_up_and_never_shuts_down(served, runtime) -> None:
    _, machine = served
    states = fixture_lifecycle.serve_nodes(runtime, {TALKER: UNCONFIGURED})
    skill = startup_of(machine)
    await skill.reset()
    await reached(skill, SkillState.READY)
    await skill.start()
    await reached(skill, SkillState.RUNNING)

    await skill.halt()
    await reached(skill, SkillState.HALTED)

    assert states == {TALKER: UNCONFIGURED}
    assert skill.terminal == ()


def commanded(runtime: FakeRuntime) -> list[str]:
    """Return the nodes a ``change_state`` request reached, in order."""
    return [service for service, _ in runtime.requests if service.endswith("change_state")]


async def test_halt_leaves_a_node_that_was_active_before_the_skill_started(runtime) -> None:
    # Halt rolls back only what this Skill did.
    async with running(runtime, Pair(name="Platform")) as (_, machine):
        states = fixture_lifecycle.serve_nodes(runtime, {TALKER: ACTIVE, LISTENER: UNCONFIGURED})
        skill = startup_of(machine)
        await skill.reset()
        await reached(skill, SkillState.READY)
        await skill.start()
        await reached(skill, SkillState.RUNNING)

        await skill.halt()
        await reached(skill, SkillState.HALTED)

        assert states == {TALKER: ACTIVE, LISTENER: UNCONFIGURED}
        assert set(commanded(runtime)) == {f"{LISTENER}/change_state"}


async def test_a_startup_that_fails_halts_without_touching_the_node_it_only_followed(runtime) -> None:
    # A late Binding halts the Skill but must not stand a working stack down.
    async with running(runtime, Pair(name="Platform")) as (_, machine):
        states = fixture_lifecycle.serve_nodes(runtime, {TALKER: ACTIVE, LISTENER: INACTIVE}, refuse={LISTENER})
        skill = startup_of(machine)
        await skill.reset()
        await reached(skill, SkillState.READY)

        await skill.start()
        await reached(skill, SkillState.HALTED)

        assert states == {TALKER: ACTIVE, LISTENER: INACTIVE}
        assert set(commanded(runtime)) == {f"{LISTENER}/change_state"}


async def test_halt_takes_the_nodes_the_skill_brought_up_down_together(runtime) -> None:
    async with running(runtime, Pair(name="Platform")) as (_, machine):
        states = fixture_lifecycle.serve_nodes(runtime, {TALKER: UNCONFIGURED, LISTENER: UNCONFIGURED})
        skill = startup_of(machine)
        await skill.reset()
        await reached(skill, SkillState.READY)
        await skill.start()
        await reached(skill, SkillState.RUNNING)

        announce(runtime, states, TALKER, start=CLEANING_UP, goal=UNCONFIGURED)
        await reached(skill, SkillState.HALTED)

        await until(lambda: states[LISTENER] == UNCONFIGURED, "the other node was left active")


async def test_halt_does_not_unconfigure_a_node_the_skill_only_activated(runtime) -> None:
    async with running(runtime, Platform(name="Platform")) as (_, machine):
        states = fixture_lifecycle.serve_nodes(runtime, {TALKER: INACTIVE})
        skill = startup_of(machine)
        await skill.reset()
        await reached(skill, SkillState.READY)
        await skill.start()
        await reached(skill, SkillState.RUNNING)

        await skill.halt()
        await reached(skill, SkillState.HALTED)

        assert states == {TALKER: INACTIVE}


async def test_reset_and_start_skip_a_node_already_in_the_target_state(runtime) -> None:
    async with running(runtime, Pair(name="Platform")) as (_, machine):
        fixture_lifecycle.serve_nodes(runtime, {TALKER: INACTIVE, LISTENER: ACTIVE})
        skill = startup_of(machine)

        await skill.reset()
        await reached(skill, SkillState.READY)
        assert commanded(runtime) == []

        await skill.start()
        await reached(skill, SkillState.RUNNING)
        assert commanded(runtime) == [f"{TALKER}/change_state"]


async def test_a_node_that_refuses_to_activate_halts_the_skill_and_names_itself(served, runtime) -> None:
    # A refusal raises no BindingEvent: the Notification already carries TRANSITION_REFUSED.
    server, machine = served
    fixture_lifecycle.serve_nodes(runtime, {TALKER: INACTIVE}, refuse={TALKER})
    skill = startup_of(machine)
    await skill.reset()
    await reached(skill, SkillState.READY)

    refusals = Collector()
    bindings = Collector()
    async with connected(server) as client:
        await subscribe_to_faults(client, server, platform_of(machine), refusals)
        await subscribe_to_events(client, server, platform_of(machine), bindings)
        await skill.start()
        await reached(skill, SkillState.HALTED)
        await until(lambda: TRANSITION_REFUSED in refusals.error_codes(), "no refusal reached the client")

    refusal = next(event for event in refusals.events if event.ErrorCode == TRANSITION_REFUSED)
    assert TALKER in refusal.Message.Text
    assert "refused to activate" in refusal.Message.Text
    assert TRANSITION_REFUSED not in bindings.codes()


# -- several nodes aggregate lowest-wins -------------------------------------------------------


async def test_running_means_every_node_is_active(runtime) -> None:
    async with running(runtime, Pair(name="Platform")) as (_, machine):
        states = fixture_lifecycle.serve_nodes(runtime, {TALKER: UNCONFIGURED, LISTENER: UNCONFIGURED})
        skill = startup_of(machine)
        await skill.reset()
        await reached(skill, SkillState.READY)
        await skill.start()
        await reached(skill, SkillState.RUNNING)
        assert states == {TALKER: ACTIVE, LISTENER: ACTIVE}

        announce(runtime, states, LISTENER, start=DEACTIVATING, goal=INACTIVE)

        await reached(skill, SkillState.HALTED)
        await until(lambda: states == {TALKER: UNCONFIGURED, LISTENER: UNCONFIGURED}, "the group was left standing")


async def test_start_does_not_claim_running_while_a_node_is_not_active(runtime) -> None:
    async with running(runtime, Pair(name="Platform")) as (server, machine):
        states = fixture_lifecycle.serve_nodes(runtime, {TALKER: INACTIVE, LISTENER: UNCONFIGURED})
        skill = startup_of(machine)
        await skill.reset()
        await reached(skill, SkillState.READY)
        states[LISTENER] = UNCONFIGURED

        refusals = Collector()
        async with connected(server) as client:
            await subscribe_to_faults(client, server, platform_of(machine), refusals)
            await skill.start()
            await reached(skill, SkillState.HALTED)
            await until(lambda: TRANSITION_REFUSED in refusals.error_codes(), "nothing said why")

        refusal = next(event for event in refusals.events if event.ErrorCode == TRANSITION_REFUSED)
        assert f"{LISTENER} did not reach active" in refusal.Message.Text
        assert EXTERNALLY_MOVED not in refusals.error_codes()
        assert states[LISTENER] == UNCONFIGURED


async def test_one_node_in_unconfigured_drags_the_whole_skill_halted(runtime) -> None:
    async with running(runtime, Pair(name="Platform")) as (_, machine):
        states = fixture_lifecycle.serve_nodes(runtime, {TALKER: ACTIVE, LISTENER: ACTIVE})
        skill = startup_of(machine)
        await skill.reset()
        await reached(skill, SkillState.READY)
        await skill.start()
        await reached(skill, SkillState.RUNNING)

        announce(runtime, states, LISTENER, start=CLEANING_UP, goal=UNCONFIGURED)

        await reached(skill, SkillState.HALTED)


# -- following the robot ------------------------------------------------------------------------


async def running_skill(machine: Any, runtime: FakeRuntime, node: int = ACTIVE) -> dict[str, int]:
    """Take the Component's StartupSkill to Running against one node, and hand back the robot."""
    states = fixture_lifecycle.serve_nodes(runtime, {TALKER: node})
    skill = startup_of(machine)
    await skill.reset()
    await reached(skill, SkillState.READY)
    await skill.start()
    await reached(skill, SkillState.RUNNING)
    return states


async def test_an_external_cleanup_halts_the_skill_and_says_nobody_asked(served, runtime) -> None:
    server, machine = served
    states = await running_skill(machine, runtime)
    skill = startup_of(machine)

    collector = Collector()
    async with connected(server) as client:
        await subscribe_to_events(client, server, platform_of(machine), collector)
        announce(runtime, states, TALKER, start=CLEANING_UP, goal=UNCONFIGURED)
        await reached(skill, SkillState.HALTED)
        await until(lambda: EXTERNALLY_MOVED in collector.codes(), "no event said who moved the node")

    assert next(event for event in collector.events if event.Code == EXTERNALLY_MOVED).ROS2Path == TALKER


async def test_an_external_deactivation_halts_the_skill_because_there_is_no_suspended(served, runtime) -> None:
    # A paused node lands in Halted, since a StartupSkill cannot be suspended.
    server, machine = served
    states = await running_skill(machine, runtime)
    skill = startup_of(machine)

    collector = Collector()
    async with connected(server) as client:
        await subscribe_to_events(client, server, platform_of(machine), collector)
        announce(runtime, states, TALKER, start=DEACTIVATING, goal=INACTIVE)
        await reached(skill, SkillState.HALTED)
        await until(lambda: EXTERNALLY_MOVED in collector.codes(), "no event said who moved the node")

    assert skill.terminal == ()


async def test_a_failed_activation_lands_halted(served, runtime) -> None:
    _, machine = served
    states = await running_skill(machine, runtime)

    announce(runtime, states, TALKER, start=ACTIVATING, goal=INACTIVE)

    await reached(startup_of(machine), SkillState.HALTED)


async def test_an_external_activation_does_not_start_the_skill(served, runtime) -> None:
    # Following is downward only.
    _, machine = served
    states = fixture_lifecycle.serve_nodes(runtime, {TALKER: ACTIVE})
    skill = startup_of(machine)

    announce(runtime, states, TALKER, start=ACTIVATING, goal=ACTIVE)
    await asyncio.sleep(0.2)

    assert skill.current_state is SkillState.HALTED


async def test_a_node_that_vanishes_does_not_halt_a_running_skill(served, runtime) -> None:
    _, machine = served
    await running_skill(machine, runtime)
    skill = startup_of(machine)

    runtime.note_publishers(f"{TALKER}/transition_event", 0)
    await asyncio.sleep(0.2)

    assert skill.current_state is SkillState.RUNNING


async def until_value(node: Node, expected: str) -> None:
    """Wait until a variable reads a value, failing rather than hanging if it never does."""
    deadline = asyncio.get_running_loop().time() + SETTLED
    while (held := await node.read_value()) != expected:
        assert asyncio.get_running_loop().time() < deadline, f"the variable stayed {held!r}"
        await asyncio.sleep(0.005)


async def test_the_state_variable_follows_the_node_and_reports_finalized_as_good(served, runtime) -> None:
    _, machine = served
    states = fixture_lifecycle.serve_nodes(runtime, {TALKER: ACTIVE})
    node = monitoring_nodes(startup_of(machine))["Talker.LcTalker.State"]

    announce(runtime, states, TALKER, start=ACTIVATING, goal=ACTIVE)
    await until_value(node, "active")

    announce(runtime, states, TALKER, start=SHUTTING_DOWN, goal=FINALIZED)
    await until_value(node, "finalized")

    status = (await node.read_data_value()).StatusCode
    assert status is not None
    assert status.is_good()


# -- terminal ------------------------------------------------------------------------------------


async def test_a_terminal_node_leaves_the_skill_halted_and_refuses_reset(served, runtime) -> None:
    _, machine = served
    states = fixture_lifecycle.serve_nodes(runtime, {TALKER: FINALIZED})
    skill = startup_of(machine)

    announce(runtime, states, TALKER, start=SHUTTING_DOWN, goal=FINALIZED)
    await until(lambda: skill.terminal == (TALKER,), "the node never latched Terminal")

    assert skill.current_state is SkillState.HALTED
    # Refused, not attempted: commanding a finalized node kills an rclpy process.
    with pytest.raises(ua.UaStatusCodeError) as refused:
        await skill.reset()

    assert refused.value.code == StatusCodes.BadInvalidState
    assert not [service for service, _ in runtime.requests if service.endswith("change_state")]


async def test_the_refusal_reaches_a_client_as_a_status_code(served, runtime) -> None:
    _, machine = served
    states = fixture_lifecycle.serve_nodes(runtime, {TALKER: FINALIZED})
    skill = startup_of(machine)
    announce(runtime, states, TALKER, start=SHUTTING_DOWN, goal=FINALIZED)
    await until(lambda: skill.terminal == (TALKER,), "the node never latched Terminal")

    answered = await skill._machine.try_trigger(cast("Any", INTERNAL_USER), "reset")

    assert answered.value == StatusCodes.BadInvalidState


async def test_one_event_carries_the_terminal_cause(served, runtime) -> None:
    server, machine = served
    states = fixture_lifecycle.serve_nodes(runtime, {TALKER: ACTIVE})
    skill = startup_of(machine)

    collector = Collector()
    async with connected(server) as client:
        await subscribe_to_events(client, server, platform_of(machine), collector)
        announce(runtime, states, TALKER, start=SHUTTING_DOWN, goal=FINALIZED)
        await until(lambda: TERMINAL in collector.codes(), "no event carried the terminal cause")
        # Once per latch, not per observation.
        announce(runtime, states, TALKER, start=SHUTTING_DOWN, goal=FINALIZED)
        await asyncio.sleep(0.1)

    assert collector.codes().count(TERMINAL) == 1
    assert next(event for event in collector.events if event.Code == TERMINAL).ROS2Path == TALKER
    assert skill.terminal == (TALKER,)


async def test_a_server_starting_against_an_already_terminal_node_still_learns(runtime) -> None:
    async with running(runtime, Platform(name="Platform")) as (_, machine):
        skill = startup_of(machine)
        assert skill.terminal == ()

        fixture_lifecycle.serve_nodes(runtime, {TALKER: FINALIZED})

        await until(lambda: skill.terminal == (TALKER,), "the state query never ran on discovery")
        assert skill.current_state is SkillState.HALTED


async def test_the_latch_lifts_only_on_a_non_terminal_state_on_that_path(served, runtime) -> None:
    _, machine = served
    states = fixture_lifecycle.serve_nodes(runtime, {TALKER: FINALIZED})
    skill = startup_of(machine)
    await until(lambda: skill.terminal == (TALKER,), "the node never latched Terminal")

    runtime.note_publishers(f"{TALKER}/transition_event", 0)
    await asyncio.sleep(0.1)
    assert skill.terminal == (TALKER,)

    announce(runtime, states, TALKER, start=int(LifecycleNodeState.UNKNOWN), goal=UNCONFIGURED)

    await until(lambda: skill.terminal == (), "the latch never lifted")


async def test_the_latch_is_per_skill_and_the_event_names_the_node(runtime) -> None:
    async with running(runtime, Pair(name="Platform")) as (_, machine):
        states = fixture_lifecycle.serve_nodes(runtime, {TALKER: ACTIVE, LISTENER: ACTIVE})
        skill = startup_of(machine)
        await skill.reset()
        await reached(skill, SkillState.READY)

        announce(runtime, states, LISTENER, start=SHUTTING_DOWN, goal=FINALIZED)
        await until(lambda: skill.terminal == (LISTENER,), "the node never latched Terminal")
        await reached(skill, SkillState.HALTED)

        answered = await skill._machine.try_trigger(cast("Any", INTERNAL_USER), "reset")
        assert answered.value == StatusCodes.BadInvalidState


# -- two Drivers, one surface -----------------------------------------------------------------


async def test_a_nav2_managed_component_presents_the_same_surface(runtime) -> None:
    async with running(runtime, Managed(name="Platform")) as (_, machine):
        skill = startup_of(machine)
        states = fixture_lifecycle.serve_nodes(runtime, {TALKER: UNCONFIGURED})
        runtime.serve_service(
            f"{MANAGER}/manage_nodes",
            lambda _request: fixture_lifecycle.ManageLifecycleNodesResponse(success=True),
        )

        assert skill.name == BaseStartupSkill.NAME
        assert list(monitoring_nodes(skill)) == ["Navigation.LcTalker.State"]

        await skill.reset()
        await reached(skill, SkillState.READY)

        assert [service for service, _ in runtime.requests if service.endswith("manage_nodes")] == [
            f"{MANAGER}/manage_nodes"
        ]
        assert not [service for service, _ in runtime.requests if service.endswith("change_state")]
        assert states == {TALKER: UNCONFIGURED}


# -- teardown ------------------------------------------------------------------------------------


async def test_shutting_the_component_down_stops_following(runtime) -> None:
    server, machine = await start(runtime)
    skill = startup_of(machine)
    await server.shutdown()

    assert not skill.observed
    assert runtime.deliver(f"{TALKER}/transition_event", fixture_lifecycle.transition_event(start=10, goal=2)) == 0
