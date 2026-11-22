# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A published topic is a Skill a UA client writes at leisure and then starts.

Most tests drive the state machine through opensmi's internal interface; the gates are tested
through a real client holding the Lock. `FAST` ticks several times per poll, `SLOW` exactly once by
the time Running is observed.
"""

from __future__ import annotations

import asyncio
import inspect
from contextlib import asynccontextmanager
from typing import TYPE_CHECKING, Any, cast

import pytest
from asyncua import ua
from asyncua.ua.status_codes import StatusCodes
from opensmi.core import SkillState
from opensmi.core.nodesets import DiNodeIds, SmartFactoryMachineSetNodeIds
from opensmi.server.base_startup_skill import BaseStartupSkill

from opensmi.ros2.runtime.errors import MissingInterfacePackageError
from opensmi.ros2.runtime.selection import FieldSelection, resolve_selection
from opensmi.ros2.server.access import SKILL_ACCESS_LEVEL
from opensmi.ros2.server.bindings import PublisherBinding
from opensmi.ros2.server.config import ROS2Config
from opensmi.ros2.server.errors import DeclarationError, NotInvertibleError
from opensmi.ros2.server.events import BINDING_EVENT_TYPE
from opensmi.ros2.server.projection import PlanVariable, ProjectionContext, join_path
from opensmi.ros2.server.publisher import (
    LOCK_ENDED,
    MAXIMUM_RATE,
    MESSAGE_UNBUILDABLE,
    MINIMUM_RATE,
    PUBLISH_FAILED,
    RATE_PARAMETER,
    SETPOINT_STALE,
    ROS2ContinuousPublisherSkill,
    ROS2FinitePublisherSkill,
)
from opensmi.ros2.server.recipes import Functions
from tests import fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.served import TestComponent, TestMachine, connected, leaf, leaves, serve, wire_value

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Callable, Mapping

    from asyncua.client.client import Client
    from opensmi.server import Server

MESSAGE_TYPES = {
    "fixture_msgs/msg/Point": fixture_messages.Point,
    "fixture_msgs/msg/Grid": fixture_messages.Grid,
    "fixture_msgs/msg/PoseStamped": fixture_messages.PoseStamped,
    "fixture_msgs/msg/Ramp": fixture_messages.Ramp,
    "fixture_msgs/msg/Path": fixture_messages.Path,
    "fixture_msgs/msg/Twist": fixture_messages.Twist,
    "fixture_msgs/msg/Vector3": fixture_messages.Vector3,
}

FAST = 100.0
SLOW = 2.0

# Timeout for every wait here; short enough that a hang fails the test.
SETTLED = 5.0

TOLERATED = 2

GOOD = ua.StatusCode(ua.UInt32(StatusCodes.Good))

DRIVE = PublisherBinding(
    name="Drive",
    path="/cmd_vel",
    message_type="fixture_msgs/msg/Point",
    continuous=True,
    rate=FAST,
    tolerated_failures=TOLERATED,
)
PUSH_MAP = PublisherBinding(
    name="PushMap",
    path="/map",
    message_type="fixture_msgs/msg/Grid",
    continuous=False,
)
PUSH_PLAN = PublisherBinding(
    name="PushPlan",
    path="/plan",
    message_type="fixture_msgs/msg/Path",
    continuous=False,
)

FRAMEWORKS_OWN = frozenset({"SuccessfulExecutionsCount"})


def status(code: int) -> ua.StatusCode:
    return ua.StatusCode(ua.UInt32(code))


class Platform(TestComponent):
    bindings = (DRIVE, PUSH_MAP)


@pytest.fixture
def runtime() -> FakeRuntime:
    return FakeRuntime(domain_id=7, message_types=MESSAGE_TYPES)


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


@asynccontextmanager
async def serving(
    runtime: FakeRuntime,
    platform: TestComponent,
    ros2_config: ROS2Config | None = None,
) -> AsyncIterator[tuple[Server, Any]]:
    """Serve one Component for the length of a block that declares its own."""
    server, machine = await start(runtime, platform, ros2_config)
    try:
        yield server, machine
    finally:
        await server.shutdown()


def skill_of(machine: Any, name: str = "Drive") -> Any:
    return machine.added[0].skill_set[name]


def names(container: Any) -> list[str]:
    return sorted(name for name in leaves(container) if name not in FRAMEWORKS_OWN)


async def until(answered: Callable[[], Any], complaint: str = "it never happened") -> None:
    """Poll until something opensmi does on a task of its own has happened."""
    try:
        async with asyncio.timeout(SETTLED):
            while True:
                answer = answered()
                if inspect.isawaitable(answer):
                    answer = await answer
                if answer:
                    return
                await asyncio.sleep(0.005)
    except TimeoutError:
        pytest.fail(complaint)


async def reached(skill: Any, state: SkillState) -> None:
    """Wait until a Skill reaches a state, failing the test rather than hanging if it does not."""
    await until(lambda: skill.current_state is state, f"the Skill stayed {skill.current_state}, never {state}")


async def ready(skill: Any) -> None:
    """Take a Skill from its initial Halted to Ready, which is what a client's Reset does."""
    await skill.reset()
    await reached(skill, SkillState.READY)


async def write_parameters(skill: Any, values: Mapping[str, object]) -> None:
    """Write Parameters the way the server itself does, which stamps them like any other write."""
    for name, value in values.items():
        variable = leaf(skill.parameter_set, name)
        await variable.write(wire_value(variable, value))


def sent(runtime: FakeRuntime, topic: str) -> list[Any]:
    """Return every message published on one topic, in order."""
    return [message for each, message in runtime.published if each == topic]


class Collector:
    """An asyncua subscription handler that keeps what it was sent."""

    def __init__(self) -> None:
        self.events: list[Any] = []

    def event_notification(self, event: Any) -> None:
        self.events.append(event)

    async def next_event(self) -> Any:
        await until(lambda: self.events, "no event reached the subscriber")
        return self.events[0]


async def events_of(client: Client, server: Server, machine: Any, event_type: ua.NodeId) -> Collector:
    """Subscribe to one type of event on the Component's Notification."""
    collector = Collector()
    subscription = await client.create_subscription(20, collector)
    await subscription.subscribe_events(
        client.get_node(machine.added[0].notification.ua_node.nodeid), client.get_node(event_type)
    )
    return collector


async def alerts(client: Client, server: Server, machine: Any) -> Collector:
    """Subscribe to the runtime errors the Component's Notification raises."""
    return await events_of(client, server, machine, server.ua_get_node_id(SmartFactoryMachineSetNodeIds.AlertType))


async def binding_events(client: Client, server: Server, machine: Any) -> Collector:
    """Subscribe to the BindingEvents the Component raises."""
    return await events_of(client, server, machine, machine.server.ua_get_node_id(BINDING_EVENT_TYPE))


# -- a client, the Lock, and the three Methods of the state machine -----------------------------


@asynccontextmanager
async def operating(server: Server, machine: Any, user: str = "operator") -> AsyncIterator[Client]:
    """Connect a client that holds the Machine's Lock, and give it back before closing."""
    namespace = server.ua_get_namespace_index(DiNodeIds)
    async with connected(server, user) as client:
        lock = client.get_node(machine.lock.ua_node.nodeid)
        await lock.call_method(f"{namespace}:InitLock", "the test suite")
        try:
            yield client
        finally:
            await lock.call_method(f"{namespace}:ExitLock")


async def press(client: Client, server: Server, skill: Any, verb: str) -> ua.StatusCode:
    """Call one Method of a Skill's state machine and return the status code it answered."""
    browse_name = f"{server.ua_get_namespace_index(SmartFactoryMachineSetNodeIds)}:{verb}"
    try:
        await client.get_node(skill.ua_state_machine.nodeid).call_method(browse_name)
    except ua.UaStatusCodeError as answer:
        return status(answer.code)
    return GOOD


# -- what a client sees before anything runs ------------------------------------------------------


async def test_a_published_topic_is_a_skill_of_the_declared_continuity(served) -> None:
    _, machine = served

    assert isinstance(skill_of(machine, "Drive"), ROS2ContinuousPublisherSkill)
    assert isinstance(skill_of(machine, "PushMap"), ROS2FinitePublisherSkill)
    assert not skill_of(machine, "Drive").is_finite
    assert skill_of(machine, "PushMap").is_finite


async def test_a_client_browses_the_skills_under_the_components_skill_set(served) -> None:
    server, machine = served

    async with connected(server, "visitor") as client:
        children = await client.get_node(machine.added[0].skill_set.ua_node.nodeid).get_children()
        browsed = {(await child.read_browse_name()).Name for child in children}

    assert {"Drive", "PushMap"} <= browsed


async def test_the_message_fields_become_the_skills_parameters(served) -> None:
    _, machine = served

    assert names(skill_of(machine, "PushMap").parameter_set) == ["Data", "Width"]
    assert names(skill_of(machine, "Drive").parameter_set) == [RATE_PARAMETER, "X", "Y", "Z"]


async def test_a_variable_is_named_for_the_field_path_and_carries_no_binding_prefix(served) -> None:
    server, machine = served
    node = leaf(skill_of(machine, "Drive").parameter_set, "X").ua_node

    async with connected(server, "visitor") as client:
        browse_name = await client.get_node(node.nodeid).read_browse_name()

    assert browse_name.Name == "X"


async def test_a_parameter_carries_the_datatype_of_its_field(served) -> None:
    server, machine = served
    node = leaf(skill_of(machine, "PushMap").parameter_set, "Width").ua_node

    async with connected(server, "visitor") as client:
        seen = client.get_node(node.nodeid)
        datatype = await seen.read_data_type()
        value_rank = await seen.read_value_rank()

    assert datatype == ua.NodeId(ua.Int32(ua.VariantType.UInt32.value))
    assert value_rank == ua.ValueRank.Scalar


async def test_a_parameter_is_writable_in_every_state(served) -> None:
    server, machine = served
    node = leaf(skill_of(machine, "PushMap").parameter_set, "Width").ua_node

    async with connected(server, "visitor") as client:
        access = await client.get_node(node.nodeid).get_access_level()

    assert ua.AccessLevel.CurrentWrite in access


async def test_the_direction_leaves_no_monitoring_and_no_result_of_its_own(served) -> None:
    _, machine = served
    skill = skill_of(machine, "Drive")

    assert not hasattr(skill, "monitoring")
    assert not hasattr(skill, "final_result_data")


async def test_the_skill_is_not_suspendable_and_says_so_by_absence(served) -> None:
    # Suspend must stop publishing: opensmi keeps the handler running through SUSPENDING.
    server, machine = served
    skill = skill_of(machine, "Drive")

    async with connected(server, "visitor") as client:
        children = await client.get_node(skill.ua_state_machine.nodeid).get_children()
        browsed = {(await child.read_browse_name()).Name for child in children}

    assert "Start" in browsed
    assert "Suspend" not in browsed


async def test_advertising_happens_at_init_and_publishing_does_not(served, runtime) -> None:
    _, machine = served

    assert skill_of(machine, "Drive").publisher is not None
    assert skill_of(machine, "PushMap").publisher is not None
    assert runtime.advertised_topics == ("/cmd_vel", "/map")
    assert runtime.published == ()


# -- the gates, which are opensmi's ---------------------------------------------------------------


async def test_a_skill_asks_for_the_skill_level(served) -> None:
    _, machine = served

    assert skill_of(machine, "Drive").minimum_access_level == SKILL_ACCESS_LEVEL == 10
    assert skill_of(machine, "PushMap").minimum_access_level == SKILL_ACCESS_LEVEL


async def test_a_reset_without_the_lock_answers_bad_requires_lock(served) -> None:
    server, machine = served
    skill = skill_of(machine, "PushMap")

    async with connected(server) as client:
        assert await press(client, server, skill, "Reset") == status(StatusCodes.BadRequiresLock)

    assert skill.current_state is SkillState.HALTED


async def test_a_start_without_the_lock_answers_bad_requires_lock_and_publishes_nothing(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine, "PushMap")
    await ready(skill)

    async with connected(server) as client:
        assert await press(client, server, skill, "Start") == status(StatusCodes.BadRequiresLock)

    assert skill.current_state is SkillState.READY
    assert runtime.published == ()


async def test_a_halt_without_the_lock_leaves_a_stream_running(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine, "Drive")
    await ready(skill)
    await skill.start()
    await reached(skill, SkillState.RUNNING)

    async with connected(server) as client:
        assert await press(client, server, skill, "Halt") == status(StatusCodes.BadRequiresLock)

    assert skill.current_state is SkillState.RUNNING
    await skill.halt()


async def test_a_user_below_the_skill_level_is_refused_even_holding_the_lock(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine, "PushMap")

    async with operating(server, machine, "planner") as client:
        assert await press(client, server, skill, "Reset") == status(StatusCodes.BadUserAccessDenied)

    assert skill.current_state is SkillState.HALTED
    assert runtime.published == ()


async def test_a_user_at_the_skill_level_holding_the_lock_runs_the_push(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine, "PushMap")

    async with operating(server, machine, "orchestrator") as client:
        assert await press(client, server, skill, "Reset") == GOOD
        await reached(skill, SkillState.READY)
        assert await press(client, server, skill, "Start") == GOOD
        await reached(skill, SkillState.COMPLETED)

    assert len(sent(runtime, "/map")) == 1


async def test_a_client_starts_and_halts_a_stream(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine, "Drive")

    async with operating(server, machine) as client:
        assert await press(client, server, skill, "Reset") == GOOD
        await reached(skill, SkillState.READY)
        assert await press(client, server, skill, "Start") == GOOD
        await until(lambda: len(sent(runtime, "/cmd_vel")) >= 2, "the stream published fewer than twice")
        assert await press(client, server, skill, "Halt") == GOOD
        await reached(skill, SkillState.HALTED)

    settled = len(sent(runtime, "/cmd_vel"))
    await asyncio.sleep(5 / FAST)
    assert len(sent(runtime, "/cmd_vel")) == settled


async def test_the_skill_level_is_the_one_the_ros2_settings_configure(runtime) -> None:
    async with serving(runtime, Platform(name="Platform"), ROS2Config(skill_access_level=20)) as (server, machine):
        skill = skill_of(machine, "PushMap")
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
    async with serving(runtime, Starting(name="Platform")) as (server, machine):
        skill = skill_of(machine, "PushMap")
        async with operating(server, machine) as client:
            assert await press(client, server, skill, "Reset") == GOOD
            await reached(skill, SkillState.READY)
            assert await press(client, server, skill, "Start") == status(StatusCodes.BadInvalidState)

        assert skill.current_state is SkillState.READY
        assert runtime.published == ()


# -- Start is the only thing that publishes -------------------------------------------------------


async def test_a_client_write_publishes_nothing_by_itself(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine, "PushMap")
    await ready(skill)

    async with operating(server, machine) as client:
        node = client.get_node(leaf(skill.parameter_set, "Width").ua_node.nodeid)
        await node.write_value(ua.Variant(4000, ua.VariantType.UInt32))
    await asyncio.sleep(0.05)

    assert await leaf(skill.parameter_set, "Width").read() == 4000
    assert runtime.published == ()


async def test_start_publishes_the_snapshot_a_client_wrote(served, runtime) -> None:
    _, machine = served
    skill = skill_of(machine, "PushMap")
    await ready(skill)
    await write_parameters(skill, {"Width": 4000})

    await skill.start()

    await reached(skill, SkillState.COMPLETED)
    (message,) = sent(runtime, "/map")
    assert message.width == 4000


async def test_a_finite_push_publishes_once_and_completes(served, runtime) -> None:
    _, machine = served
    skill = skill_of(machine, "PushMap")
    await ready(skill)

    await skill.start()
    await reached(skill, SkillState.COMPLETED)
    await asyncio.sleep(0.05)

    assert len(sent(runtime, "/map")) == 1


async def test_a_second_push_is_reset_write_start(served, runtime) -> None:
    _, machine = served
    skill = skill_of(machine, "PushMap")
    await ready(skill)
    await skill.start()
    await reached(skill, SkillState.COMPLETED)

    await ready(skill)
    await write_parameters(skill, {"Width": 7})
    await skill.start()
    await reached(skill, SkillState.COMPLETED)

    assert [message.width for message in sent(runtime, "/map")] == [0, 7]


async def test_the_parameters_survive_a_reset(served) -> None:
    _, machine = served
    skill = skill_of(machine, "PushMap")
    await ready(skill)
    await write_parameters(skill, {"Width": 11})

    await skill.start()
    await reached(skill, SkillState.COMPLETED)
    await ready(skill)

    assert await leaf(skill.parameter_set, "Width").read() == 11


# -- the stream, and the tick that is its only trigger --------------------------------------------


async def test_a_stream_publishes_on_every_tick_until_it_is_halted(served, runtime) -> None:
    _, machine = served
    skill = skill_of(machine, "Drive")
    await ready(skill)

    await skill.start()
    await reached(skill, SkillState.RUNNING)
    await until(lambda: len(sent(runtime, "/cmd_vel")) >= 3, "the stream published fewer than three times")

    await skill.halt()
    await reached(skill, SkillState.HALTED)
    settled = len(sent(runtime, "/cmd_vel"))
    await asyncio.sleep(5 / FAST)

    assert len(sent(runtime, "/cmd_vel")) == settled


async def test_every_tick_publishes_the_snapshot_as_it_stands_then(served, runtime) -> None:
    _, machine = served
    skill = skill_of(machine, "Drive")
    await ready(skill)
    await skill.start()
    await reached(skill, SkillState.RUNNING)
    await until(lambda: bool(sent(runtime, "/cmd_vel")), "the stream never published")

    await write_parameters(skill, {"X": 1.5})
    await until(
        lambda: any(message.x == 1.5 for message in sent(runtime, "/cmd_vel")),
        "the new value never reached the topic",
    )

    await skill.halt()


async def test_the_rate_is_a_client_writable_parameter_with_published_bounds(served) -> None:
    server, machine = served
    node = skill_of(machine, "Drive").parameter_set[RATE_PARAMETER].ua_node

    async with connected(server, "visitor") as client:
        seen = client.get_node(node.nodeid)
        limits = await (await seen.get_child("0:EURange")).read_value()
        access = await seen.get_access_level()
        value = await seen.read_value()

    assert value == FAST
    assert (limits.Low, limits.High) == (MINIMUM_RATE, MAXIMUM_RATE)
    assert ua.AccessLevel.CurrentWrite in access


async def test_the_rate_browses_as_rate_and_displays_as_the_reserved_name(served) -> None:
    _, machine = served
    node = skill_of(machine, "Drive").parameter_set[RATE_PARAMETER].ua_node

    assert (await node.read_browse_name()).Name == "Rate"
    assert (await node.read_display_name()).Text == "rate"


async def test_a_rate_of_zero_is_refused_where_the_write_arrives(served) -> None:
    server, machine = served
    node = skill_of(machine, "Drive").parameter_set[RATE_PARAMETER].ua_node

    async with operating(server, machine) as client, asyncio.timeout(SETTLED):
        with pytest.raises(ua.UaStatusCodeError):
            await client.get_node(node.nodeid).write_value(ua.Variant(0.0, ua.VariantType.Double))
        await client.get_node(node.nodeid).write_value(ua.Variant(5.0, ua.VariantType.Double))

    assert await skill_of(machine, "Drive").parameter_set[RATE_PARAMETER].read() == 5.0


async def test_a_new_rate_takes_effect_on_the_next_tick(runtime) -> None:
    async with serving(runtime, SlowPlatform(name="Platform")) as (_, machine):
        skill = skill_of(machine, "Crawl")
        await ready(skill)
        await skill.start()
        await reached(skill, SkillState.RUNNING)
        await until(lambda: bool(sent(runtime, "/crawl")), "the stream never published")

        await write_parameters(skill, {RATE_PARAMETER: FAST})
        await until(lambda: len(sent(runtime, "/crawl")) >= 4, "the faster rate never took effect")

        await skill.halt()


# -- a handler that returns, which is the one wrong way out of the loop ----------------------------


class Returning(ROS2ContinuousPublisherSkill):
    """A stream whose running handler returns at once, which is what the rate loop must never do."""

    async def _handle_running(self) -> None:
        return


async def test_a_continuous_handler_that_returns_leaves_the_skill_running(runtime) -> None:
    # opensmi neither completes nor halts a continuous Skill whose handler returns, so the loop must raise.
    class Quiet(Platform):
        bindings = (
            PublisherBinding(
                name="Quiet",
                path="/quiet",
                message_type="fixture_msgs/msg/Point",
                continuous=True,
                rate=FAST,
                skill_type=Returning,
            ),
        )

    async with serving(runtime, Quiet(name="Platform")) as (_, machine):
        skill = skill_of(machine, "Quiet")
        await ready(skill)

        await skill.start()
        await reached(skill, SkillState.RUNNING)
        await asyncio.sleep(0.1)

        assert skill.current_state is SkillState.RUNNING
        assert sent(runtime, "/quiet") == []
        await skill.halt()
        await reached(skill, SkillState.HALTED)


async def test_halting_a_stream_ends_in_halted_and_nothing_publishes_after(served, runtime) -> None:
    _, machine = served
    skill = skill_of(machine, "Drive")
    await ready(skill)
    await skill.start()
    await until(lambda: bool(sent(runtime, "/cmd_vel")), "the stream never published")

    await skill.halt()

    await reached(skill, SkillState.HALTED)
    settled = len(sent(runtime, "/cmd_vel"))
    await asyncio.sleep(5 / FAST)
    assert len(sent(runtime, "/cmd_vel")) == settled


# -- the header stamp ---------------------------------------------------------------------------------


async def test_the_header_timestamp_is_filled_by_the_adapter(runtime) -> None:
    runtime.set_clock(1_764_000_000, 250_000)
    async with serving(runtime, StampedPlatform(name="Platform")) as (_, machine):
        skill = skill_of(machine, "Goal")
        await ready(skill)
        await skill.start()
        await reached(skill, SkillState.COMPLETED)

        (message,) = sent(runtime, "/goal_pose")
        assert message.header.stamp == fixture_messages.Time(sec=1_764_000_000, nanosec=250_000)


async def test_a_stamp_a_client_wrote_itself_is_not_overwritten(runtime) -> None:
    runtime.set_clock(1_764_000_000, 250_000)
    async with serving(runtime, StampedPlatform(name="Platform")) as (_, machine):
        skill = skill_of(machine, "Goal")
        await ready(skill)
        await write_parameters(skill, {"Header.Stamp.Sec": 17})

        await skill.start()
        await reached(skill, SkillState.COMPLETED)

        (message,) = sent(runtime, "/goal_pose")
        assert message.header.stamp == fixture_messages.Time(sec=17, nanosec=0)


# -- failure, which the continuity answers differently ----------------------------------------------


async def test_a_finite_push_halts_on_any_publish_failure(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine, "PushMap")
    await ready(skill)
    runtime.refuse_publishing("/map")

    async with connected(server, "visitor") as client:
        collector = await alerts(client, server, machine)
        await skill.start()
        event = await collector.next_event()
        await reached(skill, SkillState.HALTED)

    assert event.ErrorCode == PUBLISH_FAILED
    assert sent(runtime, "/map") == []


async def test_a_stream_absorbs_the_declared_number_of_consecutive_failures(served, runtime) -> None:
    _, machine = served
    skill = skill_of(machine, "Drive")
    await ready(skill)
    runtime.refuse_publishing("/cmd_vel", times=TOLERATED)

    await skill.start()
    await reached(skill, SkillState.RUNNING)
    await until(lambda: bool(sent(runtime, "/cmd_vel")), "the stream never recovered")

    assert skill.current_state is SkillState.RUNNING
    assert skill.consecutive_failures == 0
    await skill.halt()


async def test_a_stream_halts_once_its_tolerance_is_spent_and_says_why(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine, "Drive")
    await ready(skill)
    runtime.refuse_publishing("/cmd_vel")

    async with connected(server, "visitor") as client:
        collector = await alerts(client, server, machine)
        await skill.start()
        event = await collector.next_event()
        await reached(skill, SkillState.HALTED)

    assert event.ErrorCode == PUBLISH_FAILED
    assert sent(runtime, "/cmd_vel") == []


async def test_a_reset_clears_the_failure_count(served, runtime) -> None:
    _, machine = served
    skill = skill_of(machine, "Drive")
    await ready(skill)
    runtime.refuse_publishing("/cmd_vel")
    await skill.start()
    await reached(skill, SkillState.HALTED)
    assert skill.consecutive_failures > 0

    await ready(skill)

    assert skill.consecutive_failures == 0


async def test_a_tolerated_failure_still_reaches_a_client_as_a_binding_event(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine, "Drive")
    await ready(skill)
    runtime.refuse_publishing("/cmd_vel", times=1)

    async with connected(server, "visitor") as client:
        collector = await binding_events(client, server, machine)
        await skill.start()
        event = await collector.next_event()

    assert (event.Code, event.Binding, event.ROS2Path) == (PUBLISH_FAILED, "Drive", "/cmd_vel")
    assert skill.current_state is SkillState.RUNNING
    await skill.halt()


async def test_a_value_that_is_not_a_list_of_poses_is_refused_on_the_write_and_never_reaches_a_message(
    runtime,
) -> None:
    async with serving(runtime, Planner(name="Platform")) as (server, machine):
        skill = skill_of(machine, "PushPlan")
        await ready(skill)
        async with operating(server, machine) as client:
            node = client.get_node(leaf(skill.parameter_set, "Poses").ua_node.nodeid)
            with pytest.raises(ua.uaerrors.BadTypeMismatch):
                await node.write_value(ua.Variant([1.0], ua.VariantType.Double))

        await skill.start()
        await reached(skill, SkillState.COMPLETED)
        assert [each.poses for each in sent(runtime, "/plan")] == [()]


# -- the Lock ending -----------------------------------------------------------------------------------


async def lock_method(client: Client, server: Server, machine: Any, verb: str, *arguments: object) -> None:
    """Call one Method of the Machine's Lock as a client."""
    namespace = server.ua_get_namespace_index(DiNodeIds)
    await client.get_node(machine.lock.ua_node.nodeid).call_method(f"{namespace}:{verb}", *arguments)


async def stream_started_by(client: Client, server: Server, machine: Any, runtime: FakeRuntime) -> Any:
    """Take the Lock as a client, start the Drive stream with it, and wait until it publishes."""
    skill = skill_of(machine, "Drive")
    await lock_method(client, server, machine, "InitLock", "the test suite")
    assert await press(client, server, skill, "Reset") == GOOD
    await reached(skill, SkillState.READY)
    assert await press(client, server, skill, "Start") == GOOD
    await until(lambda: len(sent(runtime, "/cmd_vel")) >= 2, "the stream published fewer than twice")
    return skill


async def falls_silent(runtime: FakeRuntime, topic: str = "/cmd_vel") -> None:
    """Assert that nothing more is published on a topic, over several ticks of the fastest stream."""
    settled = len(sent(runtime, topic))
    await asyncio.sleep(5 / FAST)
    assert len(sent(runtime, topic)) == settled


async def test_a_stream_halts_when_the_client_that_started_it_releases_the_lock(served, runtime) -> None:
    server, machine = served

    async with connected(server, "visitor") as watcher, connected(server, "orchestrator") as client:
        alerted = await alerts(watcher, server, machine)
        events = await binding_events(watcher, server, machine)
        skill = await stream_started_by(client, server, machine, runtime)

        await lock_method(client, server, machine, "ExitLock")

        await reached(skill, SkillState.HALTED)
        error = await alerted.next_event()
        event = await events.next_event()

    assert error.ErrorCode == LOCK_ENDED
    assert (event.Code, event.Binding, event.ROS2Path) == (LOCK_ENDED, "Drive", "/cmd_vel")
    await falls_silent(runtime)


async def test_a_stream_halts_when_another_client_breaks_the_lock(served, runtime) -> None:
    server, machine = served

    async with connected(server, "orchestrator") as client, connected(server, "operator") as breaker:
        skill = await stream_started_by(client, server, machine, runtime)

        await lock_method(breaker, server, machine, "BreakLock")

        await reached(skill, SkillState.HALTED)
        assert machine.lock.locking_user.name == "operator"
        await lock_method(breaker, server, machine, "ExitLock")
    await falls_silent(runtime)


async def test_a_stream_halts_when_the_lock_expires(served, runtime) -> None:
    server, machine = served

    async with connected(server, "orchestrator") as client:
        skill = await stream_started_by(client, server, machine, runtime)

        # opensmi checks the Lock for inactivity once a second, so the expiry lands within one sweep.
        server.config.access_control.max_inactive_lock_time_milliseconds = 1
        await reached(skill, SkillState.HALTED)

        assert not machine.lock.locked
    await falls_silent(runtime)


async def test_a_stream_halts_when_the_session_that_holds_the_lock_closes(served, runtime) -> None:
    server, machine = served

    async with connected(server, "orchestrator") as client:
        skill = await stream_started_by(client, server, machine, runtime)

    await reached(skill, SkillState.HALTED)
    await falls_silent(runtime)


async def test_renewing_the_lock_keeps_a_stream_running(served, runtime) -> None:
    server, machine = served

    async with connected(server, "orchestrator") as client:
        skill = await stream_started_by(client, server, machine, runtime)

        await lock_method(client, server, machine, "RenewLock")
        await asyncio.sleep(5 / FAST)

        assert skill.current_state is SkillState.RUNNING
        assert await press(client, server, skill, "Halt") == GOOD
        await lock_method(client, server, machine, "ExitLock")


async def test_a_stream_started_without_the_lock_is_not_tied_to_it(served) -> None:
    # The server's own interface starts a Skill while nobody holds the Lock, so there is no Lock to end.
    server, machine = served
    skill = skill_of(machine, "Drive")
    await ready(skill)
    await skill.start()
    await reached(skill, SkillState.RUNNING)

    async with operating(server, machine):
        pass
    await asyncio.sleep(5 / FAST)

    assert skill.current_state is SkillState.RUNNING
    await skill.halt()


async def test_the_lock_ending_leaves_a_finite_push_where_it_was(served, runtime) -> None:
    server, machine = served
    skill = skill_of(machine, "PushMap")

    async with connected(server, "orchestrator") as client:
        await lock_method(client, server, machine, "InitLock", "the test suite")
        assert await press(client, server, skill, "Reset") == GOOD
        await reached(skill, SkillState.READY)
        assert await press(client, server, skill, "Start") == GOOD
        await reached(skill, SkillState.COMPLETED)
        await lock_method(client, server, machine, "ExitLock")
        await asyncio.sleep(0.05)

    assert skill.current_state is SkillState.COMPLETED
    assert len(sent(runtime, "/map")) == 1


# -- a stale setpoint ----------------------------------------------------------------------------------


async def test_a_stream_whose_setpoint_went_stale_halts_itself(runtime) -> None:
    async with serving(runtime, WatchedPlatform(name="Platform")) as (server, machine):
        skill = skill_of(machine, "Watched")
        await ready(skill)

        async with connected(server, "visitor") as client:
            alerted = await alerts(client, server, machine)
            events = await binding_events(client, server, machine)
            await skill.start()

            await reached(skill, SkillState.HALTED)
            error = await alerted.next_event()
            event = await events.next_event()

        assert error.ErrorCode == SETPOINT_STALE
        assert (event.Code, event.Binding, event.ROS2Path) == (SETPOINT_STALE, "Watched", "/watched")
        await falls_silent(runtime, "/watched")


async def test_a_client_rewriting_the_same_value_keeps_its_stream_alive(runtime) -> None:
    async with serving(runtime, WatchedPlatform(name="Platform")) as (_, machine):
        skill = skill_of(machine, "Watched")
        await ready(skill)
        await skill.start()
        await reached(skill, SkillState.RUNNING)

        deadline = asyncio.get_running_loop().time() + 6 * STALE_AFTER
        while asyncio.get_running_loop().time() < deadline:
            await write_parameters(skill, {"X": 0.5})
            await asyncio.sleep(STALE_AFTER / 6)

        assert skill.current_state is SkillState.RUNNING
        await skill.halt()


async def test_a_stale_setpoint_halts_a_slow_stream_before_its_next_tick(runtime) -> None:
    # The interval is in seconds, not ticks: a stream ticking every two seconds halts well before its second tick.
    async with serving(runtime, CrawlingPlatform(name="Platform")) as (_, machine):
        skill = skill_of(machine, "Crawling")
        await ready(skill)
        loop = asyncio.get_running_loop()
        started = loop.time()
        await skill.start()

        await reached(skill, SkillState.HALTED)

        assert loop.time() - started < 1 / CRAWLING_RATE
        assert len(sent(runtime, "/crawling")) == 1


async def test_a_stream_without_stale_after_runs_on_without_writes(served) -> None:
    _, machine = served
    skill = skill_of(machine, "Drive")
    await ready(skill)
    await skill.start()

    await asyncio.sleep(4 * STALE_AFTER)

    assert skill.current_state is SkillState.RUNNING
    await skill.halt()


# -- the halt message ----------------------------------------------------------------------------------

HALT_TABLE = {"linear": {"x": 0.25}, "angular": {"z": -1}}

HALT_TWIST = fixture_messages.Twist(
    linear=fixture_messages.Vector3(x=0.25, y=0.0, z=0.0),
    angular=fixture_messages.Vector3(x=0.0, y=0.0, z=-1.0),
)


def halting(**declared: Any) -> TestComponent:
    """Return a Component carrying one Twist stream on /cmd_vel that declares a halt message."""
    keywords: dict[str, Any] = {"halt_message": HALT_TABLE, **declared}

    class Halting(TestComponent):
        bindings = (
            PublisherBinding(
                name="Drive",
                path="/cmd_vel",
                message_type="fixture_msgs/msg/Twist",
                continuous=True,
                rate=FAST,
                **keywords,
            ),
        )

    return Halting(name="Platform")


def halt_messages(runtime: FakeRuntime) -> list[Any]:
    """Return the halt messages that went out on /cmd_vel."""
    return [each for each in sent(runtime, "/cmd_vel") if each == HALT_TWIST]


async def ended_with_the_halt_message(runtime: FakeRuntime) -> None:
    """Assert that the stream fell silent and that its last message was its only halt message."""
    await falls_silent(runtime)
    assert sent(runtime, "/cmd_vel")[-1] == HALT_TWIST
    assert len(halt_messages(runtime)) == 1


async def test_a_clients_halt_ends_a_stream_with_its_halt_message_once(runtime) -> None:
    async with serving(runtime, halting()) as (server, machine):
        skill = skill_of(machine, "Drive")
        async with operating(server, machine) as client:
            assert await press(client, server, skill, "Reset") == GOOD
            await reached(skill, SkillState.READY)
            assert await press(client, server, skill, "Start") == GOOD
            await until(lambda: len(sent(runtime, "/cmd_vel")) >= 2, "the stream published fewer than twice")
            assert halt_messages(runtime) == []

            assert await press(client, server, skill, "Halt") == GOOD
            await reached(skill, SkillState.HALTED)

        await ended_with_the_halt_message(runtime)


async def test_the_end_of_the_lock_ends_a_stream_with_its_halt_message_once(runtime) -> None:
    async with serving(runtime, halting()) as (server, machine), connected(server, "orchestrator") as client:
        skill = await stream_started_by(client, server, machine, runtime)

        await lock_method(client, server, machine, "ExitLock")

        await reached(skill, SkillState.HALTED)
        await ended_with_the_halt_message(runtime)


async def test_a_stale_setpoint_ends_a_stream_with_its_halt_message_once(runtime) -> None:
    async with serving(runtime, halting(stale_after=STALE_AFTER)) as (_, machine):
        skill = skill_of(machine, "Drive")
        await ready(skill)
        await skill.start()

        await reached(skill, SkillState.HALTED)
        await ended_with_the_halt_message(runtime)


async def test_a_spent_tolerance_still_tries_the_halt_message_and_reports_that_it_failed(runtime) -> None:
    async with serving(runtime, halting()) as (server, machine):
        skill = skill_of(machine, "Drive")
        await ready(skill)
        runtime.refuse_publishing("/cmd_vel")

        async with connected(server, "visitor") as client:
            events = await binding_events(client, server, machine)
            await skill.start()
            await reached(skill, SkillState.HALTED)
            await until(lambda: len(events.events) >= 2, "the failed halt message was not reported")

        assert [event.Code for event in events.events] == [PUBLISH_FAILED, PUBLISH_FAILED]
        assert "halt message" in events.events[1].Message.Text
        assert sent(runtime, "/cmd_vel") == []


async def test_fields_the_halt_message_does_not_name_keep_the_message_types_defaults(runtime) -> None:
    async with serving(runtime, halting(halt_message={"angular": {"z": 0.5}})) as (_, machine):
        skill = skill_of(machine, "Drive")
        await ready(skill)
        await skill.start()
        await reached(skill, SkillState.RUNNING)
        await skill.halt()
        await reached(skill, SkillState.HALTED)
        await falls_silent(runtime)

        assert sent(runtime, "/cmd_vel")[-1] == fixture_messages.Twist(
            linear=fixture_messages.Vector3(x=0.0, y=0.0, z=0.0),
            angular=fixture_messages.Vector3(x=0.0, y=0.0, z=0.5),
        )


async def test_a_stream_without_a_halt_message_publishes_nothing_more_when_it_halts(served, runtime) -> None:
    _, machine = served
    skill = skill_of(machine, "Drive")
    await ready(skill)
    await write_parameters(skill, {"X": 0.5})
    await skill.start()
    await until(lambda: len(sent(runtime, "/cmd_vel")) >= 2, "the stream published fewer than twice")

    await skill.halt()
    await reached(skill, SkillState.HALTED)
    await falls_silent(runtime)

    assert {each.x for each in sent(runtime, "/cmd_vel")} == {0.5}


@pytest.mark.parametrize(
    ("table", "named"),
    [
        ({"linear": {"w": 1.0}}, "linear/w"),
        ({"speed": 1.0}, "speed"),
        ({"linear": {"x": "fast"}}, "linear/x"),
        ({"linear": 1.0}, "linear"),
    ],
)
async def test_a_halt_message_the_message_type_cannot_hold_refuses_to_boot(runtime, table: Any, named: str) -> None:
    with pytest.raises(DeclarationError, match=f"halt_message.*{named}"):
        await start(runtime, halting(halt_message=table))


# -- what is fatal at init -----------------------------------------------------------------------------


async def test_a_stale_after_refuses_to_boot_where_writes_are_not_timed(runtime) -> None:
    # asyncua's ServerTimestamp is the liveness signal and may be turned off.
    from opensmi.server import Server
    from opensmi.server.config import ServerConfiguration

    config = ServerConfiguration()
    config.ua_server.endpoint_address = "opc.tcp://127.0.0.1:0/server"
    server = Server(config=config)
    await server.init()
    try:
        server.ua_server.set_force_server_timestamp(False)
        machine = TestMachine(WatchedPlatform(name="Platform"), runtime=runtime)

        with pytest.raises(DeclarationError, match="stale_after"):
            await server.add_machine(machine)
    finally:
        await server.shutdown()


def _cannot_invert(_type: Any, _context: Any) -> tuple[()]:
    return ()


async def test_a_projection_that_cannot_invert_refuses_to_boot(runtime) -> None:
    class Dropping(TestComponent):
        bindings = (DRIVE,)
        projections = (Functions("fixture_msgs/msg/Point", shaped_by=_cannot_invert, decoded_by=_empty_decode),)

    with pytest.raises(NotInvertibleError, match="Functions"):
        await start(runtime, Dropping(name="Platform"))


def _empty_decode(_message: Any, _context: Any) -> dict[str, object]:
    return {}


def _a_rate(_message_type: Any, _context: Any) -> tuple[PlanVariable, ...]:
    return (PlanVariable(name=RATE_PARAMETER, source="rate", initial=0.0, variant_type=ua.VariantType.Double),)


async def test_a_curated_field_called_rate_refuses_to_boot_a_stream(runtime) -> None:
    class Colliding(TestComponent):
        projections = (
            Functions(
                "fixture_msgs/msg/Ramp",
                shaped_by=_a_rate,
                decoded_by=lambda *_: {},
                encode=lambda message_type, *_: cast("Any", message_type)(rate=0.0, limit=0.0),
            ),
        )
        bindings = (
            PublisherBinding(
                name="Ramp",
                path="/ramp",
                message_type="fixture_msgs/msg/Ramp",
                continuous=True,
                rate=SLOW,
            ),
        )

    with pytest.raises(DeclarationError, match=RATE_PARAMETER):
        await start(runtime, Colliding(name="Platform"))


async def test_a_message_field_called_rate_converts_onto_the_reserved_name_and_is_refused_with_a_hint(runtime) -> None:
    class Colliding(TestComponent):
        bindings = (
            PublisherBinding(
                name="Ramp", path="/ramp", message_type="fixture_msgs/msg/Ramp", continuous=True, rate=SLOW
            ),
        )

    with pytest.raises(DeclarationError, match="rename"):
        await start(runtime, Colliding(name="Platform"))


async def test_the_same_field_is_fine_on_a_finite_push(runtime) -> None:
    class PushingRamp(TestComponent):
        bindings = (
            PublisherBinding(name="Ramp", path="/ramp", message_type="fixture_msgs/msg/Ramp", continuous=False),
        )

    async with serving(runtime, PushingRamp(name="Platform")) as (_, machine):
        assert names(skill_of(machine, "Ramp").parameter_set) == ["Limit", "Rate"]


async def test_a_type_whose_interface_package_is_absent_refuses_to_boot(runtime) -> None:
    class UnknownType(TestComponent):
        bindings = (
            PublisherBinding(
                name="Nowhere",
                path="/nowhere",
                message_type="fixture_msgs/msg/Absent",
                continuous=False,
            ),
        )

    with pytest.raises(MissingInterfacePackageError):
        await start(runtime, UnknownType(name="Platform"))


async def test_a_selection_that_leaves_the_message_unbuildable_refuses_to_boot(runtime) -> None:
    class Narrowed(TestComponent):
        bindings = (
            PublisherBinding(
                name="Drive",
                path="/cmd_vel",
                message_type="fixture_msgs/msg/Point",
                continuous=False,
                exclude=("z",),
            ),
        )

    with pytest.raises(NotInvertibleError, match="z"):
        await start(runtime, Narrowed(name="Platform"))


async def test_a_refused_declaration_leaves_no_endpoint_on_the_graph(runtime) -> None:
    class Narrowed(TestComponent):
        bindings = (
            PublisherBinding(
                name="Drive",
                path="/cmd_vel",
                message_type="fixture_msgs/msg/Point",
                continuous=False,
                exclude=("z",),
            ),
        )

    with pytest.raises(NotInvertibleError):
        await start(runtime, Narrowed(name="Platform"))

    assert runtime.advertised_topics == ()


async def test_imperative_publishing_is_ordinary_runtime_use_and_declares_nothing(served, runtime) -> None:
    _, machine = served

    publisher = await machine.runtime.advertise(topic="/anywhere", message_type="fixture_msgs/msg/Point")
    await publisher.publish(fixture_messages.Point(x=1.0, y=2.0, z=3.0))
    await publisher.close()

    assert sent(runtime, "/anywhere") == [fixture_messages.Point(x=1.0, y=2.0, z=3.0)]
    assert [binding.name for binding in machine.added[0].ros2_bindings] == ["Drive", "PushMap"]


# -- the custom implementation class, and the teardown -----------------------------------------------


class CountingDrive(ROS2ContinuousPublisherSkill):
    """A custom class that adds behaviour without giving up the declaration."""

    built = 0

    async def build_message(self, values: Mapping[str, object]) -> Any:
        self.built += 1
        return await super().build_message(values)


async def test_a_declaration_may_name_a_subclass_of_the_stock_skill(runtime) -> None:
    class Custom(TestComponent):
        bindings = (
            PublisherBinding(
                name="Drive",
                path="/cmd_vel",
                message_type="fixture_msgs/msg/Point",
                continuous=True,
                rate=SLOW,
                skill_type=CountingDrive,
            ),
        )

    async with serving(runtime, Custom(name="Platform")) as (_, machine):
        skill = skill_of(machine)

        assert isinstance(skill, CountingDrive)
        assert skill.built == 1


async def test_a_message_built_by_the_declarations_own_function_is_the_one_published(runtime) -> None:
    def shouting(message_type: Any, values: Mapping[str, object], _context: Any) -> Any:
        return message_type(x=100.0, y=values["Y"], z=0.0)

    class Custom(TestComponent):
        bindings = (
            PublisherBinding(
                name="Push",
                path="/setpoint",
                message_type="fixture_msgs/msg/Point",
                continuous=False,
                message_from=shouting,
            ),
        )

    async with serving(runtime, Custom(name="Platform")) as (_, machine):
        skill = skill_of(machine, "Push")
        await ready(skill)
        await write_parameters(skill, {"Y": 2.0})
        await skill.start()
        await reached(skill, SkillState.COMPLETED)

        assert sent(runtime, "/setpoint") == [fixture_messages.Point(x=100.0, y=2.0, z=0.0)]


async def test_shutting_the_component_down_stops_advertising(served, runtime) -> None:
    _, machine = served
    skill = skill_of(machine)
    publisher = skill.publisher
    assert publisher is not None

    await machine.added[0].shutdown()

    assert publisher.closed
    assert skill.publisher is None
    assert runtime.advertised_topics == ()


async def test_a_component_with_no_publisher_declared_gains_no_skill(runtime) -> None:
    async with serving(runtime, TestComponent(name="Platform")) as (_, machine):
        assert len(machine.added[0].skill_set) == 0


# -- the Components the tests above serve ----------------------------------------------------------------

WATCHED_RATE = 20.0
STALE_AFTER = 0.3
CRAWLING_RATE = 0.5


class SlowPlatform(TestComponent):
    bindings = (
        PublisherBinding(
            name="Crawl",
            path="/crawl",
            message_type="fixture_msgs/msg/Point",
            continuous=True,
            rate=SLOW,
        ),
    )


class StampedPlatform(TestComponent):
    bindings = (
        PublisherBinding(
            name="Goal",
            path="/goal_pose",
            message_type="fixture_msgs/msg/PoseStamped",
            continuous=False,
        ),
    )


class WatchedPlatform(TestComponent):
    bindings = (
        PublisherBinding(
            name="Watched",
            path="/watched",
            message_type="fixture_msgs/msg/Point",
            continuous=True,
            rate=WATCHED_RATE,
            stale_after=STALE_AFTER,
        ),
    )


class CrawlingPlatform(TestComponent):
    bindings = (
        PublisherBinding(
            name="Crawling",
            path="/crawling",
            message_type="fixture_msgs/msg/Point",
            continuous=True,
            rate=CRAWLING_RATE,
            stale_after=STALE_AFTER,
        ),
    )


# -- a list whose length is the client's ----------------------------------------------------------------


class Planner(TestComponent):
    bindings = (PUSH_PLAN,)


def waypoints(machine: Any, *xs: float) -> Mapping[str, object]:
    """Return the values a client writes to the Parameter a message's pose list becomes."""
    message = fixture_messages.Path(
        header=fixture_messages.Header(stamp=fixture_messages.Time(sec=0, nanosec=0), frame_id="map"),
        poses=tuple(fixture_messages.pose_stamped(x=each) for each in xs),
    )
    context = ProjectionContext(
        registry=machine.added[0].ros2_projections,
        selected=resolve_selection(fixture_messages.Path, FieldSelection()),
    )
    return context.registry.find("fixture_msgs/msg/Path").decode(message, context)


async def test_a_client_writes_a_list_of_two_and_a_list_of_twenty_to_the_same_parameters(runtime) -> None:
    async with serving(runtime, Planner(name="Platform")) as (_, machine):
        skill = skill_of(machine, "PushPlan")

        for count in (2, 20):
            await ready(skill)
            await write_parameters(skill, waypoints(machine, *(float(each) for each in range(count))))
            await skill.start()
            await reached(skill, SkillState.COMPLETED)

    two, twenty = sent(runtime, "/plan")
    assert [each.pose.position.x for each in two.poses] == [0.0, 1.0]
    assert [each.pose.position.x for each in twenty.poses] == [float(each) for each in range(20)]
    assert twenty.poses[3].header.frame_id == "map"


# -- values a client wrote that cannot become a message ---------------------------------------------


def _parse_text(message_type: Any, values: Any, _context: Any) -> Any:
    """Read the String back as a number, refusing what is not one -- as a codec refuses a body."""
    return message_type(x=float(values["x"]), y=0.0, z=0.0)


def _as_numeric_text(_message_type: Any, context: Any) -> tuple[PlanVariable, ...]:
    """Render Point's x as a String whose untouched default is a number, so init can publish it."""
    return (
        PlanVariable(name="x", source=join_path(context.path, "x"), initial="0", variant_type=ua.VariantType.String),
    )


class TextPoint(TestComponent):
    bindings = (
        PublisherBinding(name="PushPoint", path="/point", message_type="fixture_msgs/msg/Point", continuous=False),
    )
    projections = (
        Functions("fixture_msgs/msg/Point", shaped_by=_as_numeric_text, decoded_by=lambda *_: {}, encode=_parse_text),
    )


async def test_parameters_that_cannot_become_a_message_halt_the_skill_with_an_error_code(runtime) -> None:
    async with serving(runtime, TextPoint(name="Platform")) as (server, machine):
        skill = skill_of(machine, "PushPoint")
        await ready(skill)
        await write_parameters(skill, {"x": "north"})

        async with connected(server, "visitor") as client:
            collector = await alerts(client, server, machine)
            await skill.start()
            event = await collector.next_event()
            await reached(skill, SkillState.HALTED)

    assert event.ErrorCode == MESSAGE_UNBUILDABLE
    assert "north" in event.Message.Text
    assert sent(runtime, "/point") == []
