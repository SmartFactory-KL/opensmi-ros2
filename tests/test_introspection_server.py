# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The introspection Component answers at the listing level and its mutators refuse.

Access is asserted through a real client: `visitor` (level 0, no Lock) may read the listings
, `operator` may take the Lock.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, Any, cast

import pytest
from asyncua import ua
from asyncua.ua.status_codes import StatusCodes
from opensmi.core.nodesets import DiNodeIds, SmartFactoryMachineSetNodeIds, SmartFactorySkillSetNodeIds

from opensmi.ros2.runtime.graph import GraphSnapshot
from opensmi.ros2.runtime.parameters import ParameterDescription, ParameterType, ParameterValue
from opensmi.ros2.server.access import LISTING_ACCESS_LEVEL, MUTATOR_ACCESS_LEVEL
from opensmi.ros2.server.bindings import MonitoringBinding
from opensmi.ros2.server.config import ROS2Config
from opensmi.ros2.server.events import Severity
from opensmi.ros2.server.introspection import INTROSPECTION_NAME, ROS2IntrospectionComponent, ROS2ListingMethod
from tests import fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.served import TestComponent, TestMachine, alone, connected, leaf, monitoring_nodes, serve

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from asyncua.client.client import Client
    from opensmi.server import Server

    from opensmi.ros2.runtime.messages import Message

MESSAGE_TYPES = {
    "fixture_msgs/msg/Pose": fixture_messages.Pose,
    "fixture_msgs/msg/Mode": fixture_messages.Mode,
}

SERVICE_TYPES = {"fixture_msgs/srv/SetMode": fixture_messages.SetMode}

GRAPH = GraphSnapshot(
    topics={
        "/current_pose": ("fixture_msgs/msg/Pose",),
        "/mode": ("fixture_msgs/msg/Mode",),
        "/confused": ("fixture_msgs/msg/Pose", "fixture_msgs/msg/Mode"),
    },
    services={"/set_mode": ("fixture_msgs/srv/SetMode",)},
    actions={"/navigate": ("fixture_msgs/action/Navigate",)},
)

POSE = MonitoringBinding(
    name="CurrentPose",
    path="/current_pose",
    message_type="fixture_msgs/msg/Pose",
    required=True,
)

LOOK = {"Name": "Look", "Topic": "/mode", "MessageType": "fixture_msgs/msg/Mode"}


class Platform(TestComponent):
    bindings = (POSE,)


def answered(_request: Message) -> Message:
    return fixture_messages.SetModeResponse(success=True, message="fine", applied=3)


@pytest.fixture
def runtime() -> FakeRuntime:
    started = FakeRuntime(
        domain_id=7,
        message_types=MESSAGE_TYPES,
        service_types=SERVICE_TYPES,
        graph=GRAPH,
    )
    started.serve_service("/set_mode", answered)
    started.serve_parameters(
        "/controller_server",
        {"use_sim_time": ParameterValue(type=ParameterType.BOOL, value=False), "unset": None},
    )
    started.describe_parameter(
        "/controller_server",
        "use_sim_time",
        ParameterDescription(
            name="use_sim_time",
            present=True,
            type=ParameterType.BOOL,
            description="whether to take time from /clock",
            additional_constraints="none at all",
            read_only=True,
            dynamic_typing=False,
        ),
    )
    return started


def robot(runtime: FakeRuntime, config: ROS2Config | None = None, machine_type: type[TestMachine] = TestMachine) -> Any:
    return machine_type(Platform(name="Platform"), runtime=runtime, ros2_config=config or ROS2Config())


@pytest.fixture
async def served(runtime: FakeRuntime) -> AsyncIterator[tuple[Server, Any]]:
    machine = robot(runtime)
    server = await serve(machine)
    try:
        yield server, machine
    finally:
        await server.shutdown()


def introspection(machine: Any) -> ROS2IntrospectionComponent:
    return cast("ROS2IntrospectionComponent", machine.components[INTROSPECTION_NAME])


def method_of(machine: Any, name: str) -> ROS2ListingMethod:
    return cast("ROS2ListingMethod", introspection(machine).method_set[name])


def platform(machine: Any) -> Any:
    return machine.added[0]


def skill_set_name(server: Server, name: str) -> str:
    return f"{server.ua_get_namespace_index(SmartFactorySkillSetNodeIds)}:{name}"


async def occupy(server: Server, client: Client, machine: Any) -> None:
    """Take the Machine's Lock, which every mutator needs as well as an access level."""
    await alone(server, client)
    await _lock_method(server, client, machine, "InitLock", "the test suite")


async def release(server: Server, client: Client, machine: Any) -> None:
    """Give the Machine's Lock back, so that the next session can take it at once."""
    await _lock_method(server, client, machine, "ExitLock")


async def _lock_method(server: Server, client: Client, machine: Any, name: str, *arguments: Any) -> None:
    lock = client.get_node(machine.lock.ua_node.nodeid)
    method = await lock.get_child(f"{server.ua_get_namespace_index(DiNodeIds)}:{name}")
    await lock.call_method(method.nodeid, *arguments)


async def called(
    server: Server,
    machine: Any,
    name: str,
    *,
    user: str = "visitor",
    lock: bool = False,
    arguments: dict[str, Any] | None = None,
) -> int:
    """Call one Method the way a client does, and return the status code it answered with.

    **The arguments are written through the same session as the Call**, deliberately. A Method
    here takes its arguments as Parameters, so *using* one is a write and then a call, and the
    write is gated separately -- by the variable rather than by the Method. Writing them
    server-side would test half of the claim and hide the half that is easy to get wrong.

    A Lock taken here is given back before the session closes: opensmi notices a closed session
    only on its next sweep, and the next call in a test would find the Lock still taken.
    """
    async with connected(server, user) as client:
        if not lock:
            return await call_with(server, client, machine, name, arguments)
        await occupy(server, client, machine)
        try:
            return await call_with(server, client, machine, name, arguments)
        finally:
            await release(server, client, machine)


async def call_with(
    server: Server,
    client: Client,
    machine: Any,
    name: str,
    arguments: dict[str, Any] | None = None,
) -> int:
    """Write one Method's arguments and call it through a session the caller holds."""
    method = method_of(machine, name)
    for argument, value in (arguments or {}).items():
        node = client.get_node(method.parameter_set[argument].ua_node.nodeid)
        try:
            await node.write_value(value)
        except ua.UaStatusCodeError as refusal:
            return refusal.code
    call = await method.ua_node.get_child(skill_set_name(server, "Call"))
    try:
        await client.get_node(method.ua_node.nodeid).call_method(call.nodeid)
    except ua.UaStatusCodeError as answer:
        return answer.code
    return StatusCodes.Good


async def entries(machine: Any, name: str) -> list[str]:
    return list(await leaf(method_of(machine, name).final_result_data, "Entries").read())


async def answer(server: Server, machine: Any, name: str, **arguments: Any) -> list[str]:
    """Call one read-only Method as the level-zero user and return the lines it wrote."""
    assert await called(server, machine, name, arguments=arguments or None) == StatusCodes.Good
    return await entries(machine, name)


async def min_access_level(server: Server, method: Any) -> int:
    node = await method.ua_node.get_child(skill_set_name(server, "MinAccessLevel"))
    return int(await node.read_value())


# -- what a client browses ---------------------------------------------------------------------

THE_NINE = [
    "ListTopics",
    "ListServices",
    "ListActions",
    "ListROS2Parameters",
    "DescribeROS2Parameter",
    "ListBindings",
    "ListRecentEvents",
    "BindTopic",
    "CallService",
]

LISTINGS = THE_NINE[:7]
MUTATORS = THE_NINE[7:]


async def test_every_machine_carries_one_introspection_component(served) -> None:
    _, machine = served

    assert isinstance(introspection(machine), ROS2IntrospectionComponent)


async def test_it_browses_ahead_of_whatever_the_machine_declares(served) -> None:
    _, machine = served

    assert [each.name for each in machine.components] == [INTROSPECTION_NAME, "Platform"]


async def test_the_component_carries_the_nine_methods_in_browse_order(served) -> None:
    _, machine = served

    assert [each.name for each in introspection(machine).method_set] == THE_NINE


async def test_a_listing_answers_with_a_count_and_an_array_of_lines(served) -> None:
    server, machine = served
    method = method_of(machine, "ListTopics")

    async with connected(server, "visitor") as client:
        entries_node = client.get_node(leaf(method.final_result_data, "Entries").ua_node.nodeid)
        rank = await entries_node.read_value_rank()

    assert sorted(each.name for each in method.final_result_data) == ["Count", "Entries"]
    assert rank == ua.ValueRank.OneDimension


# -- the access table, which is the whole point of this file ------------------------------------


async def test_the_read_only_methods_advertise_the_listing_level(served) -> None:
    server, machine = served

    for name in LISTINGS:
        assert await min_access_level(server, method_of(machine, name)) == LISTING_ACCESS_LEVEL, name


async def test_the_mutators_advertise_the_deployments_level(served) -> None:
    server, machine = served

    for name in MUTATORS:
        assert await min_access_level(server, method_of(machine, name)) == MUTATOR_ACCESS_LEVEL, name


async def test_a_listing_answers_at_level_zero_without_the_lock(served) -> None:
    # The exception: a locked listing would be unreachable while the orchestrator holds the Lock.
    server, machine = served

    assert await called(server, machine, "ListTopics", user="visitor") == StatusCodes.Good


async def test_a_listing_answers_while_somebody_else_holds_the_lock(served) -> None:
    server, machine = served

    async with connected(server, "operator") as holder:
        await occupy(server, holder, machine)

        assert await called(server, machine, "ListTopics", user="visitor") == StatusCodes.Good


async def test_every_read_only_method_is_reachable_at_level_zero(served) -> None:
    server, machine = served
    filled = {
        "ListROS2Parameters": {"Node": "/controller_server"},
        "DescribeROS2Parameter": {"Node": "/controller_server", "Name": "use_sim_time"},
    }

    for name in LISTINGS:
        assert await called(server, machine, name, user="visitor", arguments=filled.get(name)) == StatusCodes.Good, name


async def test_a_level_zero_client_can_write_a_listings_arguments(served) -> None:
    server, machine = served
    node = leaf(method_of(machine, "ListTopics").parameter_set, "Filter").ua_node

    async with connected(server, "visitor") as client:
        await client.get_node(node.nodeid).write_value("pose")

    assert await called(server, machine, "ListTopics") == StatusCodes.Good
    assert await entries(machine, "ListTopics") == ["/current_pose  fixture_msgs/msg/Pose"]


async def test_a_level_zero_client_cannot_write_a_mutators_arguments(served) -> None:
    server, machine = served
    node = leaf(method_of(machine, "BindTopic").parameter_set, "Topic").ua_node

    async with connected(server, "visitor") as client:
        with pytest.raises(ua.UaStatusCodeError) as refusal:
            await client.get_node(node.nodeid).write_value("/mode")

    assert refusal.value.code == StatusCodes.BadUserAccessDenied


async def test_a_mutator_refuses_the_level_zero_user(served) -> None:
    server, machine = served

    assert await called(server, machine, "BindTopic", user="visitor") == StatusCodes.BadUserAccessDenied


async def test_a_mutator_refuses_a_high_enough_user_who_has_not_taken_the_lock(served) -> None:
    # Two gates: operator clears the level and is still refused without the Lock.
    server, machine = served

    assert await called(server, machine, "BindTopic", user="operator") == StatusCodes.BadRequiresLock


async def test_a_mutator_refuses_a_user_holding_the_lock_below_its_level(served) -> None:
    server, machine = served

    status = await called(server, machine, "BindTopic", user="orchestrator", lock=True, arguments=LOOK)

    assert status == StatusCodes.BadUserAccessDenied
    assert monitoring_nodes(introspection(machine)) == {}


async def test_a_refused_call_is_refused_on_use_and_never_hidden(served) -> None:
    server, machine = served
    method = method_of(machine, "BindTopic")

    async with connected(server, "visitor") as client:
        node = client.get_node(method.ua_node.nodeid)
        level = await node.get_child(skill_set_name(server, "MinAccessLevel"))

        assert (await node.read_browse_name()).Name == "BindTopic"
        assert await level.read_value() == MUTATOR_ACCESS_LEVEL


async def test_every_level_is_configurable(runtime) -> None:
    config = ROS2Config(listing_access_level=10, introspection_access_level=15)
    machine = robot(runtime, config)
    server = await serve(machine)
    try:
        assert await min_access_level(server, method_of(machine, "ListTopics")) == 10
        assert await min_access_level(server, method_of(machine, "BindTopic")) == 15
        assert await called(server, machine, "ListTopics", user="visitor") == StatusCodes.BadUserAccessDenied
        assert await called(server, machine, "ListTopics", user="orchestrator") == StatusCodes.Good
    finally:
        await server.shutdown()


class Heard:
    """An asyncua subscription handler that keeps the events it was sent."""

    def __init__(self) -> None:
        self.events: list[Any] = []

    def event_notification(self, event: Any) -> None:
        self.events.append(event)


async def test_a_refusal_names_the_method_that_refused_as_the_source_of_its_notification(runtime) -> None:
    machine = robot(runtime, ROS2Config(listing_access_level=10))
    server = await serve(machine)
    heard = Heard()
    try:
        async with connected(server, "visitor") as client:
            subscription = await client.create_subscription(20, heard)
            await subscription.subscribe_events(
                client.get_node(introspection(machine).notification.ua_node.nodeid),
                client.get_node(server.ua_get_node_id(SmartFactoryMachineSetNodeIds.AlertType)),
            )
            method = method_of(machine, "ListTopics")
            assert await call_with(server, client, machine, "ListTopics") == StatusCodes.BadUserAccessDenied
            for _ in range(200):
                if heard.events:
                    break
                await asyncio.sleep(0.01)
    finally:
        await server.shutdown()

    (event,) = heard.events
    assert event.SourceNode == method.ua_node.nodeid
    assert event.SourceName == method.name
    assert "ListTopics" in event.Message.Text
    assert "|||" not in event.Message.Text


async def test_a_listings_arguments_follow_the_configured_listing_level(runtime) -> None:
    machine = robot(runtime, ROS2Config(listing_access_level=10))
    server = await serve(machine)
    try:
        status = await called(server, machine, "ListTopics", user="visitor", arguments={"Filter": "pose"})

        assert status == StatusCodes.BadUserAccessDenied
    finally:
        await server.shutdown()


def test_a_mutator_level_below_the_skills_is_refused_where_it_was_typed() -> None:
    from opensmi.ros2.server.errors import DeclarationError

    with pytest.raises(DeclarationError, match="introspection_access_level"):
        ROS2Config(introspection_access_level=0)


# -- the four listings -------------------------------------------------------------------------


async def test_listing_topics_answers_what_is_on_the_graph(served) -> None:
    server, machine = served

    lines = await answer(server, machine, "ListTopics")

    assert lines[0] == "/confused  fixture_msgs/msg/Pose, fixture_msgs/msg/Mode"
    assert "/current_pose  fixture_msgs/msg/Pose" in lines


async def test_a_listing_narrows_by_a_substring(served) -> None:
    server, machine = served

    assert await answer(server, machine, "ListTopics", Filter="pose") == ["/current_pose  fixture_msgs/msg/Pose"]


async def test_the_count_beside_the_lines_says_how_many_there_are(served) -> None:
    server, machine = served

    lines = await answer(server, machine, "ListTopics")

    assert await leaf(method_of(machine, "ListTopics").final_result_data, "Count").read() == len(lines)


async def test_listing_services_and_actions_answer_their_own_vocabularies(served) -> None:
    server, machine = served

    assert await answer(server, machine, "ListServices") == ["/set_mode  fixture_msgs/srv/SetMode"]
    assert await answer(server, machine, "ListActions") == ["/navigate  fixture_msgs/action/Navigate"]


async def test_a_listing_leaves_the_graph_poll_stopped(served, runtime) -> None:
    server, machine = served

    await answer(server, machine, "ListTopics")

    assert not runtime.graph.polling


async def test_listing_parameters_asks_the_node_rather_than_the_graph(served) -> None:
    server, machine = served

    assert await answer(server, machine, "ListROS2Parameters", Node="/controller_server") == ["use_sim_time", "unset"]


async def test_a_listing_with_no_argument_is_refused_as_a_bad_argument(served) -> None:
    server, machine = served

    status = await called(server, machine, "ListROS2Parameters", arguments={"Node": ""})

    assert status == StatusCodes.BadInvalidArgument


async def test_describing_a_parameter_carries_the_nodes_own_prose(served) -> None:
    server, machine = served

    lines = await answer(server, machine, "DescribeROS2Parameter", Node="/controller_server", Name="use_sim_time")

    assert lines[:2] == ["name: use_sim_time", "type: bool"]
    assert "description: whether to take time from /clock" in lines
    assert "read_only: True" in lines


async def test_describing_a_name_the_node_does_not_have_says_so(served) -> None:
    server, machine = served

    lines = await answer(server, machine, "DescribeROS2Parameter", Node="/controller_server", Name="typo")

    assert lines == ["/controller_server does not declare 'typo'"]


async def test_a_node_that_does_not_answer_comes_back_as_a_real_status_code(served) -> None:
    server, machine = served

    status = await called(server, machine, "ListROS2Parameters", arguments={"Node": "/nobody_is_home"})

    assert status == StatusCodes.BadNotConnected


# -- the provenance report ----------------------------------------------------------------------


async def test_the_report_names_every_binding_on_the_machine(served) -> None:
    server, machine = served

    lines = await answer(server, machine, "ListBindings")

    assert any("Platform/CurrentPose [MonitoringBinding] /current_pose" in each for each in lines)


async def test_the_report_carries_required_satisfied_and_waiting_since(served) -> None:
    server, machine = served

    line = next(each for each in await answer(server, machine, "ListBindings") if "current_pose" in each)

    assert "required, absent since " in line


async def test_a_binding_whose_topic_has_a_publisher_reads_satisfied(served, runtime) -> None:
    server, machine = served
    runtime.note_publishers("/current_pose", 1)
    for _ in range(200):
        if platform(machine).ros2_monitoring_presence.get("CurrentPose"):
            break
        await asyncio.sleep(0.005)

    line = next(each for each in await answer(server, machine, "ListBindings") if "current_pose" in each)

    assert "required, satisfied;" in line
    assert "since" not in line


async def test_the_report_carries_each_bindings_qos_provenance(served) -> None:
    server, machine = served

    line = next(each for each in await answer(server, machine, "ListBindings") if "current_pose" in each)

    assert "QoS: discovered " in line


async def test_the_report_narrows_by_a_substring(served) -> None:
    server, machine = served
    await called(server, machine, "BindTopic", user="operator", lock=True, arguments=LOOK)

    narrowed = await answer(server, machine, "ListBindings", Filter="Look")

    assert len(narrowed) == 1
    assert "Look" in narrowed[0]


# -- the recent-events listing --------------------------------------------------------------------


async def test_a_fired_binding_event_can_be_read_back_after_the_moment(served) -> None:
    server, machine = served
    await platform(machine).emit_binding_event(
        binding="CurrentPose",
        path="/current_pose",
        code="QOS_INCOMPATIBLE",
        severity=Severity.WARNING,
        text="a publisher offers QoS this subscription cannot accept",
    )

    lines = await answer(server, machine, "ListRecentEvents")

    assert any("QOS_INCOMPATIBLE" in each and "current_pose" in each for each in lines)


async def test_the_recent_events_listing_narrows_by_code(served) -> None:
    server, machine = served
    for code in ("QOS_INCOMPATIBLE", "PUBLISH_FAILED"):
        await platform(machine).emit_binding_event(
            binding="CurrentPose",
            path="/current_pose",
            code=code,
            severity=Severity.WARNING,
        )

    lines = await answer(server, machine, "ListRecentEvents", Code="PUBLISH_FAILED")

    assert len(lines) == 1
    assert "PUBLISH_FAILED" in lines[0]


async def test_the_recent_events_listing_answers_the_last_n(served) -> None:
    server, machine = served
    for index in range(4):
        await platform(machine).emit_binding_event(
            binding=f"binding_{index}",
            path="/topic",
            code="KEY_COLLISION",
            severity=Severity.WARNING,
        )

    lines = await answer(server, machine, "ListRecentEvents", Newest=ua.Variant(2, ua.VariantType.Int32))

    assert len(lines) == 2
    assert "binding_3" in lines[-1]


# -- the ad-hoc surface ----------------------------------------------------------------------------


async def test_binding_a_topic_creates_nodes_under_the_components_monitoring(served) -> None:
    server, machine = served
    component = introspection(machine)

    status = await called(server, machine, "BindTopic", user="operator", lock=True, arguments=LOOK)

    assert status == StatusCodes.Good
    assert sorted(monitoring_nodes(component)) == ["Look.Detail", "Look.Mode"]
    assert component.ad_hoc_bindings == frozenset({"Look"})


async def test_an_ad_hoc_binding_takes_its_type_from_the_graph_when_none_is_named(served) -> None:
    server, machine = served

    status = await called(
        server,
        machine,
        "BindTopic",
        user="operator",
        lock=True,
        arguments={**LOOK, "MessageType": ""},
    )

    assert status == StatusCodes.Good
    assert sorted(monitoring_nodes(introspection(machine))) == ["Look.Detail", "Look.Mode"]


async def test_a_topic_carrying_two_types_is_refused_rather_than_guessed_at(served) -> None:
    server, machine = served

    status = await called(
        server,
        machine,
        "BindTopic",
        user="operator",
        lock=True,
        arguments={"Name": "Look", "Topic": "/confused", "MessageType": ""},
    )

    assert status == StatusCodes.BadInvalidArgument
    assert monitoring_nodes(introspection(machine)) == {}


async def test_an_ad_hoc_binding_reaches_the_report(served) -> None:
    server, machine = served
    await called(server, machine, "BindTopic", user="operator", lock=True, arguments=LOOK)

    lines = await answer(server, machine, "ListBindings", Filter="Look")

    assert len(lines) == 1
    assert "Look [MonitoringBinding] /mode" in lines[0]


async def test_an_ad_hoc_binding_is_read_like_a_declared_one(served, runtime) -> None:
    server, machine = served
    await called(server, machine, "BindTopic", user="operator", lock=True, arguments=LOOK)
    runtime.note_publishers("/mode", 1)
    node = monitoring_nodes(introspection(machine))["Look.Detail"]

    for _ in range(400):
        runtime.deliver("/mode", fixture_messages.Mode(mode=2, detail="wide"))
        data_value = await node.read_data_value(raise_on_bad_status=False)
        if data_value.Value is not None and data_value.Value.Value == "wide":
            break
        await asyncio.sleep(0.005)

    final = (await node.read_data_value(raise_on_bad_status=False)).Value
    assert final is not None
    assert final.Value == "wide"


async def test_a_name_the_machinery_monitoring_reserves_is_refused(served) -> None:
    server, machine = served

    status = await called(
        server, machine, "BindTopic", user="operator", lock=True, arguments={**LOOK, "Name": "Status"}
    )

    assert status == StatusCodes.BadInvalidArgument
    assert monitoring_nodes(introspection(machine)) == {}


async def test_a_second_binding_of_one_name_is_refused(served) -> None:
    server, machine = served
    await called(server, machine, "BindTopic", user="operator", lock=True, arguments=LOOK)

    status = await called(server, machine, "BindTopic", user="operator", lock=True, arguments=LOOK)

    assert status == StatusCodes.BadInvalidArgument


class Cramped(ROS2IntrospectionComponent):
    """An introspection Component with room for one node, to reach the cap's refusal."""

    mutation_node_cap = 1


class CrampedMachine(TestMachine):
    __test__ = False

    introspection_component_type = Cramped


async def test_a_binding_a_cap_refuses_creates_nothing_and_says_so(runtime) -> None:
    machine = robot(runtime, machine_type=CrampedMachine)
    server = await serve(machine)
    try:
        status = await called(server, machine, "BindTopic", user="operator", lock=True, arguments=LOOK)

        assert status == StatusCodes.BadOutOfRange
        assert monitoring_nodes(introspection(machine)) == {}
        assert introspection(machine).ad_hoc_bindings == frozenset()
    finally:
        await server.shutdown()


async def test_binding_moves_the_machines_node_version(served) -> None:
    server, machine = served
    before = machine.node_version

    await called(server, machine, "BindTopic", user="operator", lock=True, arguments=LOOK)

    assert machine.node_version == before + 1


async def test_an_ad_hoc_binding_is_server_scoped_and_reaches_no_disk(runtime) -> None:
    machine = robot(runtime)
    server = await serve(machine)
    try:
        await called(server, machine, "BindTopic", user="operator", lock=True, arguments=LOOK)

        assert "/mode" in runtime.subscribed_topics
    finally:
        await server.shutdown()

    assert "/mode" not in runtime.subscribed_topics


# -- the ad-hoc service call, which needs no nodes at all -------------------------------------------


async def test_an_ad_hoc_call_sends_the_request_a_client_typed(served, runtime) -> None:
    server, machine = served

    status = await called(
        server,
        machine,
        "CallService",
        user="operator",
        lock=True,
        arguments={
            "Service": "/set_mode",
            "ServiceType": "fixture_msgs/srv/SetMode",
            "Request": '{"Mode.Mode": 2, "Label": "wide"}',
        },
    )

    assert status == StatusCodes.Good
    assert runtime.requests == (
        (
            "/set_mode",
            fixture_messages.SetModeRequest(mode=fixture_messages.Mode(mode=2, detail=""), label="wide"),
        ),
    )


async def test_an_ad_hoc_call_answers_the_response_as_json(served) -> None:
    server, machine = served

    await called(
        server,
        machine,
        "CallService",
        user="operator",
        lock=True,
        arguments={"Service": "/set_mode", "ServiceType": "fixture_msgs/srv/SetMode", "Request": "{}"},
    )

    lines = await entries(machine, "CallService")

    assert lines[0] == "/set_mode answered fixture_msgs/srv/SetMode_Response"
    assert lines[1] == '{"Applied": 3, "Message": "fine", "Success": true}'


async def test_an_ad_hoc_call_creates_no_nodes_at_all(served) -> None:
    server, machine = served

    status = await called(
        server,
        machine,
        "CallService",
        user="operator",
        lock=True,
        arguments={"Service": "/set_mode", "ServiceType": "", "Request": "{}"},
    )

    assert status == StatusCodes.Good
    assert monitoring_nodes(introspection(machine)) == {}


async def test_a_request_naming_a_field_the_type_has_not_got_is_refused(served) -> None:
    server, machine = served

    status = await called(
        server,
        machine,
        "CallService",
        user="operator",
        lock=True,
        arguments={
            "Service": "/set_mode",
            "ServiceType": "fixture_msgs/srv/SetMode",
            "Request": '{"mode_moed": 2}',
        },
    )

    assert status == StatusCodes.BadInvalidArgument


async def test_a_request_that_is_not_a_json_object_is_refused(served) -> None:
    server, machine = served

    status = await called(
        server,
        machine,
        "CallService",
        user="operator",
        lock=True,
        arguments={"Service": "/set_mode", "ServiceType": "fixture_msgs/srv/SetMode", "Request": "[1, 2]"},
    )

    assert status == StatusCodes.BadInvalidArgument


async def test_binding_a_topic_under_a_name_that_breaks_the_rule_is_refused_with_the_suggestion(served) -> None:
    server, machine = served
    component = introspection(machine)

    status = await called(
        server, machine, "BindTopic", user="operator", lock=True, arguments={**LOOK, "Name": "joint_states"}
    )

    assert status == StatusCodes.BadInvalidArgument
    assert not component.ad_hoc_bindings
