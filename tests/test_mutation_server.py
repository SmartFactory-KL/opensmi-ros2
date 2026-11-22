# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The status-code triple and the one place the address space may change, against a real server.

A server given a value under an error status nulls the value (OPC UA Part 4, 7.7.1), so a Bad
variable reads an empty variant. The pure rule is in `test_binding_status.py`.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua
from asyncua.ua.object_ids import ObjectIds

from opensmi.ros2.runtime.graph import GraphSnapshot
from opensmi.ros2.server.bindings import MonitoringBinding
from opensmi.ros2.server.errors import DuplicateVariableNameError, UnprojectableFieldError
from opensmi.ros2.server.events import BINDING_EVENT_TYPE
from opensmi.ros2.server.mirror import AMBIGUOUS_TOPIC_TYPE
from opensmi.ros2.server.mutation import CAP_REACHED, NodeAddition
from opensmi.ros2.server.projection import PlanObject, PlanVariable
from opensmi.ros2.server.status import GOOD, NO_COMMUNICATION, WAITING_FOR_INITIAL_DATA
from tests import fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.served import TestComponent, TestMachine, browse, connected, monitoring_nodes, names_of, serve

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Callable

    from asyncua.client.client import Client
    from opensmi.server import Server

MESSAGE_TYPES = {
    "fixture_msgs/msg/Point": fixture_messages.Point,
    "fixture_msgs/msg/Pose": fixture_messages.Pose,
}

POINT = "fixture_msgs/msg/Point"
POSE = "fixture_msgs/msg/Pose"

CURRENT_POINT = MonitoringBinding(name="Point", path="/point", message_type=POINT)
WHATEVER = MonitoringBinding(name="Whatever", path="/whatever")

MODEL_CHANGE = ua.NodeId(ua.Int32(ObjectIds.GeneralModelChangeEventType))


class Platform(TestComponent):
    bindings = (CURRENT_POINT,)


class Undeclared(Platform):
    bindings = (WHATEVER,)


class Mixed(Platform):
    bindings = (CURRENT_POINT, WHATEVER)


class Capped(Platform):
    bindings = (WHATEVER,)
    mutation_node_cap = 2


class Collector:
    """An asyncua subscription handler that keeps what it was sent."""

    def __init__(self) -> None:
        self.events: list[Any] = []
        self.changes: list[tuple[Any, ua.StatusCode]] = []

    def event_notification(self, event: Any) -> None:
        self.events.append(event)

    def datachange_notification(self, node: Any, value: Any, data: Any) -> None:
        self.changes.append((value, data.monitored_item.Value.StatusCode))

    async def next_event(self, within: float = 2.0) -> Any:
        await _until(lambda: bool(self.events), "no event reached the subscriber", within)
        return self.events[0]

    async def until_status(self, status: ua.StatusCode, within: float = 2.0) -> Any:
        await _until(
            lambda: any(seen == status for _, seen in self.changes),
            f"the subscriber was never told {status}; it saw {[str(s) for _, s in self.changes]}",
            within,
        )
        return next(value for value, seen in self.changes if seen == status)


@pytest.fixture
def runtime() -> FakeRuntime:
    # Short poll interval a test can wait out (the shipped one is 5 s).
    return FakeRuntime(domain_id=7, message_types=MESSAGE_TYPES, graph_poll_interval=0.02)


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


def platform(machine: Any) -> Any:
    return machine.added[0]


async def _until(ready: Callable[[], bool], complaint: str, within: float = 2.0) -> None:
    """Wait for a condition the server reaches on its own tasks, or say what never happened."""
    deadline = asyncio.get_running_loop().time() + within
    while not ready():
        assert asyncio.get_running_loop().time() < deadline, complaint
        await asyncio.sleep(0.005)


async def _binding_events(client: Client, item: Any, collector: Collector) -> Any:
    """Subscribe a collector to the BindingEvents one machinery item fires."""
    subscription = await client.create_subscription(20, collector)
    await subscription.subscribe_events(
        client.get_node(item.notification.ua_node.nodeid),
        client.get_node(item.server.ua_get_node_id(BINDING_EVENT_TYPE)),
    )
    return subscription


async def _model_changes(client: Client, machine: Any, collector: Collector) -> Any:
    """Subscribe a collector to the model changes a Machine announces."""
    subscription = await client.create_subscription(20, collector)
    await subscription.subscribe_events(
        client.get_node(machine.spatial_object_list.ua_node.nodeid),
        client.get_node(MODEL_CHANGE),
    )
    return subscription


# -- the status-code triple ----------------------------------------------------------------


async def test_a_topic_with_no_publisher_reads_bad_no_communication(served) -> None:
    server, machine = served
    node = monitoring_nodes(platform(machine))["Point.X"]

    assert (await node.read_data_value(raise_on_bad_status=False)).StatusCode == NO_COMMUNICATION

    async with connected(server) as client:
        await _until_status(client.get_node(node.nodeid), NO_COMMUNICATION)


async def test_a_publisher_with_no_message_reads_bad_waiting_for_initial_data(served, runtime: FakeRuntime) -> None:
    server, machine = served
    node = monitoring_nodes(platform(machine))["Point.X"]

    runtime.note_publishers("/point", 1)

    async with connected(server) as client:
        await _until_status(client.get_node(node.nodeid), WAITING_FOR_INITIAL_DATA)


async def test_the_bad_status_and_its_recovery_both_reach_a_subscription(served, runtime: FakeRuntime) -> None:
    server, machine = served
    node = monitoring_nodes(platform(machine))["Point.X"]
    collector = Collector()

    async with connected(server) as client:
        subscription = await client.create_subscription(20, collector)
        await subscription.subscribe_data_change(client.get_node(node.nodeid))
        await collector.until_status(NO_COMMUNICATION)

        runtime.note_publishers("/point", 1)
        await collector.until_status(WAITING_FOR_INITIAL_DATA)

        runtime.deliver("/point", fixture_messages.Point(x=1.5, y=0.0, z=0.0))
        recovered = await collector.until_status(GOOD)

        await subscription.delete()

    assert recovered == 1.5
    assert runtime.graph.polls == 0


async def test_a_publisher_going_away_after_a_sample_reads_bad_no_communication_again(
    served, runtime: FakeRuntime
) -> None:
    server, machine = served
    node = monitoring_nodes(platform(machine))["Point.X"]

    runtime.note_publishers("/point", 1)
    runtime.deliver("/point", fixture_messages.Point(x=2.5, y=0.0, z=0.0))

    async with connected(server) as client:
        remote = client.get_node(node.nodeid)
        await _until_status(remote, GOOD)

        runtime.note_publishers("/point", 0)
        await _until_status(remote, NO_COMMUNICATION)


# -- the graph facet, and what makes it self-disabling --------------------------------------


async def test_a_fully_declared_machine_polls_the_graph_zero_times(served, runtime: FakeRuntime) -> None:
    _, machine = served

    await asyncio.sleep(0.1)

    assert platform(machine).ros2_unbound_bindings == ()
    assert not runtime.graph.polling
    assert runtime.graph.polls == 0


async def test_a_binding_with_no_declared_type_has_no_nodes_and_waits_on_the_graph(
    runtime: FakeRuntime,
) -> None:
    server, machine = await start(runtime, Undeclared)
    try:
        assert monitoring_nodes(platform(machine)) == {}
        assert [binding.name for binding in platform(machine).ros2_unbound_bindings] == ["Whatever"]
        assert runtime.graph.polling
    finally:
        await server.shutdown()


async def test_a_binding_with_no_declared_type_gains_its_nodes_when_its_topic_appears(
    runtime: FakeRuntime,
) -> None:
    server, machine = await start(runtime, Undeclared)
    collector = Collector()
    try:
        async with connected(server) as client:
            subscription = await _model_changes(client, machine, collector)

            runtime.set_graph(GraphSnapshot(topics={"/whatever": (POINT,)}))
            await _until(
                lambda: "Whatever.X" in monitoring_nodes(platform(machine)),
                "the Binding never gained its nodes",
            )
            event = await collector.next_event()

            assert sorted(monitoring_nodes(platform(machine))) == ["Whatever.X", "Whatever.Y", "Whatever.Z"]
            assert len(collector.events) == 1
            assert len(event.Changes) == 4

            root = await browse(client.get_node(platform(machine).monitoring.ua_node.nodeid), "Whatever")
            assert {"X", "Y", "Z"} <= set(await names_of(root))
            runtime.note_publishers("/whatever", 1)
            runtime.deliver("/whatever", fixture_messages.Point(x=7.5, y=0.0, z=0.0))
            await _until_status(client.get_node(monitoring_nodes(platform(machine))["Whatever.X"].nodeid), GOOD)

            await subscription.delete()

        assert platform(machine).ros2_unbound_bindings == ()
        assert not runtime.graph.polling
        assert machine.node_version == 1
    finally:
        await server.shutdown()


async def test_a_topic_carrying_two_types_is_refused_and_bound_when_the_disagreement_clears(
    runtime: FakeRuntime,
) -> None:
    # Conflicting publishers leave the Binding waiting rather than guessing a type.
    server, machine = await start(runtime, Undeclared)
    collector = Collector()
    try:
        async with connected(server) as client:
            subscription = await _binding_events(client, platform(machine), collector)

            runtime.set_graph(GraphSnapshot(topics={"/whatever": (POINT, POSE)}))
            event = await collector.next_event()
            await subscription.delete()

        assert event.Code == AMBIGUOUS_TOPIC_TYPE
        assert monitoring_nodes(platform(machine)) == {}
        assert [binding.name for binding in platform(machine).ros2_unbound_bindings] == ["Whatever"]

        runtime.set_graph(GraphSnapshot(topics={"/whatever": (POINT,)}))
        await _until(
            lambda: "Whatever.X" in monitoring_nodes(platform(machine)),
            "the Binding never bound after the disagreement cleared",
        )

        assert platform(machine).ros2_unbound_bindings == ()
        assert not runtime.graph.polling
    finally:
        await server.shutdown()


async def test_a_declared_binding_beside_an_undeclared_one_keeps_its_nodes_where_they_were(
    runtime: FakeRuntime,
) -> None:
    # The mutation must leave a declared Binding's NodeIds where they were.
    server, machine = await start(runtime, Mixed)
    try:
        at_boot = {name: node.nodeid for name, node in monitoring_nodes(platform(machine)).items()}
        assert sorted(at_boot) == ["Point.X", "Point.Y", "Point.Z"]

        runtime.set_graph(GraphSnapshot(topics={"/whatever": (POINT,)}))
        await _until(
            lambda: "Whatever.X" in monitoring_nodes(platform(machine)),
            "the undeclared Binding never gained its nodes",
        )

        after = monitoring_nodes(platform(machine))
        assert {name: node.nodeid for name, node in after.items() if name in at_boot} == at_boot
        assert sorted(after) == ["Point.X", "Point.Y", "Point.Z", "Whatever.X", "Whatever.Y", "Whatever.Z"]

        runtime.note_publishers("/point", 1)
        runtime.deliver("/point", fixture_messages.Point(x=3.5, y=0.0, z=0.0))
        async with connected(server) as client:
            await _until_status(client.get_node(at_boot["Point.X"]), GOOD)

        assert not runtime.graph.polling
    finally:
        await server.shutdown()


# -- the chokepoint's own rules --------------------------------------------------------------


async def test_the_chokepoint_announces_nothing_at_init(served) -> None:
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        subscription = await _model_changes(client, machine, collector)

        await platform(machine).mutate_address_space([_addition(platform(machine), "Built", "A")], at_init=True)
        await asyncio.sleep(0.2)
        await subscription.delete()

    assert collector.events == []
    assert "Built.A" in monitoring_nodes(platform(machine))
    assert machine.machine_mutation_node_total == 0
    assert machine.node_version == 0


async def test_one_call_to_the_chokepoint_is_one_event_with_its_changes_batched(served) -> None:
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        subscription = await _model_changes(client, machine, collector)

        await platform(machine).mutate_address_space(
            [_addition(platform(machine), "Grown", "A", "B"), _addition(platform(machine), "Also", "C")]
        )
        event = await collector.next_event()
        await asyncio.sleep(0.2)
        await subscription.delete()

    assert len(collector.events) == 1
    assert len(event.Changes) == 5
    assert machine.machine_mutation_node_total == 5


async def test_the_chokepoint_puts_what_it_created_in_the_monitoring_container(served) -> None:
    _, machine = served

    [created] = await platform(machine).mutate_address_space([_addition(platform(machine), "Grown", "A", "B")])

    assert created is not None
    assert sorted(created.entries) == ["A", "B"]
    assert "Grown.A" in monitoring_nodes(platform(machine))
    assert platform(machine).mutation_node_counts == {"Grown": 3}


async def test_a_plan_that_opens_an_object_is_refused_and_creates_nothing(served) -> None:
    _, machine = served
    item = platform(machine)

    with pytest.raises(UnprojectableFieldError, match=r"renders .shared."):
        await item.mutate_address_space([_subtree(item, "left", "shared")])

    assert sorted(monitoring_nodes(item)) == ["Point.X", "Point.Y", "Point.Z"]
    assert machine.machine_mutation_node_total == 0


async def test_a_member_that_nests_an_object_is_refused_and_creates_nothing(served) -> None:
    _, machine = served
    item = platform(machine)
    inner = PlanObject(
        name="inner",
        children=(PlanVariable(name="x", source="x", initial=0.0, variant_type=ua.VariantType.Double),),
    )
    nested = NodeAddition(
        binding="keyed",
        path="/keyed",
        container=item.monitoring,
        plan=(PlanObject(name="member", children=(inner,)),),
        member=("Member", "member"),
        message_type=fixture_messages.Point,
    )

    with pytest.raises(UnprojectableFieldError, match=r"renders .member."):
        await item.mutate_address_space([nested])

    assert "Member" not in item.monitoring

    assert sorted(monitoring_nodes(item)) == ["Point.X", "Point.Y", "Point.Z"]
    assert machine.machine_mutation_node_total == 0


# -- the caps ---------------------------------------------------------------------------------


async def test_a_binding_cap_refuses_the_whole_addition_rather_than_half_of_it(served) -> None:
    server, machine = served
    platform(machine).mutation_node_cap = 2
    collector = Collector()

    async with connected(server) as client:
        subscription = await _binding_events(client, platform(machine), collector)

        created = await platform(machine).mutate_address_space([_addition(platform(machine), "Greedy", "A", "B", "C")])
        event = await collector.next_event()
        await subscription.delete()

    assert created == [None]
    assert not [name for name in monitoring_nodes(platform(machine)) if name.startswith("Greedy")]
    assert event.Code == CAP_REACHED
    assert machine.machine_mutation_node_total == 0


async def test_a_machine_cap_refuses_growth_no_one_component_would_have(served) -> None:
    _, machine = served
    machine.machine_mutation_node_cap = 3

    [first] = await platform(machine).mutate_address_space([_addition(platform(machine), "First", "A", "B")])
    second = await platform(machine).mutate_address_space([_addition(platform(machine), "Second", "C", "D")])

    assert first is not None
    assert sorted(first.entries) == ["A", "B"]
    assert second == [None]
    assert machine.machine_mutation_node_total == 3


async def test_the_graph_driven_case_refuses_a_topic_past_the_cap_and_says_so(
    runtime: FakeRuntime,
) -> None:
    server, machine = await start(runtime, Capped)
    collector = Collector()
    try:
        async with connected(server) as client:
            subscription = await _binding_events(client, platform(machine), collector)

            runtime.set_graph(GraphSnapshot(topics={"/whatever": (POINT,)}))
            event = await collector.next_event()
            await subscription.delete()

        assert event.Code == CAP_REACHED
        assert event.Binding == "Whatever"
        assert monitoring_nodes(platform(machine)) == {}
        assert platform(machine).ros2_subscriptions == ()
    finally:
        await server.shutdown()


def _addition(item: Any, binding: str, *names: str) -> NodeAddition:
    """Return an addition of one message rendered as a complex Variable with a field per name."""
    return NodeAddition(
        binding=binding,
        path=f"/{binding}",
        container=item.monitoring,
        root=binding,
        message_type=fixture_messages.Point,
        plan=tuple(
            PlanVariable(name=name, source=name, initial=0.0, variant_type=ua.VariantType.Double) for name in names
        ),
    )


def _member_named(item: Any, name: str, *children: str) -> NodeAddition:
    """Return an addition of one member group `name` holding one variable per child."""
    return NodeAddition(
        binding=name,
        path=f"/{name}",
        container=item.monitoring,
        member=(name, name),
        message_type=fixture_messages.Point,
        plan=tuple(
            PlanVariable(name=child, source=child, initial=0.0, variant_type=ua.VariantType.Double)
            for child in children
        ),
    )


async def test_a_name_already_taken_in_the_container_is_refused_and_creates_nothing(served) -> None:
    _, machine = served
    item = platform(machine)
    await item.mutate_address_space([_addition(item, "Taken", "X")])
    before = sorted(monitoring_nodes(item))

    with pytest.raises(DuplicateVariableNameError, match="Taken"):
        await item.mutate_address_space([_addition(item, "Taken", "X")])
    with pytest.raises(DuplicateVariableNameError, match="Taken"):
        await item.mutate_address_space([_member_named(item, "Taken", "X")])

    assert sorted(monitoring_nodes(item)) == before


async def test_a_member_is_a_group_of_its_own_with_the_fields_below_it(served) -> None:
    _, machine = served
    item = platform(machine)

    [member] = await item.mutate_address_space([_member_named(item, "Left", "X", "Y")])

    assert member is not None
    assert sorted(monitoring_nodes(item)) == ["Left.X", "Left.Y", "Point.X", "Point.Y", "Point.Z"]
    assert item.mutation_node_counts == {"Left": 3}


def _subtree(item: Any, binding: str, name: str) -> NodeAddition:
    """Return an addition whose plan opens an Object at the top."""
    return NodeAddition(
        binding=binding,
        path=f"/{binding}",
        container=item.monitoring,
        message_type=fixture_messages.Point,
        plan=(
            PlanObject(
                name=name,
                children=(PlanVariable(name="x", source="x", initial=0.0, variant_type=ua.VariantType.Double),),
            ),
        ),
    )


async def _until_status(node: Any, status: ua.StatusCode, within: float = 2.0) -> None:
    """Read a client-side node until it carries the status the mirror was driven to."""
    seen: list[ua.StatusCode | None] = []
    deadline = asyncio.get_running_loop().time() + within
    while True:
        seen.append((await node.read_data_value(raise_on_bad_status=False)).StatusCode)
        if seen[-1] == status:
            return
        assert asyncio.get_running_loop().time() < deadline, f"the variable reads {seen[-1]}, not {status}"
        await asyncio.sleep(0.005)


# -- a Machinery Monitoring group, made by the first addition that names it ----------------------


class UndeclaredInProcess(Platform):
    bindings = (MonitoringBinding(name="Whatever", path="/whatever", group="Process"),)


class CappedInProcess(UndeclaredInProcess):
    mutation_node_cap = 2


async def test_a_group_made_at_run_time_is_announced_with_the_nodes_in_it(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, UndeclaredInProcess)
    collector = Collector()
    try:
        assert "Process" not in platform(machine).monitoring
        async with connected(server) as client:
            subscription = await _model_changes(client, machine, collector)

            runtime.set_graph(GraphSnapshot(topics={"/whatever": (POINT,)}))
            event = await collector.next_event()
            await subscription.delete()

        assert sorted(monitoring_nodes(platform(machine))) == [
            "Process.Whatever.X",
            "Process.Whatever.Y",
            "Process.Whatever.Z",
        ]
        assert len(event.Changes) == 5
        assert event.Changes[0].Affected == platform(machine).monitoring.groups["Process"].ua_node.nodeid
        assert event.Changes[0].AffectedType == ua.NodeId(ua.Int32(ObjectIds.FolderType))
    finally:
        await server.shutdown()


async def test_a_capped_addition_does_not_make_its_group(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, CappedInProcess)
    collector = Collector()
    try:
        async with connected(server) as client:
            subscription = await _binding_events(client, platform(machine), collector)

            runtime.set_graph(GraphSnapshot(topics={"/whatever": (POINT,)}))
            event = await collector.next_event()
            await subscription.delete()

        assert event.Code == CAP_REACHED
        assert "Process" not in platform(machine).monitoring
    finally:
        await server.shutdown()


async def test_an_addition_that_fails_takes_back_the_group_it_made(served) -> None:
    _, machine = served
    item = platform(machine)
    failing = NodeAddition(
        binding="Broken",
        path="/broken",
        container=item.monitoring,
        plan=(PlanObject(name="inner", children=()),),
        root="Broken",
        group="Process",
        message_type=fixture_messages.Point,
    )

    with pytest.raises(UnprojectableFieldError):
        await item.mutate_address_space([failing])

    assert "Process" not in item.monitoring
