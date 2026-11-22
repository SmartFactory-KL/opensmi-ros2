# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A Keyed Binding's members appear as messages name them, against a real in-process server.

Nodes are found by walking the Component's Monitoring. The pure decisions are in `test_keyed.py`.
"""

from __future__ import annotations

import asyncio
from dataclasses import replace
from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua
from asyncua.ua.object_ids import ObjectIds
from opensmi.server.base_startup_skill import BaseStartupSkill

from opensmi.ros2.server.bindings import KeyedBinding, load_bindings
from opensmi.ros2.server.errors import DeclarationError, UnprojectableFieldError
from opensmi.ros2.server.events import BINDING_EVENT_TYPE, Severity
from opensmi.ros2.server.keyed import KEY_COLLISION, MEMBER_STATE_CHANGED
from opensmi.ros2.server.mutation import CAP_REACHED
from opensmi.ros2.server.status import GOOD, NO_COMMUNICATION, WAITING_FOR_INITIAL_DATA
from tests import fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.fixture_structures import node_id as structure_node_id
from tests.served import (
    TestComponent,
    TestMachine,
    connected,
    elements_of,
    leaf,
    serve,
    until_elements,
    until_status,
    until_value,
    value_of,
)

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Callable

    from asyncua.client.client import Client
    from asyncua.common.node import Node
    from opensmi.server import Server

MESSAGE_TYPES = {
    "fixture_msgs/msg/Units": fixture_messages.Units,
    "fixture_msgs/msg/Unit": fixture_messages.Unit,
    "fixture_msgs/msg/Pair": fixture_messages.Pair,
    "fixture_msgs/msg/Header": fixture_messages.Header,
    "fixture_msgs/msg/Time": fixture_messages.Time,
}

UNITS = "fixture_msgs/msg/Units"

#: `/diagnostics` as one declaration; the key is two fields (name, hardware_id).
DIAGNOSTICS = KeyedBinding(
    name="Diagnostics",
    path="/diagnostics",
    message_type=UNITS,
    repeated_field="units",
    key=("hardware_id", "name"),
    state_field="level",
    states={0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"},
    severities={0: Severity.INFO, 1: Severity.WARNING, 2: Severity.ERROR, 3: Severity.STALE},
)

#: The same mechanism with nothing said about state: the shape of the TF tree.
FRAMES = KeyedBinding(
    name="Frames",
    path="/frames",
    message_type=UNITS,
    repeated_field="units",
    key=("name",),
    include=("name", "message"),
)

#: One member of DIAGNOSTICS: raw key, four walked fields and the pair array.
MEMBER_VARIABLES = ("HardwareId", "Key", "Level", "Message", "Name", "Values")


class Platform(TestComponent):
    bindings = (DIAGNOSTICS, FRAMES)


class Capped(TestComponent):
    # A member is six nodes plus its Object; a cap of eight admits one member and refuses the second whole.
    bindings = (DIAGNOSTICS,)
    mutation_node_cap = 8


class Collector:
    """An asyncua subscription handler that keeps what it was sent."""

    def __init__(self) -> None:
        self.events: list[Any] = []

    def event_notification(self, event: Any) -> None:
        self.events.append(event)

    async def next_event(self, within: float = 2.0) -> Any:
        await _until(lambda: bool(self.events), "no event reached the subscriber", within)
        return self.events[0]

    async def events_of(self, code: str, count: int, within: float = 2.0) -> list[Any]:
        await _until(
            lambda: len([each for each in self.events if each.Code == code]) >= count,
            f"fewer than {count} {code} events arrived; it saw {[each.Code for each in self.events]}",
            within,
        )
        return [each for each in self.events if each.Code == code]


@pytest.fixture
def runtime() -> FakeRuntime:
    return FakeRuntime(domain_id=7, message_types=MESSAGE_TYPES)


async def start(runtime: FakeRuntime, component: TestComponent | None = None) -> tuple[Server, Any]:
    """Bring up a server carrying one Machine with one Component, and hand back both."""
    machine = TestMachine(component or Platform(name="Platform"), runtime=runtime)
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


def publish(runtime: FakeRuntime, *elements: Any, topic: str = "/diagnostics") -> None:
    """Deliver one message carrying the elements given, from a publisher that is there."""
    runtime.note_publishers(topic, 1)
    runtime.deliver(topic, fixture_messages.units(*elements))


async def nodes(item: Any) -> dict[str, Node]:
    """Return every variable in a machinery item's Monitoring by the dotted path of its browse names.

    Walks as a client does: groups, members and complex Variables are descended into, and only the
    Variables that hold no further Variable are listed.
    """
    found: dict[str, Node] = {}

    async def below(node: Node, prefix: str) -> None:
        for child in await node.get_children():
            name = f"{prefix}{(await child.read_browse_name()).Name}"
            if await child.read_node_class() == ua.NodeClass.Object:
                await below(child, f"{name}.")
            elif fields := await child.get_children(nodeclassmask=ua.NodeClass.Variable):
                await below(child, f"{name}.")
                del fields
            else:
                found[name] = child

    await below(item.monitoring.ua_node, "")
    return found


async def until_member(machine: Any, binding: str, browse: str, within: float = 2.0) -> None:
    """Wait for one member to have been created on the Component."""
    await _until(
        lambda: browse in platform(machine).ros2_members.get(binding, ()),
        f"the member {browse!r} never appeared; there are {platform(machine).ros2_members}",
        within,
    )


async def _until(ready: Callable[[], bool], complaint: str, within: float = 2.0) -> None:
    """Wait for a condition the server reaches on its own tasks, or say what never happened."""
    deadline = asyncio.get_running_loop().time() + within
    while not ready():
        assert asyncio.get_running_loop().time() < deadline, complaint
        await asyncio.sleep(0.005)


async def binding_events(client: Client, machine: Any, collector: Collector) -> Any:
    """Subscribe to the BindingEvents the Component raises."""
    subscription = await client.create_subscription(20, collector)
    await subscription.subscribe_events(
        client.get_node(platform(machine).notification.ua_node.nodeid),
        client.get_node(machine.server.ua_get_node_id(BINDING_EVENT_TYPE)),
    )
    return subscription


async def model_changes(client: Client, machine: Any, collector: Collector) -> Any:
    """Subscribe to the model changes the Machine announces."""
    subscription = await client.create_subscription(20, collector)
    await subscription.subscribe_events(
        client.get_node(machine.spatial_object_list.ua_node.nodeid),
        client.get_node(ua.NodeId(ua.Int32(ObjectIds.GeneralModelChangeEventType))),
    )
    return subscription


async def status_of(node: Node) -> int:
    status = (await node.read_data_value(raise_on_bad_status=False)).StatusCode
    assert status is not None
    return status.value


# -- membership: declared Binding, undeclared members -------------------------------------------


async def test_a_keyed_binding_has_no_members_at_boot(served) -> None:
    _, machine = served

    assert sorted(await nodes(platform(machine))) == ["Diagnostics.WorstLevel"]
    assert platform(machine).ros2_members == {"Diagnostics": (), "Frames": ()}


async def test_a_member_appears_per_key_as_messages_name_them(served, runtime: FakeRuntime) -> None:
    _, machine = served

    publish(runtime, fixture_messages.unit(hardware_id="robot_1", name="motor"))
    await until_member(machine, "Diagnostics", "robot_1_motor")

    assert sorted(await nodes(platform(machine))) == [
        "Diagnostics.WorstLevel",
        *(f"Diagnostics.robot_1_motor.{name}" for name in MEMBER_VARIABLES),
    ]


async def test_a_member_is_a_variable_group_under_the_binding(served, runtime: FakeRuntime) -> None:
    server, machine = served

    publish(runtime, fixture_messages.unit(name="motor"))
    await until_member(machine, "Diagnostics", "robot_1_motor")
    node = (await nodes(platform(machine)))["Diagnostics.robot_1_motor.Key"]

    async with connected(server) as client:
        member = await client.get_node(node.nodeid).get_parent()
        assert member is not None
        assert await member.read_node_class() == ua.NodeClass.Object
        definition = await member.read_type_definition()
        group = await member.get_parent()
        assert group is not None
        group_name = (await group.read_browse_name()).Name

    assert definition == ua.NodeId(ua.Int32(ObjectIds.BaseObjectType))
    assert group_name == "Diagnostics"


async def test_a_member_carries_no_last_update_variable(served, runtime: FakeRuntime) -> None:
    _, machine = served

    publish(runtime, fixture_messages.unit())
    await until_member(machine, "Diagnostics", "robot_1_motor")

    assert not [name for name in await nodes(platform(machine)) if "last" in name.lower()]


async def test_two_units_with_one_name_stay_two_members(served, runtime: FakeRuntime) -> None:
    _, machine = served

    publish(
        runtime,
        fixture_messages.unit(hardware_id="robot_1", name="motor"),
        fixture_messages.unit(hardware_id="robot_2", name="motor"),
    )
    await until_member(machine, "Diagnostics", "robot_2_motor")

    assert platform(machine).ros2_members["Diagnostics"] == ("robot_1_motor", "robot_2_motor")


async def test_a_member_browses_under_a_sanitised_key_and_displays_and_keeps_the_raw_one(
    served, runtime: FakeRuntime
) -> None:
    server, machine = served

    publish(runtime, fixture_messages.unit(hardware_id="robot 1: left", name="/motor"))
    await until_member(machine, "Diagnostics", "robot 1_ left__motor")
    node = (await nodes(platform(machine)))["Diagnostics.robot 1_ left__motor.Key"]

    async with connected(server) as client:
        key = client.get_node(node.nodeid)
        assert await value_of(key) == ["robot 1: left", "/motor"]
        parent = await key.get_parent()
        assert parent is not None
        assert (await parent.read_browse_name()).Name == "robot 1_ left__motor"
        assert (await parent.read_display_name()).Text == "robot 1: left_/motor"
        assert (await key.read_browse_name()).Name == "Key"
        assert (await key.read_display_name()).Text == "key"


async def test_a_member_is_append_only(served, runtime: FakeRuntime) -> None:
    _, machine = served

    publish(
        runtime,
        fixture_messages.unit(name="motor", message="both"),
        fixture_messages.unit(name="laser", message="both"),
    )
    await until_member(machine, "Diagnostics", "robot_1_laser")

    publish(runtime, fixture_messages.unit(name="motor", message="alone"))
    found = await nodes(platform(machine))
    await until_value(found["Diagnostics.robot_1_motor.Message"], "alone")

    assert platform(machine).ros2_members["Diagnostics"] == ("robot_1_motor", "robot_1_laser")
    assert await value_of(found["Diagnostics.robot_1_laser.Message"]) == "both"


async def test_a_binding_that_declares_no_state_field_gets_no_aggregate_and_only_members(
    served, runtime: FakeRuntime
) -> None:
    _, machine = served

    publish(runtime, fixture_messages.unit(name="base_link"), topic="/frames")
    await until_member(machine, "Frames", "base_link")

    assert sorted(name for name in await nodes(platform(machine)) if name.startswith("Frames")) == [
        "Frames.base_link.Key",
        "Frames.base_link.Message",
        "Frames.base_link.Name",
    ]


# -- the mutation chokepoint --------------------------------------------------------------------


async def test_a_new_member_is_announced_by_one_model_change_event(served, runtime: FakeRuntime) -> None:
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        subscription = await model_changes(client, machine, collector)

        publish(runtime, fixture_messages.unit(name="motor"))
        event = await collector.next_event()
        await subscription.delete()

    assert len(event.Changes) == 7
    assert platform(machine).mutation_node_counts == {"Diagnostics": 7}


async def test_three_members_in_one_message_are_one_model_change_event(served, runtime: FakeRuntime) -> None:
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        subscription = await model_changes(client, machine, collector)

        publish(
            runtime,
            fixture_messages.unit(name="motor"),
            fixture_messages.unit(name="laser"),
            fixture_messages.unit(name="camera"),
        )
        await until_member(machine, "Diagnostics", "robot_1_camera")
        event = await collector.next_event()
        publish(
            runtime,
            fixture_messages.unit(name="motor", message="again"),
            fixture_messages.unit(name="laser", message="again"),
            fixture_messages.unit(name="camera", message="again"),
        )
        await until_value((await nodes(platform(machine)))["Diagnostics.robot_1_camera.Message"], "again")
        await subscription.delete()

    assert len(collector.events) == 1
    assert len(event.Changes) == 21


async def test_nothing_is_announced_for_the_aggregate_created_at_init(served) -> None:
    _, machine = served

    assert platform(machine).mutation_node_counts == {}
    assert machine.machine_mutation_node_total == 0


async def test_a_cap_refuses_a_member_whole_and_says_so_once(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, Capped(name="Platform"))
    collector = Collector()
    try:
        async with connected(server) as client:
            subscription = await binding_events(client, machine, collector)

            publish(runtime, fixture_messages.unit(name="motor"))
            await until_member(machine, "Diagnostics", "robot_1_motor")
            for _ in range(3):
                publish(runtime, fixture_messages.unit(name="motor"), fixture_messages.unit(name="laser"))
            capped = await collector.events_of(CAP_REACHED, 1)
            await until_value(
                (await nodes(platform(machine)))["Diagnostics.robot_1_motor.Name"],
                "motor",
            )
            await subscription.delete()

        assert not [name for name in await nodes(platform(machine)) if "laser" in name]
        # Said once however many messages repeat it, so a 10 Hz topic causes no event storm.
        assert len(capped) == 1
    finally:
        await server.shutdown()


async def test_two_keys_that_browse_alike_are_refused_rather_than_merged(served, runtime: FakeRuntime) -> None:
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        subscription = await binding_events(client, machine, collector)

        publish(
            runtime,
            fixture_messages.unit(hardware_id="robot", name="left.motor"),
            fixture_messages.unit(hardware_id="robot", name="left:motor"),
        )
        collision = await collector.events_of(KEY_COLLISION, 1)
        await subscription.delete()

    assert platform(machine).ros2_members["Diagnostics"] == ("robot_left_motor",)
    assert collision[0].Binding == "Diagnostics"
    assert collision[0].ROS2Path == "/diagnostics"


async def test_a_key_that_browses_onto_the_aggregate_is_refused(runtime: FakeRuntime) -> None:
    class OneField(TestComponent):
        bindings = (replace(DIAGNOSTICS, key=("name",)),)

    server, machine = await start(runtime, OneField(name="Platform"))
    collector = Collector()

    try:
        async with connected(server) as client:
            subscription = await binding_events(client, machine, collector)

            publish(
                runtime,
                fixture_messages.unit(name="WorstLevel"),
                fixture_messages.unit(name="motor"),
            )
            collision = await collector.events_of(KEY_COLLISION, 1)
            await subscription.delete()
        assert platform(machine).ros2_members["Diagnostics"] == ("motor",)
        assert "already taken on this Monitoring" in collision[0].Message.Text
    finally:
        await server.shutdown()


# -- the state field: the event, and the aggregate ----------------------------------------------


async def test_a_level_change_fires_the_projects_event_off_the_member_write_path(served, runtime: FakeRuntime) -> None:
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        subscription = await binding_events(client, machine, collector)

        publish(runtime, fixture_messages.unit(name="motor", level=0))
        await until_member(machine, "Diagnostics", "robot_1_motor")
        publish(runtime, fixture_messages.unit(name="motor", level=2))
        moved = await collector.events_of(MEMBER_STATE_CHANGED, 1)
        await subscription.delete()

    assert len(moved) == 1
    assert moved[0].Severity == int(Severity.ERROR)
    assert "OK (0)" in moved[0].Message.Text
    assert "ERROR (2)" in moved[0].Message.Text


async def test_a_level_that_repeats_fires_nothing(served, runtime: FakeRuntime) -> None:
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        subscription = await binding_events(client, machine, collector)

        publish(runtime, fixture_messages.unit(name="motor", level=1, message="first"))
        moved = await collector.events_of(MEMBER_STATE_CHANGED, 1)
        for index in range(3):
            publish(runtime, fixture_messages.unit(name="motor", level=1, message=f"again {index}"))
        # The message field shows the repeats were written, so the assertion is about events not firing.
        await until_value((await nodes(platform(machine)))["Diagnostics.robot_1_motor.Message"], "again 2")
        await subscription.delete()

    assert len(moved) == 1


async def test_a_member_already_broken_when_the_server_starts_announces_itself(served, runtime: FakeRuntime) -> None:
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        subscription = await binding_events(client, machine, collector)

        publish(runtime, fixture_messages.unit(name="motor", level=3))
        moved = await collector.events_of(MEMBER_STATE_CHANGED, 1)
        await subscription.delete()

    assert moved[0].Severity == int(Severity.ERROR)


async def test_a_binding_with_no_state_field_fires_nothing_at_all(served, runtime: FakeRuntime) -> None:
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        subscription = await binding_events(client, machine, collector)

        publish(runtime, fixture_messages.unit(name="base_link", level=3), topic="/frames")
        await until_member(machine, "Frames", "base_link")
        await until_value((await nodes(platform(machine)))["Frames.base_link.Name"], "base_link")
        await subscription.delete()

    assert collector.events == []


async def test_the_aggregate_is_the_worst_level_any_member_holds(served, runtime: FakeRuntime) -> None:
    _, machine = served
    aggregate = (await nodes(platform(machine)))["Diagnostics.WorstLevel"]

    publish(
        runtime,
        fixture_messages.unit(name="motor", level=1),
        fixture_messages.unit(name="laser", level=2),
    )
    await until_member(machine, "Diagnostics", "robot_1_laser")

    await until_value(aggregate, 2)


async def test_the_aggregate_follows_a_member_back_down(served, runtime: FakeRuntime) -> None:
    _, machine = served
    aggregate = (await nodes(platform(machine)))["Diagnostics.WorstLevel"]

    publish(runtime, fixture_messages.unit(name="motor", level=2))
    await until_member(machine, "Diagnostics", "robot_1_motor")
    await until_value(aggregate, 2)
    publish(runtime, fixture_messages.unit(name="motor", level=0))

    await until_value(aggregate, 0)


async def test_an_error_level_drives_no_state_machine(served, runtime: FakeRuntime) -> None:
    _, machine = served

    publish(runtime, fixture_messages.unit(name="motor", level=2))
    await until_member(machine, "Diagnostics", "robot_1_motor")

    assert BaseStartupSkill.NAME not in platform(machine).skill_set


async def test_the_level_carries_the_four_ros2_constants_as_an_enumeration(served, runtime: FakeRuntime) -> None:
    server, machine = served

    publish(runtime, fixture_messages.unit(name="motor", level=2))
    await until_member(machine, "Diagnostics", "robot_1_motor")

    qualified = f"{platform(machine).ros2_projection_qualifier}_Unit_level"
    minted = machine.ros2_enum_datatypes[qualified]
    found = await nodes(platform(machine))

    assert minted[0].values == ((0, "OK"), (1, "WARN"), (2, "ERROR"), (3, "STALE"))
    async with connected(server) as client:
        for name in ("Diagnostics.robot_1_motor.Level", "Diagnostics.WorstLevel"):
            node = client.get_node(found[name].nodeid)
            assert await node.read_data_type() == minted[1]


# -- what a client actually reads ---------------------------------------------------------------


async def test_the_key_value_pairs_reach_a_client_as_one_array_of_a_structure(served, runtime: FakeRuntime) -> None:
    server, machine = served

    publish(
        runtime,
        fixture_messages.unit(name="motor", values=(("temperature", "41.0"), ("firmware", "3b1c"))),
    )
    await until_member(machine, "Diagnostics", "robot_1_motor")
    found = await nodes(platform(machine))
    variable = leaf(platform(machine).monitoring, "Diagnostics.robot_1_motor.Values")

    async with connected(server) as client:
        pairs = client.get_node(found["Diagnostics.robot_1_motor.Values"].nodeid)
        await until_elements(
            pairs,
            variable,
            [{"key": "temperature", "value": "41.0"}, {"key": "firmware", "value": "3b1c"}],
        )
        assert await pairs.read_value_rank() == ua.ValueRank.OneDimension
        namespace = platform(machine).ros2_namespace_index
        assert await pairs.read_data_type() == structure_node_id("fixture_msgs/msg/Pair", namespace)


async def test_a_member_reads_good_while_its_topic_has_a_publisher(served, runtime: FakeRuntime) -> None:
    _, machine = served

    publish(runtime, fixture_messages.unit(name="motor", message="all well"))
    await until_member(machine, "Diagnostics", "robot_1_motor")
    node = (await nodes(platform(machine)))["Diagnostics.robot_1_motor.Message"]

    await until_value(node, "all well")

    assert await status_of(node) == GOOD.value


async def test_the_publisher_going_away_takes_every_member_bad(served, runtime: FakeRuntime) -> None:
    _, machine = served

    publish(runtime, fixture_messages.unit(name="motor"))
    await until_member(machine, "Diagnostics", "robot_1_motor")
    found = await nodes(platform(machine))

    runtime.note_publishers("/diagnostics", 0)

    await until_status(found["Diagnostics.robot_1_motor.Name"], NO_COMMUNICATION.value)
    await until_status(found["Diagnostics.WorstLevel"], NO_COMMUNICATION.value)


async def test_the_aggregate_waits_for_initial_data_before_any_message(served, runtime: FakeRuntime) -> None:
    _, machine = served
    aggregate = (await nodes(platform(machine)))["Diagnostics.WorstLevel"]

    assert await status_of(aggregate) == NO_COMMUNICATION.value

    runtime.note_publishers("/diagnostics", 1)

    await until_status(aggregate, WAITING_FOR_INITIAL_DATA.value)


# -- a member whose element holds a list of messages, over many messages -----------------------


async def test_the_pairs_of_a_member_follow_every_message_that_names_it(served, runtime: FakeRuntime) -> None:
    server, machine = served
    publish(
        runtime,
        fixture_messages.unit(name="motor", values=(("temperature", "41.0"),)),
        fixture_messages.unit(name="brake", values=(("wear", "7"), ("cycles", "1200"))),
    )
    await until_member(machine, "Diagnostics", "robot_1_motor")
    await until_member(machine, "Diagnostics", "robot_1_brake")
    found = await nodes(platform(machine))
    motor = leaf(platform(machine).monitoring, "Diagnostics.robot_1_motor.Values")
    brake = leaf(platform(machine).monitoring, "Diagnostics.robot_1_brake.Values")

    async with connected(server, "visitor") as client:
        motor_node = client.get_node(found["Diagnostics.robot_1_motor.Values"].nodeid)
        brake_node = client.get_node(found["Diagnostics.robot_1_brake.Values"].nodeid)
        await until_elements(motor_node, motor, [{"key": "temperature", "value": "41.0"}])
        await until_elements(brake_node, brake, [{"key": "wear", "value": "7"}, {"key": "cycles", "value": "1200"}])

        publish(
            runtime,
            fixture_messages.unit(name="motor", values=(("temperature", "44.5"), ("fan", "on"), ("rpm", "3000"))),
            fixture_messages.unit(name="brake", values=()),
        )
        await until_elements(
            motor_node,
            motor,
            [{"key": "temperature", "value": "44.5"}, {"key": "fan", "value": "on"}, {"key": "rpm", "value": "3000"}],
        )
        await until_elements(brake_node, brake, [])

        publish(runtime, fixture_messages.unit(name="brake", values=(("wear", "8"),)))
        await until_elements(brake_node, brake, [{"key": "wear", "value": "8"}])
        assert elements_of(motor, await motor_node.read_value()) == [
            {"key": "temperature", "value": "44.5"},
            {"key": "fan", "value": "on"},
            {"key": "rpm", "value": "3000"},
        ]


async def test_the_pairs_go_bad_with_the_publisher_and_come_back_whole_with_it(served, runtime: FakeRuntime) -> None:
    server, machine = served
    publish(runtime, fixture_messages.unit(name="motor", values=(("temperature", "41.0"), ("fan", "on"))))
    await until_member(machine, "Diagnostics", "robot_1_motor")
    found = await nodes(platform(machine))
    variable = leaf(platform(machine).monitoring, "Diagnostics.robot_1_motor.Values")
    pairs: list[dict[str, object]] = [{"key": "temperature", "value": "41.0"}, {"key": "fan", "value": "on"}]

    async with connected(server, "visitor") as client:
        node = client.get_node(found["Diagnostics.robot_1_motor.Values"].nodeid)
        await until_elements(node, variable, pairs)
        assert await status_of(node) == GOOD.value

        runtime.note_publishers("/diagnostics", 0)
        await until_status(node, NO_COMMUNICATION.value)

        runtime.note_publishers("/diagnostics", 1)
        await until_status(node, GOOD.value)
        await until_elements(node, variable, pairs)


async def test_a_client_subscribed_to_the_pairs_is_told_when_they_change(served, runtime: FakeRuntime) -> None:
    server, machine = served
    publish(runtime, fixture_messages.unit(name="motor", values=(("temperature", "41.0"),)))
    await until_member(machine, "Diagnostics", "robot_1_motor")
    found = await nodes(platform(machine))
    variable = leaf(platform(machine).monitoring, "Diagnostics.robot_1_motor.Values")
    changes: list[Any] = []

    class Handler:
        def datachange_notification(self, node: Any, val: Any, data: Any) -> None:
            changes.append(val)

    async with connected(server, "visitor") as client:
        subscription = await client.create_subscription(20, Handler())
        await subscription.subscribe_data_change(client.get_node(found["Diagnostics.robot_1_motor.Values"].nodeid))
        publish(runtime, fixture_messages.unit(name="motor", values=(("temperature", "50.0"), ("fan", "off"))))
        await _until(
            lambda: any(
                elements_of(variable, each) == [{"key": "temperature", "value": "50.0"}, {"key": "fan", "value": "off"}]
                for each in changes
            ),
            f"the subscriber was told of {[elements_of(variable, each) for each in changes]}",
        )
        await subscription.delete()


async def test_a_level_change_is_reported_once_while_the_pairs_change_with_it(served, runtime: FakeRuntime) -> None:
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        subscription = await binding_events(client, machine, collector)
        publish(runtime, fixture_messages.unit(name="motor", level=0, values=(("temperature", "41.0"),)))
        await until_member(machine, "Diagnostics", "robot_1_motor")
        publish(runtime, fixture_messages.unit(name="motor", level=2, values=(("temperature", "99.0"), ("fan", "off"))))
        moved = await collector.events_of(MEMBER_STATE_CHANGED, 1)
        variable = leaf(platform(machine).monitoring, "Diagnostics.robot_1_motor.Values")
        node = client.get_node((await nodes(platform(machine)))["Diagnostics.robot_1_motor.Values"].nodeid)
        await until_elements(node, variable, [{"key": "temperature", "value": "99.0"}, {"key": "fan", "value": "off"}])
        await subscription.delete()

    assert len(moved) == 1
    assert "ERROR (2)" in moved[0].Message.Text


async def test_the_repeated_field_is_one_leaf_to_the_selector_so_exclude_removes_all_of_it(
    runtime: FakeRuntime,
) -> None:
    class WithoutPairs(TestComponent):
        bindings = (replace(DIAGNOSTICS, exclude=("values",)),)

    server, machine = await start(runtime, WithoutPairs(name="Platform"))
    try:
        publish(runtime, fixture_messages.unit(name="motor", values=(("temperature", "41.0"),)))
        await until_member(machine, "Diagnostics", "robot_1_motor")
        found = await nodes(platform(machine))
    finally:
        await server.shutdown()

    assert sorted(name for name in found if name.startswith("Diagnostics.robot_1_motor.")) == [
        "Diagnostics.robot_1_motor.HardwareId",
        "Diagnostics.robot_1_motor.Key",
        "Diagnostics.robot_1_motor.Level",
        "Diagnostics.robot_1_motor.Message",
        "Diagnostics.robot_1_motor.Name",
    ]


async def test_a_member_without_pairs_reads_an_empty_array_under_good(served, runtime) -> None:
    server, machine = served
    publish(runtime, fixture_messages.unit(name="motor"))
    await until_member(machine, "Diagnostics", "robot_1_motor")
    found = await nodes(platform(machine))
    variable = leaf(platform(machine).monitoring, "Diagnostics.robot_1_motor.Values")

    async with connected(server, "visitor") as client:
        node = client.get_node(found["Diagnostics.robot_1_motor.Values"].nodeid)
        await until_status(node, GOOD.value)
        assert elements_of(variable, await node.read_value()) == []


# -- what a declaration is refused for ----------------------------------------------------------


async def test_a_field_that_does_not_repeat_a_message_is_fatal_at_init(runtime: FakeRuntime) -> None:
    class Flat(TestComponent):
        bindings = (
            KeyedBinding(
                name="Flat",
                path="/diagnostics",
                message_type=UNITS,
                repeated_field="header",
                key=("a",),
            ),
        )

    with pytest.raises(UnprojectableFieldError, match="'header'"):
        await start(runtime, Flat(name="Platform"))


async def test_a_key_field_the_element_does_not_have_is_fatal_at_init(runtime: FakeRuntime) -> None:
    class Mistyped(TestComponent):
        bindings = (
            KeyedBinding(
                name="Typo",
                path="/diagnostics",
                message_type=UNITS,
                repeated_field="units",
                key=("hardwareid",),
            ),
        )

    with pytest.raises(DeclarationError, match="'hardwareid'"):
        await start(runtime, Mistyped(name="Platform"))


async def test_a_state_field_the_member_does_not_render_is_fatal_at_init(runtime: FakeRuntime) -> None:
    class Blind(TestComponent):
        bindings = (
            KeyedBinding(
                name="Blind",
                path="/diagnostics",
                message_type=UNITS,
                repeated_field="units",
                key=("name",),
                state_field="level",
                exclude=("level",),
            ),
        )

    with pytest.raises(DeclarationError, match="'level'"):
        await start(runtime, Blind(name="Platform"))


async def test_an_element_that_already_renders_a_key_is_fatal_at_init(runtime: FakeRuntime) -> None:
    class KeyNamed(TestComponent):
        bindings = (
            KeyedBinding(
                name="Pairs",
                path="/pairs",
                message_type="fixture_msgs/msg/Unit",
                repeated_field="values",
                key=("key",),
            ),
        )

    with pytest.raises(DeclarationError, match="'Key'"):
        await start(runtime, KeyNamed(name="Platform"))


def test_a_binding_that_keys_on_nothing_is_refused_where_it_is_written() -> None:
    with pytest.raises(DeclarationError, match="keys on nothing"):
        KeyedBinding(name="Keyless", path="/diagnostics", message_type=UNITS, repeated_field="units", key=())


def test_state_labels_without_a_state_field_are_refused_where_they_are_written() -> None:
    with pytest.raises(DeclarationError, match="state_field"):
        KeyedBinding(
            name="Mute",
            path="/diagnostics",
            message_type=UNITS,
            repeated_field="units",
            key=("name",),
            states={0: "OK"},
        )


def test_a_binding_file_can_declare_one_without_naming_a_python_class(tmp_path) -> None:
    path = tmp_path / "bindings.toml"
    path.write_text(
        "\n".join(
            [
                "[[keyed]]",
                'name = "Diagnostics"',
                'path = "/diagnostics"',
                f'message_type = "{UNITS}"',
                'repeated_field = "units"',
                'key = ["hardware_id", "name"]',
                'state_field = "level"',
                "",
                "[keyed.states]",
                '0 = "OK"',
                '2 = "ERROR"',
                "",
                "[keyed.severities]",
                "0 = 0",
                "2 = 999",
            ]
        ),
        encoding="utf-8",
    )

    (declared,) = load_bindings(path)

    assert isinstance(declared, KeyedBinding)
    assert declared.key == ("hardware_id", "name")
    assert declared.states == {0: "OK", 2: "ERROR"}
    assert declared.severities == {0: Severity.INFO, 2: Severity.ERROR}


async def test_a_keyed_binding_is_satisfied_by_a_publisher_and_not_by_a_member(served, runtime: FakeRuntime) -> None:
    _, machine = served

    runtime.note_publishers("/diagnostics", 1)
    await _until(
        lambda: platform(machine).ros2_keyed_presence.get("Diagnostics") is True,
        "the Keyed Binding never saw its publisher",
    )

    assert platform(machine).ros2_members["Diagnostics"] == ()


async def test_a_component_stops_reading_its_keyed_topics_when_it_shuts_down(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime)

    publish(runtime, fixture_messages.unit(name="motor"))
    await until_member(machine, "Diagnostics", "robot_1_motor")

    await server.shutdown()

    assert platform(machine).ros2_keyed_presence == {}


# -- one write call per message ------------------------------------------------------------


async def test_every_node_of_one_member_is_written_in_one_call(served, runtime: FakeRuntime) -> None:
    server, machine = served
    publish(runtime, fixture_messages.unit(hardware_id="robot_1", name="motor"))
    await until_member(machine, "Diagnostics", "robot_1_motor")
    await asyncio.sleep(0.05)
    service = server.ua_server.iserver.attribute_service
    original, calls = service.write, []

    async def counting(params: ua.WriteParameters, user: Any = None) -> list[ua.StatusCode]:
        calls.append(len(params.NodesToWrite))
        return await original(params, user)

    service.write = counting  # type: ignore[method-assign]

    publish(runtime, fixture_messages.unit(hardware_id="robot_1", name="motor", level=1, message="warm"))
    await _until(lambda: sum(calls) >= len(MEMBER_VARIABLES) + 1, f"the writes never came, saw {calls}")
    await asyncio.sleep(0.05)

    assert calls == [len(MEMBER_VARIABLES), 1]  # the member, then the aggregate


# -- the opt-in Machinery layout --------------------------------------------------------------------


class InHealth(TestComponent):
    bindings = (replace(DIAGNOSTICS, group="Health"), replace(FRAMES, group="Health"))


async def test_a_keyed_binding_in_a_group_files_its_members_below_the_folder(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, InHealth(name="Platform"))
    try:
        publish(runtime, fixture_messages.unit(hardware_id="robot_1", name="motor"))
        await until_member(machine, "Diagnostics", "robot_1_motor")

        monitoring = platform(machine).monitoring
        assert list(monitoring.groups) == ["Health"]
        folder = ua.NodeId(ua.Int32(ObjectIds.FolderType))
        assert await monitoring.groups["Health"].ua_node.read_type_definition() == folder
        assert sorted(await nodes(platform(machine))) == [
            "Health.Diagnostics.WorstLevel",
            *(f"Health.Diagnostics.robot_1_motor.{name}" for name in MEMBER_VARIABLES),
        ]
        assert sorted(monitoring.groups["Health"].groups) == ["Diagnostics", "Frames"]
    finally:
        await server.shutdown()
