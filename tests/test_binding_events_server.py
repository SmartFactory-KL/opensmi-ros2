# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""This project's namespace and its one event, against a real in-process server."""

from __future__ import annotations

import asyncio
from datetime import UTC, datetime, timedelta
from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua
from asyncua.ua.object_ids import ObjectIds
from opensmi.core.nodesets import SmartFactoryMachineSetNodeIds

from opensmi.ros2.server.bindings import MonitoringBinding
from opensmi.ros2.server.events import (
    BINDING_EVENT_TYPE,
    NAMESPACE_URI,
    NODESET_PATH,
    SIGNED_BYTE_STRING_TYPE,
    Severity,
)
from opensmi.ros2.server.nodesets import SmartFactoryROS2NodeIds
from tests import fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.served import TestComponent, TestMachine, connected, serve

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from asyncua.client.client import Client
    from opensmi.server import Server

MESSAGE_TYPES = {"fixture_msgs/msg/Pose": fixture_messages.Pose}

CURRENT_POSE = MonitoringBinding(name="CurrentPose", path="/current_pose", message_type="fixture_msgs/msg/Pose")


class Platform(TestComponent):
    bindings = (CURRENT_POSE,)


class Collector:
    """An asyncua subscription handler that keeps what it was sent."""

    def __init__(self) -> None:
        self.events: list[Any] = []

    def event_notification(self, event: Any) -> None:
        self.events.append(event)

    async def next_event(self, within: float = 2.0) -> Any:
        deadline = asyncio.get_running_loop().time() + within
        while not self.events:
            assert asyncio.get_running_loop().time() < deadline, "no event reached the subscriber"
            await asyncio.sleep(0.005)
        return self.events[0]


@pytest.fixture
async def served() -> AsyncIterator[tuple[Server, Any]]:
    machine = TestMachine(Platform(name="Platform"), runtime=FakeRuntime(domain_id=7, message_types=MESSAGE_TYPES))
    server = await serve(machine)
    try:
        yield server, machine
    finally:
        await server.shutdown()


def standard(identifier: int) -> ua.NodeId:
    """Return a NodeId in namespace zero, spelled the way asyncua's narrower types want."""
    return ua.NodeId(ua.Int32(identifier))


def platform(machine: Any) -> Any:
    return machine.added[0]


def house_notification_node(item: Any) -> ua.NodeId:
    """Return the node opensmi fires that machinery item's own messages on."""
    node_id: ua.NodeId = item.notification.ua_node.nodeid
    return node_id


async def subscribed(client: Client, node: ua.NodeId, event_type: ua.NodeId, collector: Collector) -> Any:
    subscription = await client.create_subscription(20, collector)
    await subscription.subscribe_events(client.get_node(node), client.get_node(event_type))
    return subscription


def test_the_namespace_is_shipped_as_a_file() -> None:
    assert NODESET_PATH.is_file()
    assert NAMESPACE_URI in NODESET_PATH.read_text(encoding="utf-8")


def test_the_generated_node_ids_match_the_shipped_file() -> None:
    text = NODESET_PATH.read_text(encoding="utf-8")
    assert NODESET_PATH.name == SmartFactoryROS2NodeIds.FILE_NAME
    assert f'<Model ModelUri="{SmartFactoryROS2NodeIds.URI}"' in text
    assert f'Version="{SmartFactoryROS2NodeIds.VERSION}"' in text
    assert f'NodeId="ns=1;i={int(BINDING_EVENT_TYPE.identifier)}" BrowseName="1:BindingEventType"' in text
    assert f'NodeId="ns=1;i={int(SIGNED_BYTE_STRING_TYPE.identifier)}" BrowseName="1:SignedByteString"' in text


async def test_the_server_carries_the_project_namespace(served) -> None:
    server, machine = served

    async with connected(server, "visitor") as client:
        array = await client.get_namespace_array()

    assert NAMESPACE_URI in array
    assert array.index(NAMESPACE_URI) == machine.ros2_namespace_index


async def test_the_event_type_subtypes_the_base_event_type_and_not_the_house_alert_type(served) -> None:
    server, _ = served

    async with connected(server, "visitor") as client:
        event_type = client.get_node(server.ua_get_node_id(BINDING_EVENT_TYPE))
        parents = await event_type.get_referenced_nodes(ObjectIds.HasSubtype, ua.BrowseDirection.Inverse)

    assert [node.nodeid for node in parents] == [standard(ObjectIds.BaseEventType)]
    assert server.ua_get_node_id(SmartFactoryMachineSetNodeIds.AlertType) not in [node.nodeid for node in parents]


async def test_the_event_type_carries_the_binding_the_path_and_a_code_as_three_flat_strings(served) -> None:
    server, _ = served

    async with connected(server, "visitor") as client:
        event_type = client.get_node(server.ua_get_node_id(BINDING_EVENT_TYPE))
        properties = await event_type.get_properties()
        names = [(await node.read_browse_name()).Name for node in properties]
        data_types = {await node.read_data_type() for node in properties}
        access = [await node.get_access_level() for node in properties]

    assert sorted(names) == ["Binding", "Code", "ROS2Path"]
    assert data_types == {standard(ObjectIds.String)}
    assert access == [{ua.AccessLevel.CurrentRead}] * 3


async def test_the_signed_byte_string_datatype_subtypes_the_byte_string(served) -> None:
    server, _ = served

    async with connected(server, "visitor") as client:
        blob = client.get_node(server.ua_get_node_id(SIGNED_BYTE_STRING_TYPE))
        parents = await blob.get_referenced_nodes(ObjectIds.HasSubtype, ua.BrowseDirection.Inverse)
        browse_name = await blob.read_browse_name()

    assert browse_name.Name == "SignedByteString"
    assert [node.nodeid for node in parents] == [standard(ObjectIds.ByteString)]


def test_the_error_and_stale_bands_are_the_same_number() -> None:
    assert (Severity.INFO, Severity.WARNING, Severity.ERROR) == (0, 555, 999)
    assert Severity.STALE is Severity.ERROR


async def test_an_event_fired_on_the_house_notification_node_reaches_a_subscriber(served) -> None:
    # asyncua routes events by exact NodeId, so a wrong source node is silence, not an error.
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        await subscribed(
            client,
            house_notification_node(platform(machine)),
            server.ua_get_node_id(BINDING_EVENT_TYPE),
            collector,
        )

        await platform(machine).emit_binding_event(
            binding="current_pose",
            path="/current_pose",
            code="NO_PUBLISHER",
            severity=Severity.WARNING,
        )
        event = await collector.next_event()

    assert (event.Binding, event.ROS2Path, event.Code) == ("current_pose", "/current_pose", "NO_PUBLISHER")
    assert event.Severity == 555


async def test_the_machines_own_notification_node_reaches_a_subscriber_too(served) -> None:
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        await subscribed(client, house_notification_node(machine), server.ua_get_node_id(BINDING_EVENT_TYPE), collector)

        await machine.emit_binding_event(
            binding="lifecycle", path="/map_server", code="NODE_TERMINAL", severity=Severity.ERROR
        )
        event = await collector.next_event()

    assert (event.Binding, event.ROS2Path, event.Code) == ("lifecycle", "/map_server", "NODE_TERMINAL")


async def test_the_code_is_a_field_rather_than_a_delimiter_packed_message(served) -> None:
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        await subscribed(
            client,
            house_notification_node(platform(machine)),
            server.ua_get_node_id(BINDING_EVENT_TYPE),
            collector,
        )

        await platform(machine).emit_binding_event(
            binding="current_pose",
            path="/current_pose",
            code="QOS_INCOMPATIBLE",
            severity=Severity.ERROR,
            text="the publisher offers BEST_EFFORT and the Binding declared RELIABLE",
        )
        event = await collector.next_event()

    assert event.Message.Text == "the publisher offers BEST_EFFORT and the Binding declared RELIABLE"
    assert event.Code == "QOS_INCOMPATIBLE"


async def test_a_client_reads_an_event_back_from_the_history_with_its_own_fields(served) -> None:
    server, machine = served
    item = platform(machine)
    start = datetime.now(UTC) - timedelta(seconds=1)

    await item.emit_binding_event(
        binding="current_pose", path="/current_pose", code="NO_PUBLISHER", severity=Severity.WARNING
    )
    async with connected(server) as client:
        node = client.get_node(house_notification_node(item))
        events: list[Any] = []
        async with asyncio.timeout(5):  # the history stores events asynchronously
            while not events:
                events = await node.read_event_history(start, None, 0, server.ua_get_node_id(BINDING_EVENT_TYPE))
                await asyncio.sleep(0.05)

    assert [(e.Binding, e.ROS2Path, e.Code) for e in events] == [("current_pose", "/current_pose", "NO_PUBLISHER")]


async def test_an_event_names_the_machinery_item_that_fired_it_as_its_source(served) -> None:
    server, machine = served
    collector = Collector()
    item = platform(machine)

    async with connected(server) as client:
        await subscribed(client, house_notification_node(item), server.ua_get_node_id(BINDING_EVENT_TYPE), collector)

        await item.emit_binding_event(
            binding="current_pose", path="/current_pose", code="NO_PUBLISHER", severity=Severity.WARNING
        )
        event = await collector.next_event()

    assert event.SourceNode == item.ua_node.nodeid
    assert event.SourceName == item.name


async def test_a_fired_event_is_kept_on_the_machines_ring(served) -> None:
    _, machine = served

    await platform(machine).emit_binding_event(
        binding="current_pose", path="/current_pose", code="NO_PUBLISHER", severity=Severity.WARNING
    )

    (record,) = machine.ros2_recent_events
    assert (record.binding, record.path, record.code, record.severity) == (
        "current_pose",
        "/current_pose",
        "NO_PUBLISHER",
        Severity.WARNING,
    )
    assert record.item.endswith("/Platform")


async def test_a_client_subscribed_to_the_server_object_sees_nothing(served) -> None:
    # asyncua has no event propagation up the notifier hierarchy; asserted so a new version shows.
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        await subscribed(client, standard(ObjectIds.Server), server.ua_get_node_id(BINDING_EVENT_TYPE), collector)

        await platform(machine).emit_binding_event(
            binding="current_pose", path="/current_pose", code="NO_PUBLISHER", severity=Severity.WARNING
        )
        await asyncio.sleep(0.2)

    assert collector.events == []


async def test_the_model_change_event_fires_on_the_node_the_specification_names(served) -> None:
    server, machine = served
    collector = Collector()
    monitoring = platform(machine).monitoring.ua_node.nodeid
    changed = ua.ModelChangeStructureDataType(
        Affected=monitoring,
        AffectedType=standard(ObjectIds.BaseObjectType),
        Verb=ua.Byte(1),
    )

    async with connected(server) as client:
        await subscribed(
            client,
            machine.spatial_object_list.ua_node.nodeid,
            standard(ObjectIds.GeneralModelChangeEventType),
            collector,
        )

        await machine.emit_model_change([changed])
        event = await collector.next_event()

    assert [change.Affected for change in event.Changes] == [monitoring]


async def test_a_model_change_with_nothing_in_it_is_not_announced(served) -> None:
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        await subscribed(
            client,
            machine.spatial_object_list.ua_node.nodeid,
            standard(ObjectIds.GeneralModelChangeEventType),
            collector,
        )

        await machine.emit_model_change([])
        await asyncio.sleep(0.2)

    assert collector.events == []


async def test_every_machinery_item_that_declares_bindings_can_raise_one(served) -> None:
    _, machine = served

    assert house_notification_node(platform(machine)) != house_notification_node(machine)
