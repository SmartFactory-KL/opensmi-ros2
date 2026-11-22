# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""QoS provenance where a Binding, a Runtime and a client meet.

The fake Runtime raises the match and clash events; the effect of a profile on the wire is checked
in `test_runtime_jazzy.py`.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua

from opensmi.ros2.runtime.qos import (
    DEFAULT_PROFILE,
    LATCHED,
    MOST_PERMISSIVE,
    SENSOR_DATA,
    Durability,
    Policy,
    Provenance,
    QoSProfile,
    Reliability,
)
from opensmi.ros2.server.bindings import KeyedBinding, MonitoringBinding, PublisherBinding
from opensmi.ros2.server.events import BINDING_EVENT_TYPE, Severity
from opensmi.ros2.server.provenance import QOS_INCOMPATIBLE
from tests import fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.served import TestComponent, TestMachine, connected, monitoring_nodes, serve

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from asyncua.client.client import Client
    from opensmi.server import Server

MESSAGE_TYPES = {
    "fixture_msgs/msg/Pose": fixture_messages.Pose,
    "fixture_msgs/msg/Point": fixture_messages.Point,
    "fixture_msgs/msg/Units": fixture_messages.Units,
    "fixture_msgs/msg/Unit": fixture_messages.Unit,
    "fixture_msgs/msg/Pair": fixture_messages.Pair,
    "fixture_msgs/msg/Header": fixture_messages.Header,
    "fixture_msgs/msg/Time": fixture_messages.Time,
}

SETTLED = 5.0

CURRENT_POSE = MonitoringBinding(
    name="CurrentPose",
    path="/current_pose",
    message_type="fixture_msgs/msg/Pose",
)
SCAN = MonitoringBinding(
    name="Scan",
    path="/scan",
    message_type="fixture_msgs/msg/Pose",
    qos=QoSProfile(reliability=Reliability.RELIABLE, durability=Durability.TRANSIENT_LOCAL),
)
LATE = MonitoringBinding(name="Late", path="/late")

DIAGNOSTICS = KeyedBinding(
    name="Diagnostics",
    path="/diagnostics",
    message_type="fixture_msgs/msg/Units",
    repeated_field="units",
    key=("name",),
    include=("name", "message"),
    qos=SENSOR_DATA,
)

DRIVE = PublisherBinding(
    name="Drive",
    path="/cmd_vel",
    message_type="fixture_msgs/msg/Point",
    continuous=True,
    rate=10.0,
)
PUSH = PublisherBinding(
    name="Push",
    path="/setpoint",
    message_type="fixture_msgs/msg/Point",
    continuous=False,
    qos=LATCHED,
)


class Platform(TestComponent):
    bindings = (CURRENT_POSE, SCAN, LATE, DIAGNOSTICS, DRIVE, PUSH)


class Collector:
    """An asyncua subscription handler that keeps what it was sent."""

    def __init__(self) -> None:
        self.events: list[Any] = []

    def event_notification(self, event: Any) -> None:
        self.events.append(event)

    async def next_event(self, within: float = SETTLED) -> Any:
        deadline = asyncio.get_running_loop().time() + within
        while not self.events:
            assert asyncio.get_running_loop().time() < deadline, "no event reached the subscriber"
            await asyncio.sleep(0.005)
        return self.events[0]


@pytest.fixture
def runtime() -> FakeRuntime:
    return FakeRuntime(domain_id=7, message_types=MESSAGE_TYPES)


async def start(runtime: FakeRuntime) -> tuple[Server, Any]:
    """Bring up a server carrying one Machine with one Component, and hand back both."""
    machine = TestMachine(Platform(name="Platform"), runtime=runtime)
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


def subscription(machine: Any, topic: str) -> Any:
    """Return the live subscription one Binding opened, by the topic it is on."""
    (found,) = [each for each in platform(machine).ros2_subscriptions if each.topic == topic]
    return found


async def settles() -> None:
    """Let the tasks a middleware event woke run to the point where they have written."""
    for _ in range(50):
        await asyncio.sleep(0)


async def binding_events(client: Client, machine: Any, collector: Collector) -> None:
    handle = await client.create_subscription(50, collector)
    await handle.subscribe_events(
        client.get_node(platform(machine).notification.ua_node.nodeid),
        client.get_node(machine.server.ua_get_node_id(BINDING_EVENT_TYPE)),
    )


# --- provenance, per Binding -------------------------------------------------------------------


async def test_a_declared_profile_reaches_the_subscription_exactly_as_written(served) -> None:
    _, machine = served

    assert subscription(machine, "/scan").qos == SCAN.qos
    assert subscription(machine, "/scan").qos_provenance is Provenance.DECLARED


async def test_a_binding_that_declares_nothing_discovers_from_what_is_publishing(runtime) -> None:
    runtime.set_publisher_qos("/current_pose", SENSOR_DATA, DEFAULT_PROFILE)
    server, machine = await start(runtime)
    try:
        opened = subscription(machine, "/current_pose")

        assert opened.qos.reliability is Reliability.BEST_EFFORT
        assert opened.qos_provenance is Provenance.DISCOVERED
    finally:
        await server.shutdown()


async def test_a_topic_with_no_publisher_at_init_opens_maximally_permissive(served) -> None:
    _, machine = served

    assert subscription(machine, "/current_pose").qos == MOST_PERMISSIVE


async def test_the_provenance_entry_names_every_bound_binding_and_its_profile(served) -> None:
    _, machine = served

    reported = platform(machine).ros2_qos_provenance

    assert sorted(reported) == ["CurrentPose", "Scan"]
    assert reported["Scan"].provenance is Provenance.DECLARED
    assert reported["Scan"].profile == SCAN.qos
    assert reported["Scan"].path == "/scan"
    assert reported["Scan"].adaptations == 0
    assert reported["Scan"].last_incompatibility is None


async def test_a_binding_still_waiting_for_its_topic_has_no_provenance_entry(served) -> None:
    _, machine = served

    assert "Late" not in platform(machine).ros2_qos_provenance
    assert "Late" in platform(machine).ros2_monitoring_presence


# --- the one bootstrap tightening ---------------------------------------------------------------


async def test_a_latched_publisher_appearing_later_tightens_the_subscription_once(served, runtime) -> None:
    _, machine = served
    opened = subscription(machine, "/current_pose")
    assert opened.qos == MOST_PERMISSIVE

    runtime.set_publisher_qos("/current_pose", LATCHED)
    runtime.note_publishers("/current_pose", 1)
    await settles()

    # Without this the topic is silent and empty for ever.
    assert opened.qos.durability is Durability.TRANSIENT_LOCAL
    assert platform(machine).ros2_qos_provenance["CurrentPose"].profile.durability is Durability.TRANSIENT_LOCAL


async def test_the_bootstrap_is_spent_once_and_a_later_publisher_does_not_tighten_again(served, runtime) -> None:
    _, machine = served
    opened = subscription(machine, "/current_pose")

    runtime.set_publisher_qos("/current_pose", SENSOR_DATA)
    runtime.note_publishers("/current_pose", 1)
    await settles()
    relaxed = opened.qos

    runtime.set_publisher_qos("/current_pose", LATCHED)
    runtime.note_publishers("/current_pose", 2)
    await settles()

    assert opened.qos == relaxed


async def test_a_declared_binding_never_bootstraps(served, runtime) -> None:
    _, machine = served

    runtime.set_publisher_qos("/scan", SENSOR_DATA)
    runtime.note_publishers("/scan", 1)
    await settles()

    assert subscription(machine, "/scan").qos == SCAN.qos


# --- adaptation, and the event that reports it ---------------------------------------------------


async def test_a_discovered_binding_relaxes_on_an_incompatibility_and_keeps_its_handle(served, runtime) -> None:
    _, machine = served
    opened = subscription(machine, "/current_pose")
    runtime.set_publisher_qos("/current_pose", DEFAULT_PROFILE)
    runtime.note_publishers("/current_pose", 1)
    await settles()
    assert opened.qos.reliability is Reliability.RELIABLE

    runtime.set_publisher_qos("/current_pose", DEFAULT_PROFILE, SENSOR_DATA)
    runtime.note_qos_incompatible("/current_pose", Policy.RELIABILITY)
    await settles()

    assert opened.qos.reliability is Reliability.BEST_EFFORT
    assert subscription(machine, "/current_pose") is opened
    assert platform(machine).ros2_qos_provenance["CurrentPose"].adaptations == 1


async def test_a_declared_binding_reports_a_clash_and_does_not_move(served, runtime) -> None:
    _, machine = served

    runtime.set_publisher_qos("/scan", SENSOR_DATA)
    runtime.note_qos_incompatible("/scan", Policy.DURABILITY)
    await settles()

    entry = platform(machine).ros2_qos_provenance["Scan"]
    assert entry.profile == SCAN.qos
    assert entry.adaptations == 0
    assert entry.last_incompatibility is not None
    assert entry.last_incompatibility.policy is Policy.DURABILITY
    assert entry.last_incompatibility.adapted is None


async def test_a_clash_reaches_a_subscribed_client_as_one_binding_event(served, runtime) -> None:
    server, machine = served
    collector = Collector()

    runtime.set_publisher_qos("/current_pose", DEFAULT_PROFILE)
    runtime.note_publishers("/current_pose", 1)
    await settles()

    async with connected(server) as client:
        await binding_events(client, machine, collector)

        runtime.set_publisher_qos("/current_pose", DEFAULT_PROFILE, SENSOR_DATA)
        runtime.note_qos_incompatible("/current_pose", Policy.RELIABILITY)

        event = await collector.next_event()

    assert event.Code == QOS_INCOMPATIBLE
    assert event.Binding == "CurrentPose"
    assert event.ROS2Path == "/current_pose"
    assert event.Severity == Severity.WARNING
    assert "RELIABILITY" in event.Message.Text


async def test_a_clash_a_declared_binding_cannot_recover_from_is_an_error(served, runtime) -> None:
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        await binding_events(client, machine, collector)

        runtime.set_publisher_qos("/scan", SENSOR_DATA)
        runtime.note_qos_incompatible("/scan", Policy.RELIABILITY)

        event = await collector.next_event()

    assert event.Code == QOS_INCOMPATIBLE
    assert event.Severity == Severity.ERROR
    assert "declared" in event.Message.Text


# --- the outward direction ------------------------------------------------------------------------


async def test_a_publishing_binding_that_declares_a_profile_offers_it(served) -> None:
    _, machine = served

    entry = platform(machine).skill_set["Push"].ros2_qos_provenance

    assert entry is not None
    assert entry.profile == LATCHED
    assert entry.provenance is Provenance.DECLARED


async def test_a_publishing_binding_that_declares_nothing_is_defaulted_and_never_discovered(served, runtime) -> None:
    _, machine = served
    runtime.set_publisher_qos("/cmd_vel", SENSOR_DATA)

    entry = platform(machine).skill_set["Drive"].ros2_qos_provenance

    assert entry is not None
    assert entry.profile == DEFAULT_PROFILE
    assert entry.provenance is Provenance.DEFAULTED
    assert entry.adaptations == 0


# --- nothing about QoS is fatal ---------------------------------------------------------------------


async def test_a_declared_profile_provably_incompatible_at_init_still_boots(runtime) -> None:
    # Not fatal: it would depend on whether the publisher happened to be up.
    runtime.set_publisher_qos("/scan", SENSOR_DATA)
    server, machine = await start(runtime)
    try:
        assert subscription(machine, "/scan").qos == SCAN.qos
    finally:
        await server.shutdown()


async def test_a_variable_still_reads_its_topic_after_an_adaptation(served, runtime) -> None:
    server, machine = served
    node = monitoring_nodes(platform(machine))["CurrentPose.Position.X"]

    runtime.set_publisher_qos("/current_pose", DEFAULT_PROFILE, SENSOR_DATA)
    runtime.note_publishers("/current_pose", 2)
    runtime.note_qos_incompatible("/current_pose", Policy.RELIABILITY)
    await settles()
    runtime.deliver("/current_pose", fixture_messages.pose_stamped(x=2.5).pose)

    async with connected(server) as client:
        remote = client.get_node(node.nodeid)
        deadline = asyncio.get_running_loop().time() + SETTLED
        while await read(remote) != 2.5:
            assert asyncio.get_running_loop().time() < deadline, "the value never arrived"
            await asyncio.sleep(0.01)

    assert (await node.read_value()) == 2.5


async def read(node: Any) -> object:
    """Return what a client reads from one node, with its status code and all."""
    value: ua.DataValue = await node.read_data_value()
    return None if value.Value is None else value.Value.Value


# --- a Keyed Binding is a topic Binding, and gets the same three things ---------------------------


async def test_a_keyed_binding_declares_a_profile_and_answers_the_same_entry(served) -> None:
    _, machine = served

    entry = platform(machine).ros2_keyed_qos_provenance["Diagnostics"]

    assert entry.path == "/diagnostics"
    assert entry.profile == SENSOR_DATA
    assert entry.provenance is Provenance.DECLARED
    assert entry.adaptations == 0


async def test_a_clash_on_a_keyed_binding_is_the_same_event(served, runtime) -> None:
    server, machine = served
    collector = Collector()

    async with connected(server) as client:
        await binding_events(client, machine, collector)

        runtime.set_publisher_qos("/diagnostics", LATCHED)
        runtime.note_qos_incompatible("/diagnostics", Policy.DURABILITY)

        event = await collector.next_event()

    assert event.Code == QOS_INCOMPATIBLE
    assert event.Binding == "Diagnostics"
    assert event.ROS2Path == "/diagnostics"
    assert event.Severity == Severity.ERROR


# --- and the clash is reported once, not once per re-announcement ----------------------------------


async def test_a_publisher_that_flaps_reports_its_clash_once(served, runtime) -> None:
    _, machine = served

    runtime.set_publisher_qos("/scan", SENSOR_DATA)
    for _ in range(5):
        runtime.note_qos_incompatible("/scan", Policy.RELIABILITY)
    await settles()

    assert len(subscription(machine, "/scan").qos_incompatibilities) == 1


async def test_a_second_policy_clashing_is_a_second_fault(served, runtime) -> None:
    _, machine = served

    runtime.set_publisher_qos("/scan", SENSOR_DATA)
    runtime.note_qos_incompatible("/scan", Policy.RELIABILITY)
    runtime.note_qos_incompatible("/scan", Policy.DURABILITY)
    await settles()

    reported = subscription(machine, "/scan").qos_incompatibilities
    assert [each.policy for each in reported] == [Policy.RELIABILITY, Policy.DURABILITY]
