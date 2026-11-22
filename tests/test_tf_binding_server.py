# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The TF tree as RSL Locations, against a real in-process opensmi server.

The arrangement is OPC 10000-210 Annex C.2: the Machine declares `unit_base`, a Component declares
`robot_base`. A client writes a Location with a `3DFrame`. Placement is in `test_spatial.py`.
"""

from __future__ import annotations

import asyncio
import gc
import math
from typing import TYPE_CHECKING, Any, cast

import pytest
from asyncua import ua
from asyncua.ua.attribute_ids import AttributeIds
from asyncua.ua.object_ids import ObjectIds
from opensmi.core.nodesets import RSLNodeIds
from opensmi.server.spatial_object import CartesianFrame, Orientation, Position

from opensmi.ros2.runtime.tf import TF_STATIC_TOPIC, TF_TOPIC, Transform
from opensmi.ros2.server.bindings import MonitoringBinding, PublishedFrame, TFBinding
from opensmi.ros2.server.config import ROS2Config, TransformConfig
from opensmi.ros2.server.errors import DeclarationError, FrameDirectionError
from opensmi.ros2.server.events import BINDING_EVENT_TYPE, Severity
from opensmi.ros2.server.mutation import CAP_REACHED
from opensmi.ros2.server.publisher import PUBLISH_FAILED
from opensmi.ros2.server.spatial import ALTERNATIVE_FRAMES, ATTACH_POINTS, INTERNAL_FRAMES
from opensmi.ros2.server.status import GOOD, NO_COMMUNICATION, WAITING_FOR_INITIAL_DATA
from opensmi.ros2.server.transforms import FRAME_COLLISION, METRES, SEPARATE_FRAME_TREES, inverted
from tests import fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.served import TestComponent, TestMachine, connected, serve

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Awaitable, Callable

    from asyncua.client.client import Client
    from asyncua.common.node import Node
    from opensmi.server import Server

MESSAGE_TYPES = {
    "tf2_msgs/msg/TFMessage": fixture_messages.TFMessage,
    "fixture_msgs/msg/PoseStamped": fixture_messages.PoseStamped,
}

#: The linear unit is the Machine; the arm bolted to its carriage is a Component.
UNIT = TFBinding(name="UnitFrames", head="unit_base")
ARM = TFBinding(
    name="ArmFrames",
    head="robot_base",
    alternatives=(("map", "tool0"),),
    exclude=("*_cable",),
)

#: The Machine declares where TF is read from and how often, once.
SETTINGS = ROS2Config(transforms=TransformConfig(interval=0.02))


class Arm(TestComponent):
    bindings = (ARM,)


class Capped(Arm):
    # A Location and its folder are two nodes, so a cap of two admits the first frame and refuses the rest.
    mutation_node_cap = 2


class Robot(TestMachine):
    bindings = (UNIT,)


class Untidy(Robot):
    """A Machine that declares something a Machine cannot expose."""

    bindings = (
        UNIT,
        MonitoringBinding(name="Pose", path="/pose", message_type="fixture_msgs/msg/PoseStamped"),
    )


@pytest.fixture
def runtime() -> FakeRuntime:
    return FakeRuntime(domain_id=7, message_types=MESSAGE_TYPES)


async def start(
    runtime: FakeRuntime,
    machine_type: type[TestMachine] = Robot,
    arm: TestComponent | None = None,
    config: ROS2Config = SETTINGS,
) -> tuple[Server, Any]:
    """Bring up an in-process server carrying one Machine with its arm, and hand back both."""
    machine = machine_type(arm or Arm(name="arm"), name="robot", runtime=runtime, ros2_config=config)
    return await serve(machine), machine


@pytest.fixture
async def served(runtime: FakeRuntime) -> AsyncIterator[tuple[Server, Any]]:
    server, machine = await start(runtime)
    try:
        yield server, machine
    finally:
        await server.shutdown()


def arm_of(machine: Any) -> Any:
    return machine.added[0]


#: map -> unit_base -> unit_carriage -> robot_base -> arm_link -> tool0, plus a static sensor and an excluded cable run.
def publish_tree(runtime: FakeRuntime) -> None:
    runtime.note_publishers(TF_TOPIC, 1)
    runtime.note_publishers(TF_STATIC_TOPIC, 1)
    runtime.deliver(
        TF_STATIC_TOPIC,
        fixture_messages.tf_message(
            fixture_messages.transform_stamped("unit_base", "unit_sensor", position=(0.0, 0.5, 0.0)),
        ),
    )
    runtime.deliver(
        TF_TOPIC,
        fixture_messages.tf_message(
            fixture_messages.transform_stamped("map", "unit_base"),
            fixture_messages.transform_stamped("unit_base", "unit_carriage", position=(1.0, 0.0, 0.0)),
            fixture_messages.transform_stamped("unit_carriage", "robot_base", position=(0.0, 0.0, 0.25)),
            fixture_messages.transform_stamped("robot_base", "arm_link"),
            fixture_messages.transform_stamped("arm_link", "tool0", position=(0.0, 0.0, 0.3)),
            fixture_messages.transform_stamped("arm_link", "arm_cable"),
        ),
    )


#: The same robot with no frame inside either object.
def publish_heads(runtime: FakeRuntime) -> None:
    runtime.note_publishers(TF_TOPIC, 1)
    runtime.deliver(
        TF_TOPIC,
        fixture_messages.tf_message(
            fixture_messages.transform_stamped("map", "unit_base"),
            fixture_messages.transform_stamped("unit_base", "unit_carriage", position=(1.0, 0.0, 0.0)),
            fixture_messages.transform_stamped("unit_carriage", "robot_base", position=(0.0, 0.0, 0.25)),
        ),
    )


async def rendered(machine: Any, frames: set[str], within: float = 3.0) -> None:
    """Wait for the poll to have put every named frame in the index."""
    await _until(
        lambda: frames <= set(machine.ros2_frame_nodes),
        lambda: f"the index holds {sorted(machine.ros2_frame_nodes)}",
        within,
    )


async def _until(ready: Callable[[], bool], complaint: Callable[[], str], within: float = 3.0) -> None:
    deadline = asyncio.get_running_loop().time() + within
    while not ready():
        assert asyncio.get_running_loop().time() < deadline, complaint()
        await asyncio.sleep(0.005)


async def settles_at(read: Callable[[], Awaitable[Any]], expected: object, within: float = 3.0) -> None:
    """Wait for a value the poll writes, because a bare read can land between the index and the write."""
    deadline = asyncio.get_running_loop().time() + within
    while True:
        try:
            seen = await read()
        except ua.UaStatusCodeError as bad:
            seen = bad
        else:
            if seen == expected:
                return
        assert asyncio.get_running_loop().time() < deadline, f"it reads {seen!r}, not {expected!r}"
        await asyncio.sleep(0.005)


def rsl(item: Any) -> int:
    """Return the index the server imported OPC 10000-210's namespace at."""
    return int(item.server.ua_get_namespace_index(RSLNodeIds))


async def spatial_of(item: Any) -> Node:
    """Return the SpatialObject node of one machinery item, found by walking the item itself."""
    for child in await item.ua_node.get_children():
        if (await child.read_browse_name()).Name == "SpatialObject":
            return child
    message = f"{item.name} has no SpatialObject"
    raise AssertionError(message)


async def folder_of(item: Any, name: str) -> Node:
    return await (await spatial_of(item)).get_child(ua.QualifiedName(name, rsl(item)))


async def location_in(item: Any, folder: str, browse: str) -> Node:
    return await (await folder_of(item, folder)).get_child(ua.QualifiedName(browse, rsl(item)))


async def appears(item: Any, folder: str, browse: str, within: float = 3.0) -> Node:
    """Wait for one Location to exist, which a composed one does a tick after its frame."""
    deadline = asyncio.get_running_loop().time() + within
    while True:
        try:
            return await location_in(item, folder, browse)
        except ua.UaStatusCodeError:
            assert asyncio.get_running_loop().time() < deadline, f"{browse} never appeared in {folder}"
            await asyncio.sleep(0.005)


async def head_of(item: Any) -> Node:
    return await (await spatial_of(item)).get_child(ua.QualifiedName("PositionFrame", rsl(item)))


async def browse_names(node: Node) -> set[str]:
    return {(await child.read_browse_name()).Name for child in await node.get_children()}


class Pose:
    """Reads one Location's parts the way a client walks to them, in one server's RSL namespace."""

    def __init__(self, item: Any) -> None:
        self.ns = rsl(item)

    async def base(self, node: Node) -> ua.NodeId:
        return await (await node.get_child(f"{self.ns}:Base")).read_value()

    async def position(self, node: Node) -> tuple[float, ...]:
        held = await node.get_child(f"{self.ns}:Position")
        return tuple([await (await held.get_child(each)).read_value() for each in ("X", "Y", "Z")])

    async def angles(self, node: Node) -> tuple[float, ...]:
        held = await node.get_child(f"{self.ns}:Orientation")
        return tuple([await (await held.get_child(f"{self.ns}:{each}")).read_value() for each in ("A", "B", "C")])

    async def rounded_angles(self, node: Node) -> tuple[float, ...]:
        """Return an angle triple rounded, because a quaternion round trip is not exact."""
        return tuple(round(each, 6) for each in await self.angles(node))

    async def unit(self, node: Node, part: str) -> str:
        """Return the engineering unit one half of a pose carries, as its symbol."""
        holder = "LengthUnit" if part == "Position" else "AngleUnit"
        held = await (await node.get_child(f"{self.ns}:{part}")).get_child(f"0:{holder}")
        unit: ua.EUInformation = await held.read_value()
        return str(unit.DisplayName.Text)

    async def status(self, node: Node, path: str) -> int:
        """Return the status code of one child of a Location as its number, without raising on a bad status."""
        held = node
        for each in path.split("/"):
            held = await held.get_child(each)
        value = await held.read_data_value(raise_on_bad_status=False)
        assert value.StatusCode is not None
        return value.StatusCode.value

    async def statuses(self, node: Node) -> set[int]:
        """Return what all eight value-bearing nodes of one Location read, as one set."""
        position, orientation = f"{self.ns}:Position", f"{self.ns}:Orientation"
        return {
            await self.status(node, position),
            *[await self.status(node, f"{position}/{each}") for each in ("X", "Y", "Z")],
            await self.status(node, orientation),
            *[await self.status(node, f"{orientation}/{self.ns}:{each}") for each in ("A", "B", "C")],
        }

    async def values(self, node: Node, part: str) -> tuple[Any, ...]:
        """Return what the three scalars under one composite read, bad status and all."""
        held = await node.get_child(f"{self.ns}:{part}")
        names = ("X", "Y", "Z") if part == "Position" else tuple(f"{self.ns}:{each}" for each in "ABC")
        found = []
        for each in names:
            value = await (await held.get_child(each)).read_data_value(raise_on_bad_status=False)
            found.append(value.Value.Value if value.Value is not None else None)
        return tuple(found)


async def constant(node: Node) -> bool:
    """Return whether one node claims to be constant, however the server says it does not."""
    answer = await node.read_attribute(AttributeIds.AccessLevelEx, raise_on_bad_status=False)
    if answer.StatusCode is not None and answer.StatusCode.is_bad():
        return False
    if answer.Value is None or answer.Value.Value is None:
        return False
    return bool(answer.Value.Value & int(ua.AccessLevelExType.Constant))


class Collector:
    """An asyncua subscription handler that keeps what it was sent."""

    def __init__(self) -> None:
        self.events: list[Any] = []

    def event_notification(self, event: Any) -> None:
        self.events.append(event)

    def codes(self) -> list[str]:
        return [each.Code for each in self.events]


async def binding_events_of(client: Client, item: Any, collector: Collector) -> None:
    """Subscribe to the BindingEvents one machinery item raises."""
    subscription = await client.create_subscription(20, collector)
    await subscription.subscribe_events(
        client.get_node(item.notification.ua_node.nodeid),
        client.get_node(item.server.ua_get_node_id(BINDING_EVENT_TYPE)),
    )


# --------------------------------------------------------------------------- the structural half


async def test_a_declaring_item_gains_a_spatial_object(served: tuple[Server, Any]) -> None:
    _, machine = served

    assert await spatial_of(machine) is not None
    assert await spatial_of(arm_of(machine)) is not None


async def test_the_list_and_the_spatial_objects_are_add_ins_of_their_machinery_item(
    served: tuple[Server, Any],
) -> None:
    # Per OPC 10000-210 6.1 the list and SpatialObject hang directly under the object by HasAddIn.
    _, machine = served
    ns = rsl(machine)
    add_ins = ua.NodeId(ua.Int32(ObjectIds.HasAddIn))

    async def add_in_names(item: Any) -> set[tuple[int, str]]:
        found = await item.ua_node.get_children(refs=add_ins)
        names = {((name := await each.read_browse_name()).NamespaceIndex, name.Name) for each in found}
        return {each for each in names if each[0] == ns}

    assert await add_in_names(machine) == {(ns, "SpatialObjectList"), (ns, "SpatialObject")}
    assert await add_in_names(arm_of(machine)) == {(ns, "SpatialObject")}
    identifier = await machine.spatial_object_list.ua_node.get_child(f"{ns}:Identifier")
    assert await identifier.read_value() == "robotSpatialObjectList"


async def test_the_head_frame_is_the_mandatory_position_frame(served: tuple[Server, Any]) -> None:
    _, machine = served

    node = await head_of(arm_of(machine))

    assert node.nodeid.to_string() == machine.ros2_frame_nodes["robot_base"]


async def test_both_spatial_objects_are_members_of_the_machines_list(served: tuple[Server, Any]) -> None:
    _, machine = served

    listed = {each.parent for each in machine.spatial_object_list.spatial_objects}

    assert listed == {machine, arm_of(machine)}


async def test_a_machine_may_declare_only_a_tf_binding(runtime: FakeRuntime) -> None:
    with pytest.raises(DeclarationError, match="exposes nothing of its own but where it is"):
        await start(runtime, Untidy)


async def test_nothing_subscribes_to_the_transform_topics_without_a_declaration(runtime: FakeRuntime) -> None:
    server = await serve(TestMachine(TestComponent(name="plain"), name="robot", runtime=runtime))
    try:
        assert runtime.subscribed_topics == ()
    finally:
        await server.shutdown()


async def test_a_machine_that_declares_frames_subscribes_to_both_topics(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    assert runtime.subscribed_topics == (TF_TOPIC, TF_STATIC_TOPIC)


async def test_the_transform_topics_a_deployment_remapped_are_what_every_declaration_reads(
    runtime: FakeRuntime,
) -> None:
    remapped = ROS2Config(transforms=TransformConfig(path="/robot/tf", static_path="/robot/tf_static", interval=0.02))

    server, _ = await start(runtime, config=remapped)
    try:
        assert runtime.subscribed_topics == ("/robot/tf", "/robot/tf_static")
    finally:
        await server.shutdown()


# --------------------------------------------------------------------------- the unit (item 1)


async def test_a_location_is_created_in_metres_and_degrees(served: tuple[Server, Any]) -> None:
    # Declared from the start, before anything publishes: clients read the unit even when the robot is off.
    server, machine = served
    pose = Pose(machine)

    async with connected(server) as client:
        node = client.get_node((await head_of(arm_of(machine))).nodeid)

        assert await pose.unit(node, "Position") == METRES.symbol
        assert await pose.unit(node, "Orientation") == "°"


async def test_a_written_pose_carries_the_unit_it_was_written_in(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    _, machine = served
    pose = Pose(machine)
    publish_heads(runtime)
    await rendered(machine, {"unit_carriage"})
    node = await head_of(arm_of(machine))

    await settles_at(lambda: pose.position(node), (0.0, 0.0, 0.25))

    assert await pose.unit(node, "Position") == METRES.symbol


# --------------------------------------------------------------------------- the partition


async def test_a_frame_lands_in_the_spatial_object_of_the_nearest_declared_head(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    _, machine = served
    publish_tree(runtime)

    await rendered(machine, {"unit_carriage", "unit_sensor", "arm_link", "tool0"})

    assert await browse_names(await folder_of(machine, INTERNAL_FRAMES)) == {"unit_sensor"}
    assert await browse_names(await folder_of(arm_of(machine), INTERNAL_FRAMES)) == {"arm_link", "tool0"}


async def test_an_attach_point_is_the_frame_another_objects_head_hangs_off(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    _, machine = served
    publish_heads(runtime)

    await rendered(machine, {"unit_carriage"})

    assert await browse_names(await folder_of(machine, ATTACH_POINTS)) == {"unit_carriage"}


async def test_an_excluded_frame_is_rendered_by_nobody(served: tuple[Server, Any], runtime: FakeRuntime) -> None:
    _, machine = served
    publish_tree(runtime)

    await rendered(machine, {"tool0"})

    assert "arm_cable" not in machine.ros2_frame_nodes


# --------------------------------------------------------------------------- what a Location holds


async def test_a_base_points_at_the_node_holding_the_tf_parent_wherever_it_lives(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    _, machine = served
    pose = Pose(machine)
    publish_heads(runtime)
    await rendered(machine, {"unit_carriage"})

    head = await head_of(arm_of(machine))

    await settles_at(lambda: pose.base(head), ua.NodeId.from_string(machine.ros2_frame_nodes["unit_carriage"]))


async def test_the_root_of_the_tree_is_the_lists_world_frame(served: tuple[Server, Any], runtime: FakeRuntime) -> None:
    _, machine = served
    pose = Pose(machine)
    publish_heads(runtime)
    await rendered(machine, {"unit_carriage"})

    head = await head_of(machine)

    await settles_at(lambda: pose.base(head), machine.spatial_object_list.world_frame.ua_node.nodeid)


async def test_a_position_is_the_transform_its_parent_published(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    _, machine = served
    publish_tree(runtime)
    await rendered(machine, {"tool0"})

    node = await location_in(arm_of(machine), INTERNAL_FRAMES, "tool0")

    await settles_at(lambda: Pose(machine).position(node), (0.0, 0.0, 0.3))


async def test_an_attach_points_position_is_the_transform_its_parent_published(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    _, machine = served
    publish_heads(runtime)
    await rendered(machine, {"unit_carriage"})

    node = await location_in(machine, ATTACH_POINTS, "unit_carriage")

    await settles_at(lambda: Pose(machine).position(node), (1.0, 0.0, 0.0))


async def test_an_orientation_is_the_quaternion_in_degrees(served: tuple[Server, Any], runtime: FakeRuntime) -> None:
    _, machine = served
    runtime.note_publishers(TF_TOPIC, 1)
    half = math.sin(math.pi / 8)
    runtime.deliver(
        TF_TOPIC,
        fixture_messages.tf_message(
            fixture_messages.transform_stamped("map", "unit_base"),
            fixture_messages.transform_stamped(
                "unit_base",
                "robot_base",
                rotation=(0.0, 0.0, half, math.cos(math.pi / 8)),
            ),
        ),
    )
    await rendered(machine, {"robot_base"})

    node = await head_of(arm_of(machine))

    await settles_at(lambda: Pose(machine).rounded_angles(node), (0.0, 0.0, 45.0))


async def test_the_raw_frame_id_is_the_display_name_and_its_reserved_characters_become_underscores_in_the_browse_name(
    served: tuple[Server, Any],
    runtime: FakeRuntime,
) -> None:
    _, machine = served
    runtime.note_publishers(TF_TOPIC, 1)
    runtime.deliver(
        TF_TOPIC,
        fixture_messages.tf_message(fixture_messages.transform_stamped("robot_base", "camera 1: left")),
    )
    await rendered(machine, {"camera 1: left"})

    node = await location_in(arm_of(machine), INTERNAL_FRAMES, "camera 1_ left")

    assert (await node.read_display_name()).Text == "camera 1: left"


async def test_a_static_frame_sets_the_constant_access_bit(served: tuple[Server, Any], runtime: FakeRuntime) -> None:
    server, machine = served
    ns = rsl(machine)
    publish_tree(runtime)
    await rendered(machine, {"unit_sensor", "unit_carriage"})

    static = await location_in(machine, INTERNAL_FRAMES, "unit_sensor")
    moving = await location_in(machine, ATTACH_POINTS, "unit_carriage")

    async with connected(server) as client:
        held = await client.get_node(static.nodeid).get_child(f"{ns}:Position")
        # Wait: the static bit is set by the write pass, and a head frame's Location exists before any message.
        await settles_at(lambda: constant(held), True)

        # asyncua may give a node no attribute at all, so the read must not raise on a missing bit.
        other = await client.get_node(moving.nodeid).get_child(f"{ns}:Position")
        assert await constant(other) is False


async def test_a_head_frame_published_static_is_constant_too(served: tuple[Server, Any], runtime: FakeRuntime) -> None:
    # Mark-at-creation would miss this: a Component's head frame on /tf_static has a Location before any message.
    server, machine = served
    runtime.note_publishers(TF_STATIC_TOPIC, 1)
    runtime.deliver(
        TF_STATIC_TOPIC,
        fixture_messages.tf_message(
            fixture_messages.transform_stamped("unit_base", "unit_carriage"),
            fixture_messages.transform_stamped("unit_carriage", "robot_base", position=(0.0, 0.0, 0.25)),
        ),
    )
    await rendered(machine, {"unit_carriage"})

    head = await head_of(arm_of(machine))
    async with connected(server) as client:
        held = await client.get_node(head.nodeid).get_child(f"{rsl(machine)}:Position")
        await settles_at(lambda: constant(held), True)


async def test_a_frame_first_seen_moving_becomes_constant_when_it_is_republished_static(
    served: tuple[Server, Any],
    runtime: FakeRuntime,
) -> None:
    server, machine = served
    runtime.note_publishers(TF_TOPIC, 1)
    runtime.deliver(
        TF_TOPIC,
        fixture_messages.tf_message(
            fixture_messages.transform_stamped("map", "unit_base"),
            fixture_messages.transform_stamped("unit_base", "robot_base"),
        ),
    )
    await rendered(machine, {"robot_base"})
    node = await head_of(arm_of(machine))

    async with connected(server) as client:
        held = await client.get_node(node.nodeid).get_child(f"{rsl(machine)}:Position")
        assert await constant(held) is False

        runtime.note_publishers(TF_STATIC_TOPIC, 1)
        runtime.deliver(
            TF_STATIC_TOPIC,
            fixture_messages.tf_message(fixture_messages.transform_stamped("unit_base", "robot_base")),
        )
        await settles_at(lambda: constant(held), True)


# --------------------------------------------------------------------------- alternatives


async def test_a_composed_pose_lands_in_the_alternative_frames_folder(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    _, machine = served
    pose = Pose(machine)
    publish_tree(runtime)
    runtime.serve_transform(
        target="map",
        source="tool0",
        transform=Transform(translation=(1.0, 0.0, 0.55), rotation=(0.0, 0.0, 0.0, 1.0)),
    )
    await rendered(machine, {"tool0"})

    node = await appears(arm_of(machine), ALTERNATIVE_FRAMES, "tool0_in_map")

    await settles_at(lambda: pose.position(node), (1.0, 0.0, 0.55))
    assert await pose.base(node) == machine.spatial_object_list.world_frame.ua_node.nodeid
    assert (await node.read_display_name()).Text == "tool0 in map"


async def test_an_alternative_nothing_can_answer_creates_no_node(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    _, machine = served
    publish_heads(runtime)
    await rendered(machine, {"unit_carriage"})
    polls = machine.ros2_transform_polls
    await _until(lambda: machine.ros2_transform_polls >= polls + 2, lambda: "the poll stopped")

    assert ALTERNATIVE_FRAMES not in await browse_names(await spatial_of(arm_of(machine)))


# --------------------------------------------------------------------------- the obligations


async def test_a_new_location_is_announced_with_the_node_version_bumped(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    server, machine = served
    collector = Collector()
    async with connected(server) as client:
        await (await client.create_subscription(20, collector)).subscribe_events(
            client.get_node(machine.spatial_object_list.ua_node.nodeid),
            client.get_node(ua.NodeId(ua.Int32(ObjectIds.GeneralModelChangeEventType))),
        )
        before = machine.node_version
        publish_heads(runtime)
        await rendered(machine, {"unit_carriage"})
        await _until(lambda: bool(collector.events), lambda: "no model change reached the subscriber")

        assert machine.node_version > before
        version = await (
            await machine.spatial_object_list.ua_node.get_child(ua.QualifiedName("NodeVersion", 0))
        ).read_value()
        assert version == str(machine.node_version)
        assert len(collector.events[0].Changes) == 2


async def test_one_event_carries_every_location_a_tick_created(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    server, machine = served
    collector = Collector()
    async with connected(server) as client:
        await (await client.create_subscription(20, collector)).subscribe_events(
            client.get_node(machine.spatial_object_list.ua_node.nodeid),
            client.get_node(ua.NodeId(ua.Int32(ObjectIds.GeneralModelChangeEventType))),
        )
        publish_tree(runtime)
        await rendered(machine, {"tool0"})
        await _until(lambda: len(collector.events) >= 2, lambda: f"{len(collector.events)} model changes arrived")

        assert max(len(each.Changes) for each in collector.events) == 4


async def test_a_cap_refuses_the_whole_batch_and_says_so(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, arm=Capped(name="arm"))
    collector = Collector()
    try:
        async with connected(server) as client:
            await binding_events_of(client, arm_of(machine), collector)
            publish_tree(runtime)
            await _until(
                lambda: CAP_REACHED in collector.codes(),
                lambda: f"the events were {collector.codes()}",
            )

        assert "unit_carriage" in machine.ros2_frame_nodes
        assert not {"arm_link", "tool0"} & set(machine.ros2_frame_nodes)
    finally:
        await server.shutdown()


async def test_two_frames_that_browse_alike_are_refused_and_reported(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    server, machine = served
    collector = Collector()
    async with connected(server) as client:
        await binding_events_of(client, arm_of(machine), collector)
        runtime.note_publishers(TF_TOPIC, 1)
        runtime.deliver(
            TF_TOPIC,
            fixture_messages.tf_message(
                fixture_messages.transform_stamped("map", "robot_base"),
                fixture_messages.transform_stamped("robot_base", "left.motor"),
                fixture_messages.transform_stamped("robot_base", "left:motor"),
            ),
        )
        await _until(
            lambda: FRAME_COLLISION in collector.codes(),
            lambda: f"the events were {collector.codes()}",
        )


async def test_the_first_of_two_frames_that_browse_alike_keeps_its_location(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    _, machine = served
    runtime.note_publishers(TF_TOPIC, 1)
    runtime.deliver(
        TF_TOPIC,
        fixture_messages.tf_message(
            fixture_messages.transform_stamped("map", "robot_base"),
            fixture_messages.transform_stamped("robot_base", "left.motor"),
            fixture_messages.transform_stamped("robot_base", "left:motor"),
        ),
    )
    await rendered(machine, {"left.motor"})

    assert await browse_names(await folder_of(arm_of(machine), INTERNAL_FRAMES)) == {"left_motor"}


# --------------------------------------------------------------------------- the poll


async def test_inward_is_a_poll_rather_than_a_write_per_message(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    _, machine = served
    publish_heads(runtime)
    await rendered(machine, {"unit_carriage"})
    seen = machine.ros2_transform_polls

    for _ in range(20):
        runtime.deliver(
            TF_TOPIC,
            fixture_messages.tf_message(fixture_messages.transform_stamped("unit_base", "unit_carriage")),
        )

    assert machine.ros2_transform_polls - seen < 20


async def test_a_frame_that_moves_is_written(served: tuple[Server, Any], runtime: FakeRuntime) -> None:
    _, machine = served
    publish_heads(runtime)
    await rendered(machine, {"unit_carriage"})
    node = await location_in(machine, ATTACH_POINTS, "unit_carriage")

    runtime.deliver(
        TF_TOPIC,
        fixture_messages.tf_message(
            fixture_messages.transform_stamped("unit_base", "unit_carriage", position=(0.0, 0.0, 0.9)),
        ),
    )

    await settles_at(lambda: Pose(machine).position(node), (0.0, 0.0, 0.9))


async def test_a_frame_re_parented_across_an_ownership_boundary_keeps_its_one_location(
    served: tuple[Server, Any],
    runtime: FakeRuntime,
) -> None:
    # A frame whose parent changes stays where it was filed; only its Base follows the tree.
    _, machine = served
    publish_tree(runtime)
    await rendered(machine, {"arm_link"})
    filed = machine.ros2_frame_nodes["arm_link"]

    runtime.deliver(
        TF_TOPIC,
        fixture_messages.tf_message(fixture_messages.transform_stamped("unit_base", "arm_link")),
    )
    node = await location_in(arm_of(machine), INTERNAL_FRAMES, "arm_link")
    await settles_at(lambda: Pose(machine).base(node), ua.NodeId.from_string(machine.ros2_frame_nodes["unit_base"]))

    assert machine.ros2_frame_nodes["arm_link"] == filed
    assert "arm_link" not in await browse_names(await folder_of(machine, INTERNAL_FRAMES))


# --------------------------------------------------------------------------- frame lookup (item 5)


async def test_every_rendered_location_is_found_by_its_node_id_and_stays_alive(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    server, machine = served
    publish_heads(runtime)
    await rendered(machine, {"unit_carriage"})
    gc.collect()

    for frame, text in machine.ros2_frame_nodes.items():
        found = server.get_ua_object_optional(ua.NodeId.from_string(text))
        assert isinstance(found, CartesianFrame), f"{frame} is not found by its NodeId"


# --------------------------------------------------------------------- outward: the writable half

#: A platform reading its head inward and publishing two frames of its own.
PUBLISHER = TFBinding(
    name="UnitFrames",
    head="unit_base",
    publishes=(
        PublishedFrame(base="map", frame="unit_estimate"),
        PublishedFrame(base="unit_base", frame="payload_mount", static=True, position=(0.0, 0.0, 0.54)),
    ),
)

#: A Component publishing the frame its Machine reads as a head.
CLASHING = TFBinding(
    name="ArmFrames",
    head="robot_base",
    publishes=(PublishedFrame(base="map", frame="unit_base"),),
)


class Broadcaster(Robot):
    bindings = (PUBLISHER,)


#: A Machine whose only outward frame is static.
STATIC_ONLY = TFBinding(
    name="UnitFrames",
    head="unit_base",
    publishes=(PublishedFrame(base="unit_base", frame="payload_mount", static=True, position=(0.0, 0.0, 0.54)),),
)


class StaticOnly(Robot):
    bindings = (STATIC_ONLY,)


class ClashingArm(Arm):
    bindings = (CLASHING,)


async def holding_the_lock(client: Client, machine: Any) -> None:
    """Take the Machine's Lock, which an HMI does before writing."""
    await client.get_node(machine.lock.ua_node.nodeid).call_method("2:InitLock")


async def outward(machine: Any, frame: str) -> Node:
    """Return the node of one declared outward frame, which lives in AttachPoints from boot."""
    return await location_in(machine, ATTACH_POINTS, frame)


async def writable(client: Client, node: Node, child: str) -> bool:
    """Return whether a client may write one child of a Location, as the client is told."""
    held = await client.get_node(node.nodeid).get_child(child)
    return ua.AccessLevel.CurrentWrite in await held.get_user_access_level()


async def writes(client: Client, node: Node) -> bool:
    """Return whether a client may write one node, as the client is told."""
    return ua.AccessLevel.CurrentWrite in await client.get_node(node.nodeid).get_user_access_level()


def sent(runtime: FakeRuntime, topic: str, frame: str) -> Any:
    """Return the last element for one frame that went out on one topic, or None."""
    found = None
    for each, message in runtime.published:
        if each != topic:
            continue
        for element in cast("Any", message).transforms:
            if element.child_frame_id == frame:
                found = element
    return found


def messages_on(runtime: FakeRuntime, topic: str) -> int:
    """Return how many messages went out on one topic, which is what a static frame is counted by."""
    return len([each for each, _ in runtime.published if each == topic])


def frame_value(xyz: tuple[float, float, float], abc: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> ua.DataValue:
    """Return a whole pose as a client writes it: one 3DFrame, metres and degrees."""
    return ua.DataValue(
        ua.Variant(
            ua.ThreeDFrame(
                CartesianCoordinates=ua.ThreeDCartesianCoordinates(X=xyz[0], Y=xyz[1], Z=xyz[2]),  # type: ignore[call-arg]
                Orientation=ua.ThreeDOrientation(A=abc[0], B=abc[1], C=abc[2]),  # type: ignore[call-arg]
            ),
            ua.VariantType.ExtensionObject,
        )
    )


async def write_position(
    server: Server,
    machine: Any,
    frame: str,
    xyz: tuple[float, float, float],
    abc: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    """Write one outward Location's whole pose as a client does, Lock and all."""
    node = await outward(machine, frame)
    async with connected(server) as client:
        await holding_the_lock(client, machine)
        await client.get_node(node.nodeid).write_value(frame_value(xyz, abc))


async def test_a_machine_that_declares_no_frame_of_its_own_never_appears_on_the_transform_topics(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    assert runtime.advertised_topics == ()
    assert runtime.tf.broadcasting is False


async def test_a_machine_that_publishes_a_frame_advertises_both_transform_topics(runtime: FakeRuntime) -> None:
    server, _ = await start(runtime, Broadcaster)
    try:
        assert runtime.advertised_topics == (TF_TOPIC, TF_STATIC_TOPIC)
        assert runtime.tf.broadcasting is True
    finally:
        await server.shutdown()


async def test_our_own_publisher_does_not_make_a_required_binding_satisfied(runtime: FakeRuntime) -> None:
    # Our own publisher is discounted from the matched count, or we would read ourselves as the robot.
    server, machine = await start(runtime, Broadcaster)
    try:
        runtime.note_publishers(TF_TOPIC, 1)
        assert machine.ros2_transform_presence == {"UnitFrames": False}, "one publisher on /tf is ours"

        runtime.note_publishers(TF_TOPIC, 2)
        assert machine.ros2_transform_presence == {"UnitFrames": True}
    finally:
        await server.shutdown()


async def test_a_declared_outward_frame_has_a_location_from_boot_holding_its_declared_pose(
    runtime: FakeRuntime,
) -> None:
    # Present from boot: a client may write before the robot is on.
    server, machine = await start(runtime, Broadcaster)
    try:
        assert await browse_names(await folder_of(machine, ATTACH_POINTS)) == {"unit_estimate", "payload_mount"}
        assert await Pose(machine).position(await outward(machine, "payload_mount")) == (0.0, 0.0, 0.54)
    finally:
        await server.shutdown()


async def test_the_pose_of_an_outward_location_is_writable_and_its_components_are_not(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, Broadcaster)
    ns = rsl(machine)
    try:
        node = await outward(machine, "payload_mount")
        async with connected(server) as client:
            assert await writes(client, node), "the Location holds the pose, and the pose is what is written"
            assert not await writable(client, node, f"{ns}:Position")
            assert not await writable(client, node, f"{ns}:Orientation")
            assert not await writable(client, node, f"{ns}:Base"), "Base is what the tree says, not a client"

            pose = await node.get_child(f"{ns}:Position")
            for each in ("X", "Y", "Z"):
                assert not await writable(client, pose, each), f"{each} is a component of the pose, not the pose"
    finally:
        await server.shutdown()


async def test_an_inward_location_stays_read_only(served: tuple[Server, Any], runtime: FakeRuntime) -> None:
    server, machine = served
    publish_heads(runtime)
    await rendered(machine, {"unit_carriage"})
    node = await location_in(machine, ATTACH_POINTS, "unit_carriage")

    async with connected(server) as client:
        await holding_the_lock(client, machine)
        assert not await writes(client, node), "the Location says so itself"
        with pytest.raises(ua.UaStatusCodeError):
            await client.get_node(node.nodeid).write_value(frame_value((9.0, 0.0, 0.0)))


async def test_the_declared_pose_is_what_reaches_the_frame_network_until_a_client_writes(
    runtime: FakeRuntime,
) -> None:
    server, _ = await start(runtime, Broadcaster)
    try:
        await _until(
            lambda: sent(runtime, TF_TOPIC, "unit_estimate") is not None,
            lambda: f"nothing for unit_estimate reached /tf; {runtime.published}",
        )
        element = sent(runtime, TF_TOPIC, "unit_estimate")

        assert (element.header.frame_id, element.child_frame_id) == ("map", "unit_estimate")
        assert element.transform.translation.x == 0.0
        assert element.transform.rotation.w == 1.0
    finally:
        await server.shutdown()


async def test_what_a_client_writes_into_a_location_is_what_reaches_the_frame_network(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, Broadcaster)
    try:
        node = await outward(machine, "unit_estimate")
        async with connected(server) as client:
            await holding_the_lock(client, machine)
            await client.get_node(node.nodeid).write_value(frame_value((3.0, 4.0, 0.0), (0.0, 0.0, 90.0)))

        await _until(
            lambda: (
                (element := sent(runtime, TF_TOPIC, "unit_estimate")) is not None
                and element.transform.translation.x == 3.0
                and element.transform.rotation.z != 0.0
            ),
            lambda: f"the writes never both reached /tf; {runtime.published[-1:]}",
        )
        element = sent(runtime, TF_TOPIC, "unit_estimate")

        assert (element.transform.translation.x, element.transform.translation.y) == (3.0, 4.0)
        assert element.transform.rotation.z == pytest.approx(math.sin(math.pi / 4))
        assert element.transform.rotation.w == pytest.approx(math.cos(math.pi / 4))
    finally:
        await server.shutdown()


async def test_a_write_reaches_the_frame_network_without_waiting_for_the_poll(runtime: FakeRuntime) -> None:
    slow = ROS2Config(transforms=TransformConfig(interval=60.0))
    server, machine = await start(runtime, Broadcaster, config=slow)
    try:
        await write_position(server, machine, "payload_mount", (0.0, 0.0, 0.61))

        assert sent(runtime, TF_STATIC_TOPIC, "payload_mount").transform.translation.z == 0.61
    finally:
        await server.shutdown()


async def test_both_forms_of_a_written_pose_agree_afterwards(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, Broadcaster)
    try:
        await write_position(server, machine, "unit_estimate", (1.0, 2.0, 3.0))
        node = await outward(machine, "unit_estimate")

        await settles_at(lambda: Pose(machine).position(node), (1.0, 2.0, 3.0))
        composite = await (await node.get_child(f"{rsl(machine)}:Position")).read_value()
        assert (composite.X, composite.Y, composite.Z) == (1.0, 2.0, 3.0)
        whole = (await node.read_value()).CartesianCoordinates
        assert (whole.X, whole.Y, whole.Z) == (1.0, 2.0, 3.0)
    finally:
        await server.shutdown()


async def test_the_stamp_on_a_published_frame_is_the_runtimes_clock(runtime: FakeRuntime) -> None:
    runtime.set_clock(1_700_000_000, 5)
    server, _ = await start(runtime, Broadcaster)
    try:
        await _until(lambda: sent(runtime, TF_TOPIC, "unit_estimate") is not None, lambda: "nothing reached /tf")
        stamp = sent(runtime, TF_TOPIC, "unit_estimate").header.stamp

        assert (stamp.sec, stamp.nanosec) == (1_700_000_000, 5)
    finally:
        await server.shutdown()


async def test_a_static_frame_is_published_when_it_changes_rather_than_on_every_tick(runtime: FakeRuntime) -> None:
    # The publisher is ours: StaticTransformBroadcaster appends only unsent frames and would drop this correction.
    server, machine = await start(runtime, Broadcaster)
    try:
        await _until(
            lambda: sent(runtime, TF_STATIC_TOPIC, "payload_mount") is not None,
            lambda: "the declared static frame never reached /tf_static",
        )
        polls = machine.ros2_transform_polls
        await _until(lambda: machine.ros2_transform_polls >= polls + 3, lambda: "the poll stopped")

        assert messages_on(runtime, TF_STATIC_TOPIC) == 1, "an unchanged static frame is not republished"

        await write_position(server, machine, "payload_mount", (0.0, 0.0, 0.61))

        await _until(
            lambda: messages_on(runtime, TF_STATIC_TOPIC) == 2,
            lambda: "the corrected static frame never went out",
        )
        assert sent(runtime, TF_STATIC_TOPIC, "payload_mount").transform.translation.z == 0.61
    finally:
        await server.shutdown()


async def test_our_own_broadcast_never_writes_over_the_pose_a_client_wrote(runtime: FakeRuntime) -> None:
    # Closed by construction: an outward frame is dropped from the inward record, as a TFMessage names no sender.
    server, machine = await start(runtime, Broadcaster)
    try:
        await write_position(server, machine, "unit_estimate", (7.0, 0.0, 0.0))
        node = await outward(machine, "unit_estimate")
        await settles_at(lambda: Pose(machine).position(node), (7.0, 0.0, 0.0))

        runtime.note_publishers(TF_TOPIC, 2)
        runtime.deliver(
            TF_TOPIC,
            fixture_messages.tf_message(
                fixture_messages.transform_stamped("map", "unit_estimate", position=(1.0, 1.0, 1.0)),
            ),
        )
        polls = machine.ros2_transform_polls
        await _until(lambda: machine.ros2_transform_polls >= polls + 3, lambda: "the poll stopped")

        assert await Pose(machine).position(node) == (7.0, 0.0, 0.0), "the client's pose, not the robot's copy"
    finally:
        await server.shutdown()


async def test_a_declared_outward_frame_takes_its_place_in_the_tree(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, Broadcaster)
    try:
        publish_heads(runtime)
        await rendered(machine, {"unit_carriage"})
        mount = await outward(machine, "payload_mount")

        await settles_at(lambda: Pose(machine).base(mount), (await head_of(machine)).nodeid)
    finally:
        await server.shutdown()


async def test_two_declarations_claiming_one_frame_stop_the_machine_booting(runtime: FakeRuntime) -> None:
    # Enforced at init across the whole Machine, since neither declaration can see the other.
    with pytest.raises(FrameDirectionError, match="inward or outward and never both"):
        await start(runtime, arm=ClashingArm(name="arm"))


async def test_two_unrelated_coordinate_systems_are_reported_rather_than_presented_as_one(
    served: tuple[Server, Any], runtime: FakeRuntime
) -> None:
    server, machine = served
    collector = Collector()
    async with connected(server) as client:
        await binding_events_of(client, machine, collector)
        runtime.note_publishers(TF_TOPIC, 1)
        runtime.deliver(
            TF_TOPIC,
            fixture_messages.tf_message(
                fixture_messages.transform_stamped("map", "unit_base"),
                fixture_messages.transform_stamped("world", "cell_fixture"),
            ),
        )

        await _until(
            lambda: SEPARATE_FRAME_TREES in collector.codes(),
            lambda: f"the events were {collector.codes()}",
        )
        said = next(each for each in collector.events if each.Code == SEPARATE_FRAME_TREES)
        assert "'map'" in said.Message.Text
        assert "'world'" in said.Message.Text


async def test_a_machine_whose_own_broadcasts_are_its_only_second_root_says_nothing(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, Broadcaster)
    collector = Collector()
    try:
        async with connected(server) as client:
            await binding_events_of(client, machine, collector)
            publish_heads(runtime)
            await rendered(machine, {"unit_carriage"})
            polls = machine.ros2_transform_polls
            await _until(lambda: machine.ros2_transform_polls >= polls + 2, lambda: "the poll stopped")

            assert SEPARATE_FRAME_TREES not in collector.codes()
    finally:
        await server.shutdown()


# --------------------------------------------------------------------------- the inversion helper


def test_inverting_the_identity_pose_is_the_identity_pose() -> None:
    position, orientation = inverted(Position(0.0, 0.0, 0.0, unit=METRES), Orientation(0.0, 0.0, 0.0))

    assert (position.x, position.y, position.z) == (0.0, 0.0, 0.0)
    assert (orientation.a, orientation.b, orientation.c) == (0.0, 0.0, 0.0)


def test_a_pure_translation_inverts_by_negation() -> None:
    position, orientation = inverted(Position(1.0, 2.0, 3.0, unit=METRES), Orientation(0.0, 0.0, 0.0))

    assert (position.x, position.y, position.z) == pytest.approx((-1.0, -2.0, -3.0))
    assert (orientation.a, orientation.b, orientation.c) == pytest.approx((0.0, 0.0, 0.0))


def test_inverting_a_pose_twice_is_the_pose_again() -> None:
    start_position, start_orientation = Position(1.0, -2.0, 0.5, unit=METRES), Orientation(10.0, 20.0, 30.0)

    position, orientation = inverted(*inverted(start_position, start_orientation))

    assert (position.x, position.y, position.z) == pytest.approx((start_position.x, start_position.y, start_position.z))
    assert (orientation.a, orientation.b, orientation.c) == pytest.approx(
        (start_orientation.a, start_orientation.b, start_orientation.c)
    )


def test_an_inverted_pose_undoes_the_one_it_came_from() -> None:
    position, orientation = inverted(Position(2.0, 0.0, 0.0, unit=METRES), Orientation(0.0, 0.0, 90.0))

    assert (position.x, position.y, position.z) == pytest.approx((0.0, 2.0, 0.0))
    assert (orientation.a, orientation.b, orientation.c) == pytest.approx((0.0, 0.0, -90.0))


def test_the_inverted_pose_keeps_the_unit_it_was_given() -> None:
    position, _ = inverted(Position(1.0, 0.0, 0.0, unit=METRES), Orientation(0.0, 0.0, 0.0))

    assert position.unit == METRES


# ------------------------------------------------------- what a Location says when its frame goes quiet

#: A window short enough for a test to step over, against a fast poll.
QUIET = ROS2Config(transforms=TransformConfig(interval=0.02, quiet_after=0.5))


async def test_a_frame_whose_publisher_stops_says_so_on_the_pose_and_on_every_scalar(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, config=QUIET)
    pose = Pose(machine)
    try:
        runtime.set_clock(1)
        publish_heads(runtime)
        await rendered(machine, {"unit_carriage"})
        node = await location_in(machine, ATTACH_POINTS, "unit_carriage")
        await settles_at(lambda: pose.statuses(node), {GOOD.value})

        runtime.set_clock(5)

        await settles_at(lambda: pose.statuses(node), {NO_COMMUNICATION.value})
    finally:
        await server.shutdown()


async def test_a_frame_that_is_still_arriving_stays_good_while_its_neighbour_has_gone(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, config=QUIET)
    pose = Pose(machine)
    try:
        runtime.set_clock(1)
        publish_heads(runtime)
        await rendered(machine, {"unit_carriage"})
        arm = await head_of(arm_of(machine))
        carriage = await location_in(machine, ATTACH_POINTS, "unit_carriage")

        runtime.set_clock(5)
        runtime.deliver(
            TF_TOPIC,
            fixture_messages.tf_message(
                fixture_messages.transform_stamped("unit_base", "unit_carriage", position=(2.0, 0.0, 0.0)),
            ),
        )

        await settles_at(lambda: pose.statuses(arm), {NO_COMMUNICATION.value})
        assert await pose.statuses(carriage) == {GOOD.value}
        assert machine.ros2_transform_presence == {"UnitFrames": True}, "the robot is still on"
    finally:
        await server.shutdown()


async def test_a_static_frame_stays_good_when_everything_has_gone_quiet(runtime: FakeRuntime) -> None:
    # Quiet is a static frame's normal state; a bad status would contradict its constant bit.
    server, machine = await start(runtime, config=QUIET)
    pose = Pose(machine)
    try:
        runtime.set_clock(1)
        runtime.note_publishers(TF_TOPIC, 1)
        runtime.note_publishers(TF_STATIC_TOPIC, 1)
        runtime.deliver(
            TF_STATIC_TOPIC,
            fixture_messages.tf_message(
                fixture_messages.transform_stamped("unit_carriage", "robot_base", position=(0.0, 0.0, 0.25)),
            ),
        )
        runtime.deliver(
            TF_TOPIC,
            fixture_messages.tf_message(
                fixture_messages.transform_stamped("map", "unit_base"),
                fixture_messages.transform_stamped("unit_base", "unit_carriage", position=(1.0, 0.0, 0.0)),
            ),
        )
        await rendered(machine, {"unit_carriage"})
        mounted = await head_of(arm_of(machine))
        moving = await location_in(machine, ATTACH_POINTS, "unit_carriage")

        runtime.set_clock(5)

        await settles_at(lambda: pose.statuses(moving), {NO_COMMUNICATION.value})
        assert await pose.statuses(mounted) == {GOOD.value}
    finally:
        await server.shutdown()


async def test_a_frame_nobody_has_published_on_a_robot_that_is_off_reads_bad_no_communication(
    runtime: FakeRuntime,
) -> None:
    server, machine = await start(runtime, config=QUIET)
    try:
        node = await head_of(arm_of(machine))

        await settles_at(lambda: Pose(machine).statuses(node), {NO_COMMUNICATION.value})
    finally:
        await server.shutdown()


async def test_a_frame_nobody_has_published_on_a_robot_that_is_on_reads_waiting_for_initial_data(
    runtime: FakeRuntime,
) -> None:
    server, machine = await start(runtime, config=QUIET)
    try:
        runtime.note_publishers(TF_TOPIC, 1)
        node = await head_of(arm_of(machine))

        await settles_at(lambda: Pose(machine).statuses(node), {WAITING_FOR_INITIAL_DATA.value})
    finally:
        await server.shutdown()


async def test_a_frame_that_comes_back_at_the_pose_it_left_is_written_again(runtime: FakeRuntime) -> None:
    # Compare status as well as pose: OPC UA Part 4 7.7.1 nulls a value written with a bad status.
    server, machine = await start(runtime, config=QUIET)
    pose = Pose(machine)
    try:
        runtime.set_clock(1)
        publish_heads(runtime)
        await rendered(machine, {"unit_carriage"})
        node = await location_in(machine, ATTACH_POINTS, "unit_carriage")

        runtime.set_clock(5)
        await settles_at(lambda: pose.statuses(node), {NO_COMMUNICATION.value})
        assert await pose.values(node, "Position") == (None, None, None), "a bad status nulls the value it came with"

        runtime.deliver(
            TF_TOPIC,
            fixture_messages.tf_message(
                fixture_messages.transform_stamped("unit_base", "unit_carriage", position=(1.0, 0.0, 0.0)),
            ),
        )

        await settles_at(lambda: pose.statuses(node), {GOOD.value})
        assert await pose.position(node) == (1.0, 0.0, 0.0), "the pose it held before it went quiet"
    finally:
        await server.shutdown()


async def test_a_composed_alternative_carries_the_status_of_the_frame_it_is_a_view_of(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, config=QUIET)
    pose = Pose(machine)
    try:
        runtime.set_clock(1)
        publish_tree(runtime)
        runtime.serve_transform(
            target="map",
            source="tool0",
            transform=Transform(translation=(1.0, 0.0, 0.55), rotation=(0.0, 0.0, 0.0, 1.0)),
        )
        await rendered(machine, {"tool0"})
        node = await appears(arm_of(machine), ALTERNATIVE_FRAMES, "tool0_in_map")
        await settles_at(lambda: pose.statuses(node), {GOOD.value})

        runtime.set_clock(5)

        await settles_at(lambda: pose.statuses(node), {NO_COMMUNICATION.value})
    finally:
        await server.shutdown()


async def test_an_outward_location_carries_no_status_however_long_the_robot_is_off(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, Broadcaster, config=QUIET)
    try:
        node = await outward(machine, "unit_estimate")
        polls = machine.ros2_transform_polls

        runtime.set_clock(1)
        await _until(lambda: machine.ros2_transform_polls > polls + 2, lambda: "the poll did not run again")

        assert await Pose(machine).statuses(node) == {GOOD.value}
    finally:
        await server.shutdown()


async def test_a_broadcast_rclpy_refuses_is_reported_as_a_binding_event(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, Broadcaster, config=QUIET)
    collector = Collector()
    try:
        async with connected(server) as client:
            await binding_events_of(client, machine, collector)
            runtime.refuse_publishing(TF_TOPIC)

            await _until(lambda: PUBLISH_FAILED in collector.codes(), lambda: f"the events were {collector.codes()}")
            said = next(each for each in collector.events if each.Code == PUBLISH_FAILED)

            assert said.Severity == Severity.ERROR
            assert "unit_estimate" in said.Message.Text, "it names the frames that did not reach ROS2"
    finally:
        await server.shutdown()


async def test_a_refused_broadcast_is_said_once_rather_than_once_per_tick(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, Broadcaster, config=QUIET)
    collector = Collector()
    try:
        async with connected(server) as client:
            await binding_events_of(client, machine, collector)
            runtime.refuse_publishing(TF_TOPIC)
            await _until(lambda: PUBLISH_FAILED in collector.codes(), lambda: "nothing was reported at all")
            polls = machine.ros2_transform_polls

            await _until(lambda: machine.ros2_transform_polls > polls + 5, lambda: "the poll did not run again")
            failures = [each for each in collector.codes() if each == PUBLISH_FAILED]

            assert len(failures) == 1, f"it said so {len(failures)} times over {polls} polls"
    finally:
        await server.shutdown()


async def test_a_broadcast_that_fails_again_after_recovering_is_reported_again(runtime: FakeRuntime) -> None:
    # Once per spell of failure: a publisher that breaks, recovers and breaks again is two events.
    server, machine = await start(runtime, Broadcaster, config=QUIET)
    collector = Collector()
    try:
        async with connected(server) as client:
            await binding_events_of(client, machine, collector)
            runtime.refuse_publishing(TF_TOPIC, times=1)
            await _until(
                lambda: collector.codes().count(PUBLISH_FAILED) == 1,
                lambda: "the first refusal was not reported",
            )
            went_out = messages_on(runtime, TF_TOPIC)
            await _until(
                lambda: messages_on(runtime, TF_TOPIC) > went_out,
                lambda: "no broadcast got through, so the first spell never ended",
            )

            runtime.refuse_publishing(TF_TOPIC, times=1)

            await _until(
                lambda: collector.codes().count(PUBLISH_FAILED) == 2,
                lambda: "the second refusal was swallowed by the first",
            )
    finally:
        await server.shutdown()


async def test_a_static_frame_the_middleware_refused_is_sent_again_on_the_next_tick(runtime: FakeRuntime) -> None:
    # Recorded after the wire: recorded before, a refused static frame would never be offered again.
    server, machine = await start(runtime, Broadcaster, config=QUIET)
    try:
        await _until(
            lambda: sent(runtime, TF_STATIC_TOPIC, "payload_mount") is not None,
            lambda: "the declared static frame never reached /tf_static at all",
        )
        runtime.refuse_publishing(TF_STATIC_TOPIC, times=1)

        await write_position(server, machine, "payload_mount", (0.0, 0.0, 0.61))

        await _until(
            lambda: float(sent(runtime, TF_STATIC_TOPIC, "payload_mount").transform.translation.z) == 0.61,
            lambda: "the refused correction was never offered to the wire again",
        )
    finally:
        await server.shutdown()


async def test_a_machine_whose_only_outward_frame_is_static_can_recover_from_a_refusal(runtime: FakeRuntime) -> None:
    # A tick after a refusal has nothing to send, so only a successful send can clear the streak.
    server, machine = await start(runtime, StaticOnly, config=QUIET)
    collector = Collector()
    try:
        async with connected(server) as client:
            await binding_events_of(client, machine, collector)
            await _until(
                lambda: sent(runtime, TF_STATIC_TOPIC, "payload_mount") is not None,
                lambda: "the declared static frame never reached /tf_static at all",
            )
            # One session for both writes: it holds the Lock between them.
            await holding_the_lock(client, machine)
            target = client.get_node((await outward(machine, "payload_mount")).nodeid)
            runtime.refuse_publishing(TF_STATIC_TOPIC, times=1)
            await target.write_value(frame_value((0.0, 0.0, 0.61)))
            await _until(lambda: collector.codes().count(PUBLISH_FAILED) == 1, lambda: "the refusal was not reported")

            await _until(
                lambda: float(sent(runtime, TF_STATIC_TOPIC, "payload_mount").transform.translation.z) == 0.61,
                lambda: "the pose never reached ROS2 after the middleware recovered",
            )
            runtime.refuse_publishing(TF_STATIC_TOPIC, times=1)
            await target.write_value(frame_value((0.0, 0.0, 0.62)))

            await _until(
                lambda: collector.codes().count(PUBLISH_FAILED) == 2,
                lambda: "the second spell was swallowed by a streak that never cleared",
            )
            await _until(
                lambda: float(sent(runtime, TF_STATIC_TOPIC, "payload_mount").transform.translation.z) == 0.62,
                lambda: "the second correction never reached ROS2 either",
            )
    finally:
        await server.shutdown()
