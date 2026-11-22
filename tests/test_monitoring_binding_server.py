# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A declared topic reaches an OPC UA client, against a real in-process opensmi server.

A variable whose topic has no publisher reads BadNoCommunication, so tests tell the fake Runtime a
publisher is there first. Status codes are in `test_mutation_server.py`.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua
from asyncua.ua.attribute_ids import AttributeIds
from asyncua.ua.object_ids import ObjectIds
from opensmi.core.nodesets import MachineryNodeIds

from opensmi.ros2.runtime.errors import MissingInterfacePackageError
from opensmi.ros2.server.bindings import MonitoringBinding, load_bindings
from opensmi.ros2.server.errors import (
    DeclarationError,
    UnprojectableFieldError,
)
from opensmi.ros2.server.mutation import NodeAddition
from opensmi.ros2.server.projection import PlanObject, PlanVariable
from opensmi.ros2.server.recipes import Functions
from tests import fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.served import (
    MANUFACTURER,
    TestComponent,
    TestMachine,
    browse,
    connected,
    monitoring_nodes,
    names_of,
    serve,
    until_value,
    value_of,
)

if TYPE_CHECKING:
    from collections.abc import AsyncIterator
    from pathlib import Path

    from asyncua.common.node import Node
    from opensmi.server import Server

MESSAGE_TYPES = {
    "fixture_msgs/msg/PoseStamped": fixture_messages.PoseStamped,
    "fixture_msgs/msg/PoseWithCovariance": fixture_messages.PoseWithCovariance,
    "fixture_msgs/msg/Pose": fixture_messages.Pose,
    "fixture_msgs/msg/Point": fixture_messages.Point,
    "fixture_msgs/msg/Sensor": fixture_messages.Sensor,
}

CURRENT_POSE = MonitoringBinding(
    name="CurrentPose",
    path="/current_pose",
    message_type="fixture_msgs/msg/PoseStamped",
    exclude=("header/stamp", "pose/orientation"),
)
ODOM = MonitoringBinding(
    name="Odom",
    path="/odom",
    message_type="fixture_msgs/msg/PoseWithCovariance",
    include=("pose/position/x", "covariance"),
)
SENSOR = MonitoringBinding(name="Sensor", path="/sensor", message_type="fixture_msgs/msg/Sensor")


DEPLOYED_TOML = """
[[monitoring]]
name = "TargetPose"
path = "/target_pose"
message_type = "fixture_msgs/msg/Pose"
include = ["position/x"]
"""


class Platform(TestComponent):
    bindings = (CURRENT_POSE, ODOM, SENSOR)


@pytest.fixture
def runtime() -> FakeRuntime:
    return FakeRuntime(domain_id=7, message_types=MESSAGE_TYPES)


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


def nodes(machine: Any) -> dict[str, Node]:
    return monitoring_nodes(machine.added[0])


async def test_a_binding_has_its_nodes_at_boot_with_nothing_publishing(served) -> None:
    _, machine = served

    assert sorted(nodes(machine)) == [
        "CurrentPose.Header.FrameId",
        "CurrentPose.Pose.Position.X",
        "CurrentPose.Pose.Position.Y",
        "CurrentPose.Pose.Position.Z",
        "Odom.Covariance",
        "Odom.Pose.Position.X",
        "Sensor.Labels",
        "Sensor.Ok",
        "Sensor.Payload",
        "Sensor.Readings",
    ]


async def test_a_field_called_status_below_a_bindings_root_is_free() -> None:
    # Machinery reserves Status only directly in a Machine's or Component's Monitoring.
    class Jobs(TestComponent):
        bindings = (MonitoringBinding(name="Job", path="/job", message_type="fixture_msgs/msg/Job"),)

    runtime = FakeRuntime(domain_id=7, message_types={"fixture_msgs/msg/Job": fixture_messages.Job})
    server, machine = await start(runtime, Jobs(name="Platform"))
    try:
        assert sorted(nodes(machine)) == ["Job.Progress", "Job.Status"]
    finally:
        await server.shutdown()


async def test_a_nested_field_browses_below_its_message_and_reads_via_a_client(served, runtime: FakeRuntime) -> None:
    server, machine = served
    monitoring = machine.added[0].monitoring.ua_node

    runtime.note_publishers("/current_pose", 1)
    runtime.deliver("/current_pose", fixture_messages.pose_stamped(frame_id="map", x=2.5))

    async with connected(server, "visitor") as client:
        field = await browse(client.get_node(monitoring.nodeid), "CurrentPose", "Pose", "Position", "X")
        await until_value(field, 2.5)


async def test_the_browse_name_carries_no_container_prefix(served) -> None:
    server, machine = served
    node = nodes(machine)["CurrentPose.Pose.Position.X"]

    async with connected(server) as client:
        browse_name = await client.get_node(node.nodeid).read_browse_name()

    assert browse_name.Name == "X"
    assert "Monitoring" not in str(node.nodeid.Identifier).rpartition(".")[2]


async def test_a_client_browses_the_variables_under_the_components_monitoring(served) -> None:
    server, machine = served
    monitoring = machine.added[0].monitoring.ua_node

    async with connected(server, "visitor") as client:
        names = await names_of(client.get_node(monitoring.nodeid))

    assert {"CurrentPose", "Odom", "Sensor"} <= set(names)


async def test_a_client_reads_a_nested_field_of_a_topic(served, runtime: FakeRuntime) -> None:
    server, machine = served
    node = nodes(machine)["CurrentPose.Pose.Position.X"]

    runtime.note_publishers("/current_pose", 1)
    runtime.deliver("/current_pose", fixture_messages.pose_stamped(frame_id="map", x=1.5))

    async with connected(server) as client:
        await until_value(client.get_node(node.nodeid), 1.5)


async def test_the_variables_keep_up_with_the_topic(served, runtime: FakeRuntime) -> None:
    server, machine = served
    frame = nodes(machine)["CurrentPose.Header.FrameId"]

    runtime.note_publishers("/current_pose", 1)
    for index in range(3):
        runtime.deliver("/current_pose", fixture_messages.pose_stamped(frame_id=f"frame_{index}"))

    async with connected(server) as client:
        await until_value(client.get_node(frame.nodeid), "frame_2")


async def test_a_fixed_size_array_reaches_the_client_with_its_elements(served, runtime: FakeRuntime) -> None:
    server, machine = served
    covariance = nodes(machine)["Odom.Covariance"]

    runtime.note_publishers("/odom", 1)
    runtime.deliver(
        "/odom",
        fixture_messages.PoseWithCovariance(
            pose=fixture_messages.pose_stamped().pose,
            covariance=(1.0, 2.0, 3.0, 4.0),
        ),
    )

    async with connected(server) as client:
        await until_value(client.get_node(covariance.nodeid), [1.0, 2.0, 3.0, 4.0])


async def test_a_variable_length_sequence_reaches_the_client_both_empty_and_full(served, runtime: FakeRuntime) -> None:
    server, machine = served
    labels = nodes(machine)["Sensor.Labels"]
    readings = nodes(machine)["Sensor.Readings"]
    payload = nodes(machine)["Sensor.Payload"]

    runtime.note_publishers("/sensor", 1)
    async with connected(server) as client:
        runtime.deliver("/sensor", fixture_messages.Sensor(ok=True, labels=(), readings=(), payload=b""))
        await until_value(client.get_node(labels.nodeid), [])

        runtime.deliver(
            "/sensor",
            fixture_messages.Sensor(ok=True, labels=("left", "right"), readings=(0.5, 1.5), payload=b""),
        )

        await until_value(client.get_node(labels.nodeid), ["left", "right"])
        assert await value_of(client.get_node(readings.nodeid)) == [0.5, 1.5]
        assert await value_of(client.get_node(payload.nodeid)) == b""


async def test_an_array_variable_says_it_has_exactly_one_dimension_of_unknown_length(served) -> None:
    """A ROS2 sequence is one-dimensional and the node says so."""
    server, machine = served
    readings = nodes(machine)["Sensor.Readings"]
    covariance = nodes(machine)["Odom.Covariance"]
    scalar = nodes(machine)["CurrentPose.Pose.Position.X"]

    async with connected(server) as client:

        async def rank_and_dimensions(node: Node) -> tuple[int, list[int] | None]:
            rank, dimensions = await client.get_node(node.nodeid).read_attributes(
                [AttributeIds.ValueRank, AttributeIds.ArrayDimensions]
            )
            assert rank.Value is not None
            assert dimensions.Value is not None
            return rank.Value.Value, dimensions.Value.Value

        for node in (readings, covariance):
            assert await rank_and_dimensions(node) == (ua.ValueRank.OneDimension, [0]), node.nodeid.to_string()

        rank, dimensions = await rank_and_dimensions(scalar)
        assert rank == ua.ValueRank.Scalar
        assert not dimensions


async def test_every_variable_says_its_datatype(served) -> None:
    server, machine = served
    expected = {
        "CurrentPose.Pose.Position.X": ObjectIds.Double,
        "CurrentPose.Header.FrameId": ObjectIds.String,
        "Sensor.Ok": ObjectIds.Boolean,
        "Sensor.Labels": ObjectIds.String,
        "Sensor.Readings": ObjectIds.Double,
        "Sensor.Payload": ObjectIds.ByteString,
    }

    async with connected(server) as client:
        for name, identifier in expected.items():
            datatype = await client.get_node(nodes(machine)[name].nodeid).read_data_type()
            assert datatype == ua.NodeId(ua.Int32(identifier)), name


async def test_a_variable_carries_its_declared_type_before_any_message_arrives(served, runtime) -> None:
    server, machine = served
    node = nodes(machine)["CurrentPose.Pose.Position.X"]

    async with connected(server) as client:
        remote = client.get_node(node.nodeid)
        assert (await remote.read_data_type_as_variant_type()).name == "Double"

        runtime.note_publishers("/current_pose", 1)
        runtime.deliver("/current_pose", fixture_messages.pose_stamped(x=2.25))
        await until_value(remote, 2.25)

        variant = (await remote.read_data_value()).Value

    assert variant is not None
    assert variant.VariantType.name == "Double"


async def test_a_client_cannot_write_a_monitoring_variable(served) -> None:
    server, machine = served
    node = nodes(machine)["CurrentPose.Pose.Position.X"]

    async with connected(server) as client:
        with pytest.raises(ua.UaStatusCodeError):
            await client.get_node(node.nodeid).write_value(ua.DataValue(ua.Variant(9.0, ua.VariantType.Double)))


async def test_the_machine_starts_the_runtime_at_init_and_stops_it_at_shutdown(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime)

    assert machine.runtime is runtime
    assert runtime.started

    await server.shutdown()

    assert not runtime.started


async def test_shutdown_closes_the_subscriptions_the_bindings_opened(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime)
    subscriptions: Any = list(machine.added[0].ros2_subscriptions)

    await server.shutdown()

    assert [subscription._sink.closed for subscription in subscriptions] == [True, True, True]


async def test_a_component_computes_its_bindings_from_injected_configuration(runtime: FakeRuntime) -> None:
    class PerVendor(Platform):
        def __init__(self, *, has_odometry: bool, **kwargs: Any) -> None:
            super().__init__(**kwargs)
            self.bindings = (CURRENT_POSE, ODOM) if has_odometry else (CURRENT_POSE,)

    server, machine = await start(runtime, PerVendor(name="Platform", has_odometry=False))
    try:
        assert not [name for name in nodes(machine) if name.startswith("odom/")]
        assert "CurrentPose.Pose.Position.X" in nodes(machine)
    finally:
        await server.shutdown()


async def test_a_deployment_adds_a_binding_from_a_file_without_a_code_change(
    runtime: FakeRuntime, tmp_path: Path
) -> None:
    source = tmp_path / "bindings.toml"
    source.write_text(DEPLOYED_TOML, encoding="utf-8")

    class Deployed(Platform):
        bindings_toml = source

    server, machine = await start(runtime, Deployed(name="Platform"))
    try:
        assert "TargetPose.Position.X" in nodes(machine)
        assert "CurrentPose.Pose.Position.X" in nodes(machine)
    finally:
        await server.shutdown()


async def test_a_declared_type_whose_package_is_missing_is_fatal_at_init() -> None:
    class Unknown(Platform):
        bindings = (MonitoringBinding(name="Ghost", path="/ghost", message_type="nowhere_msgs/msg/Ghost"),)

    with pytest.raises(MissingInterfacePackageError):
        await start(FakeRuntime(message_types=MESSAGE_TYPES), Unknown(name="Platform"))


async def test_a_machine_that_refuses_to_boot_stops_the_runtime_it_started() -> None:
    # Declaration faults are fatal at init: close the Context and executor thread.
    class Unknown(Platform):
        bindings = (MonitoringBinding(name="Ghost", path="/ghost", message_type="nowhere_msgs/msg/Ghost"),)

    runtime = FakeRuntime(message_types=MESSAGE_TYPES)

    with pytest.raises(MissingInterfacePackageError):
        await start(runtime, Unknown(name="Platform"))

    assert not runtime.started


async def test_the_stream_survives_a_value_the_node_will_not_take(served, runtime: FakeRuntime) -> None:
    # A runtime condition is never fatal: one unwritable value must not end the stream.
    server, machine = served
    node = nodes(machine)["CurrentPose.Pose.Position.X"]

    unwritable: Any = "not a double"
    runtime.note_publishers("/current_pose", 1)
    runtime.deliver("/current_pose", fixture_messages.pose_stamped(x=unwritable))
    runtime.deliver("/current_pose", fixture_messages.pose_stamped(x=4.25))

    async with connected(server) as client:
        await until_value(client.get_node(node.nodeid), 4.25)


async def test_a_plan_that_asks_for_an_object_is_refused_at_init() -> None:
    def nested(_type: Any, _context: Any) -> tuple[PlanObject]:
        child = PlanVariable(name="x", source="x", initial=0.0, variant_type=ua.VariantType.Double)
        return (PlanObject(name="P", children=(child,)),)

    folded = Functions("fixture_msgs/msg/Point", shaped_by=nested, decoded_by=lambda _m, _c: {})

    class Folded(Platform):
        bindings = (MonitoringBinding(name="Point", path="/point", message_type="fixture_msgs/msg/Point"),)
        projections = (folded,)

    with pytest.raises(UnprojectableFieldError, match=r"renders .P."):
        await start(FakeRuntime(message_types=MESSAGE_TYPES), Folded(name="Platform"))


async def test_a_component_whose_machine_carries_no_runtime_says_so() -> None:
    from opensmi.server import BaseMachine

    class Bare(BaseMachine):
        async def _write_identification(self) -> None:
            await self.identification.SerialNumber.write("bare")
            await self.identification.ProductInstanceUri.write("urn:opensmi-ros2:test:bare")
            await self.identification.Manufacturer.write(MANUFACTURER)

        async def _init(self) -> None:
            await super()._init()
            await self.add_component(Platform(name="Platform"))

    with pytest.raises(DeclarationError):
        await serve(Bare())


# --- max_rate ---------------------------------------------------------------------

WINDOW = 0.25
"""A rate limit of four a second: long enough that a slow CI box still lands a burst inside it."""


class _Counting:
    """A Projection of ``Point`` that keeps every message it was asked to decode."""

    def __init__(self) -> None:
        self.decoded: list[float] = []

    def functions(self) -> Functions:
        def shaped(_type: Any, _context: Any) -> tuple[PlanVariable]:
            return (
                PlanVariable(name="X", display_name="x", source="x", initial=-1.0, variant_type=ua.VariantType.Double),
            )

        def decoded(message: Any, _context: Any) -> dict[str, float]:
            self.decoded.append(message.x)
            return {"X": message.x}

        return Functions("fixture_msgs/msg/Point", shaped_by=shaped, decoded_by=decoded)


async def _limited(runtime: FakeRuntime, counting: _Counting, max_rate: float | None) -> tuple[Server, Any]:
    class Limited(Platform):
        bindings = (
            MonitoringBinding(name="Point", path="/point", message_type="fixture_msgs/msg/Point", max_rate=max_rate),
        )
        projections = (counting.functions(),)

    server, machine = await start(runtime, Limited(name="Platform"))
    runtime.note_publishers("/point", 1)
    return server, machine


def _point(x: float) -> Any:
    return fixture_messages.Point(x=x, y=0.0, z=0.0)


async def test_a_burst_writes_the_first_at_once_and_the_newest_at_the_end_of_the_window(
    runtime: FakeRuntime,
) -> None:
    counting = _Counting()
    server, _machine = await _limited(runtime, counting, 1 / WINDOW)
    try:
        for x in (1.0, 2.0, 3.0, 4.0):
            runtime.deliver("/point", _point(x))
        await asyncio.sleep(WINDOW / 4)
        assert counting.decoded == [1.0]
        await asyncio.sleep(WINDOW * 2)
        assert counting.decoded == [1.0, 4.0]
    finally:
        await server.shutdown()


async def test_a_topic_that_goes_quiet_ends_with_its_last_message_mirrored(runtime: FakeRuntime) -> None:
    counting = _Counting()
    server, machine = await _limited(runtime, counting, 1 / WINDOW)
    try:
        runtime.deliver("/point", _point(1.0))
        runtime.deliver("/point", _point(2.0))
        await asyncio.sleep(WINDOW * 2)
        assert await nodes(machine)["Point.X"].read_value() == 2.0
    finally:
        await server.shutdown()


async def test_a_message_after_a_quiet_spell_is_written_at_once(runtime: FakeRuntime) -> None:
    counting = _Counting()
    server, _machine = await _limited(runtime, counting, 1 / WINDOW)
    try:
        runtime.deliver("/point", _point(1.0))
        await asyncio.sleep(WINDOW * 2)
        runtime.deliver("/point", _point(2.0))
        await asyncio.sleep(WINDOW / 4)
        assert counting.decoded == [1.0, 2.0]
    finally:
        await server.shutdown()


async def test_a_sustained_stream_writes_at_most_the_declared_rate_and_only_decodes_what_it_writes(
    runtime: FakeRuntime,
) -> None:
    counting = _Counting()
    server, machine = await _limited(runtime, counting, 1 / WINDOW)
    try:
        sent = 0
        loop = asyncio.get_running_loop()
        started = loop.time()
        while loop.time() - started < 1.0:
            # A burst per tick keeps the count independent of timer resolution.
            for _ in range(10):
                sent += 1
                runtime.deliver("/point", _point(float(sent)))
            await asyncio.sleep(0.005)
        await asyncio.sleep(WINDOW * 2)
        seconds = loop.time() - started
        writes = len(counting.decoded)
        assert sent > 100
        assert writes <= seconds / WINDOW + 1
        assert writes >= 3
        assert sent >= writes * 20
        assert counting.decoded[-1] == float(sent)
        assert await nodes(machine)["Point.X"].read_value() == float(sent)
    finally:
        await server.shutdown()


async def test_the_first_sample_of_a_limited_binding_makes_it_good_at_once(runtime: FakeRuntime) -> None:
    counting = _Counting()
    server, machine = await _limited(runtime, counting, 1.0)
    try:
        runtime.deliver("/point", _point(1.0))
        node = nodes(machine)["Point.X"]
        deadline = asyncio.get_running_loop().time() + 0.5
        while True:
            try:
                assert await node.read_value() == 1.0
                break
            except ua.UaStatusCodeError:
                assert asyncio.get_running_loop().time() < deadline
                await asyncio.sleep(0.005)
    finally:
        await server.shutdown()


async def test_without_a_max_rate_every_message_is_written(runtime: FakeRuntime) -> None:
    counting = _Counting()
    server, _machine = await _limited(runtime, counting, None)
    try:
        for x in (1.0, 2.0, 3.0):
            runtime.deliver("/point", _point(x))
        await asyncio.sleep(0.1)
        assert counting.decoded == [1.0, 2.0, 3.0]
    finally:
        await server.shutdown()


@pytest.mark.parametrize("rate", [0.0, -1.0, 1000.5])
def test_a_rate_outside_zero_to_the_publishers_maximum_is_fatal_at_init(rate: float) -> None:
    with pytest.raises(DeclarationError, match="max_rate"):
        MonitoringBinding(name="Point", path="/point", max_rate=rate)


def test_the_publishers_maximum_is_accepted() -> None:
    assert MonitoringBinding(name="Point", path="/point", max_rate=1000.0).max_rate == 1000.0


async def test_a_binding_file_can_declare_a_max_rate(tmp_path: Path) -> None:
    source = tmp_path / "bindings.toml"
    source.write_text('[[monitoring]]\nname = "Point"\npath = "/point"\nmax_rate = 5.0\n', encoding="utf-8")
    (binding,) = load_bindings(source)
    assert isinstance(binding, MonitoringBinding)
    assert binding.max_rate == 5.0


# -- the opt-in Machinery layout: Status, Health, Process, Consumption ------------------------------

FOLDER_TYPE = ua.NodeId(ua.Int32(ObjectIds.FolderType))


class Filed(TestComponent):
    bindings = (
        MonitoringBinding(name="Odom", path="/odom", message_type="fixture_msgs/msg/Pose", group="Process"),
        MonitoringBinding(name="Target", path="/target", message_type="fixture_msgs/msg/Point", group="Process"),
        MonitoringBinding(name="Sensor", path="/sensor", message_type="fixture_msgs/msg/Sensor", group="Consumption"),
        MonitoringBinding(name="Plain", path="/plain", message_type="fixture_msgs/msg/Point"),
    )


class WithMachineryState(TestComponent):
    """A Component that provides the MachineryItemState and MachineryOperationMode AddIns, as a Machine does."""

    bindings = (MonitoringBinding(name="Ready", path="/ready", message_type="fixture_msgs/msg/Point", group="Status"),)

    async def _init(self) -> None:
        machinery = self.server.ua_get_namespace_index(MachineryNodeIds)
        blocks = await self.ua_node.add_folder(
            ua.NodeId(ua.String(f"{self.name}.MachineryBuildingBlocks"), ua.Int16(1)),
            ua.QualifiedName("MachineryBuildingBlocks", machinery),
        )
        for name, object_type in (
            ("MachineryItemState", MachineryNodeIds.MachineryItemState_StateMachineType),
            ("MachineryOperationMode", MachineryNodeIds.MachineryOperationModeStateMachineType),
        ):
            await blocks.add_object(
                ua.NodeId(ua.String(f"{self.name}.{name}"), ua.Int16(1)),
                ua.QualifiedName(name, machinery),
                objecttype=self.server.ua_get_node_id(object_type),
            )
        await super()._init()


@pytest.mark.parametrize("group", ["Status", "Health", "Process", "Consumption"])
async def test_a_binding_in_a_group_browses_below_a_folder_of_that_name(runtime: FakeRuntime, group: str) -> None:
    class InGroup(TestComponent):
        bindings = (
            MonitoringBinding(name="Target", path="/target", message_type="fixture_msgs/msg/Point", group=group),
        )

    server, machine = await start(runtime, InGroup(name="Platform"))
    try:
        async with connected(server) as client:
            monitoring = client.get_node(machine.added[0].monitoring.ua_node.nodeid)
            folder = await browse(monitoring, group)

            assert await names_of(monitoring) == [group]
            assert await folder.read_type_definition() == FOLDER_TYPE
            assert await names_of(await browse(folder, "Target")) == ["X", "Y", "Z"]
    finally:
        await server.shutdown()


async def test_bindings_naming_one_group_share_it_and_unused_groups_are_absent(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, Filed(name="Platform"))
    try:
        async with connected(server) as client:
            monitoring = client.get_node(machine.added[0].monitoring.ua_node.nodeid)

            assert sorted(await names_of(monitoring)) == ["Consumption", "Plain", "Process"]
            assert sorted(await names_of(await browse(monitoring, "Process"))) == ["Odom", "Target"]
            assert await names_of(await browse(monitoring, "Consumption")) == ["Sensor"]
            assert await names_of(await browse(monitoring, "Plain")) == ["X", "Y", "Z"]
        assert "Process.Target.X" in nodes(machine)
        assert "Plain.X" in nodes(machine)
    finally:
        await server.shutdown()


async def test_a_grouped_binding_is_mirrored_like_any_other(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, Filed(name="Platform"))
    try:
        runtime.note_publishers("/target", 1)
        runtime.deliver("/target", fixture_messages.Point(x=1.5, y=0.0, z=0.0))

        await until_value(nodes(machine)["Process.Target.X"], 1.5)
    finally:
        await server.shutdown()


async def test_status_references_the_machinery_state_the_item_provides(runtime: FakeRuntime) -> None:
    server, machine = await start(runtime, WithMachineryState(name="Platform"))
    try:
        async with connected(server) as client:
            item = client.get_node(machine.added[0].ua_node.nodeid)
            status = await browse(item, "Monitoring", "Status")
            components = await status.get_children(refs=ObjectIds.HasComponent)

            assert sorted(await names_of(status)) == ["MachineryItemState", "MachineryOperationMode", "Ready"]
            assert {each.nodeid for each in components} >= {
                (await browse(item, "MachineryBuildingBlocks", name)).nodeid
                for name in ("MachineryItemState", "MachineryOperationMode")
            }
    finally:
        await server.shutdown()


async def test_status_references_nothing_on_an_item_without_machinery_state(runtime: FakeRuntime) -> None:
    class InStatus(TestComponent):
        bindings = (
            MonitoringBinding(name="Ready", path="/ready", message_type="fixture_msgs/msg/Point", group="Status"),
        )

    server, machine = await start(runtime, InStatus(name="Platform"))
    try:
        async with connected(server) as client:
            status = await browse(client.get_node(machine.added[0].monitoring.ua_node.nodeid), "Status")

            assert await names_of(status) == ["Ready"]
    finally:
        await server.shutdown()


async def test_only_status_references_the_machinery_state(runtime: FakeRuntime) -> None:
    class InProcess(WithMachineryState):
        bindings = (
            MonitoringBinding(name="Ready", path="/ready", message_type="fixture_msgs/msg/Point", group="Process"),
        )

    server, machine = await start(runtime, InProcess(name="Platform"))
    try:
        async with connected(server) as client:
            process = await browse(client.get_node(machine.added[0].monitoring.ua_node.nodeid), "Process")

            assert await names_of(process) == ["Ready"]
    finally:
        await server.shutdown()


async def test_a_failed_addition_takes_back_its_status_group_and_keeps_the_machinery_state(
    runtime: FakeRuntime,
) -> None:
    class Bare(WithMachineryState):
        bindings = ()

    server, machine = await start(runtime, Bare(name="Platform"))
    try:
        item = machine.added[0]
        failing = NodeAddition(
            binding="Broken",
            path="/broken",
            container=item.monitoring,
            plan=(PlanObject(name="inner", children=()),),
            root="Broken",
            group="Status",
            message_type=fixture_messages.Point,
        )

        with pytest.raises(UnprojectableFieldError):
            await item.mutate_address_space([failing])

        assert "Status" not in item.monitoring
        async with connected(server) as client:
            blocks = await browse(client.get_node(item.ua_node.nodeid), "MachineryBuildingBlocks")
            assert sorted(await names_of(blocks)) == ["MachineryItemState", "MachineryOperationMode"]
    finally:
        await server.shutdown()
