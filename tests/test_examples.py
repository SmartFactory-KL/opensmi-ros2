# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The example Machine declares without ROS2 and serves every Binding kind on a stock opensmi server."""

from __future__ import annotations

import inspect
from pathlib import Path
from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua
from opensmi.server import BaseMachine

import examples.run_server as run_server
from examples.component import Platform
from examples.machine import Robot
from opensmi.ros2.runtime import ParameterType
from opensmi.ros2.runtime.parameters import ParameterValue
from opensmi.ros2.server import ROS2Config, ROS2MachineMixin, load_machine, resolve_bindings
from tests import fixture_example, fixture_lifecycle, fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.served import connected, serve, until_value

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from asyncua.client.client import Client
    from asyncua.common.node import Node
    from opensmi.server import Server

# -- ring 0: the example declares ------------------------------------------------------------


def test_the_platform_declares_one_binding_of_every_kind() -> None:
    resolved = resolve_bindings(Platform.bindings)

    assert sorted(type(each).__name__ for each in resolved) == [
        "ActionBinding",
        "KeyedBinding",
        "LifecycleBinding",
        "MethodBinding",
        "MonitoringBinding",
        "PublisherBinding",
        "ROS2ParameterBinding",
        "TFBinding",
    ]


def test_binding_names_are_unique_across_the_component() -> None:
    resolved = resolve_bindings(Platform.bindings)

    assert len(resolved) == len({each.name for each in resolved})


def test_the_robot_is_a_machine_with_the_ros2_mixin() -> None:
    assert issubclass(Robot, ROS2MachineMixin)
    assert issubclass(Robot, BaseMachine)


def test_the_ros2_settings_file_loads_into_a_ros2_config() -> None:
    config = ROS2Config.from_toml(run_server.ROS2_CONFIG_PATH)

    assert config.node_name == "opensmi_ros2_example"


def test_main_is_a_coroutine_function_that_imported_with_no_rclpy_installed() -> None:
    # A module-scope Runtime import would fail collection on machines without ROS2.
    assert inspect.iscoroutinefunction(run_server.main)


# -- ring 1: the example is served -----------------------------------------------------------

MAX_VEL_X = "FollowPath.max_vel_x"


@pytest.fixture
def runtime() -> FakeRuntime:
    fake = FakeRuntime(
        domain_id=7,
        message_types=fixture_example.MESSAGE_TYPES,
        action_types=fixture_example.ACTION_TYPES,
        service_types=fixture_example.SERVICE_TYPES,
    )
    fixture_lifecycle.serve_nodes(fake, {"/planner_server": 3, "/controller_server": 3})
    fake.serve_parameters(
        "/controller_server",
        {
            MAX_VEL_X: ParameterValue(type=ParameterType.DOUBLE, value=0.26),
            "use_sim_time": ParameterValue(type=ParameterType.BOOL, value=True),
        },
    )
    return fake


@pytest.fixture
async def served(runtime: FakeRuntime) -> AsyncIterator[Server]:
    server = await serve(Robot(runtime=runtime))
    try:
        yield server
    finally:
        await server.shutdown()


async def child(node: Node, name: str) -> Node:
    """Return the child of a node with a browse name, which a missing one fails the test on."""
    for each in await node.get_children():
        if (await each.read_browse_name()).Name == name:
            return each
    names = [(await each.read_browse_name()).Name for each in await node.get_children()]
    pytest.fail(f"{name!r} is not below {node}; there are {names}")


async def below(node: Node, *names: str) -> Node:
    """Return the node reached by following browse names one level at a time."""
    for name in names:
        node = await child(node, name)
    return node


async def names_of(node: Node) -> list[str]:
    """Return the browse names of a node's children."""
    return [(await each.read_browse_name()).Name for each in await node.get_children()]


async def platform_of(client: Client) -> Node:
    """Return the Platform Component the example adds to its Machine."""
    return await below(client.nodes.objects, "Machines", "Robot", "Components", "Platform")


async def test_the_example_machine_starts_on_a_stock_server(served: Server) -> None:
    async with connected(served) as client:
        platform = await platform_of(client)

        assert {"Monitoring", "ParameterSet", "SkillSet", "MethodSet"} <= set(await names_of(platform))


async def test_a_monitoring_binding_serves_the_odometry_without_its_covariances(
    served: Server, runtime: FakeRuntime
) -> None:
    async with connected(served) as client:
        monitoring = await below(await platform_of(client), "Monitoring")
        assert "Odom" in await names_of(monitoring)
        assert not [name for name in await names_of(await below(monitoring, "Odom")) if "Covariance" in name]
        assert not [name for name in await names_of(await below(monitoring, "Odom", "Pose")) if "Covariance" in name]

        x = await below(monitoring, "Odom", "Pose", "Pose", "Position", "X")
        runtime.note_publishers("/odom", 1)
        runtime.deliver(
            "/odom",
            fixture_example.Odometry(
                header=fixture_messages.Header(stamp=fixture_messages.Time(sec=1, nanosec=2), frame_id="odom"),
                child_frame_id="base_link",
                pose=fixture_messages.PoseWithCovariance(
                    pose=fixture_messages.Pose(
                        position=fixture_messages.Point(x=1.5, y=0.0, z=0.0),
                        orientation=fixture_messages.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                    covariance=(0.0, 0.0, 0.0, 0.0),
                ),
                twist=fixture_example.TwistWithCovariance(
                    twist=fixture_example.Twist(
                        linear=fixture_messages.Vector3(x=0.0, y=0.0, z=0.0),
                        angular=fixture_messages.Vector3(x=0.0, y=0.0, z=0.0),
                    ),
                    covariance=(0.0, 0.0, 0.0, 0.0),
                ),
            ),
        )
        await until_value(x, 1.5)


async def test_a_keyed_binding_serves_the_aggregate_and_a_member_per_diagnostic_id(
    served: Server, runtime: FakeRuntime
) -> None:
    async with connected(served) as client:
        monitoring = await below(await platform_of(client), "Monitoring")
        assert "WorstLevel" in await names_of(await below(monitoring, "Diagnostics"))

        runtime.note_publishers("/diagnostics", 1)
        runtime.deliver(
            "/diagnostics",
            fixture_example.DiagnosticArray(
                header=fixture_messages.Header(stamp=fixture_messages.Time(sec=1, nanosec=2), frame_id="robot"),
                status=(
                    fixture_example.DiagnosticStatus(
                        level=1, name="motor", message="warm", hardware_id="robot_1", values=()
                    ),
                ),
            ),
        )

        member = await below(monitoring, "Diagnostics", "robot_1_motor")
        assert "Level" in await names_of(member)


async def test_the_parameter_binding_serves_the_controller_parameters(served: Server) -> None:
    async with connected(served) as client:
        parameters = await below(await platform_of(client), "ParameterSet")
        controller = await below(parameters, "Controller")

        assert "UseSimTime" in await names_of(controller)
        await until_value(await below(controller, "FollowPath", "MaxVelX"), 0.26)


async def test_the_skill_bindings_serve_a_finite_and_a_streaming_skill(served: Server) -> None:
    async with connected(served) as client:
        skills = await below(await platform_of(client), "SkillSet")

        assert {"NavigateToPose", "Drive"} <= set(await names_of(skills))


async def test_the_action_skill_has_the_goal_as_parameters_and_the_result_as_final_result(served: Server) -> None:
    async with connected(served) as client:
        execution = await below(await platform_of(client), "SkillSet", "NavigateToPose", "SkillExecution")

        assert "BehaviorTree" in await names_of(await child(execution, "ParameterSet"))
        assert "DistanceRemaining" in await names_of(await child(execution, "Monitoring"))
        assert "ErrorCode" in await names_of(await child(execution, "FinalResultData"))


async def test_the_service_binding_serves_a_method(served: Server) -> None:
    async with connected(served) as client:
        methods = await below(await platform_of(client), "MethodSet")

        assert "ClearGlobalCostmap" in await names_of(methods)


async def test_the_lifecycle_binding_serves_the_startup_skill_with_the_managed_nodes(served: Server) -> None:
    async with connected(served) as client:
        execution = await below(await platform_of(client), "SkillSet", "StartupSkill", "SkillExecution")
        navigation = await below(await child(execution, "Monitoring"), "Navigation")

        assert await names_of(navigation) == ["PlannerServer", "ControllerServer"]
        assert await names_of(await child(navigation, "PlannerServer")) == ["State"]


async def test_the_tf_binding_follows_tf_and_gives_the_platform_its_spatial_object(
    served: Server, runtime: FakeRuntime
) -> None:
    async with connected(served) as client:
        platform = await platform_of(client)

        assert "SpatialObject" in await names_of(platform)
        assert "/tf" in runtime.subscribed_topics


async def test_the_machine_carries_the_introspection_component(served: Server) -> None:
    async with connected(served) as client:
        components = await below(client.nodes.objects, "Machines", "Robot", "Components")

        assert set(await names_of(components)) == {"ROS2Introspection", "Platform"}


def test_the_example_does_not_need_a_published_frame() -> None:
    assert not [each for each in resolve_bindings(Platform.bindings) if getattr(each, "publishes", ())]


# -- ring 2: the same Machine as a file ------------------------------------------------------

MACHINE_TOML = Path(run_server.__file__).parent / "machine.toml"


async def snapshot(node: Node, path: str = "") -> dict[str, tuple[str, str]]:
    """Return every node below one, by browse path, with its class and (for a variable) its data type."""
    found: dict[str, tuple[str, str]] = {}
    for each in await node.get_children():
        here = f"{path}/{(await each.read_browse_name()).Name}"
        node_class = await each.read_node_class()
        data_type = str(await each.read_data_type()) if node_class == ua.NodeClass.Variable else ""
        found[here] = (node_class.name, data_type)
        found.update(await snapshot(each, here))
    return found


async def identification_of(client: Client, machine: str, *components: str) -> dict[str, object]:
    """Read the identification block of the Machine, or of the Component below it."""
    node = await below(client.nodes.objects, "Machines", machine)
    for name in components:
        node = await below(node, "Components", name)
    block = await below(node, "Identification")
    values: dict[str, object] = {}
    for name in ("SerialNumber", "Manufacturer", "ProductInstanceUri"):
        if name not in await names_of(block):
            continue  # a Component's block has no product URI
        value = await (await child(block, name)).read_value()
        values[name] = value.Text if isinstance(value, ua.LocalizedText) else value
    return values


def fake_runtime_for(config: ROS2Config) -> FakeRuntime:
    """Return the fake the served tests use, for a file's settings."""
    fake = FakeRuntime(
        domain_id=config.domain_id,
        message_types=fixture_example.MESSAGE_TYPES,
        action_types=fixture_example.ACTION_TYPES,
        service_types=fixture_example.SERVICE_TYPES,
    )
    fixture_lifecycle.serve_nodes(fake, {"/planner_server": 3, "/controller_server": 3})
    fake.serve_parameters(
        "/controller_server",
        {
            MAX_VEL_X: ParameterValue(type=ParameterType.DOUBLE, value=0.26),
            "use_sim_time": ParameterValue(type=ParameterType.BOOL, value=True),
        },
    )
    return fake


def test_the_ros2_settings_of_the_file_are_the_ones_of_the_settings_file() -> None:
    machine = load_machine(MACHINE_TOML, runtime_factory=fake_runtime_for)

    assert machine.ros2_config == ROS2Config.from_toml(run_server.ROS2_CONFIG_PATH)


async def test_the_example_file_declares_the_same_bindings_as_the_python_example() -> None:
    server = await serve(load_machine(MACHINE_TOML, runtime_factory=fake_runtime_for))
    try:
        platform: Any = server.machines[0].components["Platform"]  # pyright: ignore[reportAttributeAccessIssue]

        assert sorted(platform.ros2_bindings, key=lambda each: each.name) == sorted(
            resolve_bindings(Platform.bindings), key=lambda each: each.name
        )
    finally:
        await server.shutdown()


async def test_the_example_file_serves_the_same_address_space_as_the_python_example(runtime: FakeRuntime) -> None:
    python = await serve(Robot(runtime=runtime))
    file = await serve(load_machine(MACHINE_TOML, runtime_factory=fake_runtime_for))
    try:
        async with connected(python) as from_python, connected(file) as from_file:
            expected = await snapshot(await below(from_python.nodes.objects, "Machines", "Robot"))
            found = await snapshot(await below(from_file.nodes.objects, "Machines", "Robot"))

            assert found == expected
            assert await identification_of(from_file, "Robot") == await identification_of(from_python, "Robot")
            assert await identification_of(from_file, "Robot", "Platform") == await identification_of(
                from_python, "Robot", "Platform"
            )
    finally:
        await python.shutdown()
        await file.shutdown()
