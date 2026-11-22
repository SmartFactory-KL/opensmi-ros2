# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The arm trajectory action and controller listings over a real ROS2 graph.

Marked `jazzy`; run with `pytest -m jazzy` in WSL with Jazzy sourced.
"""

from __future__ import annotations

import contextlib
import threading
from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua
from opensmi.core import SkillState

from opensmi.ros2.server.bindings import ActionBinding, MethodBinding
from tests.served import TestComponent, TestMachine, connected, leaf, serve
from tests.test_action_binding_server import GOOD, operating, press, reached, ready, results_of, skill_of, until
from tests.test_method_binding_server import called, method_of
from tests.test_structured_skills_methods import JOINTS, point_maps, write_trajectory

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Iterator

    from opensmi.server import Server

pytestmark = pytest.mark.jazzy

# A domain used only by this file, so a live robot on the default domain cannot answer it.
DOMAIN = 74
FOLLOW = "/follow_joint_trajectory"
CONTROLLERS = "/controller_manager/list_controllers"
HARDWARE = "/controller_manager/list_hardware_components"


class Peers:
    """What the robot side saw and what it answers with, filled in by rclpy callbacks on a thread."""

    def __init__(self) -> None:
        self.goals: list[list[tuple[list[float], list[float], int, int]]] = []
        self.joint_names: list[list[str]] = []


@pytest.fixture
def peers() -> Iterator[Peers]:
    """Yield a real arm: the trajectory action and the two listing services, on one node."""
    import rclpy.context
    import rclpy.executors
    import rclpy.node
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.msg import ChainConnection, ControllerState, HardwareComponentState, HardwareInterface
    from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
    from lifecycle_msgs.msg import State
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from trajectory_msgs.msg import JointTrajectoryPoint

    seen = Peers()
    context = rclpy.context.Context()
    context.init(args=None, domain_id=DOMAIN)
    node = rclpy.node.Node("arm", context=context)

    def execute(goal_handle: Any) -> Any:
        trajectory = goal_handle.request.trajectory
        seen.joint_names.append(list(trajectory.joint_names))
        seen.goals.append(
            [
                (list(each.positions), list(each.velocities), each.time_from_start.sec, each.time_from_start.nanosec)
                for each in trajectory.points
            ]
        )
        goal_handle.publish_feedback(
            FollowJointTrajectory.Feedback(
                joint_names=list(trajectory.joint_names),
                desired=JointTrajectoryPoint(positions=[0.25, 0.5]),
            )
        )
        goal_handle.succeed()
        return FollowJointTrajectory.Result(error_code=0, error_string="")

    def list_controllers(_request: Any, response: Any) -> Any:
        response.controller = [
            ControllerState(
                name="arm_controller",
                state="active",
                type="joint_trajectory_controller/JointTrajectoryController",
                update_rate=100,
                claimed_interfaces=[f"{joint}/position" for joint in JOINTS],
                required_command_interfaces=[f"{joint}/position" for joint in JOINTS],
                required_state_interfaces=[f"{joint}/position" for joint in JOINTS],
                chain_connections=[ChainConnection(name="gripper", reference_interfaces=["jaw/position"])],
            ),
            ControllerState(name="joint_state_broadcaster", state="active"),
        ]
        return response

    def list_hardware(_request: Any, response: Any) -> Any:
        response.component = [
            HardwareComponentState(
                name="arm_system",
                type="system",
                rw_rate=250,
                state=State(id=3, label="active"),
                command_interfaces=[HardwareInterface(name="shoulder_pan/position", data_type="double")],
                state_interfaces=[
                    HardwareInterface(name="shoulder_pan/position", data_type="double", is_available=True),
                    HardwareInterface(name="shoulder_pan/velocity", data_type="double", is_available=True),
                ],
            )
        ]
        return response

    action_server = ActionServer(
        node,
        FollowJointTrajectory,
        FOLLOW,
        execute_callback=execute,
        goal_callback=lambda _request: GoalResponse.ACCEPT,
        cancel_callback=lambda _handle: CancelResponse.ACCEPT,
    )
    services = [
        node.create_service(ListControllers, CONTROLLERS, list_controllers),
        node.create_service(ListHardwareComponents, HARDWARE, list_hardware),
    ]
    executor = rclpy.executors.MultiThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield seen
    finally:
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        for each in services:
            node.destroy_service(each)
        action_server.destroy()
        node.destroy_node()
        context.try_shutdown()


@pytest.fixture
async def served(peers: Peers) -> AsyncIterator[tuple[Server, Any]]:
    from opensmi.ros2.runtime import Runtime

    class Arm(TestComponent):
        bindings = (
            ActionBinding(name="Follow", path=FOLLOW, action_type="control_msgs/action/FollowJointTrajectory"),
            MethodBinding(
                name="ListControllers", path=CONTROLLERS, service_type="controller_manager_msgs/srv/ListControllers"
            ),
            MethodBinding(
                name="ListHardwareComponents",
                path=HARDWARE,
                service_type="controller_manager_msgs/srv/ListHardwareComponents",
            ),
        )

    runtime = Runtime(domain_id=DOMAIN, node_name="opensmi_under_test")
    machine = TestMachine(Arm(name="Arm"), runtime=runtime)
    server = await serve(machine)
    try:
        yield server, machine
    finally:
        await server.shutdown()
        with contextlib.suppress(Exception):
            if runtime.started:
                await runtime.stop()


async def test_a_trajectory_a_client_wrote_reaches_a_real_action_server_point_for_point(served, peers) -> None:
    server, machine = served
    skill = skill_of(machine, "Follow")
    maps = point_maps(4)

    async with operating(server, machine) as client:
        await ready(client, server, skill)
        await write_trajectory(client, skill, maps)
        assert await press(client, server, skill, "Start") == GOOD
        await reached(skill, SkillState.COMPLETED)

    assert peers.joint_names == [list(JOINTS)]
    assert peers.goals == [
        [
            (each["positions"], each["velocities"], each["time_from_start.sec"], each["time_from_start.nanosec"])
            for each in maps
        ]
    ]
    assert await results_of(skill) == {"ErrorCode": 0, "ErrorString": ""}

    async def shown() -> bool:
        return (await skill.read_monitoring())["Desired.Positions"] == [0.25, 0.5]

    await until(shown)


async def test_the_real_listing_services_come_back_as_arrays_of_structures(served) -> None:
    server, machine = served

    assert await called(server, machine, "ListControllers") == GOOD
    assert await called(server, machine, "ListHardwareComponents") == GOOD

    controllers = await method_of(machine, "ListControllers").read_results()
    components = await method_of(machine, "ListHardwareComponents").read_results()
    listed = controllers["Controller"]
    assert isinstance(listed, list)
    assert [each["name"] for each in listed] == ["arm_controller", "joint_state_broadcaster"]
    assert listed[0]["claimed_interfaces"] == [f"{joint}/position" for joint in JOINTS]
    assert listed[0]["chain_connections"] == [{"name": "gripper", "reference_interfaces": ["jaw/position"]}]
    assert listed[1]["claimed_interfaces"] == []
    hardware = components["Component"]
    assert isinstance(hardware, list)
    assert hardware[0]["state.label"] == "active"
    assert [each["name"] for each in hardware[0]["state_interfaces"]] == [
        "shoulder_pan/position",
        "shoulder_pan/velocity",
    ]
    assert hardware[0]["command_interfaces"][0]["data_type"] == "double"
    async with connected(server, "visitor") as client:
        node = client.get_node(
            leaf(method_of(machine, "ListControllers").final_result_data, "Controller").ua_node.nodeid
        )
        assert await node.read_value_rank() == ua.ValueRank.OneDimension
        assert len(await node.read_value()) == 2
