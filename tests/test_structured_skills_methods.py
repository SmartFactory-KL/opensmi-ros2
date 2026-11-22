# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Skills and Methods whose goals, results and responses are lists of structures.

Uses the `tests/fixture_motion.py` fixtures, and the installed ros2_control packages under `-m jazzy`.
"""

from __future__ import annotations

import struct
from typing import TYPE_CHECKING, Any, cast

import pytest
from asyncua import ua
from opensmi.core import SkillState

from opensmi.ros2.runtime.messages import ArrayType, NestedType, PrimitiveType, SequenceType, StringType
from opensmi.ros2.runtime.outcomes import GoalStatus
from opensmi.ros2.server.bindings import ActionBinding, MethodBinding
from tests import fixture_motion
from tests.fake_runtime import FakeRuntime
from tests.fixture_structures import encoding_id as encoding_node_id
from tests.fixture_structures import node_id as structure_node_id
from tests.served import TestComponent, TestMachine, connected, elements_of, leaf, leaves, serve, wire_value
from tests.test_action_binding_server import (
    GOOD,
    operating,
    press,
    reached,
    ready,
    results_of,
    running,
    skill_of,
    until,
)
from tests.test_method_binding_server import called, method_of
from tests.wire_client import run_client

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Awaitable, Callable

    from asyncua.client.client import Client
    from opensmi.server import Server

    from opensmi.ros2.runtime.messages import Message
    from tests.fixture_motion import MotionKit

FOLLOW = "/follow_joint_trajectory"
CONTROLLERS = "/controller_manager/list_controllers"
HARDWARE = "/controller_manager/list_hardware_components"

# The real names, for the live-graph kit: the short names the scenarios build with.
REAL_TYPES = {
    "Header": "std_msgs/msg/Header",
    "Time": "builtin_interfaces/msg/Time",
    "Duration": "builtin_interfaces/msg/Duration",
    "JointTrajectory": "trajectory_msgs/msg/JointTrajectory",
    "JointTrajectoryPoint": "trajectory_msgs/msg/JointTrajectoryPoint",
    "MultiDOFJointTrajectory": "trajectory_msgs/msg/MultiDOFJointTrajectory",
    "MultiDOFJointTrajectoryPoint": "trajectory_msgs/msg/MultiDOFJointTrajectoryPoint",
    "Transform": "geometry_msgs/msg/Transform",
    "Twist": "geometry_msgs/msg/Twist",
    "Vector3": "geometry_msgs/msg/Vector3",
    "Quaternion": "geometry_msgs/msg/Quaternion",
    "JointTolerance": "control_msgs/msg/JointTolerance",
    "JointComponentTolerance": "control_msgs/msg/JointComponentTolerance",
    "ControllerState": "controller_manager_msgs/msg/ControllerState",
    "ChainConnection": "controller_manager_msgs/msg/ChainConnection",
    "HardwareComponentState": "controller_manager_msgs/msg/HardwareComponentState",
    "HardwareInterface": "controller_manager_msgs/msg/HardwareInterface",
    "LifecycleState": "lifecycle_msgs/msg/State",
}


def live_kit() -> MotionKit:
    """Return the kit made of the installed interface packages, the types a robot's controllers speak."""
    from opensmi.ros2.runtime import Runtime
    from opensmi.ros2.runtime.messages import message_dataclass

    runtime = Runtime(domain_id=0, node_name="types_only")
    return fixture_motion.MotionKit(
        follow=runtime.describe_action("control_msgs/action/FollowJointTrajectory"),
        list_controllers=runtime.describe_service("controller_manager_msgs/srv/ListControllers"),
        list_hardware=runtime.describe_service("controller_manager_msgs/srv/ListHardwareComponents"),
        message=lambda name: message_dataclass(REAL_TYPES[name]),
    )


@pytest.fixture(params=["fixtures", pytest.param("installed", marks=pytest.mark.jazzy)])
def kit(request: pytest.FixtureRequest) -> MotionKit:
    return fixture_motion.fixture_kit() if request.param == "fixtures" else live_kit()


# -- messages, built the way a robot's controllers build them ----------------------------------


def blank(message_class: type[Message], **given: Any) -> Any:
    """Return a message holding the zero of every field but the ones given."""
    values = {
        each.name: zero(each.field_type, message_class.ros2_nested.get(each.name))
        for each in message_class.ros2.fields
        if each.name not in given
    }
    return message_class(**values, **given)


def zero(field_type: Any, nested: type[Message] | None) -> Any:
    """Return the zero of one field: empty, false, nought, or a message that is all of those."""
    match field_type:
        case StringType():
            return ""
        case PrimitiveType(name="bool"):
            return False
        case PrimitiveType(name="float32" | "float64"):
            return 0.0
        case PrimitiveType():
            return 0
        case SequenceType():
            return ()
        case ArrayType(element=element, length=length):
            return tuple(zero(element, nested) for _ in range(length))
        case NestedType():
            assert nested is not None
            return blank(nested)
    message = f"no zero for {field_type!r}"
    raise AssertionError(message)


# -- the arm, as a Component that declares three things and curates nothing --------------------


@pytest.fixture
def runtime(kit: MotionKit) -> FakeRuntime:
    started = FakeRuntime(
        domain_id=7,
        action_types={kit.follow.type_name: kit.follow},
        service_types={
            kit.list_controllers.type_name: kit.list_controllers,
            kit.list_hardware.type_name: kit.list_hardware,
        },
    )
    started.serve_action(FOLLOW)
    started.serve_service(CONTROLLERS, lambda _request: blank(kit.list_controllers.response, controller=()))
    started.serve_service(HARDWARE, lambda _request: blank(kit.list_hardware.response, component=()))
    return started


@pytest.fixture
async def served(runtime: FakeRuntime, kit: MotionKit) -> AsyncIterator[tuple[Server, Any]]:
    class Arm(TestComponent):
        bindings = (
            ActionBinding(name="Follow", path=FOLLOW, action_type=kit.follow.type_name),
            MethodBinding(name="ListControllers", path=CONTROLLERS, service_type=kit.list_controllers.type_name),
            MethodBinding(name="ListHardwareComponents", path=HARDWARE, service_type=kit.list_hardware.type_name),
        )

    machine = TestMachine(Arm(name="Arm"), runtime=runtime)
    server = await serve(machine)
    try:
        yield server, machine
    finally:
        await server.shutdown()


def follow_of(machine: Any) -> Any:
    return skill_of(machine, "Follow")


def structure_of(machine: Any, kit: MotionKit, name: str) -> ua.NodeId:
    """Return the DataType NodeId the server minted for one message type, by its short name."""
    return structure_node_id(kit.message(name).ros2.type_name, machine.added[0].ros2_namespace_index)


# -- the trajectory, the way a controller's client writes it -----------------------------------

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")


def point_maps(count: int) -> list[dict[str, object]]:
    """Return the points of a trajectory as the maps a client writes, one per waypoint."""
    return [
        {
            "positions": [0.1 * each + 0.01 * joint for joint in range(len(JOINTS))],
            "velocities": [0.5 - 0.01 * joint for joint in range(len(JOINTS))],
            "accelerations": [],
            "effort": [],
            "time_from_start.sec": each // 2,
            "time_from_start.nanosec": (each % 2) * 500_000_000 + 250,
        }
        for each in range(count)
    ]


def points_of(kit: MotionKit, maps: list[dict[str, Any]]) -> tuple[Any, ...]:
    """Return the JointTrajectoryPoint messages the maps above mean."""
    return tuple(
        blank(
            kit.message("JointTrajectoryPoint"),
            positions=tuple(each["positions"]),
            velocities=tuple(each["velocities"]),
            accelerations=tuple(each["accelerations"]),
            effort=tuple(each["effort"]),
            time_from_start=blank(
                kit.message("Duration"),
                sec=each["time_from_start.sec"],
                nanosec=each["time_from_start.nanosec"],
            ),
        )
        for each in maps
    )


def trajectory_goal(kit: MotionKit, maps: list[dict[str, Any]], **more: Any) -> Any:
    """Return the goal a robot must receive for a trajectory of these points and nothing else set."""
    trajectory = blank(
        kit.message("JointTrajectory"),
        header=blank(kit.message("Header"), frame_id="base_link"),
        joint_names=JOINTS,
        points=points_of(kit, maps),
    )
    return blank(kit.follow.goal, trajectory=trajectory, **more)


async def put(client: Client, variable: Any, value: object) -> None:
    """Write one Parameter the way a client does, with the variant type the server declared."""
    declared = (await variable.ua_node.read_data_value()).Value.VariantType
    await client.get_node(variable.ua_node.nodeid).write_value(ua.Variant(wire_value(variable, value), declared))


async def write_trajectory(client: Client, skill: Any, maps: list[dict[str, object]]) -> None:
    """Write the three Parameters of a trajectory."""
    await put(client, leaf(skill.parameter_set, "Trajectory.Header.FrameId"), "base_link")
    await put(client, leaf(skill.parameter_set, "Trajectory.JointNames"), list(JOINTS))
    await put(client, leaf(skill.parameter_set, "Trajectory.Points"), maps)


async def start_with(
    server: Server, machine: Any, runtime: FakeRuntime, kit: MotionKit, fill: Callable[[Client, Any], Awaitable[None]]
) -> Any:
    """Run one Start after ``fill`` wrote the Parameters, and return the goal the robot received."""
    skill = follow_of(machine)
    async with operating(server, machine) as client:
        await ready(client, server, skill)
        await fill(client, skill)
        await running(client, server, skill, runtime, FOLLOW, blank(kit.follow.feedback))
    assert skill.ros2_goal is not None
    action, goal = runtime.sent_goals[-1]
    assert action == FOLLOW
    return goal


# -- FollowJointTrajectory is a Skill ---------------------------------------------------------


async def test_the_trajectory_action_declares_as_a_skill_with_nothing_curated(served, kit: MotionKit) -> None:
    server, machine = served
    skill = follow_of(machine)

    assert skill.binding.skill_type is None
    assert skill.binding.goal_from is None
    assert machine.added[0].projections == ()
    parameters = set(leaves(skill.parameter_set))
    assert {"Trajectory.JointNames", "Trajectory.Points", "MultiDofTrajectory.Points"} <= parameters
    assert {"PathTolerance", "GoalTolerance", "ComponentPathTolerance", "ComponentGoalTolerance"} <= parameters
    async with connected(server, "visitor") as client:
        for name, short in (
            ("Trajectory.Points", "JointTrajectoryPoint"),
            ("MultiDofTrajectory.Points", "MultiDOFJointTrajectoryPoint"),
            ("PathTolerance", "JointTolerance"),
            ("ComponentGoalTolerance", "JointComponentTolerance"),
        ):
            node = client.get_node(leaf(skill.parameter_set, name).ua_node.nodeid)
            assert await node.read_data_type() == structure_of(machine, kit, short), name
            assert await node.read_value_rank() == ua.ValueRank.OneDimension, name
            assert await node.read_value() == [], name


async def test_the_goal_a_robot_receives_has_exactly_the_points_a_client_wrote(served, runtime, kit: MotionKit) -> None:
    server, machine = served
    maps = point_maps(4)

    async def fill(client: Client, skill: Any) -> None:
        await write_trajectory(client, skill, maps)

    goal = await start_with(server, machine, runtime, kit, fill)

    assert goal == trajectory_goal(kit, maps)
    assert len(goal.trajectory.points) == 4
    assert goal.trajectory.points[3].time_from_start.nanosec == 500_000_250
    assert goal.trajectory.points[3].positions[5] == 0.1 * 3 + 0.01 * 5


async def test_what_a_client_wrote_is_what_it_reads_back_before_it_starts(served) -> None:
    server, machine = served
    skill = follow_of(machine)
    maps = point_maps(3)

    async with operating(server, machine) as client:
        await write_trajectory(client, skill, maps)
        points = leaf(skill.parameter_set, "Trajectory.Points")
        stored = elements_of(points, await client.get_node(points.ua_node.nodeid).read_value())

    assert stored == maps


async def test_the_same_parameter_takes_a_trajectory_of_any_length_on_every_goal(served, runtime, kit) -> None:
    server, machine = served
    skill = follow_of(machine)

    async with operating(server, machine) as client:
        for count in (25, 1, 0, 3):
            maps = point_maps(count)
            await ready(client, server, skill)
            await write_trajectory(client, skill, maps)
            await running(client, server, skill, runtime, FOLLOW, blank(kit.follow.feedback))
            assert skill.ros2_goal is not None
            assert runtime.sent_goals[-1] == (FOLLOW, trajectory_goal(kit, maps)), count
            runtime.settle_goal(FOLLOW, status=GoalStatus.SUCCEEDED, result=blank(kit.follow.result))
            await reached(skill, SkillState.COMPLETED)


async def test_a_list_inside_each_point_of_a_list_keeps_its_length_per_point(served, runtime, kit) -> None:
    server, machine = served
    maps = point_maps(2)
    maps[1]["positions"] = [1.5, -1.5]
    maps[1]["velocities"] = []
    maps[0]["effort"] = [3.0, 2.0, 1.0]

    async def fill(client: Client, skill: Any) -> None:
        await write_trajectory(client, skill, maps)

    goal = await start_with(server, machine, runtime, kit, fill)

    assert goal == trajectory_goal(kit, maps)
    assert [len(each.positions) for each in goal.trajectory.points] == [6, 2]
    assert goal.trajectory.points[0].effort == (3.0, 2.0, 1.0)


async def test_tolerances_and_a_multi_dof_trajectory_are_structures_too(served, runtime, kit) -> None:
    server, machine = served
    path_tolerance = [
        {"name": "shoulder_pan", "position": 0.05, "velocity": 0.1, "acceleration": 0.0},
        {"name": "elbow", "position": 0.02, "velocity": -1.0, "acceleration": 0.0},
    ]
    component_goal = [
        {"joint_name": "base", "component": 4, "position": 0.01, "velocity": 0.0, "acceleration": 0.0},
    ]
    multi_dof = [
        {
            "transforms": [
                {
                    "translation.x": 0.1,
                    "translation.y": 0.2,
                    "translation.z": 0.3,
                    "rotation.x": 0.0,
                    "rotation.y": 0.0,
                    "rotation.z": 0.0,
                    "rotation.w": 1.0,
                }
            ],
            "velocities": [
                {
                    "linear.x": 0.5,
                    "linear.y": 0.0,
                    "linear.z": 0.0,
                    "angular.x": 0.0,
                    "angular.y": 0.0,
                    "angular.z": 0.25,
                }
            ],
            "accelerations": [],
            "time_from_start.sec": 2,
            "time_from_start.nanosec": 5,
        },
        {
            "transforms": [],
            "velocities": [],
            "accelerations": [],
            "time_from_start.sec": 3,
            "time_from_start.nanosec": 0,
        },
    ]

    async def fill(client: Client, skill: Any) -> None:
        await put(client, leaf(skill.parameter_set, "PathTolerance"), path_tolerance)
        await put(client, leaf(skill.parameter_set, "ComponentGoalTolerance"), component_goal)
        await put(client, leaf(skill.parameter_set, "MultiDofTrajectory.Points"), multi_dof)

    goal = await start_with(server, machine, runtime, kit, fill)

    vector = kit.message("Vector3")
    duration = kit.message("Duration")
    point = kit.message("MultiDOFJointTrajectoryPoint")
    expected_points = (
        blank(
            point,
            transforms=(
                blank(
                    kit.message("Transform"),
                    translation=blank(vector, x=0.1, y=0.2, z=0.3),
                    rotation=blank(kit.message("Quaternion"), w=1.0),
                ),
            ),
            velocities=(blank(kit.message("Twist"), linear=blank(vector, x=0.5), angular=blank(vector, z=0.25)),),
            time_from_start=blank(duration, sec=2, nanosec=5),
        ),
        blank(point, time_from_start=blank(duration, sec=3)),
    )
    tolerance = kit.message("JointTolerance")
    assert goal == blank(
        kit.follow.goal,
        trajectory=blank(kit.message("JointTrajectory")),
        multi_dof_trajectory=blank(kit.message("MultiDOFJointTrajectory"), points=expected_points),
        path_tolerance=(
            blank(tolerance, name="shoulder_pan", position=0.05, velocity=0.1),
            blank(tolerance, name="elbow", position=0.02, velocity=-1.0),
        ),
        component_goal_tolerance=(
            blank(kit.message("JointComponentTolerance"), joint_name="base", component=4, position=0.01),
        ),
    )


async def test_a_structure_of_the_wrong_type_is_refused_on_the_write_and_the_trajectory_stays(served) -> None:
    server, machine = served
    skill = follow_of(machine)
    tolerances = leaf(skill.parameter_set, "PathTolerance")

    async with operating(server, machine) as client:
        await write_trajectory(client, skill, point_maps(2))
        node = client.get_node(leaf(skill.parameter_set, "Trajectory.Points").ua_node.nodeid)
        wrong = [{"name": "elbow", "position": 0.1, "velocity": 0.0, "acceleration": 0.0}]
        with pytest.raises(ua.uaerrors.BadTypeMismatch):
            await node.write_value(ua.Variant(wire_value(tolerances, wrong), ua.VariantType.ExtensionObject))

    kept = leaf(skill.parameter_set, "Trajectory.Points")
    assert elements_of(kept, await kept.read()) == point_maps(2)


async def test_the_feedback_with_its_points_and_lists_is_shown_to_a_client(served, runtime, kit) -> None:
    server, machine = served
    skill = follow_of(machine)
    point = kit.message("JointTrajectoryPoint")
    duration = kit.message("Duration")
    transform = blank(kit.message("Transform"), translation=blank(kit.message("Vector3"), x=1.0, z=2.0))
    feedback = blank(
        kit.follow.feedback,
        header=blank(kit.message("Header"), frame_id="base_link"),
        joint_names=("shoulder_pan", "elbow"),
        desired=blank(
            point, positions=(0.5, 0.6), velocities=(0.1, 0.1), time_from_start=blank(duration, sec=1, nanosec=100)
        ),
        actual=blank(point, positions=(0.4, 0.6), velocities=(0.0, 0.1)),
        error=blank(point, positions=(0.1, 0.0)),
        multi_dof_joint_names=("a", "b"),
        multi_dof_desired=blank(kit.message("MultiDOFJointTrajectoryPoint"), transforms=(transform, transform)),
    )

    async with operating(server, machine) as client:
        await ready(client, server, skill)
        await running(client, server, skill, runtime, FOLLOW, feedback)

        async def arrived() -> bool:
            runtime.deliver_feedback(FOLLOW, feedback)
            return (await skill.read_monitoring())["JointNames"] == ["shoulder_pan", "elbow"]

        await until(arrived)
        shown = await skill.read_monitoring()
        node = client.get_node(leaf(skill.monitoring, "MultiDofDesired.Transforms").ua_node.nodeid)
        datatype = await node.read_data_type()

    assert shown["Desired.Positions"] == [0.5, 0.6]
    assert shown["Desired.TimeFromStart.Sec"] == 1
    assert shown["Desired.TimeFromStart.Nanosec"] == 100
    assert shown["Actual.Velocities"] == [0.0, 0.1]
    assert shown["Error.Positions"] == [0.1, 0.0]
    assert shown["Header.FrameId"] == "base_link"
    assert shown["MultiDofJointNames"] == ["a", "b"]
    expected = {
        "translation.x": 1.0,
        "translation.y": 0.0,
        "translation.z": 2.0,
        "rotation.x": 0.0,
        "rotation.y": 0.0,
        "rotation.z": 0.0,
        "rotation.w": 0.0,
    }
    assert shown["MultiDofDesired.Transforms"] == [expected, expected]
    assert datatype == structure_of(machine, kit, "Transform")


async def test_a_succeeded_goal_shows_its_result_and_an_aborted_one_shows_why(served, runtime, kit) -> None:
    server, machine = served
    skill = follow_of(machine)

    async with operating(server, machine) as client:
        await ready(client, server, skill)
        await running(client, server, skill, runtime, FOLLOW, blank(kit.follow.feedback))
        runtime.settle_goal(
            FOLLOW, status=GoalStatus.SUCCEEDED, result=blank(kit.follow.result, error_code=0, error_string="")
        )
        await reached(skill, SkillState.COMPLETED)
        assert await results_of(skill) == {"ErrorCode": 0, "ErrorString": ""}

        await ready(client, server, skill)
        await running(client, server, skill, runtime, FOLLOW, blank(kit.follow.feedback))
        runtime.settle_goal(
            FOLLOW,
            status=GoalStatus.ABORTED,
            result=blank(kit.follow.result, error_code=-4, error_string="path tolerance violated"),
        )
        await reached(skill, SkillState.HALTED)

    assert await results_of(skill) == {"ErrorCode": -4, "ErrorString": "path tolerance violated"}


async def test_a_halted_trajectory_asks_the_robot_to_cancel_it(served, runtime, kit) -> None:
    server, machine = served
    skill = follow_of(machine)

    async with operating(server, machine) as client:
        await ready(client, server, skill)
        await running(client, server, skill, runtime, FOLLOW, blank(kit.follow.feedback))
        assert await press(client, server, skill, "Halt") == GOOD
        await reached(skill, SkillState.HALTED)

    assert runtime.cancel_requests == (FOLLOW,)


# -- ListControllers and ListHardwareComponents are Methods -----------------------------------

ARM_INTERFACES = tuple(f"{joint}/position" for joint in JOINTS)
ARM_STATE_INTERFACES = tuple(f"{joint}/{kind}" for joint in JOINTS for kind in ("position", "velocity"))


def controller(kit: MotionKit, name: str, state: str, **given: Any) -> tuple[Any, dict[str, object]]:
    """Return one controller as the manager answers it, and as a client must read it."""
    claimed = given.pop("claimed", ())
    chain = given.pop("chain", ())
    fields: dict[str, Any] = {
        "name": name,
        "state": state,
        "type": "joint_trajectory_controller/JointTrajectoryController",
        "is_async": False,
        "update_rate": 100,
        "claimed_interfaces": tuple(claimed),
        "required_command_interfaces": ARM_INTERFACES,
        "required_state_interfaces": ARM_STATE_INTERFACES,
        "is_chainable": bool(chain),
        "is_chained": False,
        "exported_state_interfaces": (),
        "reference_interfaces": tuple(given.pop("references", ())),
    }
    message = blank(
        kit.message("ControllerState"),
        **fields,
        chain_connections=tuple(
            blank(kit.message("ChainConnection"), name=each, reference_interfaces=refs) for each, refs in chain
        ),
    )
    read: dict[str, object] = {
        **{key: list(value) if isinstance(value, tuple) else value for key, value in fields.items()},
        "chain_connections": [{"name": each, "reference_interfaces": list(refs)} for each, refs in chain],
    }
    return message, read


def hardware(
    kit: MotionKit, name: str, state: tuple[int, str], commands: int, states: int
) -> tuple[Any, dict[str, object]]:
    """Return one hardware component as the manager answers it, and as a client must read it."""
    interface = kit.message("HardwareInterface")

    def interfaces(kind: str, count: int) -> tuple[tuple[Any, ...], list[dict[str, object]]]:
        sent = tuple(
            blank(interface, name=f"joint_{each}/{kind}", data_type="double", is_available=True, is_claimed=each == 0)
            for each in range(count)
        )
        read = [
            {"name": f"joint_{each}/{kind}", "data_type": "double", "is_available": True, "is_claimed": each == 0}
            for each in range(count)
        ]
        return sent, read

    sent_commands, read_commands = interfaces("position", commands)
    sent_states, read_states = interfaces("velocity", states)
    message = blank(
        kit.message("HardwareComponentState"),
        name=name,
        type="system",
        is_async=False,
        rw_rate=250,
        class_type="",
        plugin_name="mock_components/GenericSystem",
        state=blank(kit.message("LifecycleState"), id=state[0], label=state[1]),
        command_interfaces=sent_commands,
        state_interfaces=sent_states,
    )
    read: dict[str, object] = {
        "name": name,
        "type": "system",
        "is_async": False,
        "rw_rate": 250,
        "class_type": "",
        "plugin_name": "mock_components/GenericSystem",
        "state.id": state[0],
        "state.label": state[1],
        "command_interfaces": read_commands,
        "state_interfaces": read_states,
    }
    return message, read


async def answer_of(server: Server, machine: Any, name: str, variable: str) -> list[dict[str, object]]:
    """Call a Method as a client does, then read the one list its response became, as a client does."""
    assert await called(server, machine, name) == GOOD
    result = method_of(machine, name).final_result_data[variable]
    async with connected(server, "visitor") as client:
        raw = await client.get_node(result.ua_node.nodeid).read_value()
    return cast("list[dict[str, object]]", elements_of(result, raw))


async def test_the_two_listing_services_declare_as_methods_with_nothing_curated(served, kit) -> None:
    server, machine = served
    controllers = method_of(machine, "ListControllers")
    components = method_of(machine, "ListHardwareComponents")

    assert machine.added[0].projections == ()
    assert [variable.name for variable in controllers.parameter_set] == []
    assert [variable.name for variable in controllers.final_result_data] == ["Controller"]
    assert [variable.name for variable in components.final_result_data] == ["Component"]
    async with connected(server, "visitor") as client:
        for method, variable, short in (
            (controllers, "Controller", "ControllerState"),
            (components, "Component", "HardwareComponentState"),
        ):
            node = client.get_node(method.final_result_data[variable].ua_node.nodeid)
            assert await node.read_data_type() == structure_of(machine, kit, short)
            assert await node.read_value_rank() == ua.ValueRank.OneDimension
            assert ua.AccessLevel.CurrentWrite not in await node.get_access_level()


async def test_a_client_reads_every_controller_with_every_list_it_carries(served, runtime, kit) -> None:
    server, machine = served
    sent, read = zip(
        controller(kit, "joint_state_broadcaster", "active"),
        controller(kit, "arm_controller", "active", claimed=ARM_INTERFACES),
        controller(
            kit,
            "arm_chain",
            "inactive",
            chain=(("arm_controller", ("a/position",)), ("gripper", ("jaw/position", "jaw/velocity"))),
            references=("a/position", "b/position"),
        ),
        strict=True,
    )
    runtime.serve_service(CONTROLLERS, lambda _request: blank(kit.list_controllers.response, controller=tuple(sent)))

    answered = await answer_of(server, machine, "ListControllers", "Controller")

    assert answered == list(read)
    arm = answered[1]
    assert arm["claimed_interfaces"] == list(ARM_INTERFACES)
    assert answered[2]["chain_connections"] == [
        {"name": "arm_controller", "reference_interfaces": ["a/position"]},
        {"name": "gripper", "reference_interfaces": ["jaw/position", "jaw/velocity"]},
    ]
    assert answered[2]["reference_interfaces"] == ["a/position", "b/position"]


async def test_a_client_reads_every_hardware_component_with_its_interfaces(served, runtime, kit) -> None:
    server, machine = served
    sent, read = zip(
        hardware(kit, "arm_system", (3, "active"), 6, 12),
        hardware(kit, "gripper", (2, "inactive"), 1, 0),
        strict=True,
    )
    runtime.serve_service(HARDWARE, lambda _request: blank(kit.list_hardware.response, component=tuple(sent)))

    answered = await answer_of(server, machine, "ListHardwareComponents", "Component")

    assert answered == list(read)
    assert [len(each["command_interfaces"]) for each in answered] == [6, 1]  # type: ignore[arg-type]
    assert answered[0]["state_interfaces"][11] == {  # type: ignore[index]
        "name": "joint_11/velocity",
        "data_type": "double",
        "is_available": True,
        "is_claimed": False,
    }
    assert answered[1]["state_interfaces"] == []
    assert answered[0]["state.label"] == "active"


async def test_the_next_call_replaces_the_list_and_an_empty_answer_is_an_empty_list(served, runtime, kit) -> None:
    server, machine = served
    answers: list[tuple[Any, ...]] = [
        tuple(controller(kit, f"controller_{each}", "active", claimed=ARM_INTERFACES[:each])[0] for each in range(40)),
        (controller(kit, "only", "active")[0],),
        (),
    ]
    runtime.serve_service(CONTROLLERS, lambda _request: blank(kit.list_controllers.response, controller=answers.pop(0)))

    counts = [len(await answer_of(server, machine, "ListControllers", "Controller")) for _ in range(3)]

    assert counts == [40, 1, 0]


async def test_a_stranger_reads_the_controller_list_from_the_wire_with_nothing_hidden(served, runtime, kit) -> None:
    # A client in another process that knows none of our types.
    server, machine = served
    sent, _ = controller(kit, "arm", "active", claimed=("a/position", "b/position"), chain=(("base", ("x",)),))
    runtime.serve_service(CONTROLLERS, lambda _request: blank(kit.list_controllers.response, controller=(sent,)))
    assert await called(server, machine, "ListControllers") == GOOD
    result = leaf(method_of(machine, "ListControllers").final_result_data, "Controller")

    answer = await run_client(server, "visitor", result.ua_node.nodeid, {"definition": True, "writes": []})

    namespace = machine.added[0].ros2_namespace_index
    encoding = encoding_node_id(kit.message("ControllerState").ros2.type_name, namespace).to_string()
    body = (
        string("arm")
        + string("active")
        + string("joint_trajectory_controller/JointTrajectoryController")
        + struct.pack("<?H", False, 100)
        + strings("a/position", "b/position")
        + strings(*ARM_INTERFACES)
        + strings(*ARM_STATE_INTERFACES)
        + struct.pack("<??", True, False)
        + strings()
        + strings()
        + struct.pack("<i", 1)
        + string("base")
        + strings("x")
    )
    assert answer[0] == ["initial", [[encoding, body.hex()]]]
    _, datatype, rank, definition = answer[1]
    assert datatype == f"ns={namespace};s={kit.message('ControllerState').ros2.type_name}"
    assert rank == 1
    assert [each[0] for each in definition] == [
        "name",
        "state",
        "type",
        "is_async",
        "update_rate",
        "claimed_interfaces",
        "required_command_interfaces",
        "required_state_interfaces",
        "is_chainable",
        "is_chained",
        "exported_state_interfaces",
        "reference_interfaces",
        "chain_connections",
    ]
    by_name = {each[0]: each for each in definition}
    string_type = "i=12"
    for name in ("claimed_interfaces", "required_command_interfaces", "exported_state_interfaces"):
        assert by_name[name][1:3] == [string_type, 1], name
    chain = by_name["chain_connections"]
    assert chain[1] == f"ns={namespace};s={kit.message('ChainConnection').ros2.type_name}"
    assert chain[2] == 1


def string(text: str) -> bytes:
    raw = text.encode()
    return struct.pack("<i", len(raw)) + raw


def strings(*texts: str) -> bytes:
    return struct.pack("<i", len(texts)) + b"".join(string(each) for each in texts)
