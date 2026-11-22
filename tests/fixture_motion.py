# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Fixtures for the ros2_control motion and controller-listing interfaces, with the real field names,
types and order."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from opensmi.ros2.runtime.messages import (
    ActionTypes,
    NestedType,
    PrimitiveType,
    SequenceType,
    ServiceTypes,
    StringType,
)
from tests.fixture_messages import _BUILT, BOOL, FLOAT64, INT32, STRING, UINT8, UINT32, build, describe, nested

if TYPE_CHECKING:
    from collections.abc import Callable

    from opensmi.ros2.runtime.messages import Message

UINT16 = (PrimitiveType(name="uint16"), "uint16")
STRINGS = (SequenceType(element=StringType()), "sequence<string>")
DOUBLES = (SequenceType(element=PrimitiveType(name="float64")), "sequence<double>")


def repeated(type_name: str) -> Any:
    """Describe a field holding any number of messages of one type."""
    return SequenceType(element=NestedType(type_name=type_name)), f"sequence<{type_name.rsplit('/', 1)[-1]}>"


Duration = build(describe("fixture_msgs/msg/Duration", sec=INT32, nanosec=UINT32))

JointTrajectoryPoint = build(
    describe(
        "fixture_msgs/msg/JointTrajectoryPoint",
        positions=DOUBLES,
        velocities=DOUBLES,
        accelerations=DOUBLES,
        effort=DOUBLES,
        time_from_start=nested("fixture_msgs/msg/Duration"),
    )
)

JointTrajectory = build(
    describe(
        "fixture_msgs/msg/JointTrajectory",
        header=nested("fixture_msgs/msg/Header"),
        joint_names=STRINGS,
        points=repeated("fixture_msgs/msg/JointTrajectoryPoint"),
    )
)

Vector3 = build(describe("fixture_msgs/msg/Vector3", x=FLOAT64, y=FLOAT64, z=FLOAT64))
Transform = build(
    describe(
        "fixture_msgs/msg/Transform",
        translation=nested("fixture_msgs/msg/Vector3"),
        rotation=nested("fixture_msgs/msg/Quaternion"),
    )
)
Twist = build(
    describe(
        "fixture_msgs/msg/Twist", linear=nested("fixture_msgs/msg/Vector3"), angular=nested("fixture_msgs/msg/Vector3")
    )
)

MultiDOFJointTrajectoryPoint = build(
    describe(
        "fixture_msgs/msg/MultiDOFJointTrajectoryPoint",
        transforms=repeated("fixture_msgs/msg/Transform"),
        velocities=repeated("fixture_msgs/msg/Twist"),
        accelerations=repeated("fixture_msgs/msg/Twist"),
        time_from_start=nested("fixture_msgs/msg/Duration"),
    )
)

MultiDOFJointTrajectory = build(
    describe(
        "fixture_msgs/msg/MultiDOFJointTrajectory",
        header=nested("fixture_msgs/msg/Header"),
        joint_names=STRINGS,
        points=repeated("fixture_msgs/msg/MultiDOFJointTrajectoryPoint"),
    )
)

JointTolerance = build(
    describe("fixture_msgs/msg/JointTolerance", name=STRING, position=FLOAT64, velocity=FLOAT64, acceleration=FLOAT64)
)

JointComponentTolerance = build(
    describe(
        "fixture_msgs/msg/JointComponentTolerance",
        joint_name=STRING,
        component=UINT16,
        position=FLOAT64,
        velocity=FLOAT64,
        acceleration=FLOAT64,
    )
)

FollowJointTrajectoryGoal = build(
    describe(
        "fixture_msgs/action/FollowJointTrajectory_Goal",
        trajectory=nested("fixture_msgs/msg/JointTrajectory"),
        multi_dof_trajectory=nested("fixture_msgs/msg/MultiDOFJointTrajectory"),
        path_tolerance=repeated("fixture_msgs/msg/JointTolerance"),
        component_path_tolerance=repeated("fixture_msgs/msg/JointComponentTolerance"),
        goal_tolerance=repeated("fixture_msgs/msg/JointTolerance"),
        component_goal_tolerance=repeated("fixture_msgs/msg/JointComponentTolerance"),
        goal_time_tolerance=nested("fixture_msgs/msg/Duration"),
    )
)

FollowJointTrajectoryResult = build(
    describe("fixture_msgs/action/FollowJointTrajectory_Result", error_code=INT32, error_string=STRING)
)

FollowJointTrajectoryFeedback = build(
    describe(
        "fixture_msgs/action/FollowJointTrajectory_Feedback",
        header=nested("fixture_msgs/msg/Header"),
        joint_names=STRINGS,
        desired=nested("fixture_msgs/msg/JointTrajectoryPoint"),
        actual=nested("fixture_msgs/msg/JointTrajectoryPoint"),
        error=nested("fixture_msgs/msg/JointTrajectoryPoint"),
        multi_dof_joint_names=STRINGS,
        multi_dof_desired=nested("fixture_msgs/msg/MultiDOFJointTrajectoryPoint"),
        multi_dof_actual=nested("fixture_msgs/msg/MultiDOFJointTrajectoryPoint"),
        multi_dof_error=nested("fixture_msgs/msg/MultiDOFJointTrajectoryPoint"),
    )
)

FollowJointTrajectory = ActionTypes(
    type_name="fixture_msgs/action/FollowJointTrajectory",
    goal=FollowJointTrajectoryGoal,
    feedback=FollowJointTrajectoryFeedback,
    result=FollowJointTrajectoryResult,
)

ChainConnection = build(describe("fixture_msgs/msg/ChainConnection", name=STRING, reference_interfaces=STRINGS))

ControllerState = build(
    describe(
        "fixture_msgs/msg/ControllerState",
        name=STRING,
        state=STRING,
        type=STRING,
        is_async=BOOL,
        update_rate=UINT16,
        claimed_interfaces=STRINGS,
        required_command_interfaces=STRINGS,
        required_state_interfaces=STRINGS,
        is_chainable=BOOL,
        is_chained=BOOL,
        exported_state_interfaces=STRINGS,
        reference_interfaces=STRINGS,
        chain_connections=repeated("fixture_msgs/msg/ChainConnection"),
    )
)

ListControllersRequest = build(describe("fixture_msgs/srv/ListControllers_Request"))
ListControllersResponse = build(
    describe("fixture_msgs/srv/ListControllers_Response", controller=repeated("fixture_msgs/msg/ControllerState"))
)

ListControllers = ServiceTypes(
    type_name="fixture_msgs/srv/ListControllers",
    request=ListControllersRequest,
    response=ListControllersResponse,
)

LifecycleState = build(describe("fixture_msgs/msg/LifecycleState", id=UINT8, label=STRING))

HardwareInterface = build(
    describe("fixture_msgs/msg/HardwareInterface", name=STRING, data_type=STRING, is_available=BOOL, is_claimed=BOOL)
)

HardwareComponentState = build(
    describe(
        "fixture_msgs/msg/HardwareComponentState",
        name=STRING,
        type=STRING,
        is_async=BOOL,
        rw_rate=UINT16,
        class_type=STRING,
        plugin_name=STRING,
        state=nested("fixture_msgs/msg/LifecycleState"),
        command_interfaces=repeated("fixture_msgs/msg/HardwareInterface"),
        state_interfaces=repeated("fixture_msgs/msg/HardwareInterface"),
    )
)

ListHardwareComponentsRequest = build(describe("fixture_msgs/srv/ListHardwareComponents_Request"))
ListHardwareComponentsResponse = build(
    describe(
        "fixture_msgs/srv/ListHardwareComponents_Response",
        component=repeated("fixture_msgs/msg/HardwareComponentState"),
    )
)

ListHardwareComponents = ServiceTypes(
    type_name="fixture_msgs/srv/ListHardwareComponents",
    request=ListHardwareComponentsRequest,
    response=ListHardwareComponentsResponse,
)


@dataclass(frozen=True)
class MotionKit:
    """The types a scenario builds its messages from, however they came to exist.

    The fixture kit takes them from this module; the live-graph kit takes the installed real
    types. A scenario constructs everything through ``message`` (by the short name of a type, for
    instance ``JointTrajectoryPoint``) and never imports a class, so one scenario runs against both.
    """

    follow: ActionTypes
    list_controllers: ServiceTypes
    list_hardware: ServiceTypes
    message: Callable[[str], type[Message]]


def fixture_kit() -> MotionKit:
    """Return the kit made of the fixtures in this module."""
    return MotionKit(
        follow=FollowJointTrajectory,
        list_controllers=ListControllers,
        list_hardware=ListHardwareComponents,
        message=lambda name: _BUILT[f"fixture_msgs/msg/{name}"],
    )
