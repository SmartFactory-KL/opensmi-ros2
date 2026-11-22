# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Hand-shaped Nav2 interfaces for the example Component's tests, so they run without ROS2."""

from __future__ import annotations

from typing import Any

from opensmi.ros2.runtime.messages import ActionTypes, ArrayType, NestedType, PrimitiveType, SequenceType
from tests import fixture_lifecycle, fixture_messages
from tests.fixture_messages import FLOAT32, INT8, STRING, build, describe, nested

UINT16 = (PrimitiveType(name="uint16"), "uint16")
COVARIANCE = (ArrayType(element=PrimitiveType(name="float64"), length=4), "double[4]")

Twist = build(
    describe(
        "fixture_msgs/msg/Twist",
        linear=nested("fixture_msgs/msg/Vector3"),
        angular=nested("fixture_msgs/msg/Vector3"),
    )
)

TwistWithCovariance = build(
    describe(
        "fixture_msgs/msg/TwistWithCovariance",
        twist=nested("fixture_msgs/msg/Twist"),
        covariance=COVARIANCE,
    )
)

Odometry = build(
    describe(
        "fixture_msgs/msg/Odometry",
        header=nested("fixture_msgs/msg/Header"),
        child_frame_id=STRING,
        pose=nested("fixture_msgs/msg/PoseWithCovariance"),
        twist=nested("fixture_msgs/msg/TwistWithCovariance"),
    )
)

DiagnosticStatus = build(
    describe(
        "fixture_msgs/msg/DiagnosticStatus",
        level=INT8,
        name=STRING,
        message=STRING,
        hardware_id=STRING,
        values=(SequenceType(element=NestedType(type_name="fixture_msgs/msg/Pair")), "sequence<Pair>"),
    )
)

DiagnosticArray = build(
    describe(
        "fixture_msgs/msg/DiagnosticArray",
        header=nested("fixture_msgs/msg/Header"),
        status=(
            SequenceType(element=NestedType(type_name="fixture_msgs/msg/DiagnosticStatus")),
            "sequence<DiagnosticStatus>",
        ),
    )
)

NavigateToPoseGoal = build(
    describe(
        "fixture_msgs/action/NavigateToPose_Goal",
        pose=nested("fixture_msgs/msg/PoseStamped"),
        behavior_tree=STRING,
    )
)
NavigateToPoseFeedback = build(
    describe(
        "fixture_msgs/action/NavigateToPose_Feedback",
        current_pose=nested("fixture_msgs/msg/PoseStamped"),
        distance_remaining=FLOAT32,
    )
)
NavigateToPoseResult = build(describe("fixture_msgs/action/NavigateToPose_Result", error_code=UINT16))

NavigateToPose = ActionTypes(
    type_name="fixture_msgs/action/NavigateToPose",
    goal=NavigateToPoseGoal,
    feedback=NavigateToPoseFeedback,
    result=NavigateToPoseResult,
)

MESSAGE_TYPES: dict[str, Any] = {
    **fixture_lifecycle.MESSAGE_TYPES,
    "nav_msgs/msg/Odometry": Odometry,
    "diagnostic_msgs/msg/DiagnosticArray": DiagnosticArray,
    "geometry_msgs/msg/Twist": Twist,
    "tf2_msgs/msg/TFMessage": fixture_messages.TFMessage,
}
"""What a fake Runtime resolves a topic type to."""

ACTION_TYPES: dict[str, ActionTypes] = {"nav2_msgs/action/NavigateToPose": NavigateToPose}
"""The same, for the navigation action."""

SERVICE_TYPES: dict[str, Any] = {
    **fixture_lifecycle.SERVICE_TYPES,
    "nav2_msgs/srv/ClearEntireCostmap": fixture_messages.Clear,
}
"""The same, for the costmap service and the lifecycle services the Nav2 manager is driven through."""
