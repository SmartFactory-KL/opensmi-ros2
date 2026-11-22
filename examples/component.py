# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""One Component exercising every Binding kind this package ships, once.

A Nav2-style mobile platform: odometry read into Monitoring, ``/diagnostics`` keyed by hardware
id, a navigation action, a costmap-clearing service, a driven ``/cmd_vel`` Skill, two tuned
controller parameters, a Nav2-managed lifecycle and the TF frame the platform is. The whole
Component is the ``bindings`` tuple below. Point the Bindings at your own robot's topics,
services and actions.
"""

from __future__ import annotations

from asyncua import ua
from opensmi.server import BaseComponent

from opensmi.ros2.runtime import ParameterType
from opensmi.ros2.server import (
    ActionBinding,
    KeyedBinding,
    LifecycleBinding,
    MethodBinding,
    MonitoringBinding,
    PublisherBinding,
    ROS2ComponentMixin,
    ROS2ParameterBinding,
    Severity,
    TFBinding,
)

ODOMETRY = MonitoringBinding(
    name="Odom",
    path="/odom",
    message_type="nav_msgs/msg/Odometry",
    exclude=("pose/covariance", "twist/covariance"),
    rename={"child_frame_id": "BaseFrame"},
    required=True,
)
"""Read continuously; required, so the platform's StartupSkill waits for a publisher on it."""

DIAGNOSTICS = KeyedBinding(
    name="Diagnostics",
    path="/diagnostics",
    message_type="diagnostic_msgs/msg/DiagnosticArray",
    repeated_field="status",
    key=("hardware_id", "name"),
    state_field="level",
    states={0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"},
    severities={0: Severity.INFO, 1: Severity.WARNING, 2: Severity.ERROR, 3: Severity.STALE},
)
"""One member Object per diagnostic id, plus ``Diagnostics.WorstLevel``."""


NAVIGATE_TO_POSE = ActionBinding(
    name="NavigateToPose",
    path="/navigate_to_pose",
    action_type="nav2_msgs/action/NavigateToPose",
)
"""A finite Skill: the goal is its Parameters, the feedback its Monitoring, the result its FinalResultData."""


CLEAR_GLOBAL_COSTMAP = MethodBinding(
    name="ClearGlobalCostmap",
    path="/global_costmap/clear_entirely_global_costmap",
    service_type="nav2_msgs/srv/ClearEntireCostmap",
)
"""A Method: the request is its Parameters, the response its FinalResultData."""


DRIVE = PublisherBinding(
    name="Drive",
    path="/cmd_vel",
    message_type="geometry_msgs/msg/Twist",
    continuous=True,
    rate=20.0,
    stale_after=0.5,
    halt_message={"linear": {"x": 0.0}, "angular": {"z": 0.0}},
)
"""A streaming Skill: a client writes the twist and ``Start``s it, and it republishes at 20 Hz
until ``Halt``, until the Lock it was started under ends, or until half a second passes without a
write, and ends with a zero twist."""

CONTROLLER_PARAMETERS = ROS2ParameterBinding(
    name="Controller",
    path="/controller_server",
    parameters={
        "FollowPath.max_vel_x": ParameterType.DOUBLE,
        "use_sim_time": ParameterType.BOOL,
    },
    read_only=("use_sim_time",),
)
"""One ROS2 parameter service, two of its parameters: readable, subscribable and writable unless ``read_only``."""


NAVIGATION_LIFECYCLE = LifecycleBinding(
    name="Navigation",
    path="/lifecycle_manager_navigation",
    nodes=("/planner_server", "/controller_server"),
    driver="nav2",
)
"""Drives the platform's StartupSkill through the Nav2 lifecycle manager; ``Running`` means both
managed nodes are ``active``."""

PLATFORM_FRAMES = TFBinding(
    name="PlatformFrames",
    head="base_link",
    alternatives=(("map", "base_link"),),
)
"""``base_link`` is the frame this platform is, and ``map`` -> ``base_link`` is a composed pose a client reads."""


class Platform(ROS2ComponentMixin, BaseComponent):
    """A mobile platform: the Bindings above, and nothing overridden."""

    bindings = (
        ODOMETRY,
        DIAGNOSTICS,
        NAVIGATE_TO_POSE,
        CLEAR_GLOBAL_COSTMAP,
        DRIVE,
        CONTROLLER_PARAMETERS,
        NAVIGATION_LIFECYCLE,
        PLATFORM_FRAMES,
    )

    async def _write_identification(self) -> None:
        """Fill the identification block opensmi asks of every Component."""
        await self.identification.SerialNumber.write("example-platform-0001")
        await self.identification.Manufacturer.write(ua.LocalizedText("Example Manufacturer", "en-US"))
