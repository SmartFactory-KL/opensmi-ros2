# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Gazebo (headless) with the HC10 and the 2F-85 on their table, the controllers, and the bridged topics.

Gazebo runs as a server only. A GUI can join it from outside (``gz sim -g`` on the same network,
see the example's compose.yaml). The bridge carries the simulation clock and the cube's pose, which
the example serves as ground truth so a client can see where the cube is.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Return the simulation: Gazebo, the robot in it, its controllers and the bridge."""
    share = FindPackageShare("hc10_robotiq_description")
    description = Command(["xacro ", PathJoinSubstitution([share, "urdf", "hc10_robotiq.urdf.xacro"])])
    world = PathJoinSubstitution([share, "worlds", "pick_place.sdf"])

    # Gazebo resolves the meshes' package:// URIs against these folders.
    meshes = [
        os.path.dirname(get_package_share_directory(each)) for each in ("motoman_hc10_support", "robotiq_description")
    ]

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]),
        launch_arguments={"gz_args": ["-r -s -v 2 ", world], "on_exit_shutdown": "true"}.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(description, value_type=str), "use_sim_time": True}],
        output="screen",
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "hc10_robotiq", "-allow_renaming", "false"],
        output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/cube/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
        ],
        remappings=[("/model/cube/pose", "/cube/pose")],
        output="screen",
    )
    controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            *("joint_state_broadcaster", "arm_controller", "gripper_controller"),
            *("--controller-manager", "/controller_manager", "--controller-manager-timeout", "120"),
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    return LaunchDescription(
        [
            *(AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", each) for each in meshes),
            gazebo,
            state_publisher,
            bridge,
            spawn,
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[controllers])),
        ]
    )
