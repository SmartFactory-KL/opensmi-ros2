# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The whole cell: the Gazebo simulation of hc10_robotiq_description, and MoveIt's move_group on top.

move_group plans with Pilz's industrial motion planner by default (PTP and LIN, deterministic) and
has OMPL as a second pipeline. It executes on the arm's trajectory controller.
"""

from __future__ import annotations

from launch.actions import IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    """Return the simulation and move_group."""
    description = FindPackageShare("hc10_robotiq_description").find("hc10_robotiq_description")
    config = (
        MoveItConfigsBuilder("hc10_robotiq", package_name="hc10_robotiq_moveit_config")
        .robot_description(file_path=f"{description}/urdf/hc10_robotiq.urdf.xacro")
        .robot_description_semantic(file_path="config/hc10_robotiq.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["pilz_industrial_motion_planner", "ompl"],
            default_planning_pipeline="pilz_industrial_motion_planner",
        )
        .to_moveit_configs()
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[config.to_dict(), {"use_sim_time": True, "publish_robot_description_semantic": True}],
        output="screen",
    )
    simulation = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("hc10_robotiq_description"), "launch", "sim.launch.py"])
    )
    return LaunchDescription([simulation, move_group])
