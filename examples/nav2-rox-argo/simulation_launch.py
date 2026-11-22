# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The simulation of the example: the ROX Argo in the example's room, Nav2 and the diagnostics stand-in.

Everything robot-specific is Neobotix's own launch files, included as they are:
``rox_bringup/bringup_sim_launch.py`` starts Gazebo (headless, always; a GUI joins from its own
container) with the Argo drive in ``world.sdf``, and ``rox_navigation/navigation.launch.py`` starts
Neobotix's Nav2 configuration with neo_localization2 on the map ``make_map.py`` drew from that world.
The Nav2 parameters are Neobotix's, with the one change install.sh makes
(``navigation.yaml``: a longer wait for Nav2's servers). Navigation starts a few seconds after the
simulation, once the robot is in the world.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

HERE = os.path.dirname(os.path.abspath(__file__))
NAVIGATION_DELAY = 10.0
"""Seconds between starting the simulation and starting Nav2."""


def generate_launch_description() -> LaunchDescription:
    """Return the simulation, Nav2 and the diagnostics node."""
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("rox_bringup"), "launch", "bringup_sim_launch.py")
        ),
        launch_arguments={
            "rox_type": "argo",
            "world": os.path.join(HERE, "world.sdf"),
            "headless_simulation": "True",
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("rox_navigation"), "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "rox_type": "argo",
            "use_sim_time": "True",
            "map": os.path.join(HERE, "map.yaml"),
            "nav2_params_file": os.path.join(HERE, "navigation.yaml"),
            "use_rviz": "False",
        }.items(),
    )
    diagnostics = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "sim_diagnostics.py"), "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )
    return LaunchDescription([simulation, diagnostics, TimerAction(period=NAVIGATION_DELAY, actions=[navigation])])
