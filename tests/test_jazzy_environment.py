# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The ROS Jazzy CI job runs on a sourced Jazzy installation, so it cannot pass by collecting nothing."""

from __future__ import annotations

import os
from pathlib import Path

import pytest


@pytest.mark.jazzy
def test_the_job_runs_against_a_sourced_ros2_jazzy_installation() -> None:
    import rclpy

    assert os.environ.get("ROS_DISTRO") == "jazzy"
    # The rclpy imported must be the sourced Jazzy installation's.
    assert rclpy.__file__ is not None
    assert "/opt/ros/jazzy/" in Path(rclpy.__file__).as_posix()
