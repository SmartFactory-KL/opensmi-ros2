# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Turning ROS2 names into browse names, and checking that siblings stay apart."""

from __future__ import annotations

import pytest

from opensmi.ros2.server.browse_names import browse_name, browse_names
from opensmi.ros2.server.errors import BindingNameError, DuplicateVariableNameError


@pytest.mark.parametrize(
    ("ros2_name", "expected"),
    [
        ("child_frame_id", "ChildFrameId"),
        ("max_vel_x", "MaxVelX"),
        ("pose2d", "Pose2d"),
        ("FollowPath", "FollowPath"),
        ("x", "X"),
    ],
)
def test_a_ros2_name_becomes_pascal_case(ros2_name: str, expected: str) -> None:
    assert browse_name(ros2_name) == expected


def test_a_name_with_a_character_outside_the_rule_is_refused_with_a_suggestion() -> None:
    with pytest.raises(BindingNameError, match=r"'/tf'.*try 'Tf'"):
        browse_name("/tf")


def test_an_empty_part_is_refused_with_a_suggestion() -> None:
    with pytest.raises(BindingNameError, match=r"'max__vel'.*try 'MaxVel'"):
        browse_name("max__vel")


def test_an_empty_name_is_refused_without_a_suggestion() -> None:
    with pytest.raises(BindingNameError) as caught:
        browse_name("")
    assert "try" not in str(caught.value)


def test_a_name_starting_with_a_digit_is_refused() -> None:
    with pytest.raises(BindingNameError, match="'2d_pose'"):
        browse_name("2d_pose")


def test_a_reserved_name_is_refused() -> None:
    with pytest.raises(BindingNameError, match="'none'"):
        browse_name("none")


def test_siblings_map_each_ros2_name_to_its_browse_name() -> None:
    assert browse_names(["child_frame_id", "twist"]) == {"child_frame_id": "ChildFrameId", "twist": "Twist"}


def test_two_siblings_converting_to_one_browse_name_are_refused_naming_both() -> None:
    with pytest.raises(DuplicateVariableNameError, match=r"'x_1'.*'x1'.*'X1'"):
        browse_names(["x_1", "x1"])


def test_a_sibling_that_cannot_be_converted_is_refused() -> None:
    with pytest.raises(BindingNameError, match="'/tf'"):
        browse_names(["depth", "/tf"])


def test_the_collision_error_points_to_the_rename_option() -> None:
    with pytest.raises(DuplicateVariableNameError, match=r"'x_1' and 'x1'.*rename option"):
        browse_names(["x_1", "x1"])
