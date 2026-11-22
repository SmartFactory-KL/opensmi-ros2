# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Structure descriptions shared by the structure codec and structure variable tests."""

from __future__ import annotations

from asyncua import ua

from opensmi.ros2.server.structures import structure_class

DURATION = structure_class("builtin_interfaces/msg/Duration", (("sec", ua.Int32, 0), ("nanosec", ua.UInt32, 0)))
"""A message nested in the point, which is a structure of its own."""

POINT = structure_class(
    "trajectory_msgs/msg/JointTrajectoryPoint",
    (
        ("positions", list[ua.Double], 0),
        ("velocities", list[ua.Double], 0),
        ("time_from_start", DURATION, 0),
        ("covariance", list[ua.Double], 3),
        ("raw", ua.ByteString, 0),
        ("valid", ua.Boolean, 0),
    ),
)
"""A point: sequences, a fixed-size array, a ByteString, a flag, and a nested duration."""

TRAJECTORY = structure_class(
    "trajectory_msgs/msg/JointTrajectory",
    (("joint_names", list[ua.String], 0), ("points", list[POINT], 0)),
)
"""A structure that nests an array of the point, which is encoded inline."""

CONTROLLER = structure_class(
    "controller_manager_msgs/msg/ControllerState",
    (("name", ua.String, 0), ("claimed_interfaces", list[ua.String], 0)),
)
"""Another structure, which a variable of points must refuse."""


def point(seed: float = 1.0, *, positions: int = 2) -> dict[str, object]:
    """Return the values of one point by flat name, filled so that one point differs from another."""
    return {
        "positions": [seed + each for each in range(positions)],
        "velocities": [],
        "time_from_start.sec": int(seed),
        "time_from_start.nanosec": 500,
        "covariance": [seed, 0.0, -seed],
        "raw": b"\x00\x01\xff",
        "valid": True,
    }


def node_id(type_name: str, namespace_index: int) -> ua.NodeId:
    """Return the NodeId of the DataType of a ROS2 type."""
    return ua.StringNodeId(Identifier=ua.String(type_name), NamespaceIndex=ua.Int16(namespace_index))


def encoding_id(type_name: str, namespace_index: int) -> ua.NodeId:
    """Return the NodeId of the Default Binary encoding of a ROS2 type's DataType."""
    return node_id(f"{type_name}.DefaultBinary", namespace_index)
