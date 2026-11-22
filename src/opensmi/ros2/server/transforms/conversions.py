# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Units and poses: what a TF transform is in OPC UA terms and back."""

from __future__ import annotations

from typing import Final

from opensmi.core import Unit
from opensmi.server.spatial_object import Orientation, Pose, Position

from opensmi.ros2.runtime.tf import Transform
from opensmi.ros2.server.spatial import (
    rotation,
)

METRES: Final = Unit.METRE
"""What a TF translation is in. opensmi's ``Position`` defaults to millimetres, so every frame is
created with metres explicitly.
"""

DEGREES: Final = Unit.DEGREE_UNIT_OF_ANGLE
"""What a TF orientation is in: the unit RSL's ``Orientation`` asks for."""

_ZERO: Final = Position(0.0, 0.0, 0.0, unit=METRES)
_LEVEL: Final = Orientation(0.0, 0.0, 0.0, unit=DEGREES)
_ORIGIN: Final = Pose(_ZERO, _LEVEL)


def inverted(position: Position, orientation: Orientation) -> tuple[Position, Orientation]:
    """Return the pose that undoes one: if the pair is the transform ``T``, this is ``T`` inverse.

    For an integrator who knows where an object sits in some frame and has to publish the opposite
    link, because a ROS2 frame has one parent (``map -> odom`` from the robot's pose in the map). It
    lives on this side of the seam because it constructs opensmi types.

    :param position: Where the frame sits, in metres.
    :param orientation: How it is turned, as ``A``, ``B`` and ``C`` in degrees.
    :return: The inverse pose, in the same units.
    """
    # Uses opensmi's `Pose.inverse()`, with the orientation brought to degrees first.
    inverse = Pose(position, orientation.to(DEGREES)).inverse()
    return inverse.position, inverse.orientation


def _as_transform(position: Position, orientation: Orientation) -> Transform:
    """Return one OPC UA pose as the transform ROS2 wants.

    Metres stay metres and degrees become a unit quaternion through
    :func:`~opensmi.ros2.server.spatial.rotation`, the inverse of the inward conversion.
    """
    position, orientation = position.to(METRES), orientation.to(DEGREES)
    return Transform(
        translation=(position.x, position.y, position.z),
        rotation=rotation((orientation.a, orientation.b, orientation.c)),
    )
