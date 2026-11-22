# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Which frame belongs to which object, which folder it lands in, and what its Base points at.

The partition rules as pure functions of a frame tree (child frame to parent frame) and
a set of declared head frames. There is no server, no ``asyncua`` node and no ROS2 here, so the
rules can be tested without any of them.

Every other frame belongs to the nearest ancestor-or-self that is a declared head. A frame that is
the TF parent of another object's head is an attach point; every other frame is an internal frame. A
Location's ``Base`` is the node holding its TF parent, and a frame removed from the middle of a
chain re-bases its children onto the nearest surviving ancestor. For the outward direction,
:func:`rotation` inverts :func:`angles`, :func:`one_direction_per_frame` checks the inward/outward
rule across a Machine, and :func:`tree_roots` finds the roots of the rendered tree.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from fnmatch import fnmatchcase
from typing import TYPE_CHECKING, Final, TypeVar

if TYPE_CHECKING:
    from collections.abc import Collection, Iterable, Mapping

    from opensmi.ros2.runtime.tf import FrameLink

__all__ = [
    "ALTERNATIVE_FRAMES",
    "ATTACH_POINTS",
    "INTERNAL_FRAMES",
    "FrameClash",
    "angles",
    "attach_points",
    "base_frame_of",
    "base_node_of",
    "excluded",
    "folder_of",
    "nearest_ancestor",
    "one_direction_per_frame",
    "owner_of",
    "owners",
    "parents_of",
    "rotation",
    "tree_roots",
]

_Node = TypeVar("_Node")
"""Whatever a node is to the caller. This module never looks at one."""

ATTACH_POINTS: Final = "AttachPoints"
"""The folder for a position another object can be attached at. Inferred, never declared."""

INTERNAL_FRAMES: Final = "InternalFrames"
"""The folder for every other frame of an object.

Where nearly every TF frame lands. 6.1.1 says an internal frame may not be used as an attach point,
so filing a wheel link under :data:`ATTACH_POINTS` would assert something false.
"""

ALTERNATIVE_FRAMES: Final = "AlternativeFrames"
"""The folder for a pose composed against some base other than the frame's own TF parent."""

_DEGREES: Final = 180.0 / math.pi
"""Radians to degrees. The RSL ``Orientation`` is in degrees."""


def parents_of(frames: Mapping[str, FrameLink]) -> Mapping[str, str]:
    """Return the frame tree as the partition reads it: each child frame mapped to its parent.

    This is the one place the module reads a :class:`~opensmi.ros2.runtime.tf.FrameLink`. Everything
    else takes a plain mapping of names, which tests can write by hand.
    """
    return {child: link.parent for child, link in frames.items()}


def nearest_ancestor(
    frame: str,
    *,
    parents: Mapping[str, str],
    admitted: Collection[str],
    include_self: bool = True,
) -> str | None:
    """Return the closest frame at or above ``frame`` that is admitted, or ``None``.

    Both the partition and the re-basing walk up until something qualifies. A cycle (which tf2
    forbids and a misbehaving publisher can still produce) ends the walk and answers ``None``, so a
    poll never hangs.

    :param frame: Where to start.
    :param parents: Child frame to parent frame.
    :param admitted: The frames that qualify.
    :param include_self: Whether ``frame`` itself may be the answer.
    """
    seen: set[str] = set()
    current: str | None = frame if include_self else parents.get(frame)
    while current is not None and current not in seen:
        if current in admitted:
            return current
        seen.add(current)
        current = parents.get(current)
    return None


def owner_of(frame: str, *, parents: Mapping[str, str], heads: Collection[str]) -> str | None:
    """Return the head frame of the object one frame belongs to, or ``None`` if no object does.

    A head frame owns itself, which makes it the object's ``PositionFrame``. Frames above every
    declared head (a ``map``, say) belong to no object and are rendered by nobody.
    """
    return nearest_ancestor(frame, parents=parents, admitted=heads, include_self=True)


def owners(*, parents: Mapping[str, str], heads: Collection[str]) -> Mapping[str, str]:
    """Return every frame that some declared object owns, mapped to that object's head frame.

    TF has one parent per frame, so no frame can be under two heads.
    """
    found = {frame: owner_of(frame, parents=parents, heads=heads) for frame in parents}
    return {frame: head for frame, head in found.items() if head is not None}


def attach_points(*, parents: Mapping[str, str], heads: Collection[str]) -> frozenset[str]:
    """Return the frames that are attach points because another object's head hangs off them.

    A head whose parent is itself a head attaches to that object's ``PositionFrame``, so nothing is
    filed for it.
    """
    return frozenset(
        parent
        for head in heads
        if (parent := parents.get(head)) is not None and parent not in heads and parent in parents
    )


def folder_of(frame: str, *, head: str, attach: Collection[str]) -> str | None:
    """Return which folder of its object's SpatialObject one frame lands in.

    ``None`` for the object's own head frame, which is the ``PositionFrame`` and in no folder.
    """
    if frame == head:
        return None
    return ATTACH_POINTS if frame in attach else INTERNAL_FRAMES


def base_frame_of(frame: str, *, parents: Mapping[str, str], kept: Collection[str]) -> str | None:
    """Return the frame whose node one Location's ``Base`` points at, or ``None`` for the world.

    The search starts strictly above ``frame``. ``None`` means the walk left the tree without
    meeting a frame that has a node, which is the root of the TF tree, i.e. the list's World Frame.
    A frame excluded from the middle of a chain is not in ``kept``, so its children walk past it.
    """
    return nearest_ancestor(frame, parents=parents, admitted=kept, include_self=False)


def base_node_of(
    frame: str,
    *,
    parents: Mapping[str, str],
    index: Mapping[str, _Node],
    world: _Node,
) -> _Node:
    """Return the node one Location's ``Base`` points at, resolved through the frame-to-node index.

    Walks up until a frame has a node and answers that node, or the World Frame when the walk leaves
    the index. Generic in what a node is: the caller's index holds NodeIds.

    :param frame: The frame whose Base is wanted.
    :param parents: Child frame to parent frame.
    :param index: Every frame that has a node, and the node it has.
    :param world: What to answer when the walk leaves the index.
    """
    found = base_frame_of(frame, parents=parents, kept=index)
    if found is None:
        return world
    return index.get(found, world)


@dataclass(frozen=True, slots=True, kw_only=True)
class FrameClash:
    """One frame that two declarations on a Machine both claim, and how each claims it.

    :ivar frame: The frame in dispute.
    :ivar outward: The name of the Binding that publishes it.
    :ivar against: The name of the Binding that claims it too.
    :ivar inward: Whether ``against`` claims it as its head frame (read from the tree) instead of
        publishing it as well.
    """

    frame: str
    outward: str
    against: str
    inward: bool


def tree_roots(parents: Mapping[str, str]) -> tuple[str, ...]:
    """Return every frame that is somebody's parent and nobody's child, in sorted order.

    Like :func:`~opensmi.ros2.runtime.tf.roots_of` but over the rendered tree, not the facet's raw
    record: what a Machine broadcasts comes back through its own subscription, so the raw record
    would report its own outward declarations as a second coordinate system. A well-formed tree has
    one root, which OPC 10000-210 calls the World Frame.
    """
    return tuple(sorted(set(parents.values()) - set(parents)))


def one_direction_per_frame(
    *,
    heads: Mapping[str, str],
    published: Mapping[str, Collection[str]],
) -> FrameClash | None:
    """Return the first frame two declarations on one Machine both claim, or ``None``.

    This is the cross-object half of the rule that a frame is inward or outward and never both. One
    declaration contradicting itself is caught by :class:`~opensmi.ros2.server.bindings.TFBinding`.
    Two cases are caught here: a frame one object publishes and another declares as its head (the
    two would fight over one Location's pose), and a frame two objects publish (tf2 would keep
    whichever transform arrived last). The result is deterministic: the caller's order over the
    objects, then alphabetical over the frames.

    :param heads: Binding name to the head frame that Binding declares.
    :param published: Binding name to the frames that Binding publishes outward.
    """
    for name, frames in published.items():
        for frame in sorted(frames):
            for other, head in heads.items():
                if head == frame:
                    return FrameClash(frame=frame, outward=name, against=other, inward=True)
            for other, others in published.items():
                if other != name and frame in others:
                    return FrameClash(frame=frame, outward=name, against=other, inward=False)
    return None


def excluded(frame: str, patterns: Iterable[str]) -> bool:
    """Return whether one frame is removed by a declaration's exclusions.

    The patterns are ``fnmatch`` globs over the whole frame id (see
    :mod:`opensmi.ros2.runtime.selection`): ``*_wheel_link`` removes four frames on a
    differential-drive base, and a bare frame id removes one.
    """
    return any(fnmatchcase(frame, pattern) for pattern in patterns)


def angles(rotation: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """Return a ROS2 quaternion as the ``A``, ``B`` and ``C`` angles RSL wants, in degrees.

    The convention is the nodeset's: fixed-axis roll-pitch-yaw (ISO 9787:2013), the same as tf2's
    ``getRPY``. The unit is degrees because opensmi's ``Orientation`` carries ``°``. A pitch at
    either pole is clamped, because floating-point drift can push the ``asin`` argument slightly
    past one.

    :param rotation: The quaternion as ROS2 orders it: x, y, z, w.
    """
    x, y, z, w = rotation
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll * _DEGREES, pitch * _DEGREES, yaw * _DEGREES)


def rotation(degrees: tuple[float, float, float]) -> tuple[float, float, float, float]:
    """Return the ``A``, ``B`` and ``C`` angles RSL holds as the quaternion ROS2 wants.

    The exact inverse of :func:`angles`, used when a pose a client writes goes out on ``/tf``. Three
    angles always name one rotation, so no clamping is needed.

    :param degrees: ``A``, ``B`` and ``C`` in degrees: roll about X, pitch about Y, yaw about Z.
    :return: The quaternion as ROS2 orders it: x, y, z, w.
    """
    half_roll, half_pitch, half_yaw = (angle / _DEGREES / 2.0 for angle in degrees)
    sin_roll, cos_roll = math.sin(half_roll), math.cos(half_roll)
    sin_pitch, cos_pitch = math.sin(half_pitch), math.cos(half_pitch)
    sin_yaw, cos_yaw = math.sin(half_yaw), math.cos(half_yaw)
    return (
        sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw,
        cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw,
        cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw,
        cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw,
    )
