# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The bookkeeping a rendered frame needs: its Location, its owner and its last reading."""

from __future__ import annotations

from collections.abc import AsyncGenerator
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, NamedTuple

from asyncua import ua
from opensmi.core.lifecycle_mixin import lifecycle
from opensmi.server.spatial_object import CartesianFrame, Orientation, Pose, Position, PositionFrame, SpatialObject

from opensmi.ros2.runtime.tf import FrameLink, Transform
from opensmi.ros2.server.bindings import PublishedFrame, TFBinding
from opensmi.ros2.server.status import transform_status
from opensmi.ros2.server.transforms.conversions import DEGREES, METRES

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable, Mapping

    from asyncua.common.node import Node


class _Observed(CartesianFrame):
    """A Location that reports a client's write and ignores this module's own bad-status writes.

    A write is announced to ``on_value_change`` once opensmi has stored it, which is how a client's
    pose reaches ROS2. Without a ``written`` callback the node is made read-only after creation,
    because opensmi makes every Cartesian frame writable. This module writes bad statuses on
    purpose, and OPC UA nulls such a value, so a null value is passed over silently instead of being
    logged as invalid.
    """

    def __init__(
        self,
        *,
        written: Callable[[Pose], Awaitable[None]] | None = None,
        minimum_access_level: int | None = None,
        **kwargs: Any,
    ) -> None:
        """Remember who to tell about a client's write, if anybody."""
        super().__init__(minimum_access_level=minimum_access_level, **kwargs)
        self._written = written

    @lifecycle(after=CartesianFrame.lifecycle_frame)
    async def lifecycle_observed(self) -> AsyncGenerator[None]:
        """Take the write bit off a Location nobody may write."""
        if self._written is None:
            await self.ua_node.set_writable(False)
        yield
        # no shutdown

    async def on_value_change(self, handle: int, dv: ua.DataValue) -> None:
        """Keep the children in step, then tell the owner what a client wrote."""
        value = dv.Value.Value if dv is not None and dv.Value is not None else None
        if not isinstance(value, ua.ThreeDFrame):
            return
        await super().on_value_change(handle, dv)
        if self._written is not None:
            await self._written(Pose.from_ua(value, length_unit=METRES, angle_unit=DEGREES))


class _Head(_Observed, PositionFrame):
    """The ``PositionFrame`` of a SpatialObject: a Location, read from the tree and so read-only."""


@dataclass(slots=True)
class _Frame:
    """One rendered Location: which frame it is, and what it last read.

    :ivar constant: Whether the constant access bit is set. It is set on the first message that says
        the frame is static, because a frame may first arrive on ``/tf``.
    :ivar status: The status last written beside the pose, or ``None``. Compared every tick, because
        a frame that goes quiet and returns unmoved has changed only this.
    :ivar bearers: The nine value-bearing nodes, browsed on the first bad-status write.
    """

    frame: str
    browse: str
    location: CartesianFrame
    folder: str | None
    position: Position | None = None
    orientation: Orientation | None = None
    base: ua.NodeId | None = None
    constant: bool = False
    status: ua.StatusCode | None = None
    bearers: _Bearers | None = None

    @property
    def node(self) -> Node:
        """Return the node this Location is."""
        return self.location.ua_node


class _Bearers(NamedTuple):
    """The nine value-bearing nodes of one Location, named so they cannot be unpacked in the wrong order.

    ``X``, ``Y`` and ``Z`` are unqualified while ``A``, ``B`` and ``C`` sit in the RSL namespace
    (OPC 10000-210's own asymmetry). ``frame`` holds the whole ``3DFrame``.
    """

    frame: Node
    position: Node
    x: Node
    y: Node
    z: Node
    orientation: Node
    a: Node
    b: Node
    c: Node


@dataclass(frozen=True, slots=True, kw_only=True)
class _Quiet:
    """One tick's answer to which frames have gone quiet.

    It is built once per poll so that all frames are judged against one clock reading.

    :ivar present: Whether anything is publishing on the transform topics at all.
    :ivar seen: When each frame was last in a message, on the Runtime's clock.
    :ivar now: The current time on that clock.
    :ivar within: How long a frame may go unpublished and still speak for itself.
    :ivar roots: The roots of the rendered tree, which no message carries.
    """

    present: bool
    seen: Mapping[str, float]
    now: float
    within: float
    roots: frozenset[str]

    def status_of(self, frame: str, link: FrameLink | None) -> ua.StatusCode:
        """Return the status code one frame's Location reads this tick.

        :param frame: The frame being judged.
        :param link: What the frame network holds for it, or ``None``. Only its ``static`` flag is
            used, because quiet is the normal state of a static frame.
        """
        return transform_status(
            present=self.present,
            static=link is not None and link.static,
            rooted=frame in self.roots,
            seen=self.seen.get(frame),
            now=self.now,
            within=self.within,
        )


@dataclass(slots=True)
class _Published:
    """One declared outward frame: the writable Location and what was last sent.

    Its pose is whatever the client last wrote, so none is cached here.

    :ivar declared: The declaration this Location exists for.
    :ivar location: The writable Location.
    :ivar base: What the ``Base`` was last written as.
    :ivar sent: The transform last put on the wire. A static frame is published only when it
        differs.
    """

    declared: PublishedFrame
    location: CartesianFrame
    base: ua.NodeId | None = None
    sent: Transform | None = None


@dataclass(slots=True)
class _Owner:
    """One machinery item's SpatialObject, live: what it declared and what it has rendered.

    :ivar folders: Frame folders created so far, so each is counted and announced once.
    """

    item: Any
    binding: TFBinding
    spatial: SpatialObject
    frames: dict[str, _Frame] = field(default_factory=dict)
    folders: set[str] = field(default_factory=set)
    claimed: dict[str, str] = field(default_factory=dict)
    refused: set[str] = field(default_factory=set)
    alternatives: dict[tuple[str, str], _Frame] = field(default_factory=dict)
    unresolved: set[tuple[str, str]] = field(default_factory=set)
    published: dict[str, _Published] = field(default_factory=dict)
    capped: bool = False
    wanted: set[str] = field(default_factory=set)

    @property
    def node(self) -> Node:
        """Return the node this SpatialObject is."""
        return self.spatial.ua_node
