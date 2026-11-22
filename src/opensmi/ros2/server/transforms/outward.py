# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Outward publishing: declared frames a client may write, and their broadcast."""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, Any

from opensmi.server.spatial_object import Orientation, Pose, Position

from opensmi.ros2.runtime.errors import PublishFailedError
from opensmi.ros2.runtime.tf import FrameLink, Transform
from opensmi.ros2.server.bindings import PublishedFrame
from opensmi.ros2.server.config import ROS2Config
from opensmi.ros2.server.events import Severity
from opensmi.ros2.server.keyed import member_browse_name
from opensmi.ros2.server.mutation import _write_display_name
from opensmi.ros2.server.publisher import PUBLISH_FAILED
from opensmi.ros2.server.spatial import (
    ATTACH_POINTS,
)
from opensmi.ros2.server.transforms.conversions import DEGREES, METRES, _as_transform
from opensmi.ros2.server.transforms.filing import _FrameFiling
from opensmi.ros2.server.transforms.records import (
    _Observed,
    _Owner,
    _Published,
)

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable, Mapping, Sequence


class _OutwardPublishing(_FrameFiling):
    """Puts the poses a client writes into declared Locations on the TF topics."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Start out with nothing refused, and pass every argument to opensmi."""
        super().__init__(*args, **kwargs)
        self._broadcast_refused = False
        self._broadcast_lock = asyncio.Lock()

    async def _open_published(self, owner: _Owner, declared: PublishedFrame) -> None:
        """Create one declared outward frame's Location, writable, in ``AttachPoints``.

        It is created at init and outside the chokepoint because it is declared. Access level and
        write check are set here, since a node cannot be re-shaped later. A client needs the
        Machine's Skill level (:attr:`~opensmi.ros2.server.config.ROS2Config.skill_access_level`),
        as for a publishing Skill, and the Lock. ``Base`` stays read-only. It starts at the World
        Frame and is re-resolved every tick, because the declared base frame may not be published
        yet.
        """
        browse = member_browse_name((declared.frame,))
        world = self._require_world()
        location = _Observed(
            name=browse,
            initial_pose=Pose(
                Position(*declared.position, unit=METRES),
                Orientation(*declared.orientation, unit=DEGREES),
            ),
            initial_base=world,
            # Opt-in writable pose; writes reach this module once stored.
            minimum_access_level=(getattr(self, "ros2_config", None) or ROS2Config()).skill_access_level,
            written=self._pose_written(owner, declared.frame),
        )
        await self._file(owner, ATTACH_POINTS, location)
        owner.folders.add(ATTACH_POINTS)
        await _write_display_name(location.ua_node, declared.frame)
        owner.claimed[browse] = declared.frame
        owner.published[declared.frame] = _Published(declared=declared, location=location, base=world)
        self._transform_index[declared.frame] = location.ua_node.nodeid

    def _pose_written(self, owner: _Owner, frame: str) -> Callable[[Pose], Awaitable[None]]:
        """Return the callback one outward Location is created with. opensmi awaits it with the stored pose."""

        async def written(pose: Pose) -> None:
            await self._broadcast_written(owner, frame, pose)

        return written

    async def _broadcast_written(self, owner: _Owner, frame: str, pose: Pose) -> None:
        """Put the pose a client wrote on the wire.

        The write is never refused for the broadcast: it is already stored, and a failed broadcast
        is a ``PUBLISH_FAILED`` BindingEvent that the next tick retries.
        """
        held = owner.published.get(frame)
        if held is None:
            return
        await self._broadcast([(held, _as_transform(pose.position, pose.orientation))])

    async def _rebase_published(self, owner: _Owner, *, parents: Mapping[str, str]) -> None:
        """Re-resolve every outward Location's ``Base``, the only part of it this module writes."""
        for held in owner.published.values():
            base = self._base_node(held.declared.frame, parents=parents)
            if base == held.base:
                continue
            await held.location.write_base(base)
            held.base = base

    async def _broadcast_published(self) -> None:
        """Re-assert what every outward Location holds, in one message per topic.

        A moving frame goes out every tick, because a pose published once ages out of listeners'
        buffers. A static frame goes out only when it differs from what last went out, which also
        retries a pose the middleware refused. That is why
        :meth:`~opensmi.ros2.runtime.tf.TFFacet.broadcast` owns its publisher instead of using
        ``tf2_ros.StaticTransformBroadcaster``. The pose is read back off the Location because it is
        the client's.
        """
        staged: list[tuple[_Published, Transform]] = []
        for owner in self._transform_objects:
            for held in owner.published.values():
                pose = await held.location.read_pose()
                transform = _as_transform(pose.position, pose.orientation)
                if held.declared.static and transform == held.sent:
                    continue
                staged.append((held, transform))
        await self._broadcast(staged)

    async def _broadcast(self, staged: Sequence[tuple[_Published, Transform]]) -> None:
        """Put outward poses on the wire, and record what went out only once it has.

        A refusal is reported as an event and never raised. An outward pose carries no status code,
        because the quiet conditions of :mod:`~opensmi.ros2.server.status` are facts about a robot
        publishing to us. Serialised, because a client's write and the tick may both broadcast.
        """
        if not staged:
            return
        sending = [
            FrameLink(
                child=held.declared.frame,
                parent=held.declared.base,
                transform=transform,
                static=held.declared.static,
            )
            for held, transform in staged
        ]
        async with self._broadcast_lock:
            try:
                await self._ros2_runtime_for_tf().tf.broadcast(sending)
            except PublishFailedError as error:
                await self._report_broadcast_refused(sending, error)
                return
            # Recorded after the wire. Recorded before, a refused static frame would compare equal on every
            # later tick and never be retried.
            for held, transform in staged:
                held.sent = transform
            self._broadcast_refused = False

    async def _report_broadcast_refused(self, sending: Sequence[FrameLink], error: Exception) -> None:
        """Say, as a BindingEvent, that the poses a client wrote did not reach ROS2.

        Once per spell of failure; the streak clears on the first broadcast that goes out. It is
        reported on the Machine's own TF Binding, because one broadcast carries every object's
        frames. It uses :data:`~opensmi.ros2.server.publisher.PUBLISH_FAILED`, the code the
        publishing Skill reports for the same fact.
        """
        binding = self.ros2_tf_binding
        if binding is None:  # pragma: no cover -- nothing broadcasts without a declaration
            return
        if self._broadcast_refused:
            return
        # After both guards, so the latch is only set by a failure that was reported.
        self._broadcast_refused = True
        frames = sorted(link.child for link in sending)
        self.logger.error("A transform broadcast did not happen", binding=binding.name, frames=frames)
        await self.emit_binding_event(
            binding=binding.name,
            path=self.ros2_transform_settings.path,
            code=PUBLISH_FAILED,
            severity=Severity.ERROR,
            text=(
                f"the poses written into {', '.join(frames)} did not reach ROS2: {error}; the Locations hold "
                f"what a client wrote and nothing on the robot has been told about it"
            ),
        )
