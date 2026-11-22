# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Inward mirroring: the poll that renders the frame network as Locations."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncGenerator
from typing import TYPE_CHECKING, Any, Final

from asyncua import ua
from opensmi.core.lifecycle_mixin import lifecycle
from opensmi.server.base_machinery_item import BaseMachineryItem
from opensmi.server.spatial_object import Orientation, Pose, Position

from opensmi.ros2.runtime.tf import FrameLink
from opensmi.ros2.server.errors import (
    FrameDirectionError,
)
from opensmi.ros2.server.events import Severity
from opensmi.ros2.server.mutation import ended
from opensmi.ros2.server.spatial import (
    angles,
    attach_points,
    excluded,
    one_direction_per_frame,
    owners,
    parents_of,
    tree_roots,
)
from opensmi.ros2.server.status import GOOD, status_value
from opensmi.ros2.server.transforms.conversions import _LEVEL, _ZERO, DEGREES, METRES
from opensmi.ros2.server.transforms.outward import _OutwardPublishing
from opensmi.ros2.server.transforms.records import (
    _Bearers,
    _Frame,
    _Owner,
    _Quiet,
)

if TYPE_CHECKING:
    from collections.abc import Iterator, Mapping, Sequence

    from asyncua.common.node import Node


SEPARATE_FRAME_TREES: Final = "SEPARATE_FRAME_TREES"
"""The BindingEvent code more than one root in the rendered frame network carries.

A well-formed TF tree has one root, which OPC 10000-210 calls the World Frame. With several,
unrelated coordinate systems are presented as one.
"""


class ROS2TransformMirrorMixin(_OutwardPublishing):
    """Keeps every SpatialObject on one Machine in step with the TF tree, on a timer.

    Machine only: a frame belongs to the nearest declared head above it, and the heads are spread
    over the Machine and its Components. The period is declared on the Machine for the same reason
    (:attr:`~opensmi.ros2.server.config.TransformConfig.interval`). How many Locations one TF
    Binding may gain is :attr:`~opensmi.ros2.server.mutation.ROS2MutationMixin.mutation_node_cap` on
    the declaring item; a robot with more frames says so once.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Start out with nothing rendered, and pass every argument to opensmi."""
        super().__init__(*args, **kwargs)
        self._transform_interval = 0.0
        self._transform_task: asyncio.Task[None] | None = None
        self._transform_polls = 0
        self._transform_outward: frozenset[str] = frozenset()
        self._transform_roots: tuple[str, ...] = ()
        self._transform_quiet_after = 0.0

    @property
    def ros2_transform_polls(self) -> int:
        """Return how often the frame network has been read, which a Machine with no TF leaves at 0."""
        return self._transform_polls

    @property
    def ros2_frame_nodes(self) -> Mapping[str, str]:
        """Return the frame-to-node index: every rendered frame and the node holding it, as NodeId text."""
        return {frame: node.to_string() for frame, node in self._transform_index.items()}

    @lifecycle(after=BaseMachineryItem.lifecycle_machinery_item)
    async def lifecycle_ros2_transform_mirror(self) -> AsyncGenerator[None]:
        """Start the poll once the whole Machine is built, and stop it before teardown.

        It starts after the Machine's ``_init`` because the Components' objects must exist first,
        and stops before ``_shutdown`` so no tick writes into a Component that is going away.
        """
        await self._init_transform_mirror()
        yield
        await self._stop_transform_mirror()

    async def _init_transform_mirror(self) -> None:
        """Collect every SpatialObject on this Machine and start the poll, if there are any.

        Nothing is created for frames the robot publishes and nothing is asked of it: the first tick
        is one interval away. Declared outward frames are created here, so a client may write them
        before the robot is on. The cross-object direction check runs first.

        :raises FrameDirectionError: When two declarations on this Machine claim one frame.
        """
        objects = self._declared_spatial_objects()
        if not objects:
            return
        listed: Any = getattr(self, "spatial_object_list", None)
        self._transform_world = listed.world_frame.ua_node.nodeid
        self._transform_objects = objects
        self._require_one_direction_per_frame(objects)
        self._transform_outward = frozenset(frame for owner in objects for frame in owner.binding.published_frames)
        for owner in objects:
            head = owner.binding.head
            self._transform_index[head] = owner.spatial.position_frame.ua_node.nodeid
            # The head's Location is the mandatory PositionFrame, which opensmi built.
            owner.frames[head] = _Frame(
                frame=head,
                browse=head,
                location=owner.spatial.position_frame,
                folder=None,
            )
        for owner in objects:
            for declared in owner.binding.publishes:
                await self._open_published(owner, declared)
        # One poll, one period: the Machine's.
        self._transform_interval = self.ros2_transform_settings.interval
        self._transform_quiet_after = self.ros2_transform_settings.quiet_after
        self._transform_task = asyncio.create_task(self._poll_transforms(), name="ros2-tf-poll")

    def _require_one_direction_per_frame(self, objects: Sequence[_Owner]) -> None:
        """Refuse a Machine on which two declarations claim one frame.

        This is the cross-object half of the inward/outward rule, which no single declaration can
        see. It runs before anything is created.

        :raises FrameDirectionError: When one frame is claimed twice.
        """
        clash = one_direction_per_frame(
            heads={owner.binding.name: owner.binding.head for owner in objects},
            published={owner.binding.name: owner.binding.published_frames for owner in objects},
        )
        if clash is None:
            return
        if clash.inward:
            against = (
                f"the TF Binding {clash.against!r} declares it as its head frame, which is read "
                f"*inward* from the tree -- so the pose a client wrote would be written over by "
                f"the robot's answer on the next tick, for ever"
            )
        else:
            against = (
                f"the TF Binding {clash.against!r} publishes it too -- and a frame has one parent "
                f"and one publisher, so two declarations asserting it would put two transforms on "
                f"the topic every tick and leave tf2 keeping whichever arrived last"
            )
        message = (
            f"the TF Binding {clash.outward!r} publishes the frame {clash.frame!r} and {against}. "
            f"A frame is inward or outward and never both, which is what makes an echo loop "
            f"impossible rather than something to suppress"
        )
        raise FrameDirectionError(message)

    async def _stop_transform_mirror(self) -> None:
        """Stop the poll. Idempotent, and it leaves the watches to the items that opened them."""
        await ended(self._transform_task)
        self._transform_task = None
        self._transform_objects = ()

    def _declared_spatial_objects(self) -> tuple[_Owner, ...]:
        """Return this Machine's SpatialObject and every Component's, in browse order."""
        found: list[_Owner] = []
        for item in _machinery_items(self):
            binding = getattr(item, "ros2_tf_binding", None)
            spatial = getattr(item, "ros2_spatial_object", None)
            if binding is None or spatial is None:
                continue
            found.append(_Owner(item=item, binding=binding, spatial=spatial))
        return tuple(found)

    async def _poll_transforms(self) -> None:
        """Read the frame network on the interval and render it, until cancelled."""
        while True:
            await asyncio.sleep(self._transform_interval)
            try:
                await self.refresh_transforms()
            except Exception:
                # A runtime condition is never fatal; a tree that failed once is likely fine on the
                # next tick.
                self.logger.exception("Could not render the TF tree; the poll continues")

    async def refresh_transforms(self) -> None:
        """Read the frame network once and bring every SpatialObject into step with it.

        Public so that tests can drive the inward path without waiting for the timer.
        """
        self._transform_polls += 1
        # A snapshot, because the facet's record is live. Outward frames are dropped from it: our own
        # broadcast comes back through our subscription and cannot be told from the robot's, so an outward
        # pose is only ever read from its Location.
        frames = {
            frame: link
            for frame, link in self._ros2_runtime_for_tf().tf.frames.items()
            if frame not in self._transform_outward
        }
        # Declared outward links join the tree, so the partition, attach points, folders and Bases treat them
        # like published ones.
        parents = dict(parents_of(frames))
        for owner in self._transform_objects:
            for declared in owner.binding.publishes:
                parents[declared.frame] = declared.base
        roots = tree_roots(parents)
        await self._report_separate_trees(roots)
        facet = self._ros2_runtime_for_tf().tf
        # One reading of the clock and of the presence per tick, so no two Locations disagree.
        quiet = _Quiet(
            present=facet.satisfied,
            seen=facet.seen,
            now=facet.now,
            within=self._transform_quiet_after,
            roots=frozenset(roots),
        )
        heads = {owner.binding.head for owner in self._transform_objects}
        held = owners(parents=parents, heads=heads)
        attach = attach_points(parents=parents, heads=heads)
        for owner in self._transform_objects:
            owner.wanted = {
                frame
                for frame, head in held.items()
                if head == owner.binding.head and not excluded(frame, owner.binding.exclude)
            }
            # Always: the head is the mandatory PositionFrame.
            owner.wanted.add(owner.binding.head)
            await self._create_new_locations(owner, parents=parents, attach=attach)
        for owner in self._transform_objects:
            await self._write_locations(owner, frames=frames, parents=parents, quiet=quiet)
            await self._write_alternatives(owner, frames=frames, quiet=quiet)
            await self._rebase_published(owner, parents=parents)
        await self._broadcast_published()

    async def _write_locations(
        self,
        owner: _Owner,
        *,
        frames: Mapping[str, FrameLink],
        parents: Mapping[str, str],
        quiet: _Quiet,
    ) -> None:
        """Write every Location of one object: its pose, and the Base its TF parent implies.

        This runs after all new Locations exist, so a Base never points at the World Frame for one
        tick and at a Location the next.
        """
        for frame, held in sorted(owner.frames.items()):
            link = frames.get(frame)
            await self._write_one(
                held,
                link,
                base=self._base_node(frame, parents=parents),
                status=quiet.status_of(frame, link),
            )

    async def _write_one(
        self,
        held: _Frame,
        link: FrameLink | None,
        *,
        base: ua.NodeId,
        status: ua.StatusCode,
    ) -> None:
        """Write one Location from the transform its TF parent published, and say how it is.

        A frame nothing has published yet still gets its Base and status.
        """
        pose = None if link is None else (link.transform.translation, link.transform.rotation)
        await self._write_pose(held, pose, base=base, status=status)
        if link is not None and link.static and not held.constant:
            # Staticness is a fact about the message, and a head frame's Location exists before any message.
            await self._mark_constant(held.node)
            held.constant = True

    async def _write_pose(
        self,
        held: _Frame,
        pose: tuple[tuple[float, float, float], tuple[float, float, float, float]] | None,
        *,
        base: ua.NodeId,
        status: ua.StatusCode,
    ) -> None:
        """Write one Location's pose, status and Base, and only what changed.

        Writing an unchanged value still wakes every MonitoredItem on the node. The status counts as
        a change: a bad status nulls its value (OPC UA Part 4 7.7.1), so a frame that returns at the
        pose it left must still be written. Used for both a frame's Location and an alternative's,
        which differ only in where the pose came from.
        """
        if pose is not None:
            translation, turned = pose
            position = Position(*translation, unit=METRES)
            orientation = Orientation(*angles(turned), unit=DEGREES)
        else:
            # No pose yet. The status still needs a value to go with (see status.py), so keep what the node
            # reads.
            position = held.position or _ZERO
            orientation = held.orientation or _LEVEL
        if position != held.position or orientation != held.orientation or status != held.status:
            await self._write_reading(held, position, orientation, status)
        if base != held.base:
            await held.location.write_base(base)
            held.base = base

    async def _write_reading(
        self,
        held: _Frame,
        position: Position,
        orientation: Orientation,
        status: ua.StatusCode,
    ) -> None:
        """Put one pose on a Location's nine value-bearing nodes, with its status.

        A Good pose goes through opensmi's ``write_pose``. opensmi cannot write a status code, so a
        bad status is written by hand, always with the value, because a status alone leaves the
        variant null (see :mod:`~opensmi.ros2.server.status`).
        """
        if status == GOOD:
            await held.location.write_pose(Pose(position, orientation))
        else:
            await self._write_faulted(held, position, orientation, status)
        held.position = position
        held.orientation = orientation
        held.status = status

    async def _write_faulted(
        self,
        held: _Frame,
        position: Position,
        orientation: Orientation,
        status: ua.StatusCode,
    ) -> None:
        """Write one bad reading onto the Location, its pose and its six scalars.

        The scalars too, because clients subscribe to ``Position/X``. Not the ``Base``, which stays
        true while the frame is unreachable. The Location is written first; opensmi's child sync
        ignores a null value, so the children keep what is written to them next.
        """
        if held.bearers is None:
            held.bearers = await self._pose_nodes(held.node)
        bearers = held.bearers
        cartesian = ua.ThreeDCartesianCoordinates(
            X=ua.Double(position.x), Y=ua.Double(position.y), Z=ua.Double(position.z)
        )
        turned = ua.ThreeDOrientation(
            A=ua.Double(orientation.a), B=ua.Double(orientation.b), C=ua.Double(orientation.c)
        )
        extension = ua.VariantType.ExtensionObject
        await bearers.frame.write_value(
            status_value(ua.ThreeDFrame(CartesianCoordinates=cartesian, Orientation=turned), extension, status)
        )
        await asyncio.gather(
            bearers.position.write_value(status_value(cartesian, extension, status)),
            bearers.orientation.write_value(status_value(turned, extension, status)),
            *[
                node.write_value(status_value(ua.Double(value), ua.VariantType.Double, status))
                for node, value in (
                    (bearers.x, position.x),
                    (bearers.y, position.y),
                    (bearers.z, position.z),
                    (bearers.a, orientation.a),
                    (bearers.b, orientation.b),
                    (bearers.c, orientation.c),
                )
            ],
        )

    async def _pose_nodes(self, node: Node) -> _Bearers:
        """Browse the nine value-bearing nodes of one Location, once, on its first bad write.

        The result is safe to keep because nodes are never moved.
        """
        ns = self._rsl_namespace
        position = await node.get_child(f"{ns}:Position")
        orientation = await node.get_child(f"{ns}:Orientation")
        # Named, so that a wrong arity is caught by the type checker.
        return _Bearers(
            frame=node,
            position=position,
            x=await position.get_child("X"),
            y=await position.get_child("Y"),
            z=await position.get_child("Z"),
            orientation=orientation,
            a=await orientation.get_child(f"{ns}:A"),
            b=await orientation.get_child(f"{ns}:B"),
            c=await orientation.get_child(f"{ns}:C"),
        )

    async def _write_alternatives(self, owner: _Owner, *, frames: Mapping[str, FrameLink], quiet: _Quiet) -> None:
        """Compose and write every alternative this object declared, creating new ones.

        This is the only place the transform buffer is asked anything. The lookup is synchronous,
        and a chain it cannot complete is skipped. An alternative carries the status of the frame it
        views, so two nodes never disagree about a quiet frame.
        """
        facet = self._ros2_runtime_for_tf().tf
        for base, frame in owner.binding.alternatives:
            composed = facet.lookup(target=base, source=frame)
            if composed is None:
                continue
            node = self._alternative_base(base, frames)
            if node is None:
                await self._report_absent_base(owner, base, frame)
                continue
            held = owner.alternatives.get((base, frame))
            if held is None:
                held = await self._open_alternative(owner, base, frame, node)
                if held is None:
                    continue
            await self._write_pose(
                held,
                (composed.translation, composed.rotation),
                base=node,
                status=quiet.status_of(frame, frames.get(frame)),
            )

    async def _report_separate_trees(self, roots: tuple[str, ...]) -> None:
        """Say, as a BindingEvent, that more than one coordinate system is being presented as one.

        Once per distinct set of roots, and there is no clearing event. Nothing is said while no
        robot is publishing: before that, a Machine that publishes two frames under different bases
        has two roots by construction. Reported on the Machine's own TF Binding.
        """
        if not self._ros2_runtime_for_tf().tf.satisfied:
            # Re-evaluated, so the condition is reported the first time it is seen against a robot that is on.
            self._transform_roots = ()
            return
        if roots == self._transform_roots:
            return
        self._transform_roots = roots
        binding = self.ros2_tf_binding
        if binding is None or len(roots) < 2:
            return
        listed = ", ".join(repr(root) for root in roots)
        self.logger.warning("The frame network has more than one root", roots=roots)
        await self.emit_binding_event(
            binding=binding.name,
            path=self.ros2_transform_settings.path,
            code=SEPARATE_FRAME_TREES,
            severity=Severity.WARNING,
            text=(
                f"the frame network has {len(roots)} roots ({listed}); a well-formed TF tree has "
                f"one, and every Location whose Base walk leaves the rendered tree lands on the "
                f"same WorldFrame -- so these unrelated coordinate systems are presented as one"
            ),
        )


def _machinery_items(machine: Any) -> Iterator[Any]:
    """Yield a Machine and every Component under it, however deeply nested."""
    yield machine
    for component in getattr(machine, "components", ()):
        yield from _machinery_items(component)
