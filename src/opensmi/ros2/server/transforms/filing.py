# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Frame filing: new Locations, their folders and the events about frames that cannot be filed."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Final

from asyncua import ua
from asyncua.server.address_space import AttributeValue
from asyncua.ua.attribute_ids import AttributeIds
from asyncua.ua.object_ids import ObjectIds
from opensmi.core.nodesets import RSLNodeIds

from opensmi.ros2.runtime.tf import FrameLink
from opensmi.ros2.server.errors import (
    DeclarationError,
)
from opensmi.ros2.server.events import Severity
from opensmi.ros2.server.keyed import member_browse_name
from opensmi.ros2.server.mutation import _write_display_name
from opensmi.ros2.server.spatial import (
    ALTERNATIVE_FRAMES,
    ATTACH_POINTS,
    INTERNAL_FRAMES,
    base_node_of,
    folder_of,
)
from opensmi.ros2.server.transforms.conversions import _ORIGIN
from opensmi.ros2.server.transforms.records import (
    _Frame,
    _Observed,
    _Owner,
)
from opensmi.ros2.server.transforms.spatial import ROS2SpatialMixin

if TYPE_CHECKING:
    from collections.abc import Mapping, Sequence

    from asyncua.common.node import Node


FRAME_COLLISION: Final = "FRAME_COLLISION"
"""The BindingEvent code a frame whose browse name is already taken carries."""

ALTERNATIVE_BASE_ABSENT: Final = "ALTERNATIVE_BASE_ABSENT"
"""The BindingEvent code a declared alternative whose base frame has no node carries."""


_CONSTANT: Final = ua.UInt32(int(ua.AccessLevelExType.CurrentRead) | int(ua.AccessLevelExType.Constant))
"""``AccessLevelEx`` for a value that is read and never changes. The ``Constant`` bit tells clients not
to subscribe.
"""


class _FrameFiling(ROS2SpatialMixin):
    """Creates Locations and files them under a SpatialObject, through the mutation chokepoint."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Start out with nothing rendered, and pass every argument to opensmi."""
        super().__init__(*args, **kwargs)
        self._transform_objects: tuple[_Owner, ...] = ()
        self._transform_index: dict[str, ua.NodeId] = {}
        self._transform_world: ua.NodeId | None = None

    async def _create_new_locations(
        self,
        owner: _Owner,
        *,
        parents: Mapping[str, str],
        attach: frozenset[str],
    ) -> None:
        """Create a Location for every frame this object owns and has not rendered yet.

        One chokepoint call per tick, so a client gets one model change naming all the new frames.
        """
        new = [
            frame
            for frame in sorted(owner.wanted)
            if frame not in owner.frames
            and frame not in owner.refused
            and frame != owner.binding.head
            # A frame has one Location, the first owner's. This excludes a frame re-parented across an
            # ownership boundary: its node stays where it was filed and its Base re-resolves.
            and frame not in self._transform_index
        ]
        if not new or owner.capped:
            return
        planned: list[tuple[str, str, str]] = []
        for frame in new:
            browse = member_browse_name((frame,))
            folder = folder_of(frame, head=owner.binding.head, attach=attach)
            taken = owner.claimed.get(browse)
            if taken is not None or any(browse == other for _, other, _ in planned):
                owner.refused.add(frame)
                await self._report_frame_collision(owner, frame, browse, taken)
                continue
            planned.append((frame, browse, folder or INTERNAL_FRAMES))
        if not planned:
            return
        if not await self._admit(owner, planned, parents=parents):
            owner.capped = True

    async def _admit(
        self,
        owner: _Owner,
        planned: Sequence[tuple[str, str, str]],
        *,
        parents: Mapping[str, str],
    ) -> bool:
        """Put one tick's new Locations through the declaring item's chokepoint.

        The declaring item's cap applies and its Notification gets the BindingEvents. The
        Machine-wide tally counts too.
        """
        folders = {name for _, _, name in planned} - owner.folders

        async def build() -> Sequence[tuple[Node, ua.NodeId | int]]:
            created = [
                await self._open_location(owner, frame, browse, folder, parents=parents)
                for frame, browse, folder in planned
            ]
            return [*await self._opened_folders(owner, folders), *created]

        return await owner.item.mutate_address_space_with(
            binding=owner.binding.name,
            path=self.ros2_transform_settings.path,
            nodes=len(planned) + len(folders),
            build=build,
        )

    async def _opened_folders(self, owner: _Owner, names: set[str]) -> list[tuple[Node, ua.NodeId | int]]:
        """Return the folder nodes opensmi created with a batch, for the model change.

        opensmi creates a folder on first use, so it is found after the batch. Recording it keeps
        later batches from counting or announcing it again.
        """
        opened: list[tuple[Node, ua.NodeId | int]] = []
        for name in sorted(names):
            node = await owner.node.get_child(ua.QualifiedName(name, self._rsl_namespace))
            opened.append((node, ObjectIds.FolderType))
            owner.folders.add(name)
        return opened

    async def _file(self, owner: _Owner, folder: str, location: _Observed) -> None:
        """File one Location in one of the three frame folders of a SpatialObject.

        ``attach_points`` is used only for a real attach point, because 6.1.1 forbids using an
        internal frame as one. The folder's ``add`` is used instead of ``add_internal_frame`` and
        its siblings, because those build a plain ``CartesianFrame`` and a Location here is a
        subclass.
        """
        if folder == ATTACH_POINTS:
            await owner.spatial.attach_points.add(location)
        elif folder == ALTERNATIVE_FRAMES:
            await owner.spatial.alternative_frames.add(location)
        else:
            await owner.spatial.internal_frames.add(location)

    async def _open_location(
        self,
        owner: _Owner,
        frame: str,
        browse: str,
        folder: str,
        *,
        parents: Mapping[str, str],
    ) -> tuple[Node, ua.NodeId | int]:
        """Create one Location and record it, with its Base already set.

        A client therefore never sees the new Location at the World Frame.
        """
        base = self._base_node(frame, parents=parents)
        location = _Observed(name=browse, initial_pose=_ORIGIN, initial_base=base)
        await self._file(owner, folder, location)
        # The raw frame id goes in DisplayName (application specific, localizable). The BrowseName is the
        # sanitised form. `Identifier` is the join key for multi-server aggregation.
        await _write_display_name(location.ua_node, frame)
        owner.claimed[browse] = frame
        owner.frames[frame] = _Frame(frame=frame, browse=browse, location=location, folder=folder, base=base)
        self._transform_index[frame] = location.ua_node.nodeid
        return location.ua_node, self._location_type()

    def _alternative_base(self, base: str, frames: Mapping[str, FrameLink]) -> ua.NodeId | None:
        """Return the node one alternative's base frame is, or ``None`` when it has none.

        It is never re-based onto an ancestor, because the pose was composed against that frame. The
        root of the TF tree is the list's World Frame.
        """
        held = self._transform_index.get(base)
        if held is not None:
            return held
        if base not in frames and any(link.parent == base for link in frames.values()):
            return self._transform_world
        return None

    async def _open_alternative(
        self,
        owner: _Owner,
        base: str,
        frame: str,
        node: ua.NodeId,
    ) -> _Frame | None:
        """Create one composed Location in ``AlternativeFrames``, through the chokepoint."""
        browse = member_browse_name((frame, "in", base))
        folders = {ALTERNATIVE_FRAMES} - owner.folders

        async def build() -> Sequence[tuple[Node, ua.NodeId | int]]:
            created = await self._open_composed(owner, base, frame, browse, node)
            return [*await self._opened_folders(owner, folders), (created, self._location_type())]

        made = await owner.item.mutate_address_space_with(
            binding=owner.binding.name,
            path=self.ros2_transform_settings.path,
            nodes=1 + len(folders),
            build=build,
        )
        return owner.alternatives.get((base, frame)) if made else None

    async def _open_composed(
        self,
        owner: _Owner,
        base: str,
        frame: str,
        browse: str,
        node: ua.NodeId,
    ) -> Node:
        """Create the node one alternative is, and record it."""
        location = _Observed(name=browse, initial_pose=_ORIGIN, initial_base=node)
        await self._file(owner, ALTERNATIVE_FRAMES, location)
        await _write_display_name(location.ua_node, f"{frame} in {base}")
        owner.alternatives[(base, frame)] = _Frame(
            frame=frame,
            browse=browse,
            location=location,
            folder=ALTERNATIVE_FRAMES,
            base=node,
        )
        return location.ua_node

    def _base_node(self, frame: str, *, parents: Mapping[str, str]) -> ua.NodeId:
        """Return the node one Location's ``Base`` points at.

        Delegates to :func:`~opensmi.ros2.server.spatial.base_node_of` and adds the frame index and
        World Frame node of the live Machine.
        """
        return base_node_of(
            frame,
            parents=parents,
            index=self._transform_index,
            world=self._require_world(),
        )

    def _require_world(self) -> ua.NodeId:
        """Return the list's World Frame node, which every Base terminates at.

        :raises DeclarationError: When the mirror was never initialised, which is a Machine
            with no spatial-object list and is refused long before this.
        """
        if self._transform_world is None:  # pragma: no cover -- init refuses this first
            message = f"{type(self).__name__} has no World Frame; its spatial object list was never built"
            raise DeclarationError(message)
        return self._transform_world

    @property
    def _rsl_namespace(self) -> int:
        """Return the index this server imported OPC 10000-210's namespace at."""
        return int(self.server.ua_get_namespace_index(RSLNodeIds))

    def _location_type(self) -> ua.NodeId:
        """Return the NodeId of the type a Location is instantiated as, for a model change.

        That is ``CartesianFrameAngleOrientationType``, the angle-based subtype opensmi's
        ``CartesianFrame`` is.
        """
        return self.server.ua_get_node_id(RSLNodeIds.CartesianFrameAngleOrientationType)

    async def _mark_constant(self, node: Node) -> None:
        """Set the constant access bit on one static frame's value, so nobody subscribes to it.

        This reaches into asyncua's address space, because asyncua gives no node an
        ``AccessLevelEx`` attribute and writing one through ``Node`` fails with
        ``BadAttributeIdInvalid``. The bit goes on the Location, its ``Position`` and
        ``Orientation`` and their scalars, not on the ``Base``, which moves when the tree is
        re-partitioned.
        """
        space: Any = getattr(getattr(self.server.ua_server, "iserver", None), "aspace", None)
        if space is None:  # pragma: no cover -- every server this runs on has one
            return
        marked = [node]
        for child in await node.get_children():
            name = await child.read_browse_name()
            if name.Name in ("Position", "Orientation"):
                marked.extend((child, *await child.get_children()))
        for each in marked:
            held = space.get(each.nodeid)
            if held is None:
                continue
            held.attributes[AttributeIds.AccessLevelEx] = AttributeValue(
                ua.DataValue(ua.Variant(_CONSTANT, ua.VariantType.UInt32))
            )

    async def _report_frame_collision(self, owner: _Owner, frame: str, browse: str, taken: str | None) -> None:
        """Say, as a BindingEvent, that a frame has no Location because its name is taken. Once per frame."""
        self.logger.error(
            "Refused a frame whose browse name is already taken",
            binding=owner.binding.name,
            frame=frame,
            browse_name=browse,
        )
        held = f"{taken!r} already holds" if taken else "is already taken in this tick"
        await owner.item.emit_binding_event(
            binding=owner.binding.name,
            path=self.ros2_transform_settings.path,
            code=FRAME_COLLISION,
            severity=Severity.ERROR,
            text=(
                f"the frame {frame!r} browses as {browse!r}, which {held}; it has no Location, "
                f"because sharing one would file two frames under one name"
            ),
        )

    async def _report_absent_base(self, owner: _Owner, base: str, frame: str) -> None:
        """Say, once, that a declared alternative has no base node to be relative to."""
        if (base, frame) in owner.unresolved:
            return
        owner.unresolved.add((base, frame))
        await owner.item.emit_binding_event(
            binding=owner.binding.name,
            path=self.ros2_transform_settings.path,
            code=ALTERNATIVE_BASE_ABSENT,
            severity=Severity.WARNING,
            text=(
                f"the alternative ({base!r}, {frame!r}) has a pose and no base: {base!r} is not a "
                f"frame this Machine renders and is not the root of the tree, so there is no "
                f"Location for it to be relative to"
            ),
        )
