# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The project's namespace, and the one event it raises about itself.

A BindingEvent is the only OPC UA event this project defines. Its subject is always a Binding: a
diagnostic changing level, a QoS mismatch, a failed publish, a reached cap, a key collision, a
lifecycle node moved by something other than us. It carries the Binding name, the ROS2 path and a
code. A fault with a state worth reading has a variable for it; the rest are moments, and a moment
is an event.

The event type subtypes the base event type, not the house ``AlertType``, which would need the OPC
UA condition lifecycle that the stack underneath hardly implements. The code goes in its
own ``Code`` field, like the house alert's ``ErrorCode``. The event is emitted by the item's
Notification node with ``SourceNode`` and ``SourceName`` set to the item itself.

The namespace is shipped as XML and registered through opensmi.server's nodeset handling; NodeIds
are read off :class:`~opensmi.ros2.server.nodesets.SmartFactoryROS2NodeIds`. Events are raised on
asyncua through the server's ``ua_server``. :class:`ROS2EventsMixin` puts the BindingEvent on the
item's own Notification node, where opensmi fires its ``ua_log_*`` events.
:class:`ROS2ModelChangeMixin` puts OPC UA's ``GeneralModelChangeEvent`` on the spatial-object list,
the node OPC 10000-210 names for it.

A client has to subscribe to the emitting node itself. asyncua does not propagate events up the
``HasNotifier`` hierarchy, so a subscription to the Server object receives none of them, and the
failure is silence.
"""

from __future__ import annotations

import asyncio
import datetime as dt
from collections import deque
from collections.abc import AsyncGenerator
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import TYPE_CHECKING, Any, Final

from asyncua import ua
from asyncua.ua.object_ids import ObjectIds
from asyncua.ua.uaerrors import BadNoMatch
from opensmi.core.lifecycle_mixin import lifecycle
from opensmi.server.base_machinery_item import BaseMachineryItem
from opensmi.server.mixins import MonitoringMixin, NotificationMixin, SpatialObjectListMixin

from opensmi.ros2.server.errors import DeclarationError
from opensmi.ros2.server.nodesets import PACKAGE, SmartFactoryROS2NodeIds

if TYPE_CHECKING:
    from collections.abc import Sequence

    from asyncua.common.node import Node
    from asyncua.server.event_generator import EventGenerator
    from opensmi.server import Server

__all__ = [
    "BINDING_EVENT_TYPE",
    "NAMESPACE_URI",
    "NODESET_PATH",
    "RECENT_EVENTS",
    "SIGNED_BYTE_STRING_TYPE",
    "BindingEventRecord",
    "ROS2EventsMixin",
    "ROS2ModelChangeMixin",
    "Severity",
    "import_nodeset",
    "root_of",
]

_NODE_VERSION: Final = "NodeVersion"
"""The Property OPC UA Part 3 defines and OPC 10000-210 makes mandatory on the list.

It is in namespace zero, hence the explicit index ``0`` in the lookup.
"""


NAMESPACE_URI: Final = SmartFactoryROS2NodeIds.URI
"""This project's own namespace URI.

It names the information model, not the distribution that ships it.
"""


NODESET_PATH: Final = Path(__file__).parent / "nodesets" / SmartFactoryROS2NodeIds.FILE_NAME
"""Where the nodeset XML that defines everything in :data:`NAMESPACE_URI` is installed."""

BINDING_EVENT_TYPE: Final = SmartFactoryROS2NodeIds.BindingEventType
"""``BindingEventType`` in this project's namespace, resolved per server with ``ua_get_node_id``."""

SIGNED_BYTE_STRING_TYPE: Final = SmartFactoryROS2NodeIds.SignedByteString
"""``SignedByteString`` in this project's namespace, resolved per server with ``ua_get_node_id``.

A ``ByteString`` subtype for bulk payloads whose elements are signed (an occupancy grid's
``int8[]``), which a client would otherwise read as large positive values. Unsigned payloads keep
the plain ``ByteString``.
"""


RECENT_EVENTS: Final = 200
"""How many BindingEvents one Machine keeps, so that they can be asked for after the fact.

A client that was not subscribed when an event fired cannot learn of it, so the introspection
Component hands back the last N from this ring. It is a ring so that memory does not grow with
every diagnostic transition; 200 covers a boot and the minutes after it.
"""


class Severity(IntEnum):
    """The house severity bands, which are the only ones this project uses.

    opensmi writes ``0`` for information, ``555`` for a warning and ``999`` for an error. ``STALE``
    is an alias of ``ERROR``: ROS2 ranks neither level, so a separate band would be this package's
    opinion presented as the robot's.
    """

    INFO = 0
    WARNING = 555
    ERROR = 999
    STALE = 999


@dataclass(frozen=True, slots=True, kw_only=True)
class BindingEventRecord:
    """One BindingEvent, kept so that it can be read after the moment it was.

    Holds the event's own fields plus when and where, so the introspection Component shows what a
    subscribed client would have received.

    :ivar at: When it fired, in UTC (wall-clock, to correlate with a robot's log).
    :ivar item: The path of the machinery item that fired it.
    :ivar binding: The Binding it is about.
    :ivar path: The ROS2 topic, service, action or node path that Binding names.
    :ivar code: What happened, as the stable identifier a client branches on.
    :ivar severity: The house band it was fired at.
    :ivar text: The human-readable detail that went into the event's ``Message``.
    """

    at: dt.datetime
    item: str
    binding: str
    path: str
    code: str
    severity: Severity
    text: str

    def __str__(self) -> str:
        """Return the one line an operator reads: when, where, what, and why."""
        when = self.at.isoformat(timespec="seconds")
        return f"{when} {self.item} {self.binding} ({self.path}) {self.code}: {self.text}"


async def import_nodeset(server: Server) -> int:
    """Register this project's nodeset with a server, import it once, and return its namespace index.

    The import is idempotent, so a second Machine in the same process reuses the first one's.

    :param server: The opensmi server to import into.
    """
    server.ua_register_nodeset_xml(namespace=SmartFactoryROS2NodeIds, package=PACKAGE)
    return await server.ua_import_nodeset(SmartFactoryROS2NodeIds)


def root_of(item: object) -> Any:
    """Return the Machine a machinery item reports to, which for a Machine is itself."""
    current = item
    while (parent := getattr(current, "parent", None)) is not None:
        current = parent
    return current


class ROS2EventsMixin(NotificationMixin):
    """Gives a machinery item the BindingEvent, on its own Notification node.

    Brings opensmi's :class:`~opensmi.server.mixins.NotificationMixin`, so a Component that raises
    BindingEvents has a Notification of its own. The namespace is imported and the generator built
    in a lifecycle step after the Notification exists and before the item's ``_init``.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Start out with no event generator, and pass every argument to opensmi."""
        super().__init__(*args, **kwargs)
        self._ros2_namespace_index: int | None = None
        self._binding_events: EventGenerator | None = None
        # Every BindingEvent fired on this Machine, oldest first. Only the Machine's ring is written.
        self._recent_binding_events: deque[BindingEventRecord] = deque(maxlen=RECENT_EVENTS)
        # The generator keeps its fields between triggers, so concurrent emits would interleave.
        self._binding_event_lock = asyncio.Lock()

    @lifecycle(
        after=NotificationMixin.lifecycle_notification,
        before=BaseMachineryItem.lifecycle_machinery_item,
    )
    async def lifecycle_ros2_binding_events(self) -> AsyncGenerator[None]:
        """Import this project's namespace and build the BindingEvent generator on the Notification.

        opensmi's generator keeps the Binding, ROS2Path and Code fields in the Notification's event history.
        """
        self._ros2_namespace_index = await import_nodeset(self.server)
        self._house_notification_node()  # refuses an item without a Notification
        self._binding_events = await self.notification.add_event_generator(
            self.server.ua_get_node_id(BINDING_EVENT_TYPE)
        )
        yield
        # no shutdown: the generator lives as long as the node it fires on

    @property
    def ros2_root_machine(self) -> Any:
        """Return the Machine this item reports to, which for a Machine is itself."""
        return root_of(self)

    @property
    def ros2_recent_events(self) -> tuple[BindingEventRecord, ...]:
        """Return the BindingEvents the Machine and its Components fired, oldest first, at most RECENT_EVENTS."""
        return tuple(self._recent_binding_events)

    @property
    def ros2_namespace_index(self) -> int:
        """Return the index this server imported :data:`NAMESPACE_URI` at.

        :raises DeclarationError: Before the item's BindingEvents are initialised, which is
            where the import happens.
        """
        if self._ros2_namespace_index is None:
            message = (
                f"{type(self).__name__} has no namespace index yet; {NAMESPACE_URI} is imported "
                f"while the machinery item's Notification is initialised"
            )
            raise DeclarationError(message)
        return self._ros2_namespace_index

    def _house_notification_node(self) -> ua.NodeId:
        """Return the node opensmi fires its own machinery-item messages on.

        :raises DeclarationError: When this machinery item has no Notification at all.
        """
        try:
            node_id: ua.NodeId = self.notification.ua_node.nodeid
        except (AssertionError, RuntimeError) as error:
            message = (
                f"{type(self).__name__} has no Notification object, so a BindingEvent would have "
                f"nowhere to be emitted from"
            )
            raise DeclarationError(message) from error
        return node_id

    async def emit_binding_event(
        self,
        *,
        binding: str,
        path: str,
        code: str,
        severity: Severity,
        text: str | None = None,
    ) -> None:
        """Fire one BindingEvent from this machinery item.

        :param binding: The name of the Binding this is about.
        :param path: The ROS2 topic, service, action or node path that Binding names.
        :param code: What happened, as a stable upper-snake-case identifier a client can branch on.
        :param severity: One of the house bands.
        :param text: A human-readable detail for the event's ``Message``, such as a node's own
            refusal text. Defaults to a summary of the other three and never carries the code.
        :raises DeclarationError: When this machinery item's BindingEvents were never
            initialised, so there is no generator to fire from.
        """
        if self._binding_events is None:
            message = f"{type(self).__name__} has no BindingEvent generator; its Notification was never initialised"
            raise DeclarationError(message)
        # The fields come from the nodeset, so they are set by name on asyncua's event object.
        message = text if text is not None else f"{code} on {binding} ({path})"
        event: Any = self._binding_events.event
        async with self._binding_event_lock:
            event.SourceNode = self.ua_node.nodeid
            event.SourceName = self.name
            event.Binding = binding
            event.ROS2Path = path
            event.Code = code
            event.Severity = int(severity)
            await self._binding_events.trigger(message=message)
        self._keep(binding=binding, path=path, code=code, severity=severity, text=message)
        self.logger.info("Fired a BindingEvent", binding=binding, path=path, code=code, severity=int(severity))

    def _keep(self, *, binding: str, path: str, code: str, severity: Severity, text: str) -> None:
        """Record one fired BindingEvent on this Machine's ring, after the trigger succeeded."""
        record = BindingEventRecord(
            at=dt.datetime.now(dt.UTC),
            item=str(self.path),
            binding=binding,
            path=path,
            code=code,
            severity=severity,
            text=text,
        )
        self.ros2_root_machine.note_binding_event(record)

    def note_binding_event(self, record: BindingEventRecord) -> None:
        """Add one fired BindingEvent to this Machine's ring. Called by every item on it."""
        self._recent_binding_events.append(record)


class ROS2ModelChangeMixin(SpatialObjectListMixin, MonitoringMixin, ROS2EventsMixin):
    """Adds the standard model-change event, on the node the companion specification names.

    For the Machine only: OPC 10000-210 marks the spatial-object list with
    ``GeneratesEvent GeneralModelChangeEventType``, and there is one per Machine. It brings opensmi's
    spatial-object list and Monitoring container and builds the list as the Machine comes up.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Start out with no model-change generator, and pass everything to opensmi."""
        super().__init__(*args, **kwargs)
        self._model_changes: EventGenerator | None = None
        self._model_change_lock = asyncio.Lock()
        self._node_version: Node | None = None
        self._node_versions = 0

    @lifecycle(
        after=MonitoringMixin.lifecycle_monitoring,
        before=BaseMachineryItem.lifecycle_machinery_item,
    )
    async def lifecycle_ros2_model_change(self) -> AsyncGenerator[None]:
        """Build the spatial-object list, and generate model-change events on it."""
        # Named after the Machine, to tell its list from another server's when a client aggregates.
        await self._init_spatial_object_list(name=f"{self.name}SpatialObjectList")
        self._model_changes = await self.server.ua_server.get_event_generator(
            ua.NodeId(ua.Int32(ObjectIds.GeneralModelChangeEventType)),
            self._specified_model_change_node(),
        )
        # asyncua types `Changes` with a DataType NodeId, which `ua.Variant` refuses; a structure
        # travels as an ExtensionObject.
        self._model_changes.event.data_types["Changes"] = ua.VariantType.ExtensionObject
        self._node_version = await self._specified_node_version()
        yield
        # no shutdown: the generator lives as long as the node it fires on

    async def _specified_node_version(self) -> Node | None:
        """Return the ``NodeVersion`` OPC 10000-210 makes mandatory on the spatial-object list.

        An event says the address space moved; the node version is what a client that was not
        listening compares against. ``None`` (with a warning) when the nodeset
        has none, since the model change is the notification that matters.
        """
        node = self.spatial_object_list.ua_node
        try:
            return await node.get_child(ua.QualifiedName(_NODE_VERSION, 0))
        except BadNoMatch:
            self.logger.warning(
                "The spatial object list carries no NodeVersion; model changes are announced without one",
                node=node.nodeid.to_string(),
            )
            return None

    def _specified_model_change_node(self) -> ua.NodeId:
        """Return the node OPC 10000-210 names as the source of ``GeneralModelChangeEvent``.

        :raises DeclarationError: When the Machine has no spatial-object list.
        """
        try:
            node_id: ua.NodeId = self.spatial_object_list.ua_node.nodeid
        except Exception as error:
            message = (
                f"{type(self).__name__} has no spatial object list, which is the node OPC "
                f"10000-210 names as the source of GeneralModelChangeEvent"
            )
            raise DeclarationError(message) from error
        return node_id

    async def emit_model_change(self, changes: Sequence[ua.ModelChangeStructureDataType]) -> None:
        """Announce that the address space moved, with every change of one mutation batched.

        One event per call and none at init. An empty batch is not announced.

        :param changes: What moved, one entry per node, as OPC UA models a model change.
        :raises DeclarationError: When this Machine's model-change generator was never
            built, so there is nothing to fire from.
        """
        if self._model_changes is None:
            message = f"{type(self).__name__} has no model-change generator; its spatial object list was never built"
            raise DeclarationError(message)
        if not changes:
            return
        event: Any = self._model_changes.event
        async with self._model_change_lock:
            await self._bump_node_version()
            event.Changes = list(changes)
            await self._model_changes.trigger()

    async def _bump_node_version(self) -> None:
        """Write the next node version, before the event that says why it moved.

        Written inside the same lock, so a client woken by the event reads a version that already
        counts it. The value is an opaque counter as text (OPC UA Part 3); only its changing means
        anything. It moves for every runtime mutation on the Machine, not only those under the
        list: a client re-browses once too often but never misses a change.
        """
        if self._node_version is None:
            return
        self._node_versions += 1
        await self._node_version.write_value(ua.Variant(str(self._node_versions), ua.VariantType.String))

    @property
    def node_version(self) -> int:
        """Return how many times this Machine's spatial-object list has been re-versioned."""
        return self._node_versions
