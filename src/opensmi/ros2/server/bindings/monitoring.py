# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A ROS2 topic read into Monitoring variables."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from opensmi.ros2.runtime.qos import QoSProfile
from opensmi.ros2.runtime.selection import FieldSelection
from opensmi.ros2.server.bindings._base import AwaitedBinding
from opensmi.ros2.server.errors import (
    DeclarationError,
)


@dataclass(frozen=True, slots=True, kw_only=True)
class MonitoringBinding(AwaitedBinding):
    """A ROS2 topic read into Monitoring variables on the declaring Component.

    :ivar name: The Binding's name. The message is one complex Variable of that name, and a
        field browses below it as ``<name>/<field path>``. ``Status``, ``Health``, ``Process``
        and ``Consumption`` are reserved for the Machinery Monitoring sub-structures (OPC 40001-1).
    :ivar group: The Machinery Monitoring sub-structure the Binding is filed in, one of ``Status``,
        ``Health``, ``Process`` and ``Consumption`` (OPC 40001-1), or ``None`` to sit directly in the
        Monitoring. With it the message browses as ``<group>/<name>/<field path>``, in a FolderType Object
        the Component's Bindings share.
    :ivar path: The ROS2 topic.
    :ivar message_type: The ROS2 type, ``package/Name`` or ``package/kind/Name``, or ``None`` to take it
        from the graph. Declaring it gives the Binding its nodes at boot, so the address space does not
        depend on start-up timing. A type whose package is not installed is fatal at init. Leaving it
        out is the graph-driven case: no nodes until the topic appears, and the only kind
        that makes the Machine poll the graph.
    :ivar include: Field patterns narrowing the message; empty means every field.
    :ivar exclude: Field patterns removing fields, after ``include`` has narrowed.
    :ivar rename: New browse names by field path, such as ``{"pose/position/x": "East"}``. A name is taken as is
        and must already follow the naming rule; a path that is not rendered is refused at init. Fixes a
        name collision without a Projection.
    :ivar depth: How many messages the subscription buffers. It is the slack a slow event loop gets
        before the oldest sample is dropped.
    :ivar qos: The QoS profile to request, or ``None`` to discover one: the most permissive request
        compatible with every current publisher, re-resolved once if the topic was empty when the
        Binding opened, and relaxed when the middleware reports a publisher it cannot accept. Declared,
        it is honoured exactly and never adapted. Neither is fatal or blocks startup.
    :ivar required: Whether the StartupSkill waits for a publisher on this topic before ``Running``.
        Presence only, never first data.
    :ivar startup_timeout: Seconds that wait lasts, or ``None`` for
        :attr:`~opensmi.ros2.server.config.ROS2Config.startup_timeout`. Refused when the Binding is
        constructed without ``required``.
    :ivar max_rate: The most messages per second written to the variables, in hertz, or ``None`` to
        mirror every message. The limit is per Binding, so one message's fields stay one snapshot. The
        first message after a quiet spell is written at once, then at most one per ``1 / max_rate`` (the
        newest), and a message still waiting when the topic stops is written at the end of its window.
        Unwritten messages are not decoded. Above zero and at most
        :data:`~opensmi.ros2.server.publisher.MAXIMUM_RATE`.
    """

    kind: ClassVar[str] = "Monitoring Binding"

    path: str
    message_type: str | None = None
    include: tuple[str, ...] = ()
    exclude: tuple[str, ...] = ()
    rename: dict[str, str] = field(default_factory=dict)
    depth: int = 10
    qos: QoSProfile | None = None
    max_rate: float | None = None
    group: str | None = None

    def __post_init__(self) -> None:
        """Refuse a startup deadline nothing waits on, and a rate the server cannot keep.

        :raises RequiredDeclarationError: When ``startup_timeout`` is declared without ``required``, or
            is not positive.
        :raises DeclarationError: When ``max_rate`` is not above zero and at most the publisher's ceiling,
            the name is one the Machinery Monitoring reserves, or ``group`` is not one of its four.
        """
        # Imported here: the publishing side annotates against the declarations.
        from opensmi.ros2.server.publisher import MAXIMUM_RATE

        if self.max_rate is not None and not 0 < self.max_rate <= MAXIMUM_RATE:
            message = (
                f"the Monitoring Binding {self.name!r} declares max_rate={self.max_rate!r}, and a "
                f"rate limit is a positive number of writes a second no greater than "
                f"{MAXIMUM_RATE}; leave it out to mirror every message"
            )
            raise DeclarationError(message)
        self._require_a_name_that_can_be_browsed()
        self._require_a_name_machinery_monitoring_leaves_free()
        self._require_a_machinery_group(self.group)
        self._require_new_names_that_follow_the_rule("rename", self.rename)
        self._require_a_deadline_that_means_something()

    @property
    def selection(self) -> FieldSelection:
        """Return this Binding's field selection."""
        return FieldSelection(include=self.include, exclude=self.exclude, rename=self.rename)

    @property
    def needs_the_graph(self) -> bool:
        """Return whether this Binding learns its shape only by watching the ROS2 graph.

        A Binding with a declared type never reads the graph, so a Machine whose declarations all answer
        ``False`` polls zero times.
        """
        return self.message_type is None
