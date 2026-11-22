# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A repeated message field rendered as one member per key."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from opensmi.ros2.runtime.qos import QoSProfile
from opensmi.ros2.runtime.selection import FieldSelection
from opensmi.ros2.server.bindings._base import AwaitedBinding
from opensmi.ros2.server.errors import (
    DeclarationError,
)
from opensmi.ros2.server.events import Severity


@dataclass(frozen=True, slots=True, kw_only=True)
class KeyedBinding(AwaitedBinding):
    """A ROS2 topic whose repeated field becomes one OPC UA member per key.

    ``/diagnostics`` keyed by ``status[]`` on ``hardware_id`` and ``name`` is the real instance. The
    repeated field and the key fields are declared; membership is taken from
    the messages. The TF tree is keyed in the same sense but has its own declaration,
    :class:`TFBinding`, because its members are OPC 10000-210 Locations. It renders the members only: for
    the fields beside the repeated one, declare a :class:`MonitoringBinding` narrowed to them.

    The number of members is capped by
    :attr:`~opensmi.ros2.server.mutation.ROS2MutationMixin.mutation_node_cap` on the Component and its
    Machine's total. A Binding whose members stop appearing has reached one of them and reports a
    BindingEvent.

    :ivar name: The Binding's name, the group its members sit in. A member browses as
        ``<name>/<key>`` (a variable group) with each of ``/ . < > : # ! &`` in the key replaced by ``_``, shows the raw
        key as its DisplayName, and holds its variables below it. The names reserved on
        :class:`MonitoringBinding` are reserved here too.
    :ivar group: The Machinery Monitoring sub-structure the Binding's group is filed in, as on
        :class:`MonitoringBinding`. A member then browses as ``<group>/<name>/<key>``.
    :ivar path: The ROS2 topic.
    :ivar message_type: The ROS2 type, ``package/Name`` or ``package/kind/Name``. Required, since the
        member shape and the aggregate come from the element type.
    :ivar repeated_field: The field whose elements become members. Anything but a repeated message
        field of ``message_type`` is fatal at init.
    :ivar key: The fields of one element that together name its member, in browse-name order. A
        sequence, since ``hardware_id`` tells two identical units apart.
    :ivar state_field: The one field of a member whose change is worth an event, or ``None``.
        Declaring it gives a BindingEvent on every write that moves it, and a variable
        ``Worst<StateField>`` with the highest value any member holds.
    :ivar states: What each value of ``state_field`` is called, which mints the enumeration a client
        reads labels from. The value is the ROS2 constant's own.
    :ivar severities: The band a transition into each state is reported at. Without it every
        transition is information.
    :ivar include: Field patterns narrowing the element; empty means every field.
    :ivar exclude: Field patterns removing element fields, after ``include`` has narrowed.
    :ivar rename: New browse names by field path, as on :class:`MonitoringBinding`.
    :ivar depth: How many messages the subscription buffers.
    :ivar qos: The QoS profile to request, or ``None`` to discover one, as on :class:`MonitoringBinding`.
    :ivar required: Whether the StartupSkill waits for a publisher on this topic before ``Running``.
        Presence only, never membership.
    :ivar startup_timeout: Seconds that wait lasts, or ``None`` for
        :attr:`~opensmi.ros2.server.config.ROS2Config.startup_timeout`.
    """

    kind: ClassVar[str] = "Keyed Binding"

    path: str
    message_type: str
    repeated_field: str
    key: tuple[str, ...]
    state_field: str | None = None
    states: dict[int, str] = field(default_factory=dict)
    severities: dict[int, Severity] = field(default_factory=dict)
    include: tuple[str, ...] = ()
    exclude: tuple[str, ...] = ()
    rename: dict[str, str] = field(default_factory=dict)
    depth: int = 10
    qos: QoSProfile | None = None
    group: str | None = None

    def __post_init__(self) -> None:
        """Refuse a contradictory declaration, and re-key the two maps to integers.

        TOML table keys are strings, and a state value is a number.

        :raises DeclarationError: When the Binding keys on nothing, names state labels or severities
            without a state field, takes a name the Machinery Monitoring reserves, or names a ``group``
            that is not one of its four.
        :raises RequiredDeclarationError: When ``startup_timeout`` is declared without ``required``, or
            is not positive.
        """
        self._require_a_name_that_can_be_browsed()
        self._require_a_name_machinery_monitoring_leaves_free()
        self._require_a_machinery_group(self.group)
        self._require_new_names_that_follow_the_rule("rename", self.rename)
        self._require_a_deadline_that_means_something()
        if not self.key:
            message = (
                f"the Keyed Binding {self.name!r} keys on nothing; the key is what names a "
                f"member, and a set whose elements have no names is a list"
            )
            raise DeclarationError(message)
        if self.state_field is None and (self.states or self.severities):
            message = (
                f"the Keyed Binding {self.name!r} declares what its states mean and names no "
                f"state_field; labels and severities describe the one field whose change is an "
                f"event, so without it they describe nothing"
            )
            raise DeclarationError(message)
        object.__setattr__(self, "states", {int(value): label for value, label in self.states.items()})
        object.__setattr__(
            self,
            "severities",
            {int(value): Severity(band) for value, band in self.severities.items()},
        )

    @property
    def selection(self) -> FieldSelection:
        """Return the field selection, which narrows one element and not the message."""
        return FieldSelection(include=self.include, exclude=self.exclude, rename=self.rename)

    @property
    def needs_the_graph(self) -> bool:
        """Return ``False``: the member plan is known at init and membership comes from the messages."""
        return False
