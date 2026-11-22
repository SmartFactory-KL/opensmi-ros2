# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A ROS2 topic published from a Skill."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import ClassVar, TypeAlias

from opensmi.ros2.runtime.messages import Message
from opensmi.ros2.runtime.qos import QoSProfile
from opensmi.ros2.runtime.selection import FieldSelection
from opensmi.ros2.server.bindings._base import NamedBinding
from opensmi.ros2.server.errors import DeclarationError
from opensmi.ros2.server.projection import ProjectionContext

MessageFunction: TypeAlias = Callable[[type[Message], Mapping[str, object], ProjectionContext], Message]
"""Builds the message to publish from a Skill's Parameters, as a pure function.

The default is :func:`opensmi.ros2.server.publisher.default_message_from`. A replacement that needs
the instance is :meth:`~opensmi.ros2.server.publisher.ROS2PublisherSkill.build_message` on a custom
Skill class.
"""


@dataclass(frozen=True, slots=True, kw_only=True)
class PublisherBinding(NamedBinding):
    """A ROS2 topic published from a Skill on the declaring Component.

    Publishing is an activity: a ``/cmd_vel`` stream runs while someone is driving, a map push happens
    once. The declaration becomes a Skill whose Parameters are the message fields, and ``Start`` is the
    only thing that publishes, so a client writes every field in any order and commits once.
    There is no debouncing or dirty tracking.

    The direction is UA to ROS2 only. The echo of a published topic is always our own message, so a
    readback would confirm the last value published, not the last delivered. To watch the topic, declare
    a :class:`MonitoringBinding` on it.

    This kind is never required: it carries neither ``required`` nor ``startup_timeout``, because the
    publisher is this server itself. To wait for the other end, declare a required
    :class:`MonitoringBinding` on the topic that answers.

    The continuity is declared, not inferred from ``rate``, so omitting ``rate`` by accident cannot
    silently give a one-shot. A continuous Skill never reaches ``Completed`` and a finite one has no
    ``Halt`` to close a stream. A second push on a finite Skill is ``Reset``, write, ``Start``.

    :ivar name: The Binding's name, also the Skill's name. Its variables are named for the field path
        alone.
    :ivar path: The ROS2 topic.
    :ivar message_type: The ROS2 type, ``package/Name`` or ``package/kind/Name``. Required, since the
        Parameters exist before anything is on the graph. A type whose package is not installed is fatal
        at init.
    :ivar continuous: Whether the Skill streams the current snapshot of its Parameters on every tick
        until ``Halt``, or publishes once and reaches ``Completed``. No default.
    :ivar include: Field patterns narrowing the message; empty means every field.
    :ivar exclude: Field patterns removing fields, after ``include`` has narrowed. A selection that
        leaves the message unbuildable is refused at init instead of filled from a default.
    :ivar rename: New browse names by field path, as on :class:`MonitoringBinding`.
    :ivar rate: Publishes per second of a continuous Skill. Required there and fatal on a finite
        Binding. It is the initial value of a client-writable ``rate`` Parameter bounded by
        :data:`~opensmi.ros2.server.publisher.MINIMUM_RATE` and
        :data:`~opensmi.ros2.server.publisher.MAXIMUM_RATE`, and takes effect on the next tick.
    :ivar qos: The QoS profile to offer, or ``None`` for the middleware's reliable, volatile default.
        Never discovered or adapted, because this declaration is how a publisher comes to exist.
    :ivar depth: The ROS2 queue depth for subscribers that have not taken messages yet.
    :ivar stale_after: Seconds a running stream may go without a client write to any of its
        Parameters before it halts itself with ``SETPOINT_STALE``, or ``None`` for never. Every write
        starts the interval again, a rewrite of the same value too, and so does ``Start``. It guards
        against a client that died or lost its connection while its session and the Lock live on,
        which would leave a robot driving on the last ``/cmd_vel``. Continuous only, and a positive,
        finite number.
    :ivar halt_message: The message a stream publishes once when it ends, whatever ended it, or
        ``None`` to just stop publishing. A table of ROS2 field names (not browse names; the
        selection and ``rename`` do not apply), nested for a nested message: ``{"linear": {"x": 0.0}}``.
        A field it does not name keeps the default the message type's generated class gives it.
        Checked against the message type at init. Continuous only.
    :ivar tolerated_failures: Consecutive publish failures a continuous Skill absorbs before halting;
        zero halts on the first. Non-zero on a finite Skill is fatal at init.
    :ivar skill_type: A subclass of :class:`~opensmi.ros2.server.publisher.ROS2ContinuousPublisherSkill`
        or :class:`~opensmi.ros2.server.publisher.ROS2FinitePublisherSkill` matching ``continuous``, or
        ``None`` for the stock one. Python-only; refused when the Binding is constructed if it is not a
        subclass.
    :ivar message_from: Builds the message from the Parameters, or ``None`` for the default.
        Python-only.
    """

    kind: ClassVar[str] = "Publishing Binding"

    path: str
    message_type: str
    continuous: bool
    include: tuple[str, ...] = ()
    exclude: tuple[str, ...] = ()
    rename: dict[str, str] = field(default_factory=dict)
    rate: float | None = None
    depth: int = 10
    qos: QoSProfile | None = None
    stale_after: float | None = None
    halt_message: Mapping[str, object] | None = None
    tolerated_failures: int = 0
    skill_type: type | None = None
    message_from: MessageFunction | None = None

    def __post_init__(self) -> None:
        """Refuse a contradictory declaration.

        Checks that need the message type (the reserved ``rate`` name, a Projection that cannot invert)
        are left to the Skill's constructor, which is still init.

        :raises BindingNameError: When ``name`` is empty or not a valid browse name.
        :raises DeclarationError: When a keyword belonging to one continuity is declared on the other, or
            a continuous declaration has no rate.
        :raises ImplementationClassError: When ``skill_type`` is not a subclass of the stock class for
            this continuity.
        """
        self._require_a_name_that_can_be_browsed()
        self._require_new_names_that_follow_the_rule("rename", self.rename)
        self._require_the_continuity_agrees()
        if self.skill_type is None:
            return
        self._require_a_subclass(
            self.skill_type,
            self.stock_skill_type,
            "a publishing Skill's continuity decides which stock class it adds to, a custom class "
            "adds behaviour rather than replacing it, and a TOML file cannot name one at all",
        )

    def _require_the_continuity_agrees(self) -> None:
        """Refuse a keyword that means nothing on the chosen continuity.

        :raises DeclarationError: On any disagreement between a keyword and the continuity.
        """
        # Imported here for the same reason as in ``MonitoringBinding``.
        from opensmi.ros2.server.publisher import MAXIMUM_RATE, MINIMUM_RATE

        if not self.continuous:
            for keyword, value in (
                ("rate", self.rate),
                ("stale_after", self.stale_after),
                ("halt_message", self.halt_message),
            ):
                if value is not None:
                    self._refuse(f"declares {keyword}={value!r}, and that exists only on a stream")
            if self.tolerated_failures:
                self._refuse(
                    f"declares tolerated_failures={self.tolerated_failures!r}, and a Skill that "
                    f"publishes once has no consecutive failures to tolerate -- it raises on the "
                    f"one it can have"
                )
            return
        if self.rate is None:
            self._refuse("streams and declares no rate; a tick needs a period, so say how often")
        elif not MINIMUM_RATE <= self.rate <= MAXIMUM_RATE:
            self._refuse(
                f"declares rate={self.rate!r}, and a publish rate is a positive number of times a "
                f"second between {MINIMUM_RATE} and {MAXIMUM_RATE}"
            )
        if self.stale_after is not None and not (math.isfinite(self.stale_after) and self.stale_after > 0):
            self._refuse(
                f"declares stale_after={self.stale_after!r}; it is the number of seconds a stream may "
                f"go without a write before it halts, so a positive, finite number, and leaving it "
                f"out is how it is turned off"
            )
        if self.tolerated_failures < 0:
            self._refuse(f"declares tolerated_failures={self.tolerated_failures!r}, which is not a count")

    def _refuse(self, why: str) -> None:
        """Raise a :class:`DeclarationError` naming the Binding."""
        shape = "continuous" if self.continuous else "finite"
        message = f"the {shape} publishing Binding {self.name!r} {why}"
        raise DeclarationError(message)

    @property
    def stock_skill_type(self) -> type:
        """Return the stock Skill class this declaration's continuity builds."""
        from opensmi.ros2.server.publisher import ROS2ContinuousPublisherSkill, ROS2FinitePublisherSkill

        return ROS2ContinuousPublisherSkill if self.continuous else ROS2FinitePublisherSkill

    @property
    def selection(self) -> FieldSelection:
        """Return this Binding's field selection."""
        return FieldSelection(include=self.include, exclude=self.exclude, rename=self.rename)

    @property
    def needs_the_graph(self) -> bool:
        """Return ``False``: we are the publisher, so the shape comes from the declaration."""
        return False
