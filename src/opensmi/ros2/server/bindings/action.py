# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A ROS2 action run as a finite Skill."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import ClassVar, TypeAlias

from opensmi.ros2.runtime.messages import Message
from opensmi.ros2.runtime.selection import FieldSelection
from opensmi.ros2.server.bindings._base import AwaitedBinding
from opensmi.ros2.server.projection import ProjectionContext

GoalFunction: TypeAlias = Callable[[type[Message], Mapping[str, object], ProjectionContext], Message]
"""Builds a goal message from a Skill's Parameters, as a pure function.

The default is :func:`opensmi.ros2.server.skill.default_goal_from`.
"""

ResultFunction: TypeAlias = Callable[[Message, ProjectionContext], Mapping[str, object]]
"""Turns a result message into the values of a Skill's FinalResultData, by variable name.

The default is :func:`opensmi.ros2.server.skill.default_result_to`.
"""


@dataclass(frozen=True, slots=True, kw_only=True)
class ActionBinding(AwaitedBinding):
    """A ROS2 action exposed as a finite Skill on the declaring Component.

    The goal becomes the Skill's Parameters (written before ``Start``), the feedback its Monitoring
    (written while the goal runs) and the result its FinalResultData. The outcome drives the state
    machine and is not data of its own. Each message is narrowed by its own include/exclude pair.

    :ivar name: The Binding's name, also the Skill's name. Its variables are named for the field path
        alone.
    :ivar path: The ROS2 action.
    :ivar action_type: The ROS2 action type, ``package/action/Name``. Required, so the address space does
        not depend on start-up timing. A type whose package is not installed is fatal at init.
    :ivar include: Field patterns narrowing the goal; empty means every field.
    :ivar exclude: Field patterns removing goal fields, after ``include`` has narrowed. A selection that
        leaves the goal unbuildable is refused at init instead of filled from a default.
    :ivar rename: New browse names by field path, as on :class:`MonitoringBinding`.
    :ivar feedback_include: Field patterns narrowing the feedback.
    :ivar feedback_exclude: Field patterns removing feedback fields.
    :ivar feedback_rename: New browse names for feedback fields, as :attr:`rename` does.
    :ivar result_include: Field patterns narrowing the result.
    :ivar result_exclude: Field patterns removing result fields.
    :ivar result_rename: New browse names for result fields, as :attr:`rename` does.
    :ivar feedback_depth: How many feedback messages the goal handle buffers before the oldest is
        dropped.
    :ivar server_timeout: Seconds ``Start`` waits for an action server before halting, so a Skill does
        not sit in ``Starting`` against a robot that is off.
    :ivar skill_type: A subclass of :class:`opensmi.ros2.server.skill.ROS2ActionSkill`, or ``None`` for
        the stock one. Python-only; refused when the Binding is constructed if it is not a
        subclass.
    :ivar goal_from: Builds the goal message from the Parameters, or ``None`` for the default.
        Python-only.
    :ivar result_to: Turns the result message into FinalResultData values, or ``None`` for the default.
        Python-only.
    :ivar required: Whether the StartupSkill waits for the action server to be discovered before
        ``Running``.
    :ivar startup_timeout: Seconds that wait lasts, or ``None`` for the deployment's default. Refused when the
        Binding is constructed without ``required``.
    """

    kind: ClassVar[str] = "action Binding"

    path: str
    action_type: str
    include: tuple[str, ...] = ()
    exclude: tuple[str, ...] = ()
    rename: dict[str, str] = field(default_factory=dict)
    feedback_include: tuple[str, ...] = ()
    feedback_exclude: tuple[str, ...] = ()
    feedback_rename: dict[str, str] = field(default_factory=dict)
    result_include: tuple[str, ...] = ()
    result_exclude: tuple[str, ...] = ()
    result_rename: dict[str, str] = field(default_factory=dict)
    feedback_depth: int = 10
    server_timeout: float = 10.0
    skill_type: type | None = None
    goal_from: GoalFunction | None = None
    result_to: ResultFunction | None = None

    def __post_init__(self) -> None:
        """Refuse a custom implementation class that is not a subclass of the stock one.

        :raises ImplementationClassError: When ``skill_type`` does not derive from
            :class:`opensmi.ros2.server.skill.ROS2ActionSkill`.
        :raises RequiredDeclarationError: When ``startup_timeout`` is declared without ``required``, or
            is not positive.
        """
        self._require_a_name_that_can_be_browsed()
        self._require_new_names_that_follow_the_rule("rename", self.rename)
        self._require_new_names_that_follow_the_rule("feedback_rename", self.feedback_rename)
        self._require_new_names_that_follow_the_rule("result_rename", self.result_rename)
        self._require_a_deadline_that_means_something()
        if self.skill_type is None:
            return
        # Imported here: the Skill annotates against this declaration.
        from opensmi.ros2.server.skill import ROS2ActionSkill

        self._require_a_subclass(
            self.skill_type,
            ROS2ActionSkill,
            "a custom class adds behaviour to the stock one rather than replacing it, "
            "and a TOML file cannot name one at all",
        )

    @property
    def selection(self) -> FieldSelection:
        """Return the goal's field selection."""
        return FieldSelection(include=self.include, exclude=self.exclude, rename=self.rename)

    @property
    def feedback_selection(self) -> FieldSelection:
        """Return the feedback's field selection."""
        return FieldSelection(include=self.feedback_include, exclude=self.feedback_exclude, rename=self.feedback_rename)

    @property
    def result_selection(self) -> FieldSelection:
        """Return the result's field selection."""
        return FieldSelection(include=self.result_include, exclude=self.result_exclude, rename=self.result_rename)

    @property
    def needs_the_graph(self) -> bool:
        """Return ``False``: the declaration carries its type."""
        return False
