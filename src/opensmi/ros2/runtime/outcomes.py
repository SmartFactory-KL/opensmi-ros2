# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""What the Runtime returns when the thing asked for happened and went badly.

The counterpart of :mod:`opensmi.ros2.runtime.errors`: an outcome is a value, and ignoring it costs
the report rather than correctness. :class:`StopOutcome` reports a stop and :class:`GoalOutcome` a
goal that was rejected, aborted or cancelled. A goal nobody received raises
:class:`~opensmi.ros2.runtime.errors.ActionServerUnavailableError` instead.

:class:`GoalStatus` restates the numbers of ``action_msgs/msg/GoalStatus`` so this half imports
without ROS2; a Jazzy test checks that the copy agrees.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from opensmi.ros2.runtime.messages import Message

__all__ = ["GoalOutcome", "GoalStatus", "Outcome", "StopOutcome"]


@dataclass(frozen=True, slots=True, kw_only=True)
class Outcome:
    """The report of an operation that ran to completion, well or badly.

    :ivar faults: One human-readable line per thing that went wrong, in the order it went wrong.
        Empty when the operation went cleanly.
    """

    faults: tuple[str, ...]

    @property
    def ok(self) -> bool:
        """Return whether the operation went cleanly."""
        return not self.faults


@dataclass(frozen=True, slots=True, kw_only=True)
class StopOutcome(Outcome):
    """The report of :meth:`Runtime.stop`.

    Stopping always leaves the Runtime stopped, so it reports and does not raise. A still-spinning
    executor thread holds a Context that a second Runtime on the same Domain ID would race.

    :ivar executor_thread_joined: Whether the executor thread ended within the stop timeout.
    """

    executor_thread_joined: bool


class GoalStatus(IntEnum):
    """Where a ROS2 goal ended up, as ``action_msgs/msg/GoalStatus`` numbers it.

    There is no ``REJECTED`` member: ROS2 gives a rejected goal no status, so
    :attr:`GoalOutcome.accepted` carries that fact.
    """

    UNKNOWN = 0
    ACCEPTED = 1
    EXECUTING = 2
    CANCELING = 3
    SUCCEEDED = 4
    CANCELED = 5
    ABORTED = 6


@dataclass(frozen=True, slots=True, kw_only=True)
class GoalOutcome(Outcome):
    """How one ROS2 goal ended, and what it ended with.

    The status is the single fact; the properties derive from it, and :attr:`Outcome.ok` agrees
    because every ending except success carries exactly one fault line.

    :ivar accepted: Whether the server took the goal at all.
    :ivar status: Where the goal ended up, as ROS2 reports it.
    :ivar result: The result message, or ``None`` when the goal was rejected. An aborted or
        cancelled goal often has a partial one.
    """

    accepted: bool
    status: GoalStatus
    result: Message | None

    @property
    def succeeded(self) -> bool:
        """Return whether the goal ran to completion and the server said so."""
        return self.accepted and self.status is GoalStatus.SUCCEEDED

    @property
    def canceled(self) -> bool:
        """Return whether the goal stopped because somebody asked it to."""
        return self.accepted and self.status is GoalStatus.CANCELED

    @property
    def aborted(self) -> bool:
        """Return whether the server gave up on the goal itself."""
        return self.accepted and self.status is GoalStatus.ABORTED
