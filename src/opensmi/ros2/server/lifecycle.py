# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The correspondence between lifecycle nodes and Skill states, written down as data.

A ROS2 lifecycle node's state machine and the SkillSet continuous-skill state machine are the same
machine:

=============  ==================  ===========  ==============================
Skill state    Lifecycle state     Trigger      Lifecycle transition
=============  ==================  ===========  ==============================
``Halted``     ``unconfigured``    ``reset``    ``configure``
``Ready``      ``inactive``        ``start``    ``activate``
``Running``    ``active``          ``halt``     ``deactivate`` + ``cleanup``
=============  ==================  ===========  ==============================

The row ``Suspended`` / ``inactive`` / ``suspend`` / ``deactivate`` is not offered, because the
StartupSkill is never suspendable. A node observed deactivating is still read as ``Suspended`` and
the Skill follows it down to ``Halted``. ``halt`` is never ``shutdown``: ``finalized`` is terminal,
so a shut-down node could never be reset, and the shutdown transitions are absent from
:class:`LifecycleNodeTransition`.

The Binding follows the state pair of a ``transition_event``, not its ``transition`` field, which
``rcl_lifecycle`` leaves empty on every event (verified live on Jazzy). Entering ``inactive`` from
``configuring`` is ``Ready``, from ``deactivating`` it is ``Suspended``, and from ``activating``
(a failed activation) it is ``Halted``. opensmi's ``SkillState`` is not ordered, so :data:`_RANKS`
names every state it covers.

Everything here is a pure function of integers and enumerations. The Drivers are in
:mod:`opensmi.ros2.server.drivers` and the Skill in :mod:`opensmi.ros2.server.startup`.
"""

from __future__ import annotations

from enum import IntEnum
from typing import TYPE_CHECKING, Final

from opensmi.core import SkillState

if TYPE_CHECKING:
    from collections.abc import Iterable, Mapping

    from opensmi.server.mixins.skill_logic_mixins import TRIGGER

__all__ = [
    "PRIMARY_STATES",
    "RESTING",
    "TRANSITIONS_FOR",
    "TRIGGERS",
    "UNDONE_BY",
    "LifecycleNodeState",
    "LifecycleNodeTransition",
    "aggregate",
    "resting_skill_state",
    "skill_state_for",
    "trigger_toward",
]


class LifecycleNodeState(IntEnum):
    """The states of a ROS2 managed node, with the identifiers ``lifecycle_msgs/msg/State`` gives.

    The four primary states are the ones a node rests in. The transition states are what it passes
    through; a ``transition_event`` names one as the state the node came from, which tells ``Ready``
    from ``Suspended``.
    """

    UNKNOWN = 0
    UNCONFIGURED = 1
    INACTIVE = 2
    ACTIVE = 3
    FINALIZED = 4
    CONFIGURING = 10
    CLEANING_UP = 11
    SHUTTING_DOWN = 12
    ACTIVATING = 13
    DEACTIVATING = 14
    ERROR_PROCESSING = 15


class LifecycleNodeTransition(IntEnum):
    """The transitions this project ever commands, with ``lifecycle_msgs/msg/Transition``'s ids.

    Four of the nine public transitions. The three ``shutdown`` transitions and ``destroy`` end in
    ``finalized``, which is terminal, so commanding one would make ``Halted`` unrecoverable;
    ``create`` is what constructing the node already did.
    """

    CONFIGURE = 1
    CLEANUP = 2
    ACTIVATE = 3
    DEACTIVATE = 4


PRIMARY_STATES: Final = frozenset(
    {
        LifecycleNodeState.UNCONFIGURED,
        LifecycleNodeState.INACTIVE,
        LifecycleNodeState.ACTIVE,
        LifecycleNodeState.FINALIZED,
    }
)
"""The states a node rests in, and therefore the only ones a Skill state is read off.

A node passing through ``configuring`` is not news: the Skill is already in its own intermediate
state for the same moment.
"""


TRIGGERS: Final[Mapping[SkillState, TRIGGER]] = {
    SkillState.HALTED: "halt",
    SkillState.READY: "reset",
    SkillState.RUNNING: "start",
}
"""Which trigger reaches each of the three Skill states the Skill can rest in.

The triggers are named for the act, so ``reset`` is the only way into ``Ready`` and out of
``Halted``. ``Suspended`` has no entry because there is no ``suspend`` trigger.
"""


TRANSITIONS_FOR: Final[Mapping[TRIGGER, tuple[LifecycleNodeTransition, ...]]] = {
    "reset": (LifecycleNodeTransition.CONFIGURE,),
    "start": (LifecycleNodeTransition.ACTIVATE,),
    "halt": (LifecycleNodeTransition.DEACTIVATE, LifecycleNodeTransition.CLEANUP),
}
"""What each Skill trigger sends at the robot, in order.

``halt`` is two ordered steps: an ``active`` node has to be deactivated before it can be cleaned
up, and an ``inactive`` one does not offer the first. The Driver's gate decides which go out.
"""


UNDONE_BY: Final[Mapping[LifecycleNodeTransition, LifecycleNodeTransition]] = {
    LifecycleNodeTransition.DEACTIVATE: LifecycleNodeTransition.ACTIVATE,
    LifecycleNodeTransition.CLEANUP: LifecycleNodeTransition.CONFIGURE,
}
"""Which transition a ``halt`` step rolls back: ``deactivate`` undoes ``activate``, ``cleanup``
undoes ``configure``.

A ``halt`` sends a step only when the StartupSkill itself performed the transition it undoes
, so a node that was already ``active`` is left as it was found.
"""


_RANKS: Final[Mapping[SkillState, int]] = {
    SkillState.HALTED: 0,
    SkillState.READY: 1,
    SkillState.SUSPENDED: 1,
    SkillState.RUNNING: 2,
}
"""How much each Skill state claims about a node, which is what "lowest wins" compares.

``Ready`` and ``Suspended`` share a rank because both are ``inactive`` and both accept ``start``.
"""


RESTING: Final = frozenset({SkillState.HALTED, SkillState.READY, SkillState.RUNNING})
"""The three Skill states a StartupSkill rests in.

``Suspended`` can be observed on a node (it ranks in :data:`_RANKS`) but never entered.
"""


def resting_skill_state(state: LifecycleNodeState | int) -> SkillState | None:
    """Return the Skill state one resting lifecycle state means, with no route to read.

    ``unconfigured`` and ``finalized`` are ``Halted``, ``active`` is ``Running`` and ``inactive``
    is ``Ready``. A node in a transition state answers ``None`` (no opinion). ``finalized`` differs
    from ``unconfigured`` only by the Binding being Terminal, which refuses ``Reset``.
    """
    if state == LifecycleNodeState.ACTIVE:
        return SkillState.RUNNING
    if state == LifecycleNodeState.INACTIVE:
        return SkillState.READY
    if state in (LifecycleNodeState.UNCONFIGURED, LifecycleNodeState.FINALIZED):
        return SkillState.HALTED
    return None


def skill_state_for(*, start: LifecycleNodeState | int, goal: LifecycleNodeState | int) -> SkillState | None:
    """Return the Skill state one observed transition puts a node in, from the pair alone.

    :param start: The state the node left, as ``transition_event.start_state.id``.
    :param goal: The state it entered, as ``transition_event.goal_state.id``.
    :return: The Skill state, or ``None`` when the node merely entered a transition state.
    """
    if goal != LifecycleNodeState.INACTIVE:
        return resting_skill_state(goal)
    if start == LifecycleNodeState.DEACTIVATING:
        return SkillState.SUSPENDED
    if start == LifecycleNodeState.ACTIVATING:
        # A failed activation lands Halted, like a failed command of ours.
        return SkillState.HALTED
    # `errorprocessing` has no branch: ROS2 resolves it to `unconfigured` or `finalized`.
    return SkillState.READY


def _rank(state: SkillState) -> int:
    """Return how much a Skill state claims: ``0`` halted, ``1`` inactive, ``2`` running.

    :raises KeyError: For a state outside the correspondence (intermediate states, ``Completed``).
    """
    return _RANKS[state]


def aggregate(states: Iterable[SkillState | None]) -> SkillState | None:
    """Return what several nodes under one Skill say together: lowest wins.

    ``Running`` means every node is active. A node nobody has heard from contributes
    ``None`` and is skipped, so a node that vanishes while the Skill is ``Running`` does not halt
    it. Between equal ranks the answer is ``Suspended`` if any node reported one.

    :return: The lowest-ranked state anything reported, or ``None`` when nothing has.
    """
    known = [state for state in states if state is not None]
    if not known:
        return None
    lowest = min(_rank(state) for state in known)
    tied = [state for state in known if _rank(state) == lowest]
    return SkillState.SUSPENDED if SkillState.SUSPENDED in tied else tied[0]


def trigger_toward(current: SkillState, target: SkillState) -> TRIGGER | None:
    """Return the trigger that moves a Skill from where it is toward what its nodes report.

    Only downward, and always ``halt``. Following a robot down keeps the
    address space truthful; following it up would start a Skill nobody asked to start.

    :param current: The state the Skill is in. An intermediate state answers ``None``, because a
        handler is already in flight.
    :param target: What :func:`aggregate` says the nodes are.
    :return: ``halt``, or ``None`` when the Skill is at or below the nodes.
    """
    if current not in RESTING or _rank(target) >= _rank(current):
        return None
    return TRIGGERS[SkillState.HALTED]
