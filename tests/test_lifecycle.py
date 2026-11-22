# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The lifecycle state correspondence as a pure function, with no Runtime or server."""

from __future__ import annotations

import pytest
from opensmi.core import SkillState

from opensmi.ros2.server.lifecycle import (
    PRIMARY_STATES,
    RESTING,
    TRANSITIONS_FOR,
    TRIGGERS,
    LifecycleNodeState,
    LifecycleNodeTransition,
    _rank,
    aggregate,
    resting_skill_state,
    skill_state_for,
    trigger_toward,
)

UNCONFIGURED = LifecycleNodeState.UNCONFIGURED
INACTIVE = LifecycleNodeState.INACTIVE
ACTIVE = LifecycleNodeState.ACTIVE
FINALIZED = LifecycleNodeState.FINALIZED


def test_the_shipped_table_is_the_one_the_adr_records() -> None:
    assert TRIGGERS == {SkillState.HALTED: "halt", SkillState.READY: "reset", SkillState.RUNNING: "start"}
    assert TRANSITIONS_FOR["reset"] == (LifecycleNodeTransition.CONFIGURE,)
    assert TRANSITIONS_FOR["start"] == (LifecycleNodeTransition.ACTIVATE,)
    assert TRANSITIONS_FOR["halt"] == (LifecycleNodeTransition.DEACTIVATE, LifecycleNodeTransition.CLEANUP)


def test_there_is_no_suspend_row_while_opensmis_startup_skill_cannot_be_suspended() -> None:
    assert SkillState.SUSPENDED not in TRIGGERS
    assert "suspend" not in TRANSITIONS_FOR
    assert SkillState.SUSPENDED not in RESTING


def test_halt_is_never_shutdown() -> None:
    commanded = {each for sequence in TRANSITIONS_FOR.values() for each in sequence}
    assert commanded == {1, 2, 3, 4}
    assert {5, 6, 7, 8}.isdisjoint({int(each) for each in LifecycleNodeTransition})


@pytest.mark.parametrize(
    ("start", "goal", "expected"),
    [
        (LifecycleNodeState.CONFIGURING, INACTIVE, SkillState.READY),
        (LifecycleNodeState.DEACTIVATING, INACTIVE, SkillState.SUSPENDED),
        (LifecycleNodeState.ACTIVATING, ACTIVE, SkillState.RUNNING),
        (LifecycleNodeState.CLEANING_UP, UNCONFIGURED, SkillState.HALTED),
        (LifecycleNodeState.SHUTTING_DOWN, FINALIZED, SkillState.HALTED),
    ],
)
def test_the_state_pair_decides_the_skill_state(start: int, goal: int, expected: SkillState) -> None:
    assert skill_state_for(start=start, goal=goal) == expected


def test_entering_inactive_from_activating_is_a_failed_activation_and_halts() -> None:
    assert skill_state_for(start=LifecycleNodeState.ACTIVATING, goal=INACTIVE) == SkillState.HALTED
    assert skill_state_for(start=LifecycleNodeState.DEACTIVATING, goal=INACTIVE) == SkillState.SUSPENDED


def test_any_other_route_into_inactive_reads_as_the_rank_it_is() -> None:
    assert skill_state_for(start=LifecycleNodeState.CLEANING_UP, goal=INACTIVE) == SkillState.READY
    assert skill_state_for(start=LifecycleNodeState.UNKNOWN, goal=INACTIVE) == SkillState.READY


def test_entering_a_transition_state_moves_nothing() -> None:
    assert skill_state_for(start=UNCONFIGURED, goal=LifecycleNodeState.CONFIGURING) is None
    assert skill_state_for(start=ACTIVE, goal=LifecycleNodeState.DEACTIVATING) is None
    assert resting_skill_state(LifecycleNodeState.ACTIVATING) is None


def test_only_the_four_primary_states_are_followed() -> None:
    assert {UNCONFIGURED, INACTIVE, ACTIVE, FINALIZED} == PRIMARY_STATES
    assert all(resting_skill_state(each) is not None for each in PRIMARY_STATES)


def test_a_bare_state_reading_has_no_route_and_says_so() -> None:
    assert resting_skill_state(UNCONFIGURED) == SkillState.HALTED
    assert resting_skill_state(INACTIVE) == SkillState.READY
    assert resting_skill_state(ACTIVE) == SkillState.RUNNING


def test_the_terminal_state_is_halted_and_not_a_fourth_state() -> None:
    assert resting_skill_state(FINALIZED) == SkillState.HALTED
    assert skill_state_for(start=LifecycleNodeState.SHUTTING_DOWN, goal=FINALIZED) == SkillState.HALTED


def test_the_rank_table_names_every_state_it_covers() -> None:
    assert _rank(SkillState.HALTED) < _rank(SkillState.READY) < _rank(SkillState.RUNNING)
    for moment in (SkillState.STARTING, SkillState.HALTING, SkillState.RESETTING, SkillState.SUSPENDING):
        with pytest.raises(KeyError):
            _rank(moment)
    with pytest.raises(KeyError):
        _rank(SkillState.COMPLETED)


def test_the_two_inactive_states_are_one_rank() -> None:
    assert _rank(SkillState.READY) == _rank(SkillState.SUSPENDED)


def test_several_nodes_aggregate_lowest_wins() -> None:
    assert aggregate([SkillState.RUNNING, SkillState.RUNNING]) == SkillState.RUNNING
    assert aggregate([SkillState.RUNNING, SkillState.READY]) == SkillState.READY
    assert aggregate([SkillState.RUNNING, SkillState.SUSPENDED]) == SkillState.SUSPENDED
    assert aggregate([SkillState.READY, SkillState.HALTED, SkillState.RUNNING]) == SkillState.HALTED


def test_a_node_nobody_has_heard_from_is_skipped_rather_than_counted_as_halted() -> None:
    # a node vanishing while the Skill is Running must not halt it (halting cascades).
    assert aggregate([SkillState.RUNNING, None]) == SkillState.RUNNING
    assert aggregate([None, None]) is None
    assert aggregate([]) is None


def test_following_is_downward_only() -> None:
    assert trigger_toward(SkillState.RUNNING, SkillState.HALTED) == "halt"
    assert trigger_toward(SkillState.READY, SkillState.HALTED) == "halt"
    assert trigger_toward(SkillState.HALTED, SkillState.RUNNING) is None
    assert trigger_toward(SkillState.READY, SkillState.RUNNING) is None


def test_an_external_deactivate_halts_because_there_is_no_suspended_to_land_in() -> None:
    assert trigger_toward(SkillState.RUNNING, SkillState.SUSPENDED) == "halt"
    assert trigger_toward(SkillState.RUNNING, SkillState.READY) == "halt"


def test_the_skill_does_not_move_between_the_two_states_of_one_rank() -> None:
    assert trigger_toward(SkillState.READY, SkillState.SUSPENDED) is None


def test_a_skill_that_is_already_moving_is_left_alone() -> None:
    for moving in (SkillState.STARTING, SkillState.HALTING, SkillState.RESETTING, SkillState.SUSPENDING):
        assert trigger_toward(moving, SkillState.HALTED) is None
