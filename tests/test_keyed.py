# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""What a Keyed Binding decides, as pure functions: member, name, transition, event band and aggregate,
plus state-map declaration."""

from __future__ import annotations

import pytest

from opensmi.ros2.server.bindings import KeyedBinding
from opensmi.ros2.server.events import Severity
from opensmi.ros2.server.keyed import member_browse_name, member_key, severity_of, state_changed, worst_state
from tests import fixture_messages


def test_a_single_field_key_is_that_field() -> None:
    element = fixture_messages.unit(name="motor", hardware_id="robot_1")

    assert member_key(element, ("name",)) == ("motor",)


def test_a_multi_field_key_keeps_every_field_in_declaration_order() -> None:
    left = fixture_messages.unit(name="motor", hardware_id="robot_1")
    right = fixture_messages.unit(name="motor", hardware_id="robot_2")

    assert member_key(left, ("hardware_id", "name")) == ("robot_1", "motor")
    assert member_key(right, ("hardware_id", "name")) != member_key(left, ("hardware_id", "name"))


def test_a_key_field_that_is_not_a_string_is_still_a_key() -> None:
    element = fixture_messages.unit(level=2)

    assert member_key(element, ("level",)) == ("2",)


@pytest.mark.parametrize(
    ("key", "expected"),
    [
        (("motor",), "motor"),
        (("robot 1: left motor",), "robot 1_ left motor"),
        (("/base_link",), "_base_link"),
        (("planner.controller_server",), "planner_controller_server"),
        (("a<b>#c!d&e",), "a_b__c_d_e"),
        (("robot_1", "motor"), "robot_1_motor"),
        (("",), "unnamed"),
    ],
)
def test_a_member_browse_name_replaces_each_reserved_character_with_an_underscore(
    key: tuple[str, ...], expected: str
) -> None:
    assert member_browse_name(key) == expected


def test_two_keys_that_differ_in_a_reserved_character_alike_produce_one_name() -> None:
    assert member_browse_name(("motor.left",)) == member_browse_name(("motor/left",))


def test_a_write_that_changes_the_state_is_a_transition() -> None:
    assert state_changed(0, 2) is True


def test_a_write_that_repeats_the_state_is_not() -> None:
    assert state_changed(2, 2) is False


def test_a_member_arriving_at_its_typed_zero_is_not_a_transition() -> None:
    assert state_changed(0, 0) is False
    assert state_changed(0, 3) is True


def test_the_aggregate_is_the_highest_state_any_member_holds() -> None:
    assert worst_state([0, 2, 1], initial=0) == 2


def test_the_aggregate_of_no_members_is_the_typed_zero() -> None:
    assert worst_state([], initial=0) == 0


def test_the_aggregate_ignores_a_state_that_is_not_a_number() -> None:
    assert worst_state(["warn", "ok"], initial="ok") == "ok"


def test_a_declared_severity_is_what_a_transition_is_reported_at() -> None:
    severities = {0: Severity.INFO, 1: Severity.WARNING, 2: Severity.ERROR, 3: Severity.STALE}

    assert severity_of(2, severities) is Severity.ERROR


def test_stale_and_error_are_reported_at_the_same_band() -> None:
    severities = {2: Severity.ERROR, 3: Severity.STALE}

    assert severity_of(3, severities) == severity_of(2, severities)


def test_a_state_nobody_declared_a_severity_for_is_a_warning() -> None:
    assert severity_of(9, {0: Severity.INFO}) is Severity.WARNING


def test_a_binding_that_declares_no_severities_reports_every_transition_the_same() -> None:
    assert severity_of(3, {}) is Severity.INFO


def keyed(**declared) -> KeyedBinding:
    """Declare a Keyed Binding over the fixture array, with whatever the test is about."""
    return KeyedBinding(
        name="Diagnostics",
        path="/diagnostics",
        message_type="fixture_msgs/msg/Units",
        repeated_field="units",
        key=("hardware_id", "name"),
        **declared,
    )


def test_the_four_ros2_constants_keep_their_own_values_and_their_own_order() -> None:
    declared = keyed(state_field="level", states={2: "ERROR", 0: "OK", 3: "STALE", 1: "WARN"})

    assert tuple(sorted(declared.states.items())) == ((0, "OK"), (1, "WARN"), (2, "ERROR"), (3, "STALE"))
    assert len(declared.states) == 4


def test_a_constant_set_that_starts_late_or_skips_a_number_keeps_saying_what_it_says() -> None:
    declared = keyed(state_field="level", states={1: "Low", 4: "High"})

    assert tuple(sorted(declared.states.items())) == ((1, "Low"), (4, "High"))


def test_a_state_map_written_in_a_file_is_re_keyed_to_the_numbers_it_meant() -> None:
    declared = keyed(state_field="level", states={"0": "OK"}, severities={"2": 999})

    assert declared.states == {0: "OK"}
    assert declared.severities == {2: Severity.ERROR}
