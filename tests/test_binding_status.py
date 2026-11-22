# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The three status codes as a pure function of two booleans."""

from __future__ import annotations

from asyncua import ua

from opensmi.ros2.server.bindings import MonitoringBinding
from opensmi.ros2.server.status import (
    GOOD,
    NO_COMMUNICATION,
    WAITING_FOR_INITIAL_DATA,
    status_code,
    status_value,
    transform_status,
)


def test_a_topic_with_no_publisher_reads_bad_no_communication() -> None:
    assert status_code(satisfied=False, sampled=False) == NO_COMMUNICATION


def test_a_topic_with_a_publisher_and_no_message_reads_bad_waiting_for_initial_data() -> None:
    assert status_code(satisfied=True, sampled=False) == WAITING_FOR_INITIAL_DATA


def test_a_topic_that_has_carried_a_sample_reads_good() -> None:
    assert status_code(satisfied=True, sampled=True) == GOOD


def test_a_publisher_that_went_away_reads_bad_no_communication_again() -> None:
    assert status_code(satisfied=False, sampled=True) == NO_COMMUNICATION


def test_a_write_carries_the_typed_value_beside_the_status() -> None:
    written = status_value(0.0, ua.VariantType.Double, NO_COMMUNICATION)

    assert written.StatusCode == NO_COMMUNICATION
    assert written.Value is not None
    assert written.Value.Value == 0.0
    assert written.Value.VariantType is ua.VariantType.Double


def test_a_binding_that_declares_its_type_never_needs_the_graph() -> None:
    declared = MonitoringBinding(name="Scan", path="/scan", message_type="sensor_msgs/msg/LaserScan")

    assert not declared.needs_the_graph


def test_a_binding_with_no_declared_type_is_the_one_that_needs_the_graph() -> None:
    undeclared = MonitoringBinding(name="Scan", path="/scan")

    assert undeclared.needs_the_graph


def test_a_frame_nothing_has_published_on_a_robot_that_is_off_reads_bad_no_communication() -> None:
    assert transform_status(present=False, static=False, rooted=False, seen=None, now=10.0, within=5.0) == (
        NO_COMMUNICATION
    )


def test_a_frame_nothing_has_published_on_a_robot_that_is_on_reads_waiting_for_initial_data() -> None:
    assert transform_status(present=True, static=False, rooted=False, seen=None, now=10.0, within=5.0) == (
        WAITING_FOR_INITIAL_DATA
    )


def test_a_frame_that_arrived_inside_the_window_reads_good() -> None:
    assert transform_status(present=True, static=False, rooted=False, seen=6.0, now=10.0, within=5.0) == GOOD


def test_a_frame_that_stopped_arriving_reads_bad_no_communication_while_its_robot_is_on() -> None:
    assert transform_status(present=True, static=False, rooted=False, seen=4.0, now=10.0, within=5.0) == (
        NO_COMMUNICATION
    )


def test_the_window_is_inclusive_at_its_edge() -> None:
    assert transform_status(present=True, static=False, rooted=False, seen=5.0, now=10.0, within=5.0) == GOOD


def test_a_frame_whose_topics_lost_every_publisher_reads_bad_however_fresh_it_is() -> None:
    assert transform_status(present=False, static=False, rooted=False, seen=10.0, now=10.0, within=5.0) == (
        NO_COMMUNICATION
    )


def test_a_static_frame_stays_good_however_long_ago_it_arrived() -> None:
    assert transform_status(present=True, static=True, rooted=False, seen=0.0, now=100_000.0, within=5.0) == GOOD


def test_a_static_frame_stays_good_after_its_transient_local_publisher_has_gone() -> None:
    assert transform_status(present=False, static=True, rooted=False, seen=0.0, now=100_000.0, within=5.0) == GOOD


def test_a_static_frame_nobody_has_published_is_still_bad() -> None:
    assert transform_status(present=True, static=True, rooted=False, seen=None, now=10.0, within=5.0) == (
        WAITING_FOR_INITIAL_DATA
    )


def test_the_root_of_the_tree_reads_good_while_anything_publishes_transforms() -> None:
    assert transform_status(present=True, static=False, rooted=True, seen=None, now=10.0, within=5.0) == GOOD


def test_the_root_of_the_tree_goes_bad_when_its_robot_does() -> None:
    assert transform_status(present=False, static=False, rooted=True, seen=None, now=10.0, within=5.0) == (
        NO_COMMUNICATION
    )
