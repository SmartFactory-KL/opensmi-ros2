# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Field selection as a pure function of a message type."""

from __future__ import annotations

import pytest

from opensmi.ros2.runtime.errors import FieldSelectionError
from opensmi.ros2.runtime.selection import FieldSelection, field_paths, resolve_selection
from tests import fixture_messages


def test_every_reachable_path_is_walked_interior_paths_included() -> None:
    assert field_paths(fixture_messages.Pose) == (
        "position",
        "position/x",
        "position/y",
        "position/z",
        "orientation",
        "orientation/x",
        "orientation/y",
        "orientation/z",
        "orientation/w",
    )


def test_a_repeated_message_field_is_a_leaf_and_is_not_walked_into() -> None:
    assert field_paths(fixture_messages.Path) == (
        "header",
        "header/stamp",
        "header/stamp/sec",
        "header/stamp/nanosec",
        "header/frame_id",
        "poses",
    )


def test_selecting_nothing_keeps_every_leaf() -> None:
    selected = resolve_selection(fixture_messages.Pose, FieldSelection())

    assert selected == frozenset(field_paths(fixture_messages.Pose))


def test_excluding_an_interior_path_excludes_its_whole_subtree() -> None:
    selected = resolve_selection(fixture_messages.Pose, FieldSelection(exclude=("orientation",)))

    assert selected == frozenset({"position", "position/x", "position/y", "position/z"})


def test_including_a_leaf_keeps_it_and_the_ancestors_that_carry_it() -> None:
    selected = resolve_selection(fixture_messages.Pose, FieldSelection(include=("position/x",)))

    assert selected == frozenset({"position", "position/x"})


def test_a_glob_matches_across_levels() -> None:
    selected = resolve_selection(fixture_messages.PoseStamped, FieldSelection(include=("*/position/*",)))

    assert selected == frozenset({"pose", "pose/position", "pose/position/x", "pose/position/y", "pose/position/z"})


def test_exclude_wins_over_include() -> None:
    selection = FieldSelection(include=("position",), exclude=("position/y",))

    selected = resolve_selection(fixture_messages.Pose, selection)

    assert selected == frozenset({"position", "position/x", "position/z"})


def test_a_pattern_matching_nothing_is_fatal_and_names_itself() -> None:
    with pytest.raises(FieldSelectionError) as raised:
        resolve_selection(fixture_messages.Pose, FieldSelection(include=("possition/x",)))

    assert "possition/x" in str(raised.value)
    assert "fixture_msgs/msg/Pose" in str(raised.value)


def test_an_exclude_pattern_matching_nothing_is_fatal_too() -> None:
    with pytest.raises(FieldSelectionError):
        resolve_selection(fixture_messages.Pose, FieldSelection(exclude=("orientation/roll",)))


def test_a_selection_that_leaves_no_field_is_fatal() -> None:
    with pytest.raises(FieldSelectionError):
        resolve_selection(fixture_messages.Pose, FieldSelection(exclude=("position", "orientation")))


def test_a_message_that_has_no_fields_resolves_to_an_empty_selection() -> None:
    """An empty message is not an emptied selection: refusing it once took the Machine down at `_init`."""
    assert resolve_selection(fixture_messages.Nothing, FieldSelection()) == frozenset()


def test_renaming_a_rendered_path_leaves_the_selection_as_it_was() -> None:
    selection = FieldSelection(rename={"position/x": "East", "orientation": "Q"})

    selected = resolve_selection(fixture_messages.Pose, selection)

    assert selected == frozenset(field_paths(fixture_messages.Pose))


def test_renaming_a_path_the_message_does_not_have_is_refused() -> None:
    with pytest.raises(FieldSelectionError, match=r"rename.*'position/q'"):
        resolve_selection(fixture_messages.Pose, FieldSelection(rename={"position/q": "East"}))


def test_renaming_a_path_the_selection_removed_is_refused() -> None:
    with pytest.raises(FieldSelectionError, match=r"rename.*'orientation/w'"):
        resolve_selection(
            fixture_messages.Pose,
            FieldSelection(exclude=("orientation",), rename={"orientation/w": "Scalar"}),
        )
