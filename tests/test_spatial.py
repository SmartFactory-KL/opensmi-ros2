# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Where a TF frame lands, as pure functions.

The tree is the spec's Annex C.2: a robot on a linear unit.
"""

from __future__ import annotations

import math

import pytest

from opensmi.ros2.runtime.tf import FrameLink, Transform
from opensmi.ros2.server.spatial import (
    ALTERNATIVE_FRAMES,
    ATTACH_POINTS,
    INTERNAL_FRAMES,
    FrameClash,
    angles,
    attach_points,
    base_frame_of,
    base_node_of,
    excluded,
    folder_of,
    nearest_ancestor,
    one_direction_per_frame,
    owner_of,
    owners,
    parents_of,
    rotation,
    tree_roots,
)

#: map -> unit_base -> unit_carriage -> robot_base -> arm_link -> tool0, plus a sensor on the unit.
TREE = {
    "unit_base": "map",
    "unit_carriage": "unit_base",
    "unit_sensor": "unit_base",
    "robot_base": "unit_carriage",
    "arm_link": "robot_base",
    "tool0": "arm_link",
}

HEADS = ("unit_base", "robot_base")


def test_a_head_frame_owns_itself() -> None:
    assert owner_of("robot_base", parents=TREE, heads=HEADS) == "robot_base"


def test_a_frame_belongs_to_the_nearest_declared_ancestor() -> None:
    assert owner_of("tool0", parents=TREE, heads=HEADS) == "robot_base"
    assert owner_of("unit_carriage", parents=TREE, heads=HEADS) == "unit_base"


def test_a_frame_above_every_head_belongs_to_nobody() -> None:
    assert owner_of("map", parents=TREE, heads=HEADS) is None


def test_the_partition_covers_every_frame_that_has_an_owner() -> None:
    assert owners(parents=TREE, heads=HEADS) == {
        "unit_base": "unit_base",
        "unit_carriage": "unit_base",
        "unit_sensor": "unit_base",
        "robot_base": "robot_base",
        "arm_link": "robot_base",
        "tool0": "robot_base",
    }


def test_a_cycle_ends_the_walk_rather_than_hanging_it() -> None:
    looped = {"a": "b", "b": "a"}

    assert owner_of("a", parents=looped, heads=("head",)) is None


def test_an_attach_point_is_a_frame_another_objects_head_hangs_off() -> None:
    assert attach_points(parents=TREE, heads=HEADS) == frozenset({"unit_carriage"})


def test_a_head_mounted_straight_onto_another_head_files_no_attach_point() -> None:
    tree = {"robot_base": "unit_base", "unit_base": "map"}

    assert attach_points(parents=tree, heads=("unit_base", "robot_base")) == frozenset()


def test_a_head_frame_is_the_position_frame_and_not_in_a_folder() -> None:
    assert folder_of("robot_base", head="robot_base", attach=()) is None


def test_every_frame_that_is_not_an_attach_point_is_an_internal_frame() -> None:
    attach = attach_points(parents=TREE, heads=HEADS)

    assert folder_of("unit_carriage", head="unit_base", attach=attach) == ATTACH_POINTS
    assert folder_of("unit_sensor", head="unit_base", attach=attach) == INTERNAL_FRAMES


def test_the_base_is_the_frames_tf_parent_wherever_it_lives() -> None:
    assert base_frame_of("robot_base", parents=TREE, kept=TREE) == "unit_carriage"


def test_the_root_of_the_tree_answers_with_the_world_rather_than_a_frame() -> None:
    assert base_frame_of("unit_base", parents=TREE, kept=TREE) is None


def test_an_exclusion_re_bases_a_child_onto_the_nearest_surviving_ancestor() -> None:
    kept = set(TREE) - {"arm_link"}

    assert base_frame_of("tool0", parents=TREE, kept=kept) == "robot_base"


def test_an_exclusion_that_removes_every_ancestor_re_bases_onto_the_world() -> None:
    kept = {"tool0"}

    assert base_frame_of("tool0", parents=TREE, kept=kept) is None


def test_nearest_ancestor_can_be_asked_to_skip_the_frame_itself() -> None:
    assert nearest_ancestor("robot_base", parents=TREE, admitted=HEADS) == "robot_base"
    assert nearest_ancestor("robot_base", parents=TREE, admitted=HEADS, include_self=False) == "unit_base"


def test_the_parents_of_a_facets_frames_is_the_tree_the_partition_reads() -> None:
    link = FrameLink(
        child="odom",
        parent="map",
        transform=Transform(translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0)),
        static=False,
    )

    assert parents_of({"odom": link}) == {"odom": "map"}


@pytest.mark.parametrize(
    ("frame", "patterns", "gone"),
    [
        ("left_wheel_link", ("*_wheel_link",), True),
        ("right_wheel_link", ("*_wheel_link",), True),
        ("base_link", ("*_wheel_link",), False),
        ("imu_link", ("imu_link",), True),
        ("imu_link", (), False),
    ],
)
def test_an_exclusion_is_a_glob_over_the_whole_frame_id(frame, patterns, gone) -> None:
    assert excluded(frame, patterns) is gone


def test_an_identity_quaternion_is_three_zeroes() -> None:
    assert angles((0.0, 0.0, 0.0, 1.0)) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    ("axis", "index"),
    [((1.0, 0.0, 0.0), 0), ((0.0, 1.0, 0.0), 1), ((0.0, 0.0, 1.0), 2)],
)
def test_a_turn_about_one_axis_lands_on_its_own_angle_and_leaves_the_others_alone(axis, index) -> None:
    # Forty-five degrees, because a pitch of exactly ninety is the gimbal-lock pole.
    half = math.sin(math.pi / 8)
    rotation = (*(each * half for each in axis), math.cos(math.pi / 8))

    turned = angles(rotation)  # type: ignore[arg-type]

    assert turned[index] == pytest.approx(45.0)
    assert [each for position, each in enumerate(turned) if position != index] == pytest.approx([0.0, 0.0])


def test_a_pitch_at_the_pole_is_ninety_degrees_rather_than_a_domain_error() -> None:
    # A unit quaternion a hair past one makes asin raise; the honest answer is the pole.
    turned = angles((0.0, math.sin(math.pi / 4) * 1.0000001, 0.0, math.cos(math.pi / 4)))

    assert turned[1] == pytest.approx(90.0, abs=1e-3)


def test_the_alternative_folder_is_named_for_what_the_specification_calls_it() -> None:
    assert (ATTACH_POINTS, INTERNAL_FRAMES, ALTERNATIVE_FRAMES) == (
        "AttachPoints",
        "InternalFrames",
        "AlternativeFrames",
    )


def test_a_base_resolves_through_the_frame_to_node_index_to_whatever_holds_the_parent() -> None:
    index = {"unit_base": "N1", "unit_carriage": "N2", "robot_base": "N3", "arm_link": "N4", "tool0": "N5"}

    assert base_node_of("robot_base", parents=TREE, index=index, world="WORLD") == "N2"
    assert base_node_of("tool0", parents=TREE, index=index, world="WORLD") == "N4"


def test_a_base_whose_walk_leaves_the_index_is_the_world_frame() -> None:
    index = {"unit_base": "N1"}

    assert base_node_of("unit_base", parents=TREE, index=index, world="WORLD") == "WORLD"


def test_an_exclusion_re_bases_through_the_index_onto_the_nearest_surviving_node() -> None:
    index = {"robot_base": "N3", "tool0": "N5"}

    assert base_node_of("tool0", parents=TREE, index=index, world="WORLD") == "N3"


# --------------------------------------------------------------------------- outward: the geometry


def test_three_zeroes_are_the_identity_quaternion() -> None:
    assert rotation((0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0, 1.0)


@pytest.mark.parametrize(
    ("index", "axis"),
    [(0, 0), (1, 1), (2, 2)],
)
def test_one_declared_angle_turns_about_its_own_axis_and_no_other(index: int, axis: int) -> None:
    declared = [0.0, 0.0, 0.0]
    declared[index] = 45.0

    x, y, z, w = rotation((declared[0], declared[1], declared[2]))

    assert (x, y, z)[axis] == pytest.approx(math.sin(math.pi / 8))
    assert [each for position, each in enumerate((x, y, z)) if position != axis] == pytest.approx([0.0, 0.0])
    assert w == pytest.approx(math.cos(math.pi / 8))


@pytest.mark.parametrize(
    "declared",
    [(0.0, 0.0, 0.0), (30.0, 0.0, 0.0), (0.0, -20.0, 0.0), (0.0, 0.0, 170.0), (10.0, 20.0, 30.0)],
)
def test_the_two_conversions_are_each_other_inverse(declared: tuple[float, float, float]) -> None:
    assert angles(rotation(declared)) == pytest.approx(declared, abs=1e-9)


# ------------------------------------------------------------ outward: one direction per frame


def test_frames_claimed_once_each_are_no_clash() -> None:
    assert (
        one_direction_per_frame(
            heads={"unit_frames": "unit_base", "arm_frames": "robot_base"},
            published={"unit_frames": ("map_estimate",), "arm_frames": ("payload_mount",)},
        )
        is None
    )


def test_publishing_a_frame_another_object_reads_as_its_head_is_a_clash() -> None:
    clash = one_direction_per_frame(
        heads={"unit_frames": "unit_base", "arm_frames": "robot_base"},
        published={"unit_frames": ("robot_base",)},
    )

    assert clash == FrameClash(frame="robot_base", outward="unit_frames", against="arm_frames", inward=True)


def test_two_objects_publishing_one_frame_is_a_clash() -> None:
    clash = one_direction_per_frame(
        heads={"unit_frames": "unit_base", "arm_frames": "robot_base"},
        published={"unit_frames": ("payload_mount",), "arm_frames": ("payload_mount",)},
    )

    assert clash == FrameClash(frame="payload_mount", outward="unit_frames", against="arm_frames", inward=False)


def test_the_clash_reported_is_the_same_one_on_every_boot() -> None:
    # Ties are broken by the caller's order over the objects, then alphabetically over the frames.
    clash = one_direction_per_frame(
        heads={"unit_frames": "unit_base", "arm_frames": "robot_base"},
        published={"unit_frames": ("robot_base", "unit_base")},
    )

    assert clash is not None
    assert clash.frame == "robot_base"


def test_the_roots_of_the_rendered_tree_are_the_frames_nobody_parents() -> None:
    assert tree_roots({"odom": "map", "base_link": "odom"}) == ("map",)


def test_two_unrelated_coordinate_systems_show_up_as_two_roots() -> None:
    assert tree_roots({"odom": "map", "cell": "world"}) == ("map", "world")
