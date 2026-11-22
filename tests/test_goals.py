# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The goal handle, its outcome, and the two pure default goal and result mappings."""

from __future__ import annotations

from typing import Any, cast

import pytest
from asyncua import ua

from opensmi.ros2.runtime.errors import GoalClosedError
from opensmi.ros2.runtime.goal import Goal, GoalSink, rejected_outcome, settled_outcome
from opensmi.ros2.runtime.outcomes import GoalStatus
from opensmi.ros2.runtime.selection import FieldSelection, resolve_selection
from opensmi.ros2.server.projection import PlanVariable, ProjectionContext
from opensmi.ros2.server.projections import DEFAULT_REGISTRY
from opensmi.ros2.server.recipes import Functions
from opensmi.ros2.server.skill import default_goal_from, default_result_to
from tests import fixture_messages

GOAL = fixture_messages.NavigateGoal(target=fixture_messages.Point(x=1.0, y=2.0, z=3.0), label="kitchen")
RESULT = fixture_messages.NavigateResult(total_distance=9.5, waypoints=4)


def context(message_type: Any, registry: Any = DEFAULT_REGISTRY) -> ProjectionContext:
    """Return a context selecting every field of a type, which is what no declaration means."""
    return ProjectionContext(registry=registry, selected=resolve_selection(message_type, FieldSelection()))


def opened(depth: int = 10) -> tuple[Goal, GoalSink, list[str]]:
    """Return a Goal handle over a sink, and the record of what was asked of its two callables."""
    sink = GoalSink(depth=depth)
    asked: list[str] = []

    async def request_cancel() -> None:
        asked.append("cancel")

    async def release() -> None:
        asked.append("release")

    handle = Goal(
        action="/navigate",
        types=fixture_messages.Navigate,
        sink=sink,
        request_cancel=request_cancel,
        release=release,
    )
    return handle, sink, asked


# -- the outcome, which is one fact read four ways ---------------------------------------------


@pytest.mark.parametrize(
    ("status", "succeeded", "aborted", "canceled"),
    [
        (GoalStatus.SUCCEEDED, True, False, False),
        (GoalStatus.ABORTED, False, True, False),
        (GoalStatus.CANCELED, False, False, True),
    ],
)
def test_the_four_questions_are_derived_from_one_status(
    status: GoalStatus, succeeded: bool, aborted: bool, canceled: bool
) -> None:
    outcome = settled_outcome(status, RESULT)

    assert (outcome.succeeded, outcome.aborted, outcome.canceled) == (succeeded, aborted, canceled)
    assert outcome.accepted is True


def test_a_successful_outcome_carries_no_faults_and_every_other_one_does() -> None:
    assert settled_outcome(GoalStatus.SUCCEEDED, RESULT).ok
    assert not settled_outcome(GoalStatus.ABORTED, RESULT).ok
    assert not rejected_outcome("/navigate").ok


def test_a_rejected_goal_is_not_given_a_status_ros2_does_not_have() -> None:
    outcome = rejected_outcome("/navigate")

    assert not outcome.accepted
    assert outcome.status is GoalStatus.UNKNOWN
    assert outcome.result is None


# -- the handle --------------------------------------------------------------------------------


async def test_feedback_is_a_stream_that_ends_when_the_goal_ends() -> None:
    handle, sink, _ = opened()
    sink.deliver(fixture_messages.NavigateFeedback(distance_remaining=2.0))
    sink.deliver(fixture_messages.NavigateFeedback(distance_remaining=1.0))
    sink.settle(settled_outcome(GoalStatus.SUCCEEDED, RESULT))

    seen = [cast("Any", feedback).distance_remaining async for feedback in handle]

    assert seen == [2.0, 1.0]


async def test_feedback_past_the_depth_drops_the_oldest() -> None:
    handle, sink, _ = opened(depth=2)
    for remaining in (3.0, 2.0, 1.0):
        sink.deliver(fixture_messages.NavigateFeedback(distance_remaining=remaining))
    sink.settle(settled_outcome(GoalStatus.SUCCEEDED, RESULT))

    seen = [cast("Any", feedback).distance_remaining async for feedback in handle]

    assert seen == [2.0, 1.0]


async def test_awaiting_feedback_on_an_ended_goal_raises_rather_than_hanging() -> None:
    handle, sink, _ = opened()
    sink.settle(settled_outcome(GoalStatus.ABORTED, RESULT))

    with pytest.raises(GoalClosedError):
        await handle.receive_feedback()


async def test_the_result_is_awaited_once_and_readable_afterwards() -> None:
    handle, sink, _ = opened()
    sink.settle(settled_outcome(GoalStatus.SUCCEEDED, RESULT))

    outcome = await handle.result()

    assert outcome.result == RESULT
    assert handle.outcome is outcome


async def test_the_first_ending_wins() -> None:
    # A cancel answered after the server aborted must not rewrite the verdict.
    handle, sink, _ = opened()
    sink.settle(settled_outcome(GoalStatus.ABORTED, RESULT))
    sink.settle(settled_outcome(GoalStatus.CANCELED, None))

    assert (await handle.result()).aborted


async def test_cancel_asks_once_and_says_nothing_about_the_ending() -> None:
    handle, _sink, asked = opened()

    await handle.cancel()

    assert asked == ["cancel"]
    assert handle.outcome is None


async def test_cancelling_a_goal_that_has_already_ended_asks_nothing() -> None:
    handle, sink, asked = opened()
    sink.settle(settled_outcome(GoalStatus.SUCCEEDED, RESULT))

    await handle.cancel()

    assert asked == []


async def test_closing_a_running_goal_settles_it_rather_than_leaving_a_waiter() -> None:
    handle, _sink, asked = opened()

    await handle.close()
    await handle.close()

    assert asked == ["release"]  # idempotent, and it never commands anything
    assert (await handle.result()).canceled


# -- the two pure default mappings ---------------------------------------------------------------


def test_the_default_goal_builder_is_a_pure_function_of_the_values() -> None:
    values = {"Target.X": 1.0, "Target.Y": 2.0, "Target.Z": 3.0, "Label": "kitchen"}

    assert default_goal_from(fixture_messages.NavigateGoal, values, context(fixture_messages.NavigateGoal)) == GOAL


def test_the_default_result_mapping_is_a_pure_function_of_the_message() -> None:
    read = default_result_to(RESULT, context(fixture_messages.NavigateResult))

    assert read == {"TotalDistance": 9.5, "Waypoints": 4}


def test_the_two_defaults_are_each_other_s_inverse() -> None:
    goal_context = context(fixture_messages.NavigateGoal)
    values = default_result_to(GOAL, goal_context)

    assert default_goal_from(fixture_messages.NavigateGoal, values, goal_context) == GOAL


def _one_value(_message_type: Any, context: ProjectionContext) -> tuple[PlanVariable, ...]:
    """Render a Voltage as one Double named for the field it sits in."""
    return (PlanVariable(name="", source=context.path, initial=0.0, variant_type=ua.VariantType.Double),)


def _read_one_value(message: Any, _context: Any) -> dict[str, object]:
    return {"": message.data}


def _build_from_one_value(message_type: Any, values: Any, _context: Any) -> Any:
    return message_type(data=float(values[""]))


def test_a_curated_projection_is_what_the_defaults_render_through() -> None:
    # The defaults are the Projection engine and not a second rendering, so curating a type
    # changes what a client writes and what the goal carries, with no change to either function
    # -- and it does so where the type is *nested*, which is what type-keyed means.
    collapsed = Functions(
        "fixture_msgs/msg/Voltage",
        shaped_by=_one_value,
        decoded_by=_read_one_value,
        encode=_build_from_one_value,
    )
    registry = DEFAULT_REGISTRY.extended([collapsed])
    goal_context = context(fixture_messages.ChargeGoal, registry)

    built = default_goal_from(fixture_messages.ChargeGoal, {"Limit": 4.0}, goal_context)

    assert built == fixture_messages.ChargeGoal(limit=fixture_messages.Voltage(data=4.0))
    assert default_result_to(built, goal_context) == {"Limit": 4.0}
