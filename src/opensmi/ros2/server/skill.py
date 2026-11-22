# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""An action is a finite Skill: a client starts it, watches it, and reads what it produced.

======================  ========================================================================
``Start``               Read the Parameters, build the goal, send it. ``Running`` is reached only
                        once the server has accepted the goal.
``Running``             Feedback is written to Monitoring as it arrives; the goal's ending is
                        awaited.
``Completed``           The goal succeeded. The result fields are in FinalResultData.
``Halt``                Cancel the goal. An aborted or externally cancelled goal also ends here.
``Reset``               Clear Monitoring and FinalResultData. The Parameters are left alone, so a
                        goal a client wrote before ``Reset`` is not discarded silently.
``Suspend``             Not available: ROS2 actions cannot be paused, so the Skill is never
                        suspendable and opensmi removes the method.
======================  ========================================================================

The outcome drives the state machine: ``Completed`` is success and ``Halted`` is every other
ending. FinalResultData holds only the fields of the result message. Three outcome booleans
(``succeeded``, ``aborted``, ``canceled``) can be added as a debugging aid with
:attr:`~opensmi.ros2.server.config.ROS2Config.debug_goal_outcome_booleans`.

opensmi runs the state machine and the ``_handle_*`` methods below are its hooks. ``Start``,
``Halt`` and ``Reset`` need the Skill access level (``skill_access_level``, default
:data:`~opensmi.ros2.server.access.SKILL_ACCESS_LEVEL`) and the Machine's Lock; ``Start`` also needs
a completed startup. An ``OpenSmiRuntimeError`` from a handler halts the Skill and appears as a
runtime error in the Notification of the Component, with its code as ``ErrorCode``. Any other
exception and a cancelled handler halt it too.

:func:`default_goal_from` and :func:`default_result_to` are pure functions of a message and its
Projection context. A declaration may name its own pair; a replacement that needs the instance
overrides :meth:`ROS2ActionSkill.build_goal` or :meth:`ROS2ActionSkill.result_values` on a custom
class. The containers themselves are built in :mod:`opensmi.ros2.server.containers`.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, ClassVar, Final

from asyncua import ua
from opensmi.core.errors import OpenSmiRuntimeError
from opensmi.server import BaseMachineryItem, BaseSkill, FiniteSkillLogicMixin
from opensmi.server.mixins import FinalResultDataMixin, MonitoringMixin, ParameterSetMixin, ParentMixin

from opensmi.ros2.runtime.guard import require_sendable
from opensmi.ros2.server.access import SKILL_ACCESS_LEVEL
from opensmi.ros2.server.containers import ROS2ContainerMixin, VariableContainer
from opensmi.ros2.server.mutation import add_plan_variable, require_unused_name
from opensmi.ros2.server.projection import PlanVariable, narrowed, require_invertible, shaped

if TYPE_CHECKING:
    from collections.abc import Mapping

    from opensmi.ros2.runtime.goal import Goal
    from opensmi.ros2.runtime.interface import RuntimeInterface
    from opensmi.ros2.runtime.messages import ActionTypes, Message
    from opensmi.ros2.runtime.outcomes import GoalOutcome
    from opensmi.ros2.server.bindings import ActionBinding
    from opensmi.ros2.server.config import ROS2Config
    from opensmi.ros2.server.projection import ProjectionContext

__all__ = [
    "GOAL_UNBUILDABLE",
    "OUTCOME_BOOLEANS",
    "ROS2ActionSkill",
    "default_goal_from",
    "default_result_to",
]


OUTCOME_BOOLEANS: Final = ("succeeded", "aborted", "canceled")
"""The three result booleans the opt-in debugging flag adds to FinalResultData.

They are derived from one outcome, so they cannot disagree with each other.
"""


GOAL_UNBUILDABLE: Final = "GoalUnbuildable"
"""The error code of a ``Start`` whose Parameters could not be built into a goal.

The Skill halts and the goal never reaches ROS2. The error arrives in the Component's
Notification with this code and a message naming the field and the reason.
"""


def default_goal_from(
    goal_type: type[Message],
    values: Mapping[str, object],
    context: ProjectionContext,
) -> Message:
    """Build a goal message out of what a client wrote into a Skill's Parameters.

    The Projection covering the goal type, run backwards. A pure function: no server, ``rclpy``
    or event loop.

    :raises NotInvertibleError: When a field of the goal has no value, because the selection
        removed it or no Parameter carries it. Such a field is refused instead of filled with a
        default.
    """
    return context.registry.find(goal_type.ros2.type_name).inverse_of(goal_type, values, context)


def default_result_to(result: Message, context: ProjectionContext) -> Mapping[str, object]:
    """Read a result message into the FinalResultData variables its type was shaped into."""
    return context.registry.find(type(result).ros2.type_name).decode(result, context)


class ROS2ActionSkill(
    ROS2ContainerMixin,
    ParentMixin[BaseMachineryItem],
    MonitoringMixin,
    FinalResultDataMixin,
    ParameterSetMixin,
    FiniteSkillLogicMixin,
    BaseSkill,
):
    """The finite Skill one :class:`~opensmi.ros2.server.bindings.ActionBinding` becomes.

    Constructed by the declaring Component. A declaration may name a subclass of this class to add
    behaviour; the subclass is checked when the declaration is read.
    """

    callable_kind: ClassVar[str] = "Skill"

    def __init__(
        self,
        *,
        binding: ActionBinding,
        runtime: RuntimeInterface,
        context: ProjectionContext,
        config: ROS2Config,
        minimum_access_level: int = SKILL_ACCESS_LEVEL,
    ) -> None:
        """Resolve everything about the declaration that can be wrong, before any node exists.

        A missing interface package, a pattern that matches nothing or a goal that cannot be built
        backwards fails here instead of on the first ``Start``.

        :param binding: What to build.
        :param runtime: Where the goal is sent. The Machine's, reached through the Component.
        :param context: The Projection context of the Component, with nothing selected yet. The
            Skill narrows it once per surface (goal, feedback, result).
        :param config: The deployment's ROS2 settings.
        :param minimum_access_level: The level ``Start``, ``Halt`` and ``Reset`` need.
        :raises MessageTypeError: When the action type cannot be resolved.
        :raises FieldSelectionError: When a selection pattern matches nothing.
        :raises NotInvertibleError: When the goal type reaches a Projection that cannot run
            backwards.
        """
        super().__init__(
            name=binding.name,
            minimum_access_level=minimum_access_level,
            # ROS2 has no pause; opensmi deletes the Suspend method from a Skill that says so.
            suspendable=False,
        )
        self._binding = binding
        self._runtime = runtime
        self._config = config
        self._types: ActionTypes = runtime.describe_action(binding.action_type)
        self._goal_context = narrowed(context, self._types.goal, binding.selection)
        self._feedback_context = narrowed(context, self._types.feedback, binding.feedback_selection)
        self._result_context = narrowed(context, self._types.result, binding.result_selection)
        require_invertible(self._types.goal, self._goal_context)
        self._goal_plan = shaped(self._types.goal, self._goal_context)
        self._feedback_plan = shaped(self._types.feedback, self._feedback_context)
        self._result_plan = shaped(self._types.result, self._result_context)
        # opensmi's containers exist from first use; `_init` fills these records.
        self._parameters = VariableContainer(name="ParameterSet", container=self.parameter_set)
        self._monitoring = VariableContainer(name="Monitoring", container=self.monitoring)
        self._results = VariableContainer(name="FinalResultData", container=self.final_result_data)
        self._goal: Goal | None = None

    @property
    def binding(self) -> ActionBinding:
        """Return the declaration this Skill was built from."""
        return self._binding

    @property
    def action_types(self) -> ActionTypes:
        """Return the goal, feedback and result types of the action this Skill is bound to."""
        return self._types

    @property
    def ros2_goal(self) -> Goal | None:
        """Return the goal currently in flight, or ``None`` when none is.

        Held on the Skill because opensmi cancels the running handler's task before the halting
        handler runs, so a handle kept only there would already be gone.
        """
        return self._goal

    async def _init(self) -> None:
        """Let opensmi build the Skill, create the variables, and prove the defaults can be sent.

        Building a goal from the initial Parameters must work, so a value ROS2 would abort on
        fails here, on the machine of whoever wrote the declaration.
        """
        await super()._init()
        await self._create(self._goal_plan, self._parameters, message_type=self._types.goal, writable=True)
        await self._create(self._feedback_plan, self._monitoring, message_type=self._types.feedback)
        await self._create(self._result_plan, self._results, message_type=self._types.result)
        await self._create_outcome_booleans()
        require_sendable(await self.build_goal(await self.read_parameters()))

    async def read_parameters(self) -> Mapping[str, object]:
        """Return what the ParameterSet holds now, by variable name."""
        return await self._read_values(self._parameters)

    async def read_monitoring(self) -> Mapping[str, object]:
        """Return what the Monitoring holds now, by variable name."""
        return await self._read_values(self._monitoring)

    async def read_results(self) -> Mapping[str, object]:
        """Return what the FinalResultData holds now, by variable name."""
        return await self._read_values(self._results)

    async def build_goal(self, values: Mapping[str, object]) -> Message:
        """Return the goal message to send, given what a client wrote into the Parameters.

        Override on a custom class when building the goal needs the instance. The declaration's
        ``goal_from`` is the pure-function form, and :func:`default_goal_from` is the default.
        An override must also work with the declaration's defaults, because ``_init`` calls it
        once with them.
        """
        build = self._binding.goal_from or default_goal_from
        return build(self._types.goal, values, self._goal_context)

    async def result_values(self, result: Message) -> Mapping[str, object]:
        """Return the FinalResultData values one result message carries, by variable name.

        Overridable like :meth:`build_goal`; the declaration's ``result_to`` is the pure-function
        form and :func:`default_result_to` the default.
        """
        read = self._binding.result_to or default_result_to
        return read(result, self._result_context)

    async def _handle_starting(self) -> None:
        """Read the Parameters, build the goal and send it; reach Running only if it is taken.

        A goal the server refuses halts the Skill. Parameters that cannot be built into a goal halt
        it with :data:`GOAL_UNBUILDABLE`; every refusal of a client's value is a ``ValueError``,
        which opensmi would otherwise report without an error code.
        """
        try:
            goal = await self.build_goal(await self.read_parameters())
        except ValueError as refused:
            raise OpenSmiRuntimeError(str(refused), error_code=GOAL_UNBUILDABLE) from refused
        self._goal = await self._runtime.send_goal(
            action=self._binding.path,
            action_type=self._types,
            goal=goal,
            depth=self._binding.feedback_depth,
            server_timeout=self._binding.server_timeout,
        )
        if self._goal.accepted:
            return
        outcome = self._goal.outcome
        await self._let_go()
        raise OpenSmiRuntimeError(_fault_text(outcome), error_code="GoalRejected")

    async def _handle_running(self) -> None:
        """Follow the feedback and wait for the ending, then let the ending decide the state.

        A cancel this Skill asked for never arrives here, because opensmi cancels this handler
        before running the halting one. Any non-success outcome that does arrive comes from
        outside and halts the Skill.
        """
        goal = self._goal
        if goal is None:  # pragma: no cover -- starting either sets one or halts
            gone = "the goal was gone before it was awaited"
            raise OpenSmiRuntimeError(gone, error_code="GoalMissing")
        following = asyncio.create_task(self._follow_feedback(goal), name=f"ros2-feedback-{self.name}")
        try:
            outcome = await goal.result()
            # Awaited, not cancelled: the stream ends by itself after draining the buffer, so the
            # last progress report is not lost.
            await following
        finally:
            following.cancel()
            await asyncio.gather(following, return_exceptions=True)
        # ROS2 sends a (often partial) result for aborted and cancelled goals too.
        await self._write_result(outcome)
        if not outcome.succeeded:
            raise OpenSmiRuntimeError(_fault_text(outcome), error_code=outcome.status.name)
        await self._let_go()

    async def _handle_halting(self) -> None:
        """Ask the server to cancel whatever is in flight, then let the handle go."""
        goal = self._goal
        if goal is None:
            return
        try:
            await goal.cancel()
        finally:
            await self._let_go()

    async def _handle_resetting(self) -> None:
        """Clear what the last run produced and keep the Parameters a client wrote.

        Monitoring and FinalResultData go back to their declared initial values.
        """
        await self._let_go()
        await self._reset_values(self._monitoring)
        await self._reset_values(self._results)
        await self._write_outcome_booleans(None)

    async def _follow_feedback(self, goal: Goal) -> None:
        """Write every feedback message this goal produces, until it ends."""
        async for feedback in goal:
            try:
                await self._write_feedback(feedback)
            except Exception:
                # A runtime condition is never fatal: the next message is likely fine.
                self.logger.exception(
                    "Could not write action feedback to its Monitoring variables",
                    binding=self._binding.name,
                    action=self._binding.path,
                )

    async def _write_feedback(self, feedback: Message) -> None:
        """Write one feedback message into the Monitoring variables its type was shaped into."""
        projection = self._feedback_context.registry.find(type(feedback).ros2.type_name)
        await self._write_values(self._monitoring, projection.decode(feedback, self._feedback_context))

    async def _write_result(self, outcome: GoalOutcome) -> None:
        """Write what the goal produced, and the debugging booleans if they were asked for."""
        if outcome.result is not None:
            await self._write_values(self._results, await self.result_values(outcome.result))
        await self._write_outcome_booleans(outcome)

    async def _create_outcome_booleans(self) -> None:
        """Create the three booleans if the debugging aid is on."""
        if not self._config.debug_goal_outcome_booleans:
            return
        for label in OUTCOME_BOOLEANS:
            require_unused_name(label, (self.final_result_data,), self._named(self._results))
            entry = PlanVariable(name=label, source="", initial=False, variant_type=ua.VariantType.Boolean)
            await add_plan_variable(self.final_result_data, label, entry, None)

    async def _write_outcome_booleans(self, outcome: GoalOutcome | None) -> None:
        """Write the three booleans from one outcome; ``None`` means a reset."""
        if not self._config.debug_goal_outcome_booleans:
            return
        answers = (False, False, False) if outcome is None else (outcome.succeeded, outcome.aborted, outcome.canceled)
        for label, value in zip(OUTCOME_BOOLEANS, answers, strict=True):
            if label in self.final_result_data:  # pragma: no branch -- created together with the flag
                await self._write(self.final_result_data[label], value, ua.VariantType.Boolean)

    async def _let_go(self) -> None:
        """Release the goal handle, if there is one. It never commands anything."""
        goal, self._goal = self._goal, None
        if goal is not None:
            await goal.close()


def _fault_text(outcome: GoalOutcome | None) -> str:
    """Return what a halting Skill reports about a goal, out of that goal's own faults."""
    if outcome is None:  # pragma: no cover -- an unaccepted goal always carries one
        return "the goal ended without an outcome"
    return "; ".join(outcome.faults) or "the goal did not succeed"
