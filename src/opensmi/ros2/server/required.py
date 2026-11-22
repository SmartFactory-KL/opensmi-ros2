# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Required Bindings: the StartupSkill's contract, and what a client sees at startup.

A Binding marked ``required=True`` means that the StartupSkill of the machinery item that declared
it does not reach ``Running`` until the Binding is Satisfied, which means its ROS2 counterpart is
present. If its deadline passes first, the Skill halts and opensmi's own supervision reports the
Machine ``OutOfService``. Satisfied is presence, never first data: a publisher that has
not spoken satisfies a Binding. :func:`satisfaction_of` holds the answer per kind, and both the
StartupSkill and the introspection Component's provenance report use it, so they cannot disagree.

Nothing waits before the endpoint opens. Every ``_init`` creates its nodes and returns, and the wait
happens inside ``_handle_starting``, so the Skill is observably ``Starting`` meanwhile. The Machine
and Component mixins add :class:`ROS2StartupSkill` themselves, because opensmi does not require a
StartupSkill; its starting handler waits for the declarations first and asks every child for its
verdict instead of stopping at the first.

Every Binding fires :data:`REQUIRED_BINDING_ABSENT` at its own deadline, and the halt at the last
one names every Binding that missed. A Binding lost after startup is reported
(:data:`REQUIRED_BINDING_LOST`) but never halts, so a restarted sensor node cannot tear a robot down
mid-goal; recovery is ``Reset`` plus ``Start``, and the check re-runs on every start. A normal boot
is quiet: no event when a Binding becomes satisfied, and none about a Binding that is not required.
The satisfaction facts are polled every :data:`CHECK_INTERVAL` seconds, since the kinds learn
presence in different ways and share no event stream.
"""

from __future__ import annotations

import asyncio
import contextlib
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Final, cast

from opensmi.core import SkillState
from opensmi.core.errors import OpenSmiError, OpenSmiRuntimeError
from opensmi.server.base_startup_skill import BaseStartupSkill

from opensmi.ros2.runtime.graph import EMPTY_GRAPH
from opensmi.ros2.server.access import SKILL_ACCESS_LEVEL
from opensmi.ros2.server.bindings import (
    ActionBinding,
    KeyedBinding,
    LifecycleBinding,
    MethodBinding,
    MonitoringBinding,
    ROS2ParameterBinding,
    TFBinding,
    is_required,
)
from opensmi.ros2.server.config import ROS2Config
from opensmi.ros2.server.errors import DeclarationError
from opensmi.ros2.server.events import Severity
from opensmi.ros2.server.lifecycle import RESTING

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable, Sequence

    from opensmi.server.base_machinery_item import BaseMachineryItem

    from opensmi.ros2.runtime.graph import GraphSnapshot, GraphWatch
    from opensmi.ros2.server.bindings import Binding

__all__ = [
    "CHECK_INTERVAL",
    "COMPONENT_NOT_RUNNING",
    "LOSS_INTERVAL",
    "REQUIRED_BINDING_ABSENT",
    "REQUIRED_BINDING_LOST",
    "Prerequisite",
    "ROS2StartupSkill",
    "StartupGate",
    "close_startup_skills",
    "prerequisites_of",
    "satisfaction_of",
    "wait_for",
]

REQUIRED_BINDING_ABSENT: Final = "REQUIRED_BINDING_ABSENT"
"""What a required Binding reports when its own deadline passed and it was still not Satisfied.

One event per Binding, at its own deadline. It is also the error code the halt carries on opensmi's
own Notification.
"""

REQUIRED_BINDING_LOST: Final = "REQUIRED_BINDING_LOST"
"""What a required Binding reports when it was Satisfied after startup and then was not.

A warning, since nothing is halted. Reported once per loss: a sensor that flaps twice is reported
twice. Coming back is not reported.
"""

COMPONENT_NOT_RUNNING: Final = "COMPONENT_NOT_RUNNING"
"""The error code a Machine's halt carries when a Component's StartupSkill did not come up.

It is not a BindingEvent, since its subject is a Component. The fault text names every Component
that did not reach ``Running``.
"""

CHECK_INTERVAL: Final = 0.05
"""Seconds between two readings of anything a starting StartupSkill is waiting on.

That is the satisfaction facts and a child Skill's ``CurrentState``. It is the granularity of "the
robot came up" for a client watching ``CurrentState``; each Binding's deadline is its own.
"""

LOSS_INTERVAL: Final = 1.0
"""Seconds between two readings of whether a Binding is still Satisfied, once nothing waits on it."""


@dataclass(frozen=True, slots=True, eq=False)
class Prerequisite:
    """One required Binding, as the wait sees it: a deadline and a question to ask.

    :func:`prerequisites_of` resolves the kinds once, so the wait is a loop over these. Compared by
    identity, because two Components may carry Bindings of the same name.

    :ivar binding: The Binding's name, which is what a ``BindingEvent`` says this is about.
    :ivar path: The ROS2 topic, service, action or node path it names.
    :ivar timeout: How many seconds from the start of the wait its deadline is.
    :ivar satisfied: Whether its ROS2 counterpart is present, asked now. Cheap and synchronous.
    :ivar reads_the_graph: Whether the answer comes from the Runtime's graph facet, which has to be
        watched for it to be read at all.
    """

    binding: str
    path: str
    timeout: float
    satisfied: Callable[[], bool]
    reads_the_graph: bool = False


def prerequisites_of(machinery_item: Any) -> tuple[Prerequisite, ...]:
    """Return what one machinery item's StartupSkill waits for, one entry per required Binding.

    :param machinery_item: The Component or Machine whose declarations these are. One without
        Bindings, such as every Machine, answers with nothing.
    :raises DeclarationError: When a required Binding's kind has no answer to what Satisfied means
        for it.
    """
    bindings: Sequence[Binding] = getattr(machinery_item, "ros2_bindings", ())
    required = [binding for binding in bindings if is_required(binding)]
    if not required:
        return ()
    default = _deployment_deadline(machinery_item)
    return tuple(_prerequisite(machinery_item, binding, default) for binding in required)


def satisfaction_of(machinery_item: Any, binding: Any) -> Callable[[], bool] | None:
    """Return how to ask whether one Binding is Satisfied, or ``None`` when the word does not apply.

    Nothing is asked here, so this works while the Component is still being built. ``None`` means
    the kind has no presence to read, as with a publishing Binding (we are the publisher). The
    probe reads through ``getattr``, so an item that is not one of ours answers absent.

    ====================== ======================================================================
    Monitoring             The subscription's publisher count, off its DDS match events. A Binding
                           that is not bound yet is not satisfied.
    Keyed                  The subscription's publisher count, never membership.
    Method                 The service is on the Runtime's last graph snapshot. This reads the
                           snapshot and starts no poll, so it answers absent where nothing watches
                           the graph; the StartupSkill's wait holds a watch open while it asks.
    ROS2 parameter         Every declared name is confirmed present on the node, read off
                           the parameter mirror.
    Action                 The action is on the Runtime's last graph snapshot, like a service.
    Lifecycle              A publisher on each declared node's ``transition_event``.
    TF                     Something other than this server publishes on a transform topic, never a
                           particular frame.
    ====================== ======================================================================
    """
    if isinstance(binding, MonitoringBinding):
        return lambda: bool(getattr(machinery_item, "ros2_monitoring_presence", {}).get(binding.name))
    if isinstance(binding, KeyedBinding):
        return lambda: bool(getattr(machinery_item, "ros2_keyed_presence", {}).get(binding.name))
    if isinstance(binding, MethodBinding):
        return lambda: binding.path in _graph_of(machinery_item).services
    if isinstance(binding, ROS2ParameterBinding):
        return lambda: _every_name_is_confirmed(machinery_item, binding)
    if isinstance(binding, ActionBinding):
        return lambda: binding.path in _graph_of(machinery_item).actions
    if isinstance(binding, LifecycleBinding):
        return lambda: _every_node_is_there(machinery_item, binding)
    if isinstance(binding, TFBinding):
        return lambda: bool(getattr(machinery_item, "ros2_transform_presence", {}).get(binding.name))
    return None


def _reads_the_graph(binding: Any) -> bool:
    """Return whether a kind's presence is read off the graph snapshot, which has to be watched."""
    return isinstance(binding, (MethodBinding, ActionBinding))


def _prerequisite(machinery_item: Any, binding: Any, default: float) -> Prerequisite:
    """Return one declaration as the wait sees it, with the probe its kind answers through.

    :raises DeclarationError: When the Binding is required and its kind has no answer to what
        Satisfied means for it.
    """
    satisfied = satisfaction_of(machinery_item, binding)
    if satisfied is None:
        message = (
            f"the Binding {binding.name!r} is marked required and its kind ({type(binding).__name__}) has no "
            f"answer to what Satisfied means for it; every kind that can be required answers in satisfaction_of"
        )
        raise DeclarationError(message)
    return Prerequisite(
        binding=binding.name,
        path=binding.path,
        timeout=binding.startup_timeout if binding.startup_timeout is not None else default,
        satisfied=satisfied,
        reads_the_graph=_reads_the_graph(binding),
    )


def _deployment_deadline(machinery_item: Any) -> float:
    """Return the default deadline of the deployment one machinery item is part of.

    An item outside a Machine of ours gets the :class:`~opensmi.ros2.server.config.ROS2Config`
    default.
    """
    config: Any = getattr(machinery_item, "ros2_config", None) or ROS2Config()
    return float(config.startup_timeout)


def _every_name_is_confirmed(machinery_item: Any, binding: ROS2ParameterBinding) -> bool:
    """Return whether every ROS2 parameter one declaration names is confirmed on its node.

    A Binding whose mirror is not built yet (a Component mid-init) answers with its node path, so it
    is not satisfied.
    """
    unconfirmed: Any = getattr(machinery_item, "ros2_unconfirmed_parameters", {})
    return unconfirmed.get(binding.name, (binding.path,)) == ()


def _every_node_is_there(machinery_item: Any, binding: LifecycleBinding) -> bool:
    """Return whether every node one lifecycle declaration follows is on the graph.

    Asked of the Component's own StartupSkill, which follows the nodes.
    """
    try:
        startup: Any = machinery_item.skill_set[BaseStartupSkill.NAME]
    except (AttributeError, KeyError):
        return False
    presence: Any = getattr(startup, "node_presence", {})
    return all(presence.get(path, False) for path in binding.observed)


def _graph_of(machinery_item: Any) -> GraphSnapshot:
    """Return the Runtime's last graph snapshot, or an empty one without a Runtime."""
    runtime = getattr(machinery_item, "runtime", None)
    return EMPTY_GRAPH if runtime is None else runtime.graph.snapshot


async def wait_for(
    prerequisites: Sequence[Prerequisite],
    *,
    report: Callable[[Prerequisite], Awaitable[None]],
    interval: float = CHECK_INTERVAL,
) -> tuple[Prerequisite, ...]:
    """Wait until every prerequisite is Satisfied or its own deadline has passed.

    There is no fail-fast and no shared deadline. A prerequisite that misses its deadline is
    reported at once and counted as missed, even if it arrives later. A prerequisite that is met is
    dropped and not asked again; if it goes away later, the gate's watch reports it as lost.

    :param prerequisites: What to wait for. Empty returns at once.
    :param report: How to announce one that missed its deadline, awaited at that deadline.
    :param interval: Seconds between two readings.
    :return: The prerequisites that missed their deadlines, in the order they did. Empty means the
        wait was met.
    """
    if not prerequisites:
        return ()
    loop = asyncio.get_running_loop()
    deadlines = {each: loop.time() + each.timeout for each in prerequisites}
    pending = list(prerequisites)
    missed: list[Prerequisite] = []
    while pending:
        now = loop.time()
        waiting: list[Prerequisite] = []
        for each in pending:
            if each.satisfied():
                continue
            if deadlines[each] <= now:
                missed.append(each)
                await report(each)
                continue
            waiting.append(each)
        pending = waiting
        if pending:
            await asyncio.sleep(max(0.0, min(interval, min(deadlines[each] for each in pending) - loop.time())))
    return tuple(missed)


class StartupGate:
    """One machinery item's required Bindings, as the wait and the watch that follow from them.

    Held by its :class:`ROS2StartupSkill`. :meth:`wait` is called from the starting handler and
    :meth:`close` from the halting one and from shutdown. The graph watch and the loss watcher exist
    only between them.
    """

    def __init__(self, machinery_item: BaseMachineryItem) -> None:
        """Resolve what this item will wait for, and refuse an item that could not report it.

        This is resolved at init, not at the first start, because both failures are declaration
        errors. Only how to ask is resolved here, never the answer.

        :raises DeclarationError: When a required Binding's kind has no answer to what Satisfied
            means for it, or when the item cannot raise a BindingEvent.
        """
        self._item = machinery_item
        self._emit: Any = getattr(machinery_item, "emit_binding_event", None)
        self._prerequisites = prerequisites_of(machinery_item)
        self._watch: GraphWatch | None = None
        self._watcher: asyncio.Task[None] | None = None
        self._lost: set[Prerequisite] = set()
        if self._prerequisites and self._emit is None:
            message = (
                f"{type(machinery_item).__name__} declares a required Binding and cannot raise a "
                f"BindingEvent, which is where a prerequisite that never arrived is reported; a "
                f"machinery item carrying ROS2 Bindings mixes in ROS2ComponentMixin"
            )
            raise DeclarationError(message)

    @property
    def prerequisites(self) -> tuple[Prerequisite, ...]:
        """Return what this item's StartupSkill waits for, one entry per required Binding."""
        return self._prerequisites

    @property
    def watching(self) -> bool:
        """Return whether the gate is still following its prerequisites after a startup."""
        return self._watcher is not None

    async def wait(self) -> None:
        """Wait for every required Binding, then keep an eye on them.

        It re-runs on every start, so a resume after a crashed stack cannot claim ``Running`` on an
        old check.

        :raises OpenSmiRuntimeError: When anything missed its deadline, carrying
            :data:`REQUIRED_BINDING_ABSENT` as its code.
        """
        await self.close()
        prerequisites = self._prerequisites
        if not prerequisites:
            return
        try:
            await self._open_the_graph(prerequisites)
            missed = await wait_for(prerequisites, report=self._report_absent)
        except BaseException:
            await self.close()
            raise
        if missed:
            await self.close()
            raise OpenSmiRuntimeError(_verdict(missed), error_code=REQUIRED_BINDING_ABSENT)
        self._watcher = asyncio.create_task(self._follow(prerequisites), name="ros2-required-bindings")

    async def close(self) -> None:
        """Stop watching and let go of the graph. Idempotent.

        Called on the way to ``Halted`` and again at shutdown, for a Component torn down while the
        process goes on serving.
        """
        watcher, self._watcher = self._watcher, None
        if watcher is not None:
            watcher.cancel()
            await asyncio.gather(watcher, return_exceptions=True)
        if self._watch is not None:
            await self._watch.close()
            self._watch = None
        self._lost = set()

    async def _open_the_graph(self, prerequisites: Sequence[Prerequisite]) -> None:
        """Start reading the graph, if anything here can only be answered from it.

        The watch keeps the facet polling during the wait, and the read stops a service that is
        already there from costing a poll interval. Machines with only topic, parameter or node
        Bindings open neither.
        """
        if not any(each.reads_the_graph for each in prerequisites):
            return
        runtime = cast("Any", self._item).runtime  # a Component with Bindings always has one
        self._watch = runtime.graph.watch()
        with contextlib.suppress(Exception):
            # Never fatal: the poll behind the watch retries, and the wait has its own deadline.
            await runtime.graph.refresh()

    async def _follow(self, prerequisites: Sequence[Prerequisite]) -> None:
        """Report a prerequisite that goes away after startup, and never act on it."""
        while True:
            await asyncio.sleep(LOSS_INTERVAL)
            for each in prerequisites:
                if each.satisfied():
                    self._lost.discard(each)
                    continue
                if each in self._lost:
                    continue
                self._lost.add(each)
                await self._report(
                    each,
                    REQUIRED_BINDING_LOST,
                    Severity.WARNING,
                    f"{each.path} was there when {self._item.name} started and is not any more; a "
                    f"required Binding lost after startup is reported and never halts, because "
                    f"halting would let a restarted node tear this robot down mid-goal",
                )

    async def _report_absent(self, prerequisite: Prerequisite) -> None:
        """Announce one prerequisite that missed its own deadline."""
        await self._report(
            prerequisite,
            REQUIRED_BINDING_ABSENT,
            Severity.ERROR,
            f"{prerequisite.path} is required by {self._item.name} and was still not satisfied "
            f"{prerequisite.timeout} seconds after its StartupSkill began starting; what is "
            f"waited for is presence -- a publisher, a discovered server, a node answering -- and "
            f"never first data, so this is absent rather than silent",
        )

    async def _report(self, prerequisite: Prerequisite, code: str, severity: Severity, text: str) -> None:
        """Fire one BindingEvent, and never let the reporting be the fault."""
        try:
            await self._emit(
                binding=prerequisite.binding,
                path=prerequisite.path,
                code=code,
                severity=severity,
                text=text,
            )
        except Exception:
            self._item.logger.exception(
                "Could not fire a BindingEvent about a required Binding",
                binding=prerequisite.binding,
                path=prerequisite.path,
                code=code,
            )


def _verdict(missed: Sequence[Prerequisite]) -> str:
    """Return the fault text of a startup that ran out of time, naming every cause."""
    listed = "; ".join(f"{each.binding} ({each.path}) after {each.timeout}s" for each in missed)
    return (
        f"{listed}. Every required Binding is waited for to its own deadline and the verdict is "
        f"rendered at the last one, so this names every prerequisite that was not met rather "
        f"than the first; recovery is Reset and Start, which checks again"
    )


async def close_startup_skills(machinery_item: Any) -> None:
    """Release every StartupSkill of ours on one machinery item. Idempotent.

    A Machine's and a Component's shutdown both call it, to release the watch behind a required
    Binding and the lifecycle Skill's subscriptions.
    """
    for skill in getattr(machinery_item, "skill_set", ()):
        if isinstance(skill, ROS2StartupSkill):
            await skill.close_startup()


class ROS2StartupSkill(BaseStartupSkill):
    """The StartupSkill this package gives a machinery item that has not got a narrower one.

    It stays ``Running`` once started. It waits for this item's required Bindings through the gate
    and starts each child's StartupSkill and waits for it to run, because opensmi's own starting
    handler stops at the first child that did not come up. A Machine gets one unless it
    registers its own. A Component gets one when it declares a required Binding, and
    :class:`opensmi.ros2.server.startup.ROS2LifecycleStartupSkill` when it declares a lifecycle one.
    It is not suspendable, because opensmi's StartupSkill never is.
    """

    def __init__(self, *, minimum_access_level: int = SKILL_ACCESS_LEVEL, **kwargs: Any) -> None:
        """Take the access level and hand opensmi the rest; the gate is built once there is a parent.

        The default is the ROS2 settings' ``skill_access_level`` default, not opensmi's 2, because
        the orchestrator must be able to start a machine with no operator present.

        :param minimum_access_level: The level ``Start``, ``Halt`` and ``Reset`` need.
        """
        super().__init__(minimum_access_level=minimum_access_level, **kwargs)
        self._startup_gate: StartupGate | None = None
        # Trigger tasks fired at children, held so the loop does not collect one mid-transition.
        self._fired: set[asyncio.Task[None]] = set()

    @property
    def gate(self) -> StartupGate:
        """Return the required Bindings this Skill answers for, as the gate that waits on them."""
        assert self._startup_gate is not None, "the gate is built when the Skill is added to its item"
        return self._startup_gate

    async def _init(self) -> None:
        """Let opensmi build the Skill, then resolve what its item declared required.

        :raises DeclarationError: When the item declares a required Binding and cannot raise a
            BindingEvent.
        """
        await super()._init()
        self._startup_gate = StartupGate(self.parent)

    async def close_startup(self) -> None:
        """Release everything this Skill holds outside the state machine. Idempotent.

        The declaring item's shutdown calls it; a subclass that holds more releases it here and
        calls up. opensmi's supervision task is stopped too, since it has no shutdown of its own and
        would fail on nodes that are gone.
        """
        if self._startup_gate is not None:
            await self._startup_gate.close()
        status, self._handle_status_change_task = self._handle_status_change_task, None
        if status is not None:
            status.cancel()
            await asyncio.gather(status, return_exceptions=True)

    async def _handle_starting(self) -> None:
        """Wait for this item's prerequisites, command its own robot, then bring its children up.

        The prerequisites come first: a managed node that is not on the graph cannot be activated,
        and the Skill should report that it is waiting for the node, not that the node did not
        answer. Each child waits for its own prerequisites in its own handler.
        """
        await self.gate.wait()
        await self._start_the_ros2_side()
        await self._reset_children()
        await self._start_children()

    async def _start_the_ros2_side(self) -> None:
        """Command whatever this item owns on the ROS2 side. The lifecycle Skill activates its nodes here."""

    async def _handle_running(self) -> None:
        """Do nothing: a started StartupSkill has nothing left to tick."""

    async def _handle_halting(self) -> None:
        """Stop answering for the prerequisites, then halt the Components and the other Skills.

        This is opensmi's own halting handler, minus this Skill. opensmi's ``halt_skills`` waits for
        every Skill of the item to be ``Halted`` and has not got an exception for the caller, so
        called from here it would wait for the halt it is part of and fail after its timeout.
        """
        if self._startup_gate is not None:
            await self._startup_gate.close()
        await self.parent.halt_components()
        await self.parent.halt_skills([skill for skill in self.parent.skill_set if skill is not self])

    async def _reset_children(self) -> None:
        """Run opensmi's own starting handler, without letting one child be the verdict.

        Its reset raises at the first Component that halted. That is swallowed here, because
        :meth:`_start_children` names every child that did not come up, and the rest of opensmi's
        housekeeping still has to run.
        """
        try:
            await super()._handle_starting()
        except (OpenSmiError, TimeoutError) as error:
            self.logger.debug(
                "A child did not come up while opensmi was resetting it; the verdict is "
                "rendered once every child has been asked",
                fault=str(error),
            )
            await self.parent.reset_skills()

    async def _start_children(self) -> None:
        """Start every child's StartupSkill and wait for it to reach ``Running``.

        :raises OpenSmiRuntimeError: When any child's StartupSkill did not reach ``Running``,
            carrying :data:`COMPONENT_NOT_RUNNING` and naming every one of them.
        """
        faults = [fault for child in self._children() if (fault := await self._bring_up(child)) is not None]
        if not faults:
            return
        message = (
            f"{'; '.join(faults)}. A Machine is Running when its Components are, so a Component "
            f"that did not come up halts this Skill rather than being started around; the "
            f"Component's own StartupSkill and its events say which prerequisite it was waiting for"
        )
        raise OpenSmiRuntimeError(message, error_code=COMPONENT_NOT_RUNNING)

    def _children(self) -> tuple[Any, ...]:
        """Return the child machinery items that have a StartupSkill of their own, in order."""
        return tuple(child for child in self.parent.components if BaseStartupSkill.NAME in child.skill_set)

    async def _bring_up(self, child: Any) -> str | None:
        """Bring one child's StartupSkill to ``Running``, and say why if it did not get there.

        Each step is skipped where it has already happened, since resetting a running Component
        would tear down a working robot.

        :return: ``None`` when the child is running, or a phrase naming it and where it stopped.
        """
        startup: Any = child.skill_set[BaseStartupSkill.NAME]
        deadline = self._deadline_for(child)
        if startup.current_state is not SkillState.RUNNING:
            if startup.current_state is not SkillState.READY:
                await self._reach(startup, SkillState.READY, deadline, startup.reset(), f"reset-{child.name}")
            if startup.current_state is SkillState.READY:
                await self._reach(startup, SkillState.RUNNING, deadline, startup.start(), f"start-{child.name}")
        if startup.current_state is SkillState.RUNNING:
            return None
        return f"the Component {child.name} is {startup.current_state.localized_text.Text} and not Running"

    async def _reach(self, skill: Any, target: SkillState, deadline: float, trigger: Any, label: str) -> None:
        """Fire one trigger at another Skill and watch until it settles, wherever that is.

        The trigger is fired from a task of its own and watched, not awaited: ``AsyncMachine``
        cancels the tasks processing a transition when the next one for that model begins and
        swallows the ``CancelledError``. opensmi's ``wait_for_state`` is not used, because it raises
        on ``Halted`` only if the Skill was not already there, so a child that halted early would
        be reported as a timeout. The caller reads ``CurrentState`` for the verdict.
        """
        task: asyncio.Task[None] = asyncio.create_task(trigger, name=f"ros2-startup-{self.name}-{label}")
        self._fired.add(task)
        task.add_done_callback(self._fired.discard)
        loop = asyncio.get_running_loop()
        until = loop.time() + deadline
        started_from = skill.current_state
        moved = False
        while loop.time() < until:
            await asyncio.sleep(CHECK_INTERVAL)
            state = skill.current_state
            if state is target:
                return
            if task.done() and not task.cancelled() and task.exception() is not None:
                # The trigger was refused and nothing will move, so do not wait out the deadline.
                return
            moved = moved or state is not started_from
            if moved and state in RESTING:
                return

    def _deadline_for(self, child: Any) -> float:
        """Return how long this Skill waits for one child: its longest required deadline plus ``startup_timeout``."""
        slack = _deployment_deadline(self.parent)
        return slack + max((each.timeout for each in prerequisites_of(child)), default=0.0)
