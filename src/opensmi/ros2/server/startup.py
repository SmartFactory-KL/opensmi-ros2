# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The StartupSkill a lifecycle node drives, and that drives the lifecycle node back.

The Binding is the StartupSkill. It derives from ``BaseStartupSkill``, which carries opensmi's
status supervision. ``Reset`` configures every node that is not configured yet
(``Halted`` to ``Ready`` is ``unconfigured`` to ``inactive``) and is refused while any node is
Terminal. ``Start`` activates every node that is not active. ``Running`` means every node
is active, and nothing ticks there. ``Halt`` takes back what this Skill did: ``deactivate`` for a
node it activated, ``cleanup`` for one it configured, never ``shutdown`` (``finalized`` is terminal,
so the Skill could never be reset), and nodes that were already active are left as found. The Skill
is not suspendable, because opensmi's ``BaseStartupSkill`` is not.

Following is downward only. A transition somebody else caused moves the Skill toward what the nodes
say and stops where the client has taken it: an external ``cleanup`` or ``deactivate`` halts the
Skill, an external ``activate`` does not start it. Recovery is the operator's ``Reset``
and ``Start``. Handlers command whoever fired the trigger, so a Skill halted by one external
``cleanup`` also takes back the other nodes it brought up. This is safe because a Driver sends
nothing to a node that is already where the trigger wants it, which makes every handler idempotent.
Transitions that arrive while a handler is commanding are its own: they are recorded and move
nothing, and one settle runs when the handler is done.

The state correspondence is in :mod:`opensmi.ros2.server.lifecycle`, the services in
:mod:`opensmi.ros2.server.drivers`. A refused transition has no BindingEvent: opensmi's failure path
reports it with :data:`TRANSITION_REFUSED`.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, Any, Final, cast

from asyncua import ua
from asyncua.ua.status_codes import StatusCodes
from opensmi.core import SkillState
from opensmi.core.errors import OpenSmiRuntimeError
from opensmi.core.ua_node_util import write_value
from opensmi.server.mixins import MonitoringMixin
from opensmi.server.user import INTERNAL_USER

from opensmi.ros2.server.browse_names import browse_name
from opensmi.ros2.server.errors import DeclarationError
from opensmi.ros2.server.events import Severity
from opensmi.ros2.server.lifecycle import (
    PRIMARY_STATES,
    RESTING,
    LifecycleNodeState,
    LifecycleNodeTransition,
    aggregate,
    resting_skill_state,
    skill_state_for,
    trigger_toward,
)
from opensmi.ros2.server.mutation import add_plan_variable, ensure_groups, require_unused_name
from opensmi.ros2.server.projection import NodeLevel, PlanVariable
from opensmi.ros2.server.publisher import binding_event_emitter
from opensmi.ros2.server.required import ROS2StartupSkill

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable, Sequence

    from asyncua.common.node import Node
    from opensmi.server.mixins.skill_logic_mixins import TRIGGER
    from opensmi.server.protocols import UserAuthorization

    from opensmi.ros2.runtime.interface import RuntimeInterface
    from opensmi.ros2.server.bindings import LifecycleBinding
    from opensmi.ros2.server.drivers import LifecycleDriver

__all__ = [
    "EXTERNALLY_MOVED",
    "SETTLE_INTERVAL",
    "STATE_VARIABLE",
    "TERMINAL",
    "TRANSITION_REFUSED",
    "UNKNOWN_STATE",
    "ROS2LifecycleStartupSkill",
]

TERMINAL: Final = "TERMINAL"
"""What a lifecycle Binding reports when one of its nodes reaches ``finalized``.

Refusing ``Reset`` leaves ``CurrentState`` looking like an ordinary, resettable ``Halted``, so
without this event a client learns only by trying. Fired once per node per latch; the latch lifts
when a non-terminal state is observed on that node's path.
"""

EXTERNALLY_MOVED: Final = "EXTERNALLY_MOVED"
"""What a lifecycle Binding reports when something other than us moved one of its nodes.

``CurrentState`` says where the Skill ended up, this says that nobody asked for it. A warning, since
an ordinary robot doing an ordinary thing causes it.
"""

TRANSITION_REFUSED: Final = "TRANSITION_REFUSED"
"""The error code a halt carries when a node or a manager would not take a transition.

It is the code on opensmi's own error message, which joins every fault of one trigger into one text
on this Skill's Notification. There is no BindingEvent beside it.
"""

STATE_VARIABLE: Final = "state"
"""The leaf under which every node's lifecycle state is published in the StartupSkill's Monitoring.

A variable browses as ``<Binding>/<node path>/State``, one grouping Object per segment of the node
path, with ``state`` as its DisplayName. ``finalized`` is an ordinary Good value, since
the node is answering.
"""

UNKNOWN_STATE: Final = "unknown"
"""What a node's state variable holds before anything has been heard from that node."""

SETTLE_INTERVAL: Final = 0.1
"""Seconds an observation waits when the Skill is mid-transition and cannot act on it yet.

Neither state machine has a completion callback, so a held observation is re-offered at this
interval. It runs only while something is pending and polls the Skill, never ROS2.
"""


class ROS2LifecycleStartupSkill(MonitoringMixin, ROS2StartupSkill):
    """One Component's lifecycle declarations, as the StartupSkill they all land on.

    The declaring Component constructs it. A Component may name a subclass through
    ``startup_skill_type``. It extends :class:`opensmi.ros2.server.required.ROS2StartupSkill` with
    the managed nodes, so a Component with both required and lifecycle Bindings waits for the
    required ones first and then activates its nodes (see :meth:`_start_the_ros2_side`).
    """

    def __init__(
        self,
        *,
        bindings: Sequence[LifecycleBinding],
        runtime: RuntimeInterface,
        **kwargs: Any,
    ) -> None:
        """Build a Driver per declaration, before any node exists.

        :param bindings: Every lifecycle declaration on the Component. They share this one Skill,
            and their nodes aggregate lowest-wins.
        :param runtime: The Machine's Runtime, reached through the Component.
        :param kwargs: What :class:`~opensmi.ros2.server.required.ROS2StartupSkill` takes.
        :raises DeclarationError: When two declarations follow the same ROS2 node.
        :raises MessageTypeError: When an interface package a declared flavour needs is missing.
        """
        super().__init__(**kwargs)
        self._bindings = tuple(bindings)
        self._drivers = tuple(_driver_for(binding, runtime) for binding in self._bindings)
        self._owner = _require_one_owner_per_node(self._drivers)
        self._emit: Callable[..., Awaitable[None]] | None = None
        # What each node last said about itself, as the Skill state it means. A node nobody has
        # heard from is absent (no opinion), so a vanished node cannot tear a robot down.
        self._observed: dict[str, SkillState] = {}
        # Whether each node is on the graph, which is the answer to Satisfied for a lifecycle
        # Binding. A vanished node keeps its last state and has presence False.
        self._present: dict[str, bool] = {}
        self._terminal: dict[str, str] = {}
        # The transitions this Skill itself sent each node, which is all a halt may take back
        # .
        self._performed: dict[str, set[LifecycleNodeTransition]] = {}
        self._variables: dict[str, Node] = {}
        self._commanding = asyncio.Lock()
        self._pending = asyncio.Event()
        self._firing: TRIGGER | None = None
        # Nodes that announced a transition while a handler was commanding. Their event carries a
        # route and the read-back does not, so the read-back defers to them.
        self._announced: set[str] = set()
        self._tasks: list[asyncio.Task[None]] = []

    @property
    def bindings(self) -> tuple[LifecycleBinding, ...]:
        """Return the declarations this Skill was built from, in declaration order."""
        return self._bindings

    @property
    def drivers(self) -> tuple[LifecycleDriver, ...]:
        """Return one Driver per declaration, in the same order."""
        return self._drivers

    @property
    def observed(self) -> dict[str, SkillState]:
        """Return a copy of what each node last said about itself, as the Skill state it means."""
        return dict(self._observed)

    @property
    def node_presence(self) -> dict[str, bool]:
        """Return a copy of whether each declared node is on the graph, by node path.

        A vanished node keeps its last state, because absence is not a lifecycle state. Presence
        comes from the node's ``transition_event`` subscription and answers a required lifecycle
        Binding (:func:`opensmi.ros2.server.required.satisfaction_of`).
        """
        return dict(self._present)

    @property
    def terminal(self) -> tuple[str, ...]:
        """Return the node paths currently latched Terminal, in sorted order.

        While this is non-empty, ``Reset`` is refused.
        """
        return tuple(sorted(self._terminal))

    async def _init(self) -> None:
        """Build the Skill, give every node a state variable, and start following.

        Nothing is asked of the robot here, so a Component whose robot is off still builds.
        The first question goes out when a ``transition_event`` subscription gets a
        publisher.

        :raises DeclarationError: When the Component cannot raise a ``BindingEvent``.
        """
        await super()._init()
        self._emit = binding_event_emitter(
            self.parent,
            "a lifecycle Binding is declared on a Component that cannot raise a BindingEvent, "
            "and the Terminal event is the only thing that tells a Halted StartupSkill which can "
            "be reset from one which never can; a Component carrying ROS2 Bindings mixes in "
            "ROS2ComponentMixin",
        )
        await self._create_state_variables()
        for driver in self._drivers:
            await driver.open()
        self._start_following()

    async def close_startup(self) -> None:
        """Release the gate this Skill inherits, and then its own subscriptions. Idempotent."""
        await super().close_startup()
        await self.close_drivers()

    async def close_drivers(self) -> None:
        """Stop following and release every subscription. Idempotent."""
        tasks, self._tasks = self._tasks, []
        fired, self._fired = self._fired, set()
        for task in (*tasks, *fired):
            task.cancel()
        await asyncio.gather(*tasks, *fired, return_exceptions=True)
        for driver in self._drivers:
            await driver.close()

    async def _condition_reset_allowed(self, _user: UserAuthorization) -> bool:
        """Refuse ``Reset`` while any node behind this Skill is Terminal.

        Commanding a transition at a ``finalized`` node returns ``success=False`` on a C++ node and
        kills the process on an rclpy one.

        :raises UaStatusCodeError: ``BadInvalidState``. It is raised because a bare ``False`` would
            reach ``try_trigger`` as an unexplained ``Good``.
        """
        if not self._terminal:
            return True
        listed = ", ".join(self.terminal)
        refused = (
            f"Denied, {listed} reached the terminal lifecycle state and can never be commanded "
            f"again; the node object has to be destroyed and recreated, which is a process "
            f"restart and not something this adapter or OPC UA can do"
        )
        await self.ua_log_error(refused, code=TERMINAL)
        raise ua.UaStatusCodeError(StatusCodes.BadInvalidState)

    async def _handle_resetting(self) -> None:
        """Configure every node."""
        await self._drive("reset")

    async def _start_the_ros2_side(self) -> None:
        """Activate every node, and check they all got there.

        This is the lifecycle half of ``Starting``, run after the required Bindings were waited for.
        A required lifecycle Binding is thus waited for before its nodes are commanded: a node that
        is not on the graph does not answer, and that is a fault rather than a wait. Nodes of other
        required Bindings have their publishers by now, because a managed node creates them on
        ``configure``, which ``Reset`` has already done.
        """
        await self._drive("start")
        self._require_every_node_active()

    def _require_every_node_active(self) -> None:
        """Refuse to reach ``Running`` unless every node actually reached ``active``.

        The Driver's gate is quiet: a node that offers no ``activate`` is skipped. That is right for
        ``halt`` but not for ``start``, where a node left ``unconfigured`` would let ``Start``
        succeed while the Skill claims a robot it does not have. The read-back is checked, not the
        command.

        :raises OpenSmiRuntimeError: When any node reports anything other than ``active``.
        """
        idle = [path for path, state in self._observed.items() if state is not SkillState.RUNNING]
        if not idle:
            return
        message = (
            f"start: {', '.join(idle)} did not reach active, and Running means every node is "
            f"active; a node that offers no activate is not commanded, so one resting somewhere "
            f"else has to halt the Skill rather than be started around"
        )
        raise OpenSmiRuntimeError(message, error_code=TRANSITION_REFUSED)

    async def _handle_halting(self) -> None:
        """Take back what this Skill brought up, and let opensmi halt what it halts.

        Only transitions the Skill performed itself are rolled back, so a node that was already
        active when the Skill started is not touched. Never ``shutdown``, since
        ``finalized`` is terminal and the Skill could not be reset. opensmi's own halting runs
        whatever the robot did, and the faults are raised afterwards.
        """
        faults = await self._command("halt")
        await super()._handle_halting()
        _require_nothing_refused(faults, "halt")

    async def _drive(self, trigger: TRIGGER) -> None:
        """Command one trigger at every node, and halt this Skill if anything refused.

        :raises OpenSmiRuntimeError: When a node or a manager refused or did not answer.
        """
        _require_nothing_refused(await self._command(trigger), trigger)

    async def _command(self, trigger: TRIGGER) -> tuple[str, ...]:
        """Ask every Driver to drive its nodes toward what one trigger means, and collect faults.

        Idempotent, since each Driver gates every transition on what its nodes report as available.
        """
        faults: list[str] = []
        async with self._commanding:
            self._announced.clear()
            for driver in self._drivers:
                faults.extend(await driver.apply(trigger, self._performed))
            await self._read_back()
        self._pending.set()
        return tuple(faults)

    async def _read_back(self) -> None:
        """Ask every node where it ended up, rather than assuming the command took it there.

        A node announces the transitions we caused too, and a Skill that ignored its own commands
        would read its successful ``Reset`` as a robot that never moved. The read has no route, so
        it gives a rank but not which state at that rank. A node that announced a transition inside
        this window is skipped: its event carries a route, and overwriting it would turn somebody
        else's failed activation (``Halted``) into a routeless ``inactive`` (``Ready``).
        """
        for driver in self._drivers:
            for path in driver.paths:
                if path in self._announced:
                    continue
                state = await driver.read_state(path)
                if state is not None:
                    await self._note(path, state, resting_skill_state(state))

    def _start_following(self) -> None:
        """Start one task per node per signal (transitions and appearances), plus the settling one.

        An appearance is the subscription's own DDS match event, which is the "one state query on
        first discovery".
        """
        for driver in self._drivers:
            for path in driver.paths:
                self._spawn(self._follow_transitions(driver, path), f"transitions{path}")
                self._spawn(self._follow_appearances(driver, path), f"appearances{path}")
        self._spawn(self._settling(), "settling")

    def _spawn(self, work: Any, label: str) -> None:
        """Run one following coroutine as a task this Skill owns and cancels.

        These tasks stay out of opensmi's own task bookkeeping, which is cancelled when the Skill
        halts, because a Skill that stopped watching on halt could never see its nodes come back.
        """
        self._tasks.append(asyncio.create_task(work, name=f"ros2-lifecycle-{self.name}-{label}"))

    async def _follow_transitions(self, driver: LifecycleDriver, path: str) -> None:
        """Record every transition one node announces, reading the pair and never the field."""
        async for transition in driver.transitions(path):
            await self._note(
                path,
                transition.goal,
                skill_state_for(start=transition.start, goal=transition.goal),
                routed=True,
            )

    async def _follow_appearances(self, driver: LifecycleDriver, path: str) -> None:
        """Ask a node what state it is in each time it appears, and only then.

        A server starting against an already ``finalized`` node gets no transition event, and
        without this it would offer a ``Reset`` that cannot work. The node's
        ``transition_event`` publisher is still matched, so it can be seen.
        """
        there = False
        self._present.setdefault(path, False)
        async for publishers in driver.appearances(path):
            self._present[path] = publishers > 0
            if publishers == 0:
                # Gone: what the node last said stands, because absence is not a lifecycle state.
                there = False
                continue
            if there:
                # A second publisher on the topic is not a node arriving.
                continue
            there = True
            state = await driver.read_state(path)
            if state is None:
                continue
            await self._note(path, state, resting_skill_state(state))

    async def _note(
        self,
        path: str,
        state: LifecycleNodeState | int,
        means: SkillState | None,
        *,
        routed: bool = False,
    ) -> None:
        """Record one thing a node said about itself, and ask for the Skill to be settled.

        Transition states are not news: the Skill has its own intermediate states for them.

        :param routed: Whether this came from a ``transition_event`` and so carries the state pair.
            A routed observation outranks the routeless read-back of :meth:`_read_back`.
        """
        if state not in PRIMARY_STATES:
            return
        if routed:
            self._announced.add(path)
        await self._write_state(path, state)
        await self._latch_terminal(path, state)
        if means is not None:
            self._observed[path] = means
        self._pending.set()

    async def _latch_terminal(self, path: str, state: LifecycleNodeState | int) -> None:
        """Latch or lift one node's Terminal condition, firing the event exactly on the latch.

        Only observing another primary state on that path lifts it. The node vanishing does not,
        since the last thing known is still ``finalized``.
        """
        binding = self._owner[path]
        if state != LifecycleNodeState.FINALIZED:
            self._terminal.pop(path, None)
            return
        if path in self._terminal:
            return
        self._terminal[path] = binding.name
        await self._report(
            binding.name,
            path,
            TERMINAL,
            f"{path} reached the terminal lifecycle state; its process is alive and no transition "
            f"can move it again, so Reset is refused until a restart puts a non-terminal state on "
            f"that path",
            severity=Severity.ERROR,
        )

    async def _settling(self) -> None:
        """Move the Skill toward what its nodes say, whenever something new has been recorded.

        An observation the Skill cannot take yet (mid-transition, or a handler is commanding) is
        held and re-offered every :data:`SETTLE_INTERVAL`.
        """
        holding = False
        while True:
            try:
                await asyncio.wait_for(self._pending.wait(), timeout=SETTLE_INTERVAL if holding else None)
            except TimeoutError:
                # Nothing new arrived; offer the held observation again.
                pass
            else:
                self._pending.clear()
            holding = not await self._settle()

    async def _settle(self) -> bool:
        """Fire at most one trigger toward the aggregate, and say whether the Skill could act.

        :return: ``False`` when the Skill is not resting and the observation has to be re-offered;
            ``True`` when it acted, or when there was nothing for it to do.
        """
        if self._commanding.locked() or self._firing is not None:
            return False
        target = aggregate(self._observed.values())
        if target is None:
            return True
        current = self.current_state
        trigger = trigger_toward(current, target)
        if trigger is None:
            return current in RESTING
        binding, path = self._weakest(target)
        await self._report(
            binding.name,
            path,
            EXTERNALLY_MOVED,
            f"{path} moved to what this Skill reads as {target.localized_text.Text} without being asked "
            f"to, so the StartupSkill is following it from {current.localized_text.Text} by halting",
            severity=Severity.WARNING,
        )
        self._fire(trigger)
        # The Skill has not moved yet, so the observation is held until it has.
        return False

    def _fire(self, trigger: TRIGGER) -> None:
        """Fire one trigger from a task of its own, never from the task that decided to.

        ``AsyncMachine`` cancels the tasks processing a transition when a new one begins and
        swallows the ``CancelledError``, so a long-lived task awaiting a trigger would silently stop
        following.
        """
        self._firing = trigger
        task = asyncio.create_task(
            self._trigger_once(trigger),
            name=f"ros2-lifecycle-{self.name}-{trigger}",
        )
        self._fired.add(task)
        task.add_done_callback(self._fired.discard)

    async def _trigger_once(self, trigger: TRIGGER) -> None:
        """Offer one trigger to the state machine, tolerating a state it is not valid from.

        ``try_trigger`` answers with a status code instead of raising, so a transition arriving
        mid-handler is harmless.
        """
        try:
            await self._machine.try_trigger(cast("Any", INTERNAL_USER), trigger)
        finally:
            self._firing = None

    def _weakest(self, target: SkillState) -> tuple[LifecycleBinding, str]:
        """Return the Binding and node path that dragged the aggregate down to ``target``.

        The first in declaration order wins a tie.
        """
        path = next(path for path, state in self._observed.items() if state == target)
        return self._owner[path], path

    async def _create_state_variables(self) -> None:
        """Give every declared node one Monitoring variable holding its lifecycle state.

        Created at init from the declaration alone, so they are readable against a robot that is
        off. They hold :data:`UNKNOWN_STATE` until the node speaks, then the ROS2 label.

        :raises DuplicateVariableNameError: When two declarations claim one state variable, which is
            the same Binding name and node path. This differs from two declarations following one
            node (see :func:`_require_one_owner_per_node`).
        """
        for binding in self._bindings:
            for path in binding.observed:
                entry = PlanVariable(
                    name=browse_name(STATE_VARIABLE),
                    display_name=STATE_VARIABLE,
                    above=(
                        NodeLevel(name=binding.name, display_name=binding.name, group=True),
                        *(
                            NodeLevel(name=browse_name(segment), display_name=segment, group=True)
                            for segment in path.strip("/").split("/")
                        ),
                    ),
                    source=path,
                    initial=UNKNOWN_STATE,
                    variant_type=ua.VariantType.String,
                )
                group = await ensure_groups(self.monitoring, entry.above)
                require_unused_name(entry.name, (group,), "this StartupSkill's Monitoring")
                variable = await add_plan_variable(group, entry.name, entry, None)
                self._variables[path] = variable.ua_node

    async def _write_state(self, path: str, state: LifecycleNodeState | int) -> None:
        """Publish one node's lifecycle state as the label ROS2 spells it, with Good status."""
        await write_value(
            self._variables[path], LifecycleNodeState(int(state)).name.lower(), variant_type=ua.VariantType.String
        )

    async def _report(
        self,
        binding: str,
        path: str,
        code: str,
        text: str,
        severity: Severity = Severity.ERROR,
    ) -> None:
        """Fire one BindingEvent about one node, and never let the reporting be the fault.

        A failure to raise the event is only logged.
        """
        if self._emit is None:  # pragma: no cover -- resolved at init, before anything follows
            return
        try:
            await self._emit(binding=binding, path=path, code=code, severity=severity, text=text)
        except Exception:
            self.logger.exception(
                "Could not fire a BindingEvent about a lifecycle node",
                binding=binding,
                node=path,
                code=code,
            )


def _driver_for(binding: LifecycleBinding, runtime: RuntimeInterface) -> LifecycleDriver:
    """Return the Driver one declaration asked for: its own class, or the flavour it named."""
    driver_type: Any = binding.driver_class
    built: LifecycleDriver = driver_type(binding=binding, runtime=runtime)
    return built


def _require_one_owner_per_node(drivers: Sequence[LifecycleDriver]) -> dict[str, LifecycleBinding]:
    """Return which declaration owns each node path.

    :raises DeclarationError: When one node path appears under two declarations.
    """
    owner: dict[str, LifecycleBinding] = {}
    for driver in drivers:
        for path in driver.paths:
            existing = owner.get(path)
            if existing is not None:
                message = (
                    f"the lifecycle node {path!r} is followed by both {existing.name!r} and "
                    f"{driver.binding.name!r}; one node has one Driver, because two would "
                    f"command it twice and disagree about what it last said"
                )
                raise DeclarationError(message)
            owner[path] = driver.binding
    return owner


def _require_nothing_refused(faults: Sequence[str], trigger: str) -> None:
    """Halt the Skill when anything refused a transition, naming every cause.

    :raises OpenSmiRuntimeError: When there is at least one fault.
    """
    if not faults:
        return
    refused = f"{trigger}: {'; '.join(faults)}"
    raise OpenSmiRuntimeError(refused, error_code=TRANSITION_REFUSED)
