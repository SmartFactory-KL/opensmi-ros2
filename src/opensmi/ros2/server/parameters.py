# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A ROS2 parameter is a Parameter: an operator tunes a value, and the node decides the write.

Each declared ROS2 parameter becomes one UA Parameter on the declaring Component's ParameterSet,
writable unless the declaration says otherwise. A client write goes out as ``set_parameters`` and a
change observed on ``/parameter_events`` comes back in, so the Parameter means the current value on
the node, whoever moved it.

A client's write is decided by ROS2 before it is stored. Every Parameter carries an async
write check, which opensmi awaits before storing and which answers the client with the status code it
raises. A value equal to what ROS2 holds is accepted at once (this is how the inward mirror passes).
The node's descriptor refuses without a round trip: ``BadNotWritable`` for a read-only parameter,
``BadTypeMismatch`` for another type, ``BadOutOfRange`` outside the declared range. Otherwise
``set_parameters`` goes to the node with a timeout below the client's own. A refusal is
``BadOutOfRange`` when the node's reason is a range and ``BadInvalidArgument`` otherwise, and no
answer in time is ``BadTimeout``. Nothing wrong is ever stored, so nothing is reverted, and writes to
one Parameter are serialised.

The node is also read and followed, and faults a client did not cause are BindingEvents. A node that
does not answer sets every Parameter of the Binding to ``BadNoCommunication`` and fires
:data:`PARAMETER_UNREACHABLE`. A declared name that is absent fires :data:`PARAMETER_ABSENT`, and one
held at another type fires :data:`PARAMETER_TYPE_MISMATCH`; both leave the Parameter
``BadNoCommunication``, since types are declared and never discovered.

Nothing waits at init. The Parameters exist from boot with typed zeros under ``BadNoCommunication``,
and the first read happens on a task. A name that is not confirmed is read again every
``retry_interval`` seconds, and nothing is read once every name is confirmed. This is not a liveness
probe: a node that vanishes silently is not noticed here, which is the StartupSkill's and a
lifecycle Binding's job.
"""

from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Final, NoReturn

from asyncua import ua
from asyncua.ua import UaStatusCodeError
from asyncua.ua.status_codes import StatusCodes
from opensmi.server.mixins import ParameterSetMixin

from opensmi.ros2.runtime.errors import FieldTypeMismatchError, FieldValueOutOfRangeError, ROS2RuntimeError
from opensmi.ros2.runtime.parameters import ParameterType, ParameterValue, absolute
from opensmi.ros2.server.errors import DeclarationError
from opensmi.ros2.server.events import ROS2EventsMixin, Severity
from opensmi.ros2.server.mutation import add_plan_variable, ended, ensure_groups, require_unused_name
from opensmi.ros2.server.projection import (
    NodeLevel,
    PlanVariable,
    join_path,
    ros2_value,
    ua_shape,
    ua_value,
)
from opensmi.ros2.server.status import status_code, status_value

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable, Iterable, Mapping

    from opensmi.server import UaVariable

    from opensmi.ros2.runtime.interface import RuntimeInterface
    from opensmi.ros2.runtime.parameters import ParameterDescription, ParameterReading, ParameterWatch
    from opensmi.ros2.server.bindings import ROS2ParameterBinding

__all__ = [
    "PARAMETER_ABSENT",
    "PARAMETER_TYPE_MISMATCH",
    "PARAMETER_UNREACHABLE",
    "ROS2ParameterMirrorMixin",
    "refusal_code",
    "ros2_parameter_value",
    "ua_parameter_shape",
]

PARAMETER_UNREACHABLE: Final = "PARAMETER_UNREACHABLE"
"""The BindingEvent code for a parameter call that did not happen at all."""

PARAMETER_ABSENT: Final = "PARAMETER_ABSENT"
"""The BindingEvent code for a declared name the node does not have."""

PARAMETER_TYPE_MISMATCH: Final = "PARAMETER_TYPE_MISMATCH"
"""The BindingEvent code for a declared name the node holds at another type."""

_ELEMENTS: Final[Mapping[ParameterType, ParameterType]] = {
    ParameterType.BOOL_ARRAY: ParameterType.BOOL,
    ParameterType.INTEGER_ARRAY: ParameterType.INTEGER,
    ParameterType.DOUBLE_ARRAY: ParameterType.DOUBLE,
    ParameterType.STRING_ARRAY: ParameterType.STRING,
}
"""What one element of each array parameter type is, for the local type check."""

_RANGED: Final[frozenset[ParameterType]] = frozenset({ParameterType.INTEGER, ParameterType.DOUBLE})
"""The parameter types a node's declared range is checked against here.

An array is left to the node, which knows whether it applies its range per element.
"""


def ua_parameter_shape(parameter_type: ParameterType) -> tuple[ua.VariantType, object]:
    """Return the OPC UA type one declared ROS2 parameter type renders as, and its typed zero.

    This is :func:`opensmi.ros2.server.projection.ua_shape` over the field type the parameter type
    is, so a ROS2 ``integer`` renders like an ``int64`` message field. It is not a Projection:
    the declaration already chose the union member. An array's zero is a fresh list on
    every call.
    """
    zero, variant_type = ua_shape(parameter_type.field_type)
    return variant_type, zero


def ros2_parameter_value(value: object, parameter_type: ParameterType) -> ParameterValue:
    """Return what a client wrote into a Parameter as the ROS2 value it stands for.

    This is :func:`opensmi.ros2.server.projection.ros2_value` over the declared field type, which
    narrows OPC UA scalar types to the exact Python types the Runtime's guard tests. A value of a
    kind the type cannot hold is left as it came, for the guard to refuse.
    """
    return ParameterValue(type=parameter_type, value=ros2_value(value, parameter_type.field_type))


def refusal_code(reason: str) -> int:
    """Return the status code a node's refusal of a write is answered with.

    ``BadOutOfRange`` when the node's reason mentions a range, ``BadInvalidArgument`` otherwise. A
    Write response carries only a status code.
    """
    return StatusCodes.BadOutOfRange if "range" in reason.lower() else StatusCodes.BadInvalidArgument


@dataclass(slots=True)
class _Parameter:
    """One declared ROS2 parameter, live: its node, what the robot last said, and what that means.

    ``ros2_name`` is what the node calls it (dotted), ``name`` its node path joined with dots.
    """

    ros2_name: str
    name: str
    parameter_type: ParameterType
    variant_type: ua.VariantType
    # The value last known from ROS2, which is also the value the variable holds.
    value: object
    writable: bool
    variable: UaVariable[Any] | None = None
    # What the node said about this name the last time it was confirmed, or nothing yet.
    description: ParameterDescription | None = None
    satisfied: bool = False
    sampled: bool = False
    # Conditions already reported, so a retry does not make an event storm. Cleared with the condition.
    reported: set[str] = field(default_factory=set)
    # Bumped by every event applied, so a read or write in flight can tell newer news landed.
    epoch: int = 0
    # Serialises everything that decides or writes this value: client writes and inward writes.
    writing: asyncio.Lock = field(default_factory=asyncio.Lock)
    # The task holding `writing` for an inward write, so its own check does not wait on the lock.
    inward: asyncio.Task[Any] | None = None

    @property
    def status(self) -> ua.StatusCode:
        """Return the status code this Parameter currently reads."""
        return status_code(satisfied=self.satisfied, sampled=self.sampled)

    @property
    def node(self) -> UaVariable[Any]:
        """Return the opensmi variable this Parameter is, which exists from init on."""
        if self.variable is None:  # pragma: no cover -- created before anything reads it
            message = f"the Parameter {self.name!r} has no variable yet"
            raise RuntimeError(message)
        return self.variable

    def holds(self, value: object) -> None:
        """Record that the node has this value, which is the only good news there is."""
        self.satisfied = True
        self.sampled = True
        self.reported.clear()
        self.value = value


@dataclass(slots=True)
class _Tuned:
    """One ROS2-parameter Binding, live: its watch, its Parameters and the tasks driving them."""

    binding: ROS2ParameterBinding
    watch: ParameterWatch
    parameters: dict[str, _Parameter] = field(default_factory=dict)
    unconfirmed: asyncio.Event = field(default_factory=asyncio.Event)
    tasks: list[asyncio.Task[None]] = field(default_factory=list)

    @property
    def confirmed(self) -> bool:
        """Return whether every declared name is confirmed present, which is Satisfied here."""
        return all(each.satisfied for each in self.parameters.values())


class ROS2ParameterMirrorMixin(ParameterSetMixin, ROS2EventsMixin):
    """Keeps a Component's ParameterSet in sync with the ROS2 nodes it declared.

    Mixed in ahead of opensmi's own class. It extends the events mixin, because a node that stops
    answering is a BindingEvent.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Start out tuning nothing, and pass every argument to opensmi's constructor."""
        super().__init__(*args, **kwargs)
        self._tuned: list[_Tuned] = []
        self._parameter_runtime: RuntimeInterface | None = None

    @property
    def ros2_unconfirmed_parameters(self) -> Mapping[str, tuple[str, ...]]:
        """Return, per Binding name, the declared ROS2 parameter names not confirmed present.

        Empty everywhere means every ROS2-parameter Binding is Satisfied. A name here is a typo in a
        declaration or a robot that is not answering; the BindingEvent says which.
        """
        return {
            tuned.binding.name: tuple(name for name, parameter in tuned.parameters.items() if not parameter.satisfied)
            for tuned in self._tuned
        }

    async def _init_parameter_mirror(
        self,
        bindings: Iterable[ROS2ParameterBinding],
        *,
        runtime: RuntimeInterface,
    ) -> None:
        """Create every declared Parameter, then start following the nodes they came from.

        Fatal on anything wrong with a declaration, such as a name written twice; what was opened is
        closed again first. No ROS2 call happens here, so the endpoint opens before anything waits
        for a robot.
        """
        self._parameter_runtime = runtime
        try:
            for binding in bindings:
                self._tuned.append(await self._open_parameters(binding, runtime))
        except BaseException:
            await self._stop_parameter_mirror()
            raise
        for tuned in self._tuned:
            await self._start_tuning(tuned)

    async def _stop_parameter_mirror(self) -> None:
        """Stop following every ROS2-parameter Binding and close its watch. Idempotent.

        It runs before the Runtime is stopped, for a Component torn down while the process goes on
        serving.
        """
        for tuned in self._tuned:
            for task in tuned.tasks:
                task.cancel()
        for tuned in self._tuned:
            for task in tuned.tasks:
                await ended(task)
            tuned.tasks = []
            await tuned.watch.close()
        self._tuned = []

    async def _open_parameters(self, binding: ROS2ParameterBinding, runtime: RuntimeInterface) -> _Tuned:
        """Create one Binding's Parameters and register interest in the node they live on.

        :raises DuplicateVariableNameError: When a name is already taken on this ParameterSet.
        """
        tuned = _Tuned(binding=binding, watch=runtime.parameters.watch(binding.path))
        try:
            for parameter, parameter_type in binding.parameters.items():
                tuned.parameters[parameter] = await self._add_parameter(tuned, parameter, parameter_type)
        except BaseException:
            await tuned.watch.close()
            raise
        return tuned

    async def _add_parameter(self, tuned: _Tuned, parameter: str, parameter_type: ParameterType) -> _Parameter:
        """Create one Parameter, with the access level its declaration asked for and its check.

        Read-only is applied here because an access level cannot be moved after ``_init``.
        A read-only Parameter still gets the write check, which answers a client's
        write ``BadNotWritable``.
        """
        binding = tuned.binding
        variant_type, zero = ua_parameter_shape(parameter_type)
        *namespaces, leaf = binding.node_path(parameter)
        *raw_namespaces, raw_leaf = parameter.split(".")
        entry = PlanVariable(
            name=leaf,
            display_name=raw_leaf,
            above=tuple(
                NodeLevel(name=name, display_name=raw, group=True)
                for name, raw in zip((binding.name, *namespaces), (binding.name, *raw_namespaces), strict=True)
            ),
            source=join_path(absolute(binding.path), parameter),
            initial=zero,
            variant_type=variant_type,
        )
        group = await ensure_groups(self.parameter_set, entry.above)
        require_unused_name(leaf, (group,), "this ParameterSet")
        live = _Parameter(
            ros2_name=parameter,
            name=entry.full_name,
            parameter_type=parameter_type,
            variant_type=variant_type,
            value=zero,
            writable=binding.writable(parameter),
        )
        # Created as an inward write: opensmi writes the typed zero through the check.
        async with live.writing:
            live.inward = asyncio.current_task()
            try:
                live.variable = await add_plan_variable(
                    group,
                    leaf,
                    entry,
                    None,
                    writable=live.writable,
                    write_check=self._write_check(tuned, live),
                )
            finally:
                live.inward = None
        return live

    async def _start_tuning(self, tuned: _Tuned) -> None:
        """Put the first status on one Binding's Parameters, and start its tasks.

        The status is written here so that a Parameter says ``BadNoCommunication`` from the moment
        it exists, whatever order the tasks run in.
        """
        for parameter in tuned.parameters.values():
            async with parameter.writing:
                await self._write_inward(parameter)
        # Nothing is confirmed yet, so the refresh task reads on its first pass.
        tuned.unconfirmed.set()
        name = tuned.binding.name
        tuned.tasks = [
            asyncio.create_task(self._follow_parameters(tuned), name=f"ros2-parameter-events-{name}"),
            asyncio.create_task(self._refresh_parameters(tuned), name=f"ros2-parameter-refresh-{name}"),
        ]

    def _write_check(self, tuned: _Tuned, parameter: _Parameter) -> Callable[[Any], Awaitable[None]]:
        """Return the async check opensmi awaits before it stores a value written to one Parameter."""

        async def decide(value: Any) -> None:
            await self._decide_write(tuned, parameter, value)

        return decide

    async def _decide_write(self, tuned: _Tuned, parameter: _Parameter, value: object) -> None:
        """Decide one write to one Parameter: return to store it, raise to answer it with a code.

        The equality test comes first, because it needs no call. It only
        counts once ROS2 has said something: before that the Parameter holds a typed zero, and a
        client writing that zero goes to the node like any other write. Inward writes pass as the
        lock holder's own.

        :raises UaStatusCodeError: With the code the client is answered with.
        """
        if parameter.inward is not None and parameter.inward is asyncio.current_task():
            if _same(value, parameter.value):
                return
            # An inward write carries the value just recorded, so this is our fault; refuse rather than deadlock.
            _refuse(StatusCodes.BadInternalError)
        if _known(parameter, value):
            return
        self._check_locally(parameter, value)
        async with parameter.writing:
            if _known(parameter, value):
                # An event or another client's write landed while this one waited.
                return
            await self._send_parameter_write(tuned, parameter, value)

    def _check_locally(self, parameter: _Parameter, value: object) -> None:
        """Refuse what the declaration and the node's descriptor already rule out, without a call.

        :raises UaStatusCodeError: ``BadTypeMismatch``, ``BadNotWritable`` or ``BadOutOfRange``.
        """
        if not _carries(value, parameter.parameter_type):
            _refuse(StatusCodes.BadTypeMismatch)
        description = parameter.description
        if not parameter.writable or (description is not None and description.read_only):
            _refuse(StatusCodes.BadNotWritable)
        declared = None if description is None else description.range
        if declared is not None and parameter.parameter_type in _RANGED and not declared.admits(value):  # type: ignore[arg-type]
            _refuse(StatusCodes.BadOutOfRange)

    async def _send_parameter_write(self, tuned: _Tuned, parameter: _Parameter, value: object) -> None:
        """Ask the node to take one value, and answer with what it said.

        Called with the Parameter's lock held. Returning means the node took the value.

        :raises UaStatusCodeError: ``BadTypeMismatch`` or ``BadOutOfRange`` when the guard behind
            the seam refused to put the value in a message; ``BadOutOfRange`` or
            ``BadInvalidArgument`` when the node refused it; ``BadTimeout`` when the node did not
            answer in time.
        """
        runtime = self._require_parameter_runtime()
        binding = tuned.binding
        sent = ros2_parameter_value(value, parameter.parameter_type)
        # An event that landed during the call is newer than the value sent (an `on_set` callback
        # may have adjusted it), so the value is not claimed then.
        epoch = parameter.epoch
        try:
            async with asyncio.timeout(binding.write_timeout):
                outcomes = await runtime.parameters.set(node=binding.path, values={parameter.ros2_name: sent})
        except FieldTypeMismatchError:
            _refuse(StatusCodes.BadTypeMismatch)
        except FieldValueOutOfRangeError:
            _refuse(StatusCodes.BadOutOfRange)
        except (TimeoutError, ROS2RuntimeError) as error:
            self.logger.warning(
                "A ROS2 node did not answer a parameter write in time",
                binding=binding.name,
                node=binding.path,
                parameter=parameter.ros2_name,
                error=str(error) or type(error).__name__,
            )
            self._lose_track(tuned, parameter)
            _refuse(StatusCodes.BadTimeout)
        outcome = outcomes.get(parameter.ros2_name)
        if outcome is None or not outcome.answered:
            # Unknown whether the value landed, as after a timeout: fetch the truth, do not guess.
            self._lose_track(tuned, parameter)
            _refuse(StatusCodes.BadTimeout)
        if not outcome.successful:
            self.logger.info(
                "A ROS2 node refused a parameter write",
                binding=binding.name,
                node=binding.path,
                parameter=parameter.ros2_name,
                reason=outcome.reason,
            )
            _refuse(refusal_code(outcome.reason))
        if parameter.epoch == epoch:
            parameter.holds(ua_value(sent.value))

    def _lose_track(self, tuned: _Tuned, parameter: _Parameter) -> None:
        """Note that what the node holds for one name is unknown, so the refresh reads it again."""
        parameter.satisfied = False
        tuned.unconfirmed.set()

    async def _follow_parameters(self, tuned: _Tuned) -> None:
        """Mirror every change the node announces, for the names this Binding declared.

        Events caused by a client's write are applied like any other, since the node just confirmed
        the value. Other names are skipped.
        """
        async for update in tuned.watch:
            for reading in update.readings:
                parameter = tuned.parameters.get(reading.name)
                if parameter is None:
                    continue
                parameter.epoch += 1
                try:
                    await self._apply(tuned, parameter, reading)
                except Exception:
                    # Runs on a task nobody awaits; one unwritable value must not stop the Binding following.
                    self.logger.exception(
                        "Could not apply a ROS2 parameter change",
                        binding=tuned.binding.name,
                        node=tuned.binding.path,
                        parameter=reading.name,
                    )

    async def _refresh_parameters(self, tuned: _Tuned) -> None:
        """Read every unconfirmed name, and keep reading until they are all confirmed.

        It is self-disabling: while every name is confirmed it waits on an event and makes no calls.
        A name going unconfirmed (absent node, timed-out write, deleted parameter) sets the event.
        Without this, a node that was off at boot and comes up silently would never be seen.
        """
        while True:
            await self._read_the_unconfirmed(tuned)
            if tuned.confirmed:
                # Nothing awaits between the clear and the wait, so no name can slip in between.
                tuned.unconfirmed.clear()
                await tuned.unconfirmed.wait()
            await asyncio.sleep(tuned.binding.retry_interval)

    async def _read_the_unconfirmed(self, tuned: _Tuned) -> None:
        """Read each name that is not confirmed present, one parameter per request.

        One request per name, since the batch call answers with an empty list when any one name is
        undeclared, which would blank every value on one typo. A transport failure ends
        the pass, as the remaining names would time out for the same reason.
        """
        for parameter in tuple(tuned.parameters.values()):
            if parameter.satisfied:
                continue
            if not await self._look(tuned, parameter):
                return

    async def _look(self, tuned: _Tuned, parameter: _Parameter) -> bool:
        """Read one name and its descriptor off the node and make its Parameter say what came back.

        :return: Whether the node answered at all. ``False`` is about the whole Binding, so a caller
            working through several names stops.
        """
        runtime = self._require_parameter_runtime()
        node = tuned.binding.path
        # An event that landed during the read is newer news and must not be overwritten.
        epoch = parameter.epoch
        try:
            reading = await runtime.parameters.get(node=node, name=parameter.ros2_name)
            if reading.present:
                parameter.description = await runtime.parameters.describe(node=node, name=parameter.ros2_name)
        except ROS2RuntimeError as error:
            await self._unreachable(tuned, str(error))
            return False
        if parameter.epoch == epoch:
            await self._apply(tuned, parameter, reading)
        return True

    async def _apply(self, tuned: _Tuned, parameter: _Parameter, reading: ParameterReading) -> None:
        """Take one look at one ROS2 parameter and make the Parameter say what it found.

        Runs under the Parameter's lock, so a client's write waiting on the node is answered first.
        """
        async with parameter.writing:
            # The node answered, so it is no longer unreachable and a later silence is news again.
            parameter.reported.discard(PARAMETER_UNREACHABLE)
            if not reading.present:
                parameter.satisfied = False
                await self._report(
                    tuned,
                    parameter,
                    PARAMETER_ABSENT,
                    Severity.ERROR,
                    f"{tuned.binding.path} has no parameter called {parameter.ros2_name!r}, so this "
                    f"Parameter has nothing to show; the names are declared, so this is a "
                    f"declaration to correct rather than a value to wait for",
                )
            elif reading.value is None:
                # Present but empty: satisfied, not sampled, so the Parameter says it is waiting.
                parameter.satisfied = True
                parameter.sampled = False
                parameter.reported.clear()
            elif reading.value.type is not parameter.parameter_type:
                parameter.satisfied = False
                await self._report(
                    tuned,
                    parameter,
                    PARAMETER_TYPE_MISMATCH,
                    Severity.ERROR,
                    f"{parameter.ros2_name!r} on {tuned.binding.path} is a "
                    f"{reading.value.type.value!r} and this Binding declares it a "
                    f"{parameter.parameter_type.value!r}; a type is declared and never discovered, "
                    f"so the Parameter stays bad rather than showing a value of the wrong kind",
                )
            else:
                parameter.holds(ua_value(reading.value.value))
            if not parameter.satisfied:
                tuned.unconfirmed.set()
            await self._write_inward(parameter)

    async def _unreachable(self, tuned: _Tuned, text: str) -> None:
        """Say that a node did not answer, and take every one of its Parameters bad with it.

        Reported once per Binding, since the fact is about the node.
        """
        for parameter in tuned.parameters.values():
            async with parameter.writing:
                parameter.satisfied = False
                await self._write_inward(parameter)
        tuned.unconfirmed.set()
        await self._report_once(
            tuned,
            PARAMETER_UNREACHABLE,
            Severity.ERROR,
            f"{tuned.binding.path} did not answer, so what it holds is no longer known: {text}",
        )

    async def _write_inward(self, parameter: _Parameter) -> None:
        """Write one Parameter's value and status, as an ordinary write through opensmi.

        Called with the Parameter's lock held, and the write passes the check as the holder's own.
        The value always travels with the status, because a status alone leaves the variant null.
        """
        parameter.inward = asyncio.current_task()
        try:
            await parameter.node.ua_node.write_value(
                status_value(parameter.value, parameter.variant_type, parameter.status),
            )
        finally:
            parameter.inward = None

    async def _report(
        self,
        tuned: _Tuned,
        parameter: _Parameter,
        code: str,
        severity: Severity,
        text: str,
    ) -> None:
        """Fire one BindingEvent about one Parameter, at most once per condition per name.

        The latch is cleared when the name reads well again. The parameter goes in the event's
        path (the node plus the dotted name), not in the text.
        """
        if code in parameter.reported:
            return
        parameter.reported.add(code)
        await self._emit(tuned.binding, code, severity, text, parameter=parameter.ros2_name)

    async def _report_once(self, tuned: _Tuned, code: str, severity: Severity, text: str) -> None:
        """Fire one BindingEvent about a whole Binding, at most once per condition.

        The latch lives on the Parameters, because every one of them clears it when it reads well.
        """
        if all(code in each.reported for each in tuned.parameters.values()):
            return
        for each in tuned.parameters.values():
            each.reported.add(code)
        await self._emit(tuned.binding, code, severity, text)

    async def _emit(
        self,
        binding: ROS2ParameterBinding,
        code: str,
        severity: Severity,
        text: str,
        parameter: str | None = None,
    ) -> None:
        """Raise one BindingEvent, and never let the reporting itself be the fault.

        :param parameter: The ROS2 parameter this is about, when it is about one rather than the
            whole node. It is joined onto the event's ``ROS2Path``.
        """
        node = absolute(binding.path)
        path = node if parameter is None else join_path(node, parameter)
        try:
            await self.emit_binding_event(
                binding=binding.name,
                path=path,
                code=code,
                severity=severity,
                text=text,
            )
        except Exception:
            self.logger.exception(
                "Could not fire a BindingEvent about a ROS2 parameter",
                binding=binding.name,
                node=path,
                code=code,
            )

    def _require_parameter_runtime(self) -> RuntimeInterface:
        """Return the Runtime this mirror was initialised with.

        :raises DeclarationError: When there is none, which is a wiring fault.
        """
        if self._parameter_runtime is None:  # pragma: no cover -- every path here runs after init
            message = f"{type(self).__name__} has no Runtime; its ParameterSet was never initialised"
            raise DeclarationError(message)
        return self._parameter_runtime


def _refuse(code: int) -> NoReturn:
    """Answer the write being checked with one status code, which opensmi hands to the client."""
    raise UaStatusCodeError(code)


def _known(parameter: _Parameter, value: object) -> bool:
    """Return whether a written value is the one ROS2 last said it holds, having said one at all."""
    return parameter.sampled and _same(value, parameter.value)


def _same(written: object, held: object) -> bool:
    """Return whether a written value is the value held, kind for kind.

    Exact, so ``True`` is not ``1`` and ``1`` is not ``1.0``. ``NaN`` equals ``NaN``, because a
    double parameter can hold one and its inward write has to pass.
    """
    if isinstance(written, (list, tuple)) and isinstance(held, (list, tuple)):
        return len(written) == len(held) and all(_same(a, b) for a, b in zip(written, held, strict=True))
    if type(written) is not type(held):
        return False
    if isinstance(written, float) and isinstance(held, float) and math.isnan(written) and math.isnan(held):
        return True
    return written == held


def _carries(value: object, parameter_type: ParameterType) -> bool:
    """Return whether a written value is of the declared ROS2 parameter type.

    A scalar needs exactly its Python type, so a boolean is not an integer and an integer is not a
    double.
    """
    if parameter_type is ParameterType.BYTE_ARRAY:
        return isinstance(value, (bytes, bytearray))
    element = _ELEMENTS.get(parameter_type)
    if element is not None:
        return isinstance(value, (list, tuple)) and all(_carries(each, element) for each in value)
    match parameter_type:
        case ParameterType.BOOL:
            return isinstance(value, bool)
        case ParameterType.INTEGER:
            return isinstance(value, int) and not isinstance(value, bool)
        case ParameterType.DOUBLE:
            return isinstance(value, float)
        case _:
            return isinstance(value, str)
