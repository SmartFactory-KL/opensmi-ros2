# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A published topic is a Skill: a client writes the fields, then starts the publishing.

The message fields are the Skill's Parameters, writable in any state, and nothing goes out while
they are written. ``Start`` reads them, builds the message and publishes it: the first tick of a
stream on a continuous Skill, the only publish on a finite one. ``Halt`` closes a stream. ``Reset``
clears the failure count and leaves the Parameters alone. There is no ``Suspend`` (opensmi keeps
running the handler through ``SUSPENDING``, so a paused stream would go on publishing) and no
Monitoring sibling: the echo of a topic we publish is our own message, so a readback would only
confirm the last value published. A consumer who wants to see the topic declares a
``MonitoringBinding`` on it. There is no FinalResultData either (it is optional in the SkillSet
model), since a publish has no result.

The trigger is the tick alone: there is no debounce, coalescing or dirty tracking. A finite push
has one publish, so any failure halts. A stream absorbs ``tolerated_failures`` consecutive ones, and
every failure fires a BindingEvent whether it was absorbed or not.

opensmi leaves a continuous Skill in ``Running`` when its handler returns, so the rate loop never
returns: every exit raises an ``OpenSmiRuntimeError``, which halts the Skill and reaches the
Component's Notification with its code. A stream exits when its tolerance for failed publishes is
spent, when the Lock it was started under ends (:data:`LOCK_ENDED`), and when a declared
``stale_after`` passes without a write (:data:`SETPOINT_STALE`), each with a BindingEvent of the same
code. ``Halt`` cancels the handler instead.

Access is a Skill's like any other: ``Start``, ``Halt`` and ``Reset`` need the Skill level
(``skill_access_level``) and the Machine's Lock, and ``Start`` also needs a completed startup. The
ParameterSet is built in :mod:`opensmi.ros2.server.containers`; the header stamp and the send-time
value guard are in :mod:`opensmi.ros2.runtime.publisher`.
"""

from __future__ import annotations

import asyncio
import contextlib
import math
from collections.abc import Mapping
from dataclasses import replace
from datetime import UTC, datetime
from typing import TYPE_CHECKING, Any, ClassVar, Final, cast

from asyncua import ua
from asyncua.ua.attribute_ids import AttributeIds
from opensmi.core.errors import OpenSmiRuntimeError
from opensmi.core.nodesets import DiNodeIds
from opensmi.server import BaseMachineryItem, BaseSkill, ContinuousSkillLogicMixin, FiniteSkillLogicMixin
from opensmi.server.mixins import ParameterSetMixin, ParentMixin

from opensmi.ros2.runtime.errors import FieldValueError, PublishFailedError, ROS2RuntimeError
from opensmi.ros2.runtime.guard import require_sendable
from opensmi.ros2.runtime.messages import ArrayType, NestedType, PrimitiveType, SequenceType, is_byte_container
from opensmi.ros2.runtime.selection import SEPARATOR
from opensmi.ros2.server.access import SKILL_ACCESS_LEVEL
from opensmi.ros2.server.containers import ROS2ContainerMixin, VariableContainer
from opensmi.ros2.server.errors import DeclarationError
from opensmi.ros2.server.events import Severity
from opensmi.ros2.server.mutation import add_plan_variable, require_unused_name
from opensmi.ros2.server.projection import PlanVariable, narrowed, require_invertible, shaped
from opensmi.ros2.server.provenance import BindingQoS

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Awaitable, Callable

    from opensmi.server.protocols import UserAuthorization
    from opensmi.server.ua_variable import UaVariable

    from opensmi.ros2.runtime.interface import RuntimeInterface
    from opensmi.ros2.runtime.messages import ElementType, FieldType, Message
    from opensmi.ros2.runtime.publisher import Publisher
    from opensmi.ros2.server.bindings import PublisherBinding
    from opensmi.ros2.server.projection import PlanEntry, ProjectionContext

__all__ = [
    "LOCK_ENDED",
    "MAXIMUM_RATE",
    "MESSAGE_UNBUILDABLE",
    "MINIMUM_RATE",
    "PUBLISH_FAILED",
    "PUBLISH_FAULTS",
    "RATE_PARAMETER",
    "SETPOINT_STALE",
    "ROS2ContinuousPublisherSkill",
    "ROS2FinitePublisherSkill",
    "ROS2PublisherSkill",
    "default_message_from",
    "halt_message_from",
]

RATE_PARAMETER: Final = "Rate"
"""The browse name of the one Parameter of a continuous publishing Skill that is not a message field.

It is reserved: a message with a field that converts to it (``rate``) is refused at init. Its
DisplayName is ``rate``.
"""

MINIMUM_RATE: Final = 0.001
"""The slowest publish rate a declaration or a client may set, in hertz (one tick every 16 minutes).

The bound is published as an ``EURange`` on the Parameter, so a write of zero is refused before a
period is divided by it.
"""

MAXIMUM_RATE: Final = 1000.0
"""The fastest publish rate, in hertz: the 1 ms period is the floor ``asyncio.sleep`` can keep."""

PUBLISH_FAILED: Final = "PUBLISH_FAILED"
"""The BindingEvent ``Code`` of every failed publish, and the ``ErrorCode`` of the error that halts
the Skill once its tolerance is spent. A TF Binding's failed broadcast reports under it too.
"""

MESSAGE_UNBUILDABLE: Final = "MessageUnbuildable"
"""The ``ErrorCode`` of a tick whose Parameters could not be built into a message.

It halts the Skill and nothing is published; the error message names the field and the reason.
"""

LOCK_ENDED: Final = "LOCK_ENDED"
"""The BindingEvent ``Code`` and ``ErrorCode`` of a stream halted because the Lock it was started
under ended: released, broken, expired, or lost with the holder's session.
"""

SETPOINT_STALE: Final = "SETPOINT_STALE"
"""The BindingEvent ``Code`` and ``ErrorCode`` of a stream halted because no client wrote any of its
Parameters within the Binding's ``stale_after``.
"""

PUBLISH_FAULTS: Final = (PublishFailedError, FieldValueError)
"""The faults that mean this tick did not reach the topic: the middleware refused the message, or
the guard refused a field value. Any other Runtime error (a released publisher, a stopped Runtime)
cannot be mended by another tick and is not tolerated.
"""


def default_message_from(
    message_type: type[Message],
    values: Mapping[str, object],
    context: ProjectionContext,
) -> Message:
    """Build the message to publish out of what a client wrote into a Skill's Parameters.

    This runs the Projection of the message type backwards, as an action's goal and a service's
    request do. It is a pure function (no server, no ``rclpy``, no event loop). Values the message
    has no field for, such as the reserved ``rate``, are ignored.

    :raises NotInvertibleError: When a field of the message has no value, because the selection
        removed it or no Parameter carries it. A default is never invented.
    """
    return context.registry.find(message_type.ros2.type_name).inverse_of(message_type, values, context)


def halt_message_from(
    binding: PublisherBinding, message_type: type[Message], runtime: RuntimeInterface
) -> Message | None:
    """Build the message a stream publishes once when it ends, or ``None`` when none is declared.

    It starts from the message as the type's generated class constructs it, so a field the table
    does not name keeps that class's default, and sets the named fields. A number goes
    into a floating-point field as a float, so ``z = -1`` in a file is ``-1.0``. The result passes
    the same send-time guard as every message the Skill publishes, so a value the field cannot hold
    fails here, at init, and not on the robot.

    :raises DeclarationError: When the table names a field the message type does not have, gives a
        table where a value belongs or the reverse, or holds a value its field cannot carry.
    """
    table = binding.halt_message
    if table is None:
        return None
    try:
        message = _overlaid(runtime.default_message(message_type), table, runtime, "")
        require_sendable(message)
    except (_HaltMessageRefusedError, FieldValueError) as refused:
        text = (
            f"the continuous publishing Binding {binding.name!r} declares a halt_message that "
            f"{message_type.ros2.type_name} cannot hold: {refused}"
        )
        raise DeclarationError(text) from refused
    return message


class _HaltMessageRefusedError(ValueError):
    """A halt message table that does not fit its message type, with the field path at fault."""


def _overlaid(default: Message, table: Mapping[str, object], runtime: RuntimeInterface, where: str) -> Message:
    """Return a message with the fields a table names set to its values, the rest left as they are."""
    if not isinstance(table, Mapping):
        message = f"{where or 'the message'} is a {default.ros2.type_name}, so it takes a table of its fields"
        raise _HaltMessageRefusedError(message)
    fields = {field.name: field for field in default.ros2.fields}
    changes: dict[str, object] = {}
    for name, value in table.items():
        path = f"{where}{SEPARATOR}{name}" if where else name
        field = fields.get(name)
        if field is None:
            message = f"{path} is not a field of {default.ros2.type_name}, which has {', '.join(fields)}"
            raise _HaltMessageRefusedError(message)
        nested = default.ros2_nested.get(name)
        changes[name] = _field_value(field.field_type, nested, getattr(default, name), value, runtime, path)
    # A message dataclass is built at run time, so the checker sees only its base.
    return replace(cast("Any", default), **changes)


def _field_value(
    field_type: FieldType,
    nested: type[Message] | None,
    current: object,
    value: object,
    runtime: RuntimeInterface,
    path: str,
) -> object:
    """Return what one field of a halt message holds, given what the table says for it."""
    if is_byte_container(field_type):
        if not isinstance(value, (list, tuple)):
            message = f"{path} holds bytes, so it takes an array of numbers from 0 to 255"
            raise _HaltMessageRefusedError(message)
        try:
            return bytes(value)  # pyright: ignore[reportUnknownArgumentType]
        except (TypeError, ValueError) as error:
            message = f"{path} holds bytes, so it takes an array of numbers from 0 to 255"
            raise _HaltMessageRefusedError(message) from error
    match field_type:
        case NestedType():
            return _overlaid(cast("Message", current), cast("Mapping[str, object]", value), runtime, path)
        case ArrayType(element=element) | SequenceType(element=element):
            if not isinstance(value, (list, tuple)):
                message = f"{path} is a list, so it takes an array"
                raise _HaltMessageRefusedError(message)
            items = cast("list[object] | tuple[object, ...]", value)
            return tuple(
                _element_value(element, nested, each, runtime, f"{path}[{index}]") for index, each in enumerate(items)
            )
        case _:
            return _element_value(field_type, nested, value, runtime, path)


def _element_value(
    element: ElementType,
    nested: type[Message] | None,
    value: object,
    runtime: RuntimeInterface,
    path: str,
) -> object:
    """Return one scalar or one element of a list, a nested element built from its own default."""
    if isinstance(element, NestedType):
        assert nested is not None
        return _overlaid(runtime.default_message(nested), cast("Mapping[str, object]", value), runtime, path)
    if isinstance(value, Mapping):
        message = f"{path} is a single value, not a message, so it takes a value rather than a table"
        raise _HaltMessageRefusedError(message)
    if isinstance(element, PrimitiveType) and element.name in _FLOATING and type(value) is int:
        return float(value)
    return value


_FLOATING: Final = frozenset({"float32", "float64"})


class ROS2PublisherSkill(
    ROS2ContainerMixin,
    ParentMixin[BaseMachineryItem],
    ParameterSetMixin,
):
    """What both publishing Skills share, everything except the tick.

    It is not a Skill on its own. The two subclasses below add opensmi's skill logic and differ
    only in what ``Running`` does; the declaration's ``continuous`` picks one.
    """

    callable_kind: ClassVar[str] = "Skill"

    def __init__(
        self,
        *,
        binding: PublisherBinding,
        runtime: RuntimeInterface,
        context: ProjectionContext,
        minimum_access_level: int = SKILL_ACCESS_LEVEL,
    ) -> None:
        """Resolve everything about the declaration that can be wrong, before any node exists.

        :param binding: What to build.
        :param runtime: Where the message is published (the Machine's, reached through the
            Component).
        :param context: The Projection context this Component renders through, with nothing
            selected yet.
        :param minimum_access_level: The level ``Start``, ``Halt`` and ``Reset`` need.
        :raises MessageTypeError: When the message type cannot be resolved.
        :raises FieldSelectionError: When a selection pattern matches nothing.
        :raises NotInvertibleError: When the message type reaches a Projection that cannot run
            backwards, so no client could ever publish through it.
        :raises DeclarationError: When a field of the message would claim the ``rate`` Parameter's
            name on a continuous Skill.
        """
        super().__init__(
            name=binding.name,
            minimum_access_level=minimum_access_level,
            suspendable=False,
        )
        self._binding = binding
        self._runtime = runtime
        self._message_type: type[Message] = runtime.describe_message(binding.message_type)
        self._context = narrowed(context, self._message_type, binding.selection)
        require_invertible(self._message_type, self._context)
        self._plan = shaped(self._message_type, self._context)
        if binding.continuous:
            _require_the_rate_name_is_free(self._plan, self._message_type, binding)
        # opensmi's container exists from first use, so the record can be made here and filled as
        # `_init` creates the variables.
        self._parameters = VariableContainer(name="ParameterSet", container=self.parameter_set)
        self._emit: Callable[..., Awaitable[None]] | None = None
        self._publisher: Publisher | None = None
        self._failures = 0

    @property
    def binding(self) -> PublisherBinding:
        """Return the declaration this Skill was built from."""
        return self._binding

    @property
    def message_type(self) -> type[Message]:
        """Return the message dataclass this Skill publishes."""
        return self._message_type

    @property
    def publisher(self) -> Publisher | None:
        """Return the ROS2 publisher this Skill owns, or ``None`` before init and after shutdown.

        It is advertised at init and held, so subscribers discover it while a client is still
        writing the fields and a one-shot push does not race their discovery.
        """
        return self._publisher

    @property
    def ros2_qos_provenance(self) -> BindingQoS | None:
        """Return what this Skill's publisher offers and where that came from, or ``None``.

        ``None`` before ``_init`` has advertised. The QoS is declared or defaulted, never
        discovered, so it cannot change.
        """
        publisher = self._publisher
        if publisher is None:
            return None
        return BindingQoS.of(self._binding.name, self._binding.path, publisher)

    @property
    def consecutive_failures(self) -> int:
        """Return how many publishes in a row have failed, as the tolerance counts them."""
        return self._failures

    async def _init(self) -> None:
        """Let opensmi build the Skill, create the ParameterSet variables, and prove them.

        The Parameters a client has not written yet hold the plan's initial values, so building a
        message from them is the message ``Start`` would publish now. A value ROS2 would abort the
        process over is thus a traceback here, not a core dump on a robot. The publisher is
        advertised last, so a declaration that cannot boot leaves no endpoint on the graph.

        :raises DeclarationError: When the Component cannot raise a BindingEvent.
        """
        # The mixin sits ahead of opensmi's Skill and is not a subclass of it, so the call resolves
        # statically to `object`; the house spelling of that suppression is the one the Component uses.
        await super()._init()  # pyright: ignore[reportAttributeAccessIssue]
        self._emit = binding_event_emitter(
            self.parent,
            f"the Binding {self._binding.name!r} publishes, and the Component it is on cannot raise a "
            f"BindingEvent; a Component carrying ROS2 Bindings mixes in ROS2ComponentMixin",
        )
        await self._create(self._plan, self._parameters, message_type=self._message_type, writable=True)
        await self._create_own_parameters()
        require_sendable(await self.build_message(await self.read_parameters()))
        self._publisher = await self._runtime.advertise(
            topic=self._binding.path,
            message_type=self._message_type,
            depth=self._binding.depth,
            qos=self._binding.qos,
        )

    async def read_parameters(self) -> Mapping[str, object]:
        """Return what the ParameterSet holds now, by variable name."""
        return await self._read_values(self._parameters)

    async def build_message(self, values: Mapping[str, object]) -> Message:
        """Return the message to publish, given what a client wrote into the Parameters.

        Override this on a custom implementation class when building needs the instance (another
        node's value, something injected into the Component). The declaration's ``message_from`` is
        the pure-function form of the same hook, and :func:`default_message_from` is the default.

        An override must work at init, which calls it once with the declaration's defaults. On a
        continuous Skill the values include the reserved ``Rate``, which is not a message field.
        """
        build = self._binding.message_from or default_message_from
        return build(self._message_type, values, self._context)

    async def close_publisher(self) -> None:
        """Release this Skill's ROS2 publisher. Idempotent.

        The declaring Component's shutdown calls it, for a Component torn down while the process
        goes on serving. Stopping the Runtime closes every publisher too.
        """
        publisher, self._publisher = self._publisher, None
        if publisher is not None:
            await publisher.close()

    async def _create_own_parameters(self) -> None:
        """Create the Parameters that are not message fields. A finite Skill has none."""

    async def _handle_resetting(self) -> None:
        """Clear the failure count. The ParameterSet stays, so nothing a client wrote is lost."""
        self._failures = 0

    async def _publish(self) -> None:
        """Publish one snapshot of the Parameters, and let the continuity decide what a failure is.

        :raises OpenSmiRuntimeError: When the publish failed and the tolerance is spent.
        """
        publisher = self._publisher
        if publisher is None:  # pragma: no cover -- _init either advertises or raises
            missing = "this Skill has no ROS2 publisher, so there is nothing to publish through"
            raise OpenSmiRuntimeError(missing, error_code="PublisherMissing")
        try:
            message = await self.build_message(await self.read_parameters())
        except ValueError as refused:
            # Every refusal of a client's value is a ValueError; left to opensmi it would halt
            # without an error code.
            raise OpenSmiRuntimeError(str(refused), error_code=MESSAGE_UNBUILDABLE) from refused
        try:
            await publisher.publish(message)
        except PUBLISH_FAULTS as error:
            await self._note_a_failure(error)
            return
        self._failures = 0

    async def _note_a_failure(self, error: Exception) -> None:
        """Count one failed publish, report it, and halt when the tolerance is spent.

        Every failure is reported, tolerated or not: a tolerated one has no other surface, since
        the Skill stays ``Running``.

        :raises OpenSmiRuntimeError: When this failure is one too many.
        """
        self._failures += 1
        tolerated = self._binding.tolerated_failures
        spent = self._failures > tolerated
        self.logger.warning(
            "A ROS2 publish did not happen",
            binding=self._binding.name,
            topic=self._binding.path,
            consecutive=self._failures,
            tolerated=tolerated,
        )
        await self._report(
            PUBLISH_FAILED,
            severity=Severity.ERROR if spent else Severity.WARNING,
            text=str(error),
        )
        if spent:
            raise OpenSmiRuntimeError(str(error), error_code=PUBLISH_FAILED)

    async def _report(self, code: str, *, severity: Severity, text: str) -> None:
        """Fire one BindingEvent about this Binding, and never let the reporting be the fault.

        A failure to raise the event is only logged: a stream must not die because the
        report about a tolerated failure could not be made.
        """
        emit = self._emit
        if emit is None:  # pragma: no cover -- _init resolves one or raises
            return
        try:
            await emit(
                binding=self._binding.name,
                path=self._binding.path,
                code=code,
                severity=severity,
                text=text,
            )
        except Exception:
            self.logger.exception(
                "Could not fire a BindingEvent about a publish",
                binding=self._binding.name,
                topic=self._binding.path,
                code=code,
            )


class ROS2ContinuousPublisherSkill(ROS2PublisherSkill, ContinuousSkillLogicMixin, BaseSkill):
    """The stream: one publish per tick of the ``rate`` Parameter, until ``Halt`` or nobody is in charge.

    opensmi has no periodic hook on a running Skill, so this one owns a wait loop that the halting
    transition cancels. Two things end it besides ``Halt`` and a failed publish: the Lock
    it was started under ends (:data:`LOCK_ENDED`), and a declared ``stale_after`` passes without a
    write to any Parameter (:data:`SETPOINT_STALE`). Both are checked before every publish, and the
    wait between ticks wakes for either, so a halt does not wait for the next tick. A stream whose
    Binding declares a ``halt_message`` publishes it once as it ends, whatever ended it; without one
    it just stops. The robot's own command timeout stays what brings it to rest.
    """

    def __init__(self, **kwargs: Any) -> None:
        """Build the Skill, and start out with nothing known about the client's writing."""
        super().__init__(**kwargs)
        self._rate: UaVariable[float] | None = None
        self._locking_user_node: ua.NodeId | None = None
        self._holder: UserAuthorization | None = None
        self._lock_ended = asyncio.Event()
        self._fresh_since = datetime.now(UTC)
        self._halt_message = halt_message_from(self._binding, self._message_type, self._runtime)
        self._streaming = False

    async def _create_own_parameters(self) -> None:
        """Create the ``rate`` Parameter, and check a declared ``stale_after``.

        The rate is bounded by an ``EURange``, which opensmi's write path enforces, so a client
        cannot write a period of zero.
        """
        require_unused_name(RATE_PARAMETER, (self.parameter_set,), self._named(self._parameters))
        # The declaration refuses a continuous Binding without a rate, so the fallback is
        # unreachable; it is spelled out because `rate` is optional on the declaration.
        declared = self._binding.rate if self._binding.rate is not None else MINIMUM_RATE
        entry = PlanVariable(
            name=RATE_PARAMETER,
            display_name="rate",
            source="",
            initial=float(declared),
            variant_type=ua.VariantType.Double,
        )
        self._rate = await add_plan_variable(
            self.parameter_set,
            RATE_PARAMETER,
            entry,
            None,
            writable=True,
            unit="Hz",
            value_range=(MINIMUM_RATE, MAXIMUM_RATE),
        )
        self._require_write_times_are_recorded()

    async def _handle_running(self) -> None:
        """Publish the current snapshot on every tick, until something ends the stream.

        Publishes first and waits afterwards, so ``Start`` sends at once; ``Start`` also counts as a
        fresh setpoint. This never returns; every exit raises.
        """
        self._fresh_since = datetime.now(UTC)
        self._streaming = True
        async with self._tied_to_the_lock():
            while True:
                await self._publish()
                await self._until_the_next_tick(await self._period())

    async def _handle_halting(self) -> None:
        """Publish the declared halt message once, when a stream that ran comes to an end.

        opensmi halts through here whatever ended the stream: ``Halt``, the end of the Lock, a stale
        setpoint or a spent tolerance. The rate loop has been cancelled or has raised by now, so the
        halt message is the last one. A halt message that cannot be published is reported and does
        not hold the halt up.
        """
        streamed, self._streaming = self._streaming, False
        message = self._halt_message
        publisher = self._publisher
        if not streamed or message is None or publisher is None:
            return
        try:
            await publisher.publish(message)
        except ROS2RuntimeError as error:
            self.logger.warning("The halt message was not published", binding=self._binding.name)
            text = f"the halt message of {self.name!r} was not published on {self._binding.path}: {error}"
            await self._report(PUBLISH_FAILED, severity=Severity.ERROR, text=text)

    async def _period(self) -> float:
        """Return how long until the next tick, read off the ``rate`` Parameter on every one.

        Reading it per tick makes a rate change effective on the next tick. The range check is not
        a second enforcement of the ``EURange``: it only refuses to divide by a number no client
        write could have produced, such as one written by a consumer's own code.

        :raises OpenSmiRuntimeError: When the Parameter holds a rate outside its published bounds.
        """
        assert self._rate is not None
        rate = float(await self._rate.read())
        if not MINIMUM_RATE <= rate <= MAXIMUM_RATE:
            refused = (
                f"the publish rate of {self.name!r} is {rate}, and a rate is a number of times a "
                f"second between {MINIMUM_RATE} and {MAXIMUM_RATE}"
            )
            raise OpenSmiRuntimeError(refused, error_code="RateOutOfRange")
        return 1.0 / rate

    async def _until_the_next_tick(self, period: float) -> None:
        """Wait one period, and halt on the way if the Lock ends or the setpoint goes stale.

        :raises OpenSmiRuntimeError: When either happens before the period is over.
        """
        loop = asyncio.get_running_loop()
        tick = loop.time() + period
        while True:
            await self._require_the_lock_holds()
            fresh_for = await self._require_a_fresh_setpoint()
            remaining = tick - loop.time()
            if remaining <= 0:
                return
            with contextlib.suppress(TimeoutError):
                async with asyncio.timeout(min(remaining, fresh_for)):
                    await self._lock_ended.wait()

    @contextlib.asynccontextmanager
    async def _tied_to_the_lock(self) -> AsyncIterator[None]:
        """Tie the stream to whoever holds the Machine's Lock now, for as long as the block runs.

        A client needs the Lock to call ``Start``, so its holder is the one in charge of the stream.
        A Skill started through the server's own interface while nobody holds the Lock is tied to
        nothing, since there is no Lock to end.

        opensmi signals no change of the Lock, but every change (``ExitLock``, ``BreakLock``, the
        inactivity timeout, a closed session) writes the Lock's ``LockingUser`` variable, so the
        stream watches that write in asyncua's address space, as a subscription does.
        """
        node = await self._locking_user()
        self._lock_ended.clear()
        self._holder = self.lock.locking_user
        if self._holder is None:
            yield
            return
        space: Any = self.server.ua_server.iserver.aspace
        _, handle = space.add_datachange_callback(node, AttributeIds.Value, self._on_lock_written)
        try:
            yield
        finally:
            space.delete_datachange_callback(handle)

    async def _locking_user(self) -> ua.NodeId:
        """Return the NodeId of the Lock's ``LockingUser`` variable, looked up once."""
        if self._locking_user_node is None:
            namespace = self.server.ua_get_namespace_index(DiNodeIds)
            node = await self.lock.ua_node.get_child(f"{namespace}:LockingUser")
            self._locking_user_node = node.nodeid
        return self._locking_user_node

    async def _on_lock_written(self, *_change: object) -> None:
        """Note that the Lock ended when its holder is no longer the one the stream started under.

        It runs inside the Lock's own update, so it only sets a flag; the running loop halts.
        """
        if self.lock.locking_user != self._holder:
            self._lock_ended.set()

    async def _require_the_lock_holds(self) -> None:
        """Halt the stream when the Lock it was started under has ended.

        :raises OpenSmiRuntimeError: When the Lock was released, broken, expired or lost with its
            session.
        """
        if not self._lock_ended.is_set():
            return
        assert self._holder is not None
        now = self.lock.locking_user
        holding = "nobody holds it now" if now is None else f"{now.name!r} holds it now"
        ended = (
            f"the Lock {self._holder.name!r} held when {self.name!r} started has ended (released, broken, "
            f"expired or its session closed) and {holding}, so nobody is in charge of {self._binding.path} "
            f"any more and this stream is halting rather than republishing what it last said"
        )
        await self._report(LOCK_ENDED, severity=Severity.WARNING, text=ended)
        raise OpenSmiRuntimeError(ended, error_code=LOCK_ENDED)

    async def _require_a_fresh_setpoint(self) -> float:
        """Halt the stream when no Parameter has been written for ``stale_after`` seconds.

        Without this, a stream whose client died keeps republishing the last values (on
        ``/cmd_vel``, a robot driving on a command nobody sends) for as long as the client's session
        and Lock outlive it. It compares when each Parameter was last written, not what it holds: a
        client steering straight ahead writes the same numbers every time. Every Parameter counts,
        ``rate`` included. The write times are wall-clock ``ServerTimestamp`` values, so a clock
        stepped forward can halt a stream early, which is the safe side.

        :return: The seconds left before the setpoint goes stale, infinite without ``stale_after``.
        :raises OpenSmiRuntimeError: When the setpoint is stale.
        """
        seconds = self._binding.stale_after
        if seconds is None:
            return math.inf
        written = await self._newest_write()
        if written is not None and written > self._fresh_since:
            self._fresh_since = written
        left = seconds - (datetime.now(UTC) - self._fresh_since).total_seconds()
        if left > 0:
            return left
        stale = (
            f"no Parameter of {self.name!r} has been written for {seconds} s, so the client driving "
            f"{self._binding.path} has stopped and this stream is halting rather than republishing "
            f"what it last said"
        )
        await self._report(SETPOINT_STALE, severity=Severity.WARNING, text=stale)
        raise OpenSmiRuntimeError(stale, error_code=SETPOINT_STALE)

    async def _newest_write(self) -> datetime | None:
        """Return when any Parameter was last written (its ``ServerTimestamp``), or ``None``."""
        newest: datetime | None = None
        for variable in self.parameter_set:
            written = (await variable.ua_node.read_data_value()).ServerTimestamp
            if written is not None and (newest is None or written > newest):
                newest = written
        return newest

    def _require_write_times_are_recorded(self) -> None:
        """Refuse a declared ``stale_after`` on a server that does not stamp the writes it takes.

        The check reads asyncua's ``ServerTimestamp``, which an application may turn off. Without
        it every stream would halt after ``stale_after`` while its client writes.

        :raises DeclarationError: When the server does not record write times.
        """
        if self._binding.stale_after is None:
            return
        space: Any = getattr(getattr(self.server.ua_server, "iserver", None), "aspace", None)
        if getattr(space, "force_server_timestamp", True):
            return
        refused = (
            f"the Binding {self._binding.name!r} declares stale_after, and this server does not "
            f"record when a value was written -- force_server_timestamp is off, so every Parameter "
            f"would look untouched and the stream would halt while its client was writing; leave "
            f"stale_after out, or leave asyncua's server timestamps on"
        )
        raise DeclarationError(refused)


class ROS2FinitePublisherSkill(ROS2PublisherSkill, FiniteSkillLogicMixin, BaseSkill):
    """The one-shot push: publish once on ``Start``, then reach ``Completed``.

    A second push is ``Reset``, write, ``Start``. ``Ready`` is reachable only by an explicit reset,
    because resetting automatically would discard what a client wrote.
    """

    async def _handle_running(self) -> None:
        """Publish once. Returning reaches ``Completed``; any failure raises and halts.

        A finite Skill has one publish and nothing to absorb, which is why a non-zero
        ``tolerated_failures`` on a finite declaration is refused.
        """
        await self._publish()


def binding_event_emitter(machinery_item: object, message: str) -> Callable[..., Awaitable[None]]:
    """Return the BindingEvent emitter of the machinery item that declared a Binding.

    Resolved at init so that a missing reporting surface is not first noticed by a report going
    nowhere. A Binding is only built by a Component that mixes ours in, so this is unreachable
    through the declarative API.

    :param message: The error text for a machinery item that cannot raise one.
    :raises DeclarationError: When the machinery item has no BindingEvent emitter.
    """
    emit = getattr(machinery_item, "emit_binding_event", None)
    if emit is None:
        raise DeclarationError(message)
    return emit


def _require_the_rate_name_is_free(
    plan: tuple[PlanEntry, ...],
    message_type: type[Message],
    binding: PublisherBinding,
) -> None:
    """Refuse a continuous declaration whose message already claims the ``rate`` Parameter's name.

    Checked against the plan, not the message's fields, because a curated Projection chooses its
    own names and those are what collide. A finite Skill has no ``rate`` node, so it is not checked.

    :raises DeclarationError: When the plan has a top-level entry called ``rate``.
    """
    if not any(entry.full_name == RATE_PARAMETER for entry in plan):
        return
    message = (
        f"the continuous publishing Binding {binding.name!r} renders "
        f"{message_type.ros2.type_name} into a Parameter called {RATE_PARAMETER!r}, and that name "
        f"is the tick's period on a stream; exclude the field, give it another name with the rename option, "
        f"curate the type so the name differs, or declare this Binding finite"
    )
    raise DeclarationError(message)
