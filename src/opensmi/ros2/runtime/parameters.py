# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The parameter facet: a ROS2 node's parameters, read one at a time and written non-atomically.

A ROS2 parameter is a named, typed configuration value on a ROS2 node, read and written over the
node's parameter services and observable on ``/parameter_events``. Three behaviours of the
library (verified on Jazzy) shape this module:

* A read takes one name. ``get_parameters`` and ``describe_parameters`` answer with an empty list
  and no error when any one name is undeclared, so a batched read would blank every value on one
  typo.
* A write is never atomic. ``set_parameters`` reports per parameter, while the atomic variant
  discards the whole batch when one name is undeclared. The atomic call is not wrapped.
* One subscription to ``/parameter_events`` serves all watches and demultiplexes by node. It is
  open only while a watch exists. The topic also fires for changes a node makes to itself, and
  these are passed on.

:class:`ParameterType` has the nine declarable members of ``rcl_interfaces/msg/ParameterType``.
A value crossing this module is a :class:`ParameterValue`: one of those types and a plain Python
value. The facet reads and writes through callables the Runtime supplies, so nothing here imports
``rclpy``.
"""

from __future__ import annotations

import asyncio
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any, Final, TypeAlias, final

import structlog

from opensmi.ros2.runtime.errors import SubscriptionClosedError
from opensmi.ros2.runtime.messages import PrimitiveType, SequenceType, StringType

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator, AsyncIterator, Awaitable, Callable, Iterable, Mapping, Sequence

    from opensmi.ros2.runtime.messages import FieldType, Message

__all__ = [
    "CALL_TIMEOUT",
    "DESCRIBE_PARAMETERS_TYPE",
    "EVENT_DEPTH",
    "GET_PARAMETERS_TYPE",
    "LIST_PARAMETERS_TYPE",
    "PARAMETER_EVENTS_TOPIC",
    "PARAMETER_EVENT_TYPE",
    "SERVER_TIMEOUT",
    "SET_PARAMETERS_TYPE",
    "DescribeFunction",
    "EventFunction",
    "ListFunction",
    "ParameterDescription",
    "ParameterFacet",
    "ParameterRange",
    "ParameterReading",
    "ParameterType",
    "ParameterUpdate",
    "ParameterValue",
    "ParameterWatch",
    "ReadFunction",
    "SetOutcome",
    "WriteFunction",
    "absolute",
    "describe_request",
    "description_of",
    "get_request",
    "list_request",
    "names_of",
    "outcomes_of",
    "reading_of",
    "set_request",
    "update_of",
]

PARAMETER_EVENTS_TOPIC: Final = "/parameter_events"
"""The one topic every ROS2 node announces its parameter changes on."""

PARAMETER_EVENT_TYPE: Final = "rcl_interfaces/msg/ParameterEvent"
"""The type carried on :data:`PARAMETER_EVENTS_TOPIC`."""

GET_PARAMETERS_TYPE: Final = "rcl_interfaces/srv/GetParameters"
"""The service type behind ``<node>/get_parameters``."""

SET_PARAMETERS_TYPE: Final = "rcl_interfaces/srv/SetParameters"
"""The service type behind ``<node>/set_parameters``, which reports per parameter."""

LIST_PARAMETERS_TYPE: Final = "rcl_interfaces/srv/ListParameters"
"""The service type behind ``<node>/list_parameters``, which answers what a node declares."""

DESCRIBE_PARAMETERS_TYPE: Final = "rcl_interfaces/srv/DescribeParameters"
"""The service type behind ``<node>/describe_parameters``, asked one name at a time."""

SERVER_TIMEOUT: Final = 10.0
"""Seconds a parameter call waits for the node's service to appear.

ROS2 supplies no deadline: a parameter-service call to a node that is not running produces no
error and no result.
"""

CALL_TIMEOUT: Final = 30.0
"""Seconds to wait for the answer once a parameter request has gone."""

EVENT_DEPTH: Final = 100
"""How many parameter events one subscription and one watch buffer.

Deeper than a Monitoring Binding's because the buffer is shared by every node on the graph, and
a dropped event leaves a UA Parameter holding a value the node no longer has.
"""

_LOGGER = structlog.get_logger("opensmi.ros2.runtime.parameters")


class ParameterType(Enum):
    """The declarable members of ``rcl_interfaces/msg/ParameterType``.

    Nine of the ten codes: ``PARAMETER_NOT_SET`` is a reading, never a declaration, and arrives
    as a :class:`ParameterReading` that is present and holds nothing. The member value is the
    constant name without its ``PARAMETER_`` prefix, lower-cased, for use in declarations.
    """

    BOOL = "bool"
    INTEGER = "integer"
    DOUBLE = "double"
    STRING = "string"
    BYTE_ARRAY = "byte_array"
    BOOL_ARRAY = "bool_array"
    INTEGER_ARRAY = "integer_array"
    DOUBLE_ARRAY = "double_array"
    STRING_ARRAY = "string_array"

    @property
    def code(self) -> int:
        """Return the number ``rcl_interfaces/msg/ParameterType`` gives this type."""
        return _CODES[self]

    @property
    def value_field(self) -> str:
        """Return the field of ``rcl_interfaces/msg/ParameterValue`` that carries this type."""
        return f"{self.value}_value"

    @property
    def field_type(self) -> FieldType:
        """Return the ROS2 field type a value of this type is.

        An ``integer`` is an ``int64``, a ``double`` is a ``float64`` and a ``byte_array`` is a
        sequence of ``uint8``, so a parameter value goes through the same conversion as a message
        field (:func:`opensmi.ros2.server.projection.ros2_value`).
        """
        return _FIELD_TYPES[self]


_FIELD_TYPES: Final[Mapping[ParameterType, FieldType]] = {
    ParameterType.BOOL: PrimitiveType(name="bool"),
    ParameterType.INTEGER: PrimitiveType(name="int64"),
    ParameterType.DOUBLE: PrimitiveType(name="float64"),
    ParameterType.STRING: StringType(),
    ParameterType.BYTE_ARRAY: SequenceType(element=PrimitiveType(name="uint8")),
    ParameterType.BOOL_ARRAY: SequenceType(element=PrimitiveType(name="bool")),
    ParameterType.INTEGER_ARRAY: SequenceType(element=PrimitiveType(name="int64")),
    ParameterType.DOUBLE_ARRAY: SequenceType(element=PrimitiveType(name="float64")),
    ParameterType.STRING_ARRAY: SequenceType(element=StringType()),
}
"""What each declarable parameter type is, in the package's field-type model."""

_CODES: Final[Mapping[ParameterType, int]] = {
    ParameterType.BOOL: 1,
    ParameterType.INTEGER: 2,
    ParameterType.DOUBLE: 3,
    ParameterType.STRING: 4,
    ParameterType.BYTE_ARRAY: 5,
    ParameterType.BOOL_ARRAY: 6,
    ParameterType.INTEGER_ARRAY: 7,
    ParameterType.DOUBLE_ARRAY: 8,
    ParameterType.STRING_ARRAY: 9,
}
"""The constants of ``rcl_interfaces/msg/ParameterType``, written out so member order cannot renumber them."""

_BY_CODE: Final[Mapping[int, ParameterType]] = {code: each for each, code in _CODES.items()}

_EMPTY_VALUE: Final[Mapping[str, object]] = {
    "type": 0,  # PARAMETER_NOT_SET
    "bool_value": False,
    "integer_value": 0,
    "double_value": 0.0,
    "string_value": "",
    "byte_array_value": b"",
    "bool_array_value": (),
    "integer_array_value": (),
    "double_array_value": (),
    "string_array_value": (),
}
"""Every field of ``rcl_interfaces/msg/ParameterValue`` at its zero.

Message dataclasses have no defaults, so a value is built with all ten fields and the relevant one
overwritten. A field added upstream raises a missing-argument ``TypeError`` here.
"""


@dataclass(frozen=True, slots=True, kw_only=True)
class ParameterValue:
    """One ROS2 parameter value, as the declaration typed it.

    :ivar type: Which of the union's typed fields this value belongs in.
    :ivar value: The value as a plain Python object: ``bool``, ``int``, ``float``, ``str``,
        ``bytes`` for a byte array, or ``tuple`` for the other arrays.
    """

    type: ParameterType
    value: object


@dataclass(frozen=True, slots=True, kw_only=True)
class ParameterReading:
    """What one look at one ROS2 parameter found: not present, present without a value, or a value.

    :ivar name: The ROS2 parameter name, dotted as the node spells it.
    :ivar present: Whether the node declares that name at all.
    :ivar value: What the node holds, or ``None`` when the name is declared but unset
        (``PARAMETER_NOT_SET``). Always ``None`` when ``present`` is ``False``.
    """

    name: str
    present: bool
    value: ParameterValue | None = None


@dataclass(frozen=True, slots=True, kw_only=True)
class SetOutcome:
    """What a ROS2 node answered about one parameter of a write.

    A refusal is an answer. A call that does not happen (no server, or one that went quiet) is
    raised by the Runtime instead.

    :ivar successful: ``SetParametersResult.successful``.
    :ivar reason: ``SetParametersResult.reason``, verbatim. It is only ever the node's own text,
        for example "speed_limit out of range. Min: 0, Max: 100, value: 5000".
    :ivar answered: Whether the node said anything about this parameter. ``False`` means it
        answered the batch with fewer results than names; the write is neither a success nor a
        refusal with words.
    """

    successful: bool
    reason: str = ""
    answered: bool = True


@dataclass(frozen=True, slots=True, kw_only=True)
class ParameterRange:
    """The range a ROS2 node declares for one numeric parameter, inclusive at both ends.

    A ``ParameterDescriptor`` carries at most one ``floating_point_range`` or ``integer_range``;
    this is either of them.

    :ivar low: ``from_value``, the smallest value the node accepts.
    :ivar high: ``to_value``, the largest value the node accepts.
    :ivar step: The node's ``step``. Zero means any value between the two bounds.
    """

    low: float
    high: float
    step: float = 0

    def admits(self, value: float) -> bool:
        """Return whether a value lies between the two bounds, both included.

        The step is not checked: the node applies it with a tolerance it does not publish.
        """
        return self.low <= value <= self.high


@dataclass(frozen=True, slots=True, kw_only=True)
class ParameterDescription:
    """What a ROS2 node says about one of its parameters, beyond the value it holds.

    Everything an operator needs to write a declaration for the parameter.

    :ivar name: The ROS2 parameter name, dotted as the node spells it.
    :ivar present: Whether the node declares that name at all.
    :ivar type: The type the node declares it as, or ``None`` when the node answered a code with
        no member here (``PARAMETER_NOT_SET``). Always ``None`` when ``present`` is ``False``.
    :ivar description: The node's own one-line description, verbatim; empty if it gave none.
    :ivar additional_constraints: The node's own free text about what it accepts, verbatim.
    :ivar read_only: Whether the node refuses every write. Such a parameter is still declarable,
        and every write to it is then refused, so this is a warning and not an error.
    :ivar dynamic_typing: Whether the node lets the parameter change type. A declaration names
        one type, so declaring such a parameter narrows what the node allows.
    :ivar range: The numeric range the node declares, or ``None``.
    """

    name: str
    present: bool
    type: ParameterType | None = None
    description: str = ""
    additional_constraints: str = ""
    read_only: bool = False
    dynamic_typing: bool = False
    range: ParameterRange | None = None


@dataclass(frozen=True, slots=True, kw_only=True)
class ParameterUpdate:
    """What one ``/parameter_events`` message says about one node.

    :ivar node: The fully qualified name of the node the change happened on.
    :ivar readings: One reading per parameter the event names. A deleted parameter arrives as a
        reading that is not present, like a read of an undeclared name.
    """

    node: str
    readings: tuple[ParameterReading, ...]


ReadFunction: TypeAlias = "Callable[[str, str], Awaitable[ParameterReading]]"
"""How the facet reads one parameter: the node path and one name."""

WriteFunction: TypeAlias = "Callable[[str, Mapping[str, ParameterValue]], Awaitable[Mapping[str, SetOutcome]]]"
"""How the facet writes: the node path and a batch, answered per parameter."""

EventFunction: TypeAlias = "Callable[[], AsyncGenerator[ParameterUpdate, None]]"
"""How the facet follows ``/parameter_events``: one generator, closed with the last watch."""

ListFunction: TypeAlias = "Callable[[str], Awaitable[tuple[str, ...]]]"
"""How the facet lists: the node path, and every parameter name that node declares."""

DescribeFunction: TypeAlias = "Callable[[str, str], Awaitable[ParameterDescription]]"
"""How the facet describes: the node path and one name."""


@final
class ParameterWatch:
    """One consumer's view of one ROS2 node's parameter changes, and its lifetime.

    Handed out by :meth:`ParameterFacet.watch`. Close it when done: the shared subscription closes
    with the last watch.
    """

    def __init__(self, *, node: str, facet: ParameterFacet, release: Callable[[ParameterWatch], None]) -> None:
        """Bind a watch to one node's name and the facet's deregistration."""
        self._node = node
        self._facet = facet
        self._release = release
        self._updates: deque[ParameterUpdate] = deque(maxlen=EVENT_DEPTH)
        self._arrival = asyncio.Event()
        self._closed = False

    def __repr__(self) -> str:
        """Return a representation naming the node and the watch's state."""
        return f"<ParameterWatch {self._node} ({'closed' if self._closed else 'open'})>"

    @property
    def node(self) -> str:
        """Return the fully qualified ROS2 node name this watch follows."""
        return self._node

    @property
    def closed(self) -> bool:
        """Return whether this watch's lifetime has ended."""
        return self._closed

    def deliver(self, update: ParameterUpdate) -> None:
        """Hand this watch one update. Only the facet's demux calls it."""
        if len(self._updates) == self._updates.maxlen:
            # A dropped event leaves a UA Parameter stale, so it is logged, not just counted.
            _LOGGER.warning(
                "A parameter event was dropped because its consumer is behind",
                node=self._node,
                depth=EVENT_DEPTH,
            )
        self._updates.append(update)
        self._arrival.set()

    async def receive(self) -> ParameterUpdate:
        """Await the next parameter change on this node.

        :raises SubscriptionClosedError: When the watch has been closed.
        """
        while not self._updates:
            if self._closed:
                message = f"this parameter watch on {self._node} is closed; its events are no longer delivered"
                raise SubscriptionClosedError(message)
            # No await between clear and wait, so `deliver` (same loop) cannot slip in between.
            self._arrival.clear()
            await self._arrival.wait()
        return self._updates.popleft()

    def __aiter__(self) -> AsyncIterator[ParameterUpdate]:
        """Return an iterator yielding updates until the watch is closed."""
        return self._stream()

    async def _stream(self) -> AsyncIterator[ParameterUpdate]:
        """Yield updates until the watch is closed, then stop cleanly."""
        while True:
            try:
                yield await self.receive()
            except SubscriptionClosedError:
                return

    async def close(self) -> None:
        """End this watch, and with it the shared subscription if it was the last one.

        Idempotent.
        """
        if self._closed:
            return
        self._closed = True
        self._arrival.set()
        self._release(self)
        await self._facet.settle()


@final
class ParameterFacet:
    """The ROS2 parameters of every node on one Machine's graph.

    Obtain it from :attr:`opensmi.ros2.runtime.runtime.Runtime.parameters`; the constructor
    arguments are the Runtime's plumbing.
    """

    def __init__(
        self,
        *,
        read: ReadFunction,
        write: WriteFunction,
        events: EventFunction,
        list_names: ListFunction,
        describe: DescribeFunction,
    ) -> None:
        """Describe a facet without calling anything.

        :param read: How to read one parameter of one node.
        :param write: How to write a batch to one node, per parameter.
        :param events: How to follow ``/parameter_events``. Opened on the first watch and closed
            with the last.
        :param list_names: How to ask one node what parameters it declares.
        :param describe: How to ask one node about one of those names.
        """
        self._read = read
        self._write = write
        self._events = events
        self._list = list_names
        self._describe = describe
        self._watches: list[ParameterWatch] = []
        self._stream: AsyncGenerator[ParameterUpdate, None] | None = None
        self._task: asyncio.Task[None] | None = None
        self._reads = 0
        self._writes = 0

    def __repr__(self) -> str:
        """Return a representation naming how many watches are open and what it has done."""
        return f"<ParameterFacet watching {len(self._watches)} nodes after {self._reads} reads, {self._writes} writes>"

    @property
    def _watching(self) -> bool:
        """Return whether ``/parameter_events`` is subscribed (exactly while a watch is open)."""
        return self._task is not None

    async def get(self, *, node: str, name: str) -> ParameterReading:
        """Read one parameter of one ROS2 node.

        A name the node does not have is not an error: it comes back as a reading that is not
        present.

        :param node: The ROS2 node path. A leading ``/`` is added when it is missing.
        :param name: The ROS2 parameter name, dotted as the node spells it.
        :raises ServiceUnavailableError: When the node's parameter services do not answer.
        :raises ServiceCallTimeoutError: When the request went and no answer came back in time.
        """
        self._reads += 1
        return await self._read(absolute(node), name)

    async def set(self, *, node: str, values: Mapping[str, ParameterValue]) -> Mapping[str, SetOutcome]:
        """Write parameters to one ROS2 node, never atomically, and report per parameter.

        Every valid entry lands and each refusal is named, even when others in the batch fail.

        :param node: The ROS2 node path. A leading ``/`` is added when it is missing.
        :param values: What to write, by ROS2 parameter name.
        :return: One outcome per name asked for. An empty batch is not a call and answers empty.
        :raises ServiceUnavailableError: When the node's parameter services do not answer.
        :raises ServiceCallTimeoutError: When the request went and no answer came back in time.
        """
        if not values:
            return {}
        self._writes += 1
        return await self._write(absolute(node), values)

    async def list(self, *, node: str) -> tuple[str, ...]:
        """Return every ROS2 parameter name one node declares, in the order the node gave them.

        This covers the whole node at once, since no names are asked for. A node with no
        parameters and a node that answered nothing both give an empty tuple. Counted as a read.

        :param node: The ROS2 node path. A leading ``/`` is added when it is missing.
        :raises ServiceUnavailableError: When the node's parameter services do not answer.
        :raises ServiceCallTimeoutError: When the request went and no answer came back in time.
        """
        self._reads += 1
        return await self._list(absolute(node))

    async def describe(self, *, node: str, name: str) -> ParameterDescription:
        """Describe one parameter of one ROS2 node: its type, description and constraints.

        A name the node does not have comes back as a description that is not present. Counted
        as a read.

        :param node: The ROS2 node path. A leading ``/`` is added when it is missing.
        :param name: The ROS2 parameter name, dotted as the node spells it.
        :raises ServiceUnavailableError: When the node's parameter services do not answer.
        :raises ServiceCallTimeoutError: When the request went and no answer came back in time.
        """
        self._reads += 1
        return await self._describe(absolute(node), name)

    def watch(self, node: str) -> ParameterWatch:
        """Follow one ROS2 node's parameter changes, subscribing if nothing was watching yet.

        :param node: The ROS2 node path. A leading ``/`` is added when it is missing.
        :return: A handle carrying the updates and its own lifetime. Close it when the consumer
            is done: the subscription closes with the last watch.
        """
        watch = ParameterWatch(node=absolute(node), facet=self, release=self._watches.remove)
        self._watches.append(watch)
        self._start()
        return watch

    async def settle(self) -> None:
        """Close the shared subscription if nothing is watching. Called by a watch as it closes."""
        if self._watches or self._task is None:
            return
        # Both taken before the await: a watch opened meanwhile starts its own task and stream,
        # and reading the fields afterwards would close that new subscription instead.
        task, self._task = self._task, None
        stream, self._stream = self._stream, None
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        # Closed here, not in the task: finalising a generator during cancellation can be
        # interrupted and would leak the subscription.
        if stream is not None:
            await stream.aclose()

    async def close(self) -> None:
        """End every watch and close the subscription, which is what stopping the Runtime does."""
        for watch in tuple(self._watches):
            await watch.close()
        await self.settle()

    def _start(self) -> None:
        """Open the one subscription, unless it is already open or nothing is watching."""
        if self._task is not None or not self._watches:
            return
        self._task = asyncio.create_task(self._follow(), name="ros2-parameter-events")

    async def _follow(self) -> None:
        """Demux ``/parameter_events`` onto the watches, by node, until cancelled."""
        stream = self._events()
        self._stream = stream
        try:
            async for update in stream:
                for watch in tuple(self._watches):
                    if watch.node == update.node:
                        watch.deliver(update)
        except Exception:
            # Nobody awaits this task, so log: otherwise every Parameter silently stops following.
            _LOGGER.exception("Stopped following /parameter_events")


def absolute(node: str) -> str:
    """Return a ROS2 node path with the leading ``/`` that ``/parameter_events`` uses.

    A path without the slash would match no event.
    """
    return node if node.startswith("/") else f"/{node}"


def get_request(request_type: type[Message], name: str) -> Message:
    """Return the ``GetParameters`` request for exactly one name."""
    return _built(request_type, names=(name,))


def reading_of(name: str, response: Message) -> ParameterReading:
    """Return what a ``GetParameters`` response says about the one name it was asked for.

    An empty list means the node has no such parameter (and arrives without an error).
    """
    values: Sequence[Message] = _field(response, "values")
    if not values:
        return ParameterReading(name=name, present=False)
    return ParameterReading(name=name, present=True, value=_value_of(values[0]))


def _value_of(raw: Message) -> ParameterValue | None:
    """Return one ``rcl_interfaces/msg/ParameterValue`` as a typed value, or ``None`` when unset.

    A code with no member here also gives ``None``, so a wrong value is never shown.
    """
    parameter_type = _BY_CODE.get(int(_field(raw, "type")))
    if parameter_type is None:
        return None
    return ParameterValue(type=parameter_type, value=_field(raw, parameter_type.value_field))


def list_request(request_type: type[Message]) -> Message:
    """Return the ``ListParameters`` request for every name (no prefixes, depth zero is ``DEPTH_RECURSIVE``)."""
    return _built(request_type, prefixes=(), depth=0)


def names_of(response: Message) -> tuple[str, ...]:
    """Return the parameter names a ``ListParameters`` response carries, in the node's own order."""
    result = _field(response, "result")
    return tuple(str(each) for each in _field(result, "names"))


def describe_request(request_type: type[Message], name: str) -> Message:
    """Return the ``DescribeParameters`` request asking about exactly one name."""
    return _built(request_type, names=(name,))


def description_of(name: str, response: Message) -> ParameterDescription:
    """Return what a ``DescribeParameters`` response says about the one name it was asked for.

    An empty list means the node has no such parameter, as for a read.
    """
    descriptors: Sequence[Message] = _field(response, "descriptors")
    if not descriptors:
        return ParameterDescription(name=name, present=False)
    descriptor = descriptors[0]
    return ParameterDescription(
        name=name,
        present=True,
        type=_BY_CODE.get(int(_field(descriptor, "type"))),
        description=str(_field(descriptor, "description")),
        additional_constraints=str(_field(descriptor, "additional_constraints")),
        read_only=bool(_field(descriptor, "read_only")),
        dynamic_typing=bool(_field(descriptor, "dynamic_typing")),
        range=_range_of(descriptor),
    )


def _range_of(descriptor: Message) -> ParameterRange | None:
    """Return the one range a ``ParameterDescriptor`` declares, or ``None``.

    At most one of the two range fields is filled, with at most one element.
    """
    for named in ("floating_point_range", "integer_range"):
        declared: Sequence[Message] = _field(descriptor, named)
        if declared:
            each = declared[0]
            return ParameterRange(
                low=_field(each, "from_value"),
                high=_field(each, "to_value"),
                step=_field(each, "step"),
            )
    return None


def set_request(request_type: type[Message], values: Mapping[str, ParameterValue]) -> Message:
    """Return the ``SetParameters`` request writing a batch, in the order it was given.

    The order matters: results come back positionally and :func:`outcomes_of` pairs them by it.
    """
    parameter_type = request_type.ros2_nested["parameters"]
    value_type = parameter_type.ros2_nested["value"]
    written = tuple(
        _built(parameter_type, name=name, value=_raw_value(value_type, each)) for name, each in values.items()
    )
    return _built(request_type, parameters=written)


def outcomes_of(names: Iterable[str], response: Message) -> Mapping[str, SetOutcome]:
    """Return what a ``SetParameters`` response says, per parameter, paired back with its name.

    Names missing from a short answer are marked unsuccessful and unanswered, with no reason text,
    because :attr:`SetOutcome.reason` must only ever hold the node's own words.
    """
    results: Sequence[Message] = _field(response, "results")
    outcomes: dict[str, SetOutcome] = {}
    for index, name in enumerate(names):
        if index >= len(results):
            outcomes[name] = SetOutcome(successful=False, answered=False)
            continue
        result = results[index]
        outcomes[name] = SetOutcome(
            successful=bool(_field(result, "successful")),
            reason=str(_field(result, "reason")),
        )
    return outcomes


def update_of(message: Message) -> ParameterUpdate:
    """Return one ``/parameter_events`` message as an update.

    New and changed parameters, then deleted ones as readings that are not present.
    """
    readings = [
        ParameterReading(name=str(_field(each, "name")), present=True, value=_value_of(_field(each, "value")))
        for named in ("new_parameters", "changed_parameters")
        for each in _field(message, named)
    ]
    readings += [
        ParameterReading(name=str(_field(each, "name")), present=False)
        for each in _field(message, "deleted_parameters")
    ]
    return ParameterUpdate(node=str(_field(message, "node")), readings=tuple(readings))


def _raw_value(value_type: type[Message], value: ParameterValue) -> Message:
    """Return one typed value as the ``rcl_interfaces/msg/ParameterValue`` that carries it."""
    return _built(value_type, **{**_EMPTY_VALUE, "type": value.type.code, value.type.value_field: value.value})


def _built(message_type: type[Message], **fields: object) -> Message:
    """Build one message dataclass from its field names (they are only known at run time)."""
    return message_type(**fields)


def _field(message: Message, name: str) -> Any:
    """Return one field of a message dataclass by name (its fields are not statically known)."""
    return getattr(message, name)
