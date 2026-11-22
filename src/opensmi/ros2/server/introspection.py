# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""One Component per Machine that answers what is on the ROS2 graph and what this adapter is bound to.

Every Machine with a Runtime gets this Component, on by default. It carries nine Methods:

- ``ListTopics``, ``ListServices`` and ``ListActions``: what is on the ROS2 graph, with an optional
  substring filter. The graph is refreshed once per call, not polled.
- ``ListROS2Parameters``: the ROS2 parameters one node declares.
- ``DescribeROS2Parameter``: type, description, constraints and flags of one such parameter.
- ``ListBindings``: every Binding on this Machine: kind, ROS2 path, required or not, satisfied or
  not, how long it has been waiting, and its QoS provenance.
- ``ListRecentEvents``: the last N BindingEvents this Machine fired.
- ``BindTopic``: bind a topic to look at it. Ephemeral: the nodes hang under this Component's own
  Monitoring and live until the server restarts.
- ``CallService``: call a service once. Request and response are JSON in the Method's own
  ParameterSet and FinalResultData, so no nodes are created.

The seven read-only Methods answer at the listing level with the Machine's Lock bypassed, and
their arguments are writable the same way. The two mutators need the Lock and a higher level. No
Method waits for startup to complete, so a Machine whose startup hangs can still be inspected.
Every Method browses for every authenticated user, and a client without the access level is
refused on use. Ad-hoc Bindings go through
:meth:`~opensmi.ros2.server.mirror.ROS2InwardMirrorMixin.bind_topic_now`, so they are capped and
counted like any other mutation. There is no ad-hoc goal sending: an action worth driving is worth
declaring.
"""

from __future__ import annotations

import asyncio
import datetime as dt
import json
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, ClassVar, Final, NoReturn

from asyncua import ua
from asyncua.ua.status_codes import StatusCodes
from opensmi.server import BaseComponent, FinalResultData, ParameterSet, UaVariable
from opensmi.server.mixins import FinalResultDataMixin, ParameterSetMixin
from typing_extensions import override

from opensmi.ros2.runtime.errors import ROS2RuntimeError
from opensmi.ros2.runtime.guard import require_sendable
from opensmi.ros2.runtime.selection import FieldSelection
from opensmi.ros2.server.access import LISTING_ACCESS_LEVEL, ROS2ArgumentVariable, ROS2GatedMethod
from opensmi.ros2.server.bindings import MonitoringBinding, is_required
from opensmi.ros2.server.component import ROS2ComponentMixin
from opensmi.ros2.server.errors import DeclarationError
from opensmi.ros2.server.events import root_of
from opensmi.ros2.server.method import TRANSPORT_STATUS, default_request_from, default_response_to
from opensmi.ros2.server.mutation import add_plan_variable
from opensmi.ros2.server.projection import PlanObject, PlanVariable, flat_name, narrowed, shaped
from opensmi.ros2.server.required import LOSS_INTERVAL, satisfaction_of

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable, Iterator, Mapping, Sequence

    from opensmi.server.protocols import UserAuthorization

    from opensmi.ros2.runtime.graph import GraphSnapshot
    from opensmi.ros2.runtime.interface import RuntimeInterface
    from opensmi.ros2.runtime.messages import Message
    from opensmi.ros2.server.bindings import Binding
    from opensmi.ros2.server.events import BindingEventRecord
    from opensmi.ros2.server.projection import PlanEntry, ProjectionContext
    from opensmi.ros2.server.provenance import BindingQoS

__all__ = [
    "INTROSPECTION_ID",
    "INTROSPECTION_NAME",
    "BindTopic",
    "BindingReport",
    "CallService",
    "DescribeROS2Parameter",
    "ListActions",
    "ListBindings",
    "ListROS2Parameters",
    "ListRecentEvents",
    "ListServices",
    "ListTopics",
    "ROS2GraphListing",
    "ROS2IntrospectionComponent",
    "ROS2ListingMethod",
    "ROS2MutatorMethod",
    "WaitingClock",
    "machinery_items",
    "qos_entries_of",
    "reports_of",
    "waiting_key",
]

INTROSPECTION_NAME: Final = "ROS2Introspection"
"""What the Component browses as. It is the same on every Machine."""

INTROSPECTION_ID: Final = "ros2_introspection"
"""The Component's asset id, which it writes into its identification."""

_ENTRIES: Final = "Entries"
"""The array of lines every Method here answers with."""

_COUNT: Final = "Count"
"""How many lines :data:`_ENTRIES` carries, so a client can branch without reading them."""

_AD_HOC_DEPTH: Final = 1
"""How many messages an ad-hoc subscription buffers (a declared Binding buffers ten)."""


def _utc_now() -> dt.datetime:
    """Return the wall clock, in UTC."""
    return dt.datetime.now(dt.UTC)


@dataclass(frozen=True, slots=True, kw_only=True)
class BindingReport:
    """One Binding, as the provenance report answers for it.

    Everything is read off state that is already kept current, so a report puts nothing on the wire.

    :ivar item: The path of the machinery item that declares it.
    :ivar binding: The Binding's name.
    :ivar kind: The declaration class.
    :ivar path: The ROS2 topic, service, action or node path it names.
    :ivar required: Whether its item's StartupSkill will not reach ``Running`` without it.
    :ivar satisfied: Whether its ROS2 counterpart is present, or ``None`` where presence does not
        apply (a publishing Binding, where this adapter is the publisher).
    :ivar waiting_since: When it last stopped being Satisfied, or ``None`` when it is satisfied
        now or was never answerable. See :class:`WaitingClock`.
    :ivar qos: What its ROS2 endpoint asks for and where that came from, or ``None`` for a kind
        with no single endpoint of its own (a Method, an action, a ROS2-parameter Binding, the TF
        Binding) or a Binding still waiting for its topic.
    """

    item: str
    binding: str
    kind: str
    path: str
    required: bool
    satisfied: bool | None
    waiting_since: dt.datetime | None = None
    qos: BindingQoS | None = None

    def __str__(self) -> str:
        """Return the one line an operator reads out of the report."""
        state = "n/a" if self.satisfied is None else ("satisfied" if self.satisfied else "absent")
        waiting = f" since {self.waiting_since.isoformat(timespec='seconds')}" if self.waiting_since else ""
        marked = "required" if self.required else "optional"
        profile = "no endpoint" if self.qos is None else f"{self.qos.provenance.value} {self.qos.profile}"
        adapted = f", adapted {self.qos.adaptations} time(s)" if self.qos and self.qos.adaptations else ""
        return (
            f"{self.item}/{self.binding} [{self.kind}] {self.path} -- "
            f"{marked}, {state}{waiting}; QoS: {profile}{adapted}"
        )


class WaitingClock:
    """When each Binding last stopped being Satisfied.

    The introspection Component samples the same booleans the status codes are written from, once
    every :data:`~opensmi.ros2.server.required.LOSS_INTERVAL` seconds and again whenever a report
    is asked for, and remembers the first sample at which each Binding was not satisfied. Sampling
    reads flags other tasks already maintain, so it puts nothing on the wire and never starts the
    graph poll.
    """

    def __init__(self, now: Callable[[], dt.datetime] = _utc_now) -> None:
        """Start out knowing nothing.

        :param now: Reads the wall clock. Tests inject their own.
        """
        self._now = now
        self._since: dict[tuple[str, str], dt.datetime] = {}

    def __repr__(self) -> str:
        """Return a representation naming how many Bindings are currently waiting."""
        return f"<WaitingClock, {len(self._since)} waiting>"

    def note(self, key: tuple[str, str], *, satisfied: bool | None) -> None:
        """Record one sample of one Binding's satisfaction.

        A satisfied Binding, or one whose kind has no presence to read, forgets its waiting time,
        so a sensor that returns and leaves again is reported waiting since the second departure.
        """
        if satisfied is None or satisfied:
            self._since.pop(key, None)
            return
        self._since.setdefault(key, self._now())

    def waiting_since(self, key: tuple[str, str]) -> dt.datetime | None:
        """Return when one Binding last stopped being Satisfied, or ``None`` if it has not."""
        return self._since.get(key)


def waiting_key(machinery_item: Any, binding: str) -> tuple[str, str]:
    """Return how the clock files one Binding: the item's path, and the Binding's name."""
    path = getattr(machinery_item, "path", None)
    return (type(machinery_item).__name__ if path is None else str(path), binding)


def machinery_items(machine: Any) -> Iterator[Any]:
    """Yield a Machine and every Component under it, depth first, in the order they were added."""
    yield machine
    for component in getattr(machine, "components", ()):
        yield from machinery_items(component)


def qos_entries_of(machinery_item: Any) -> Mapping[str, BindingQoS]:
    """Return every provenance entry one machinery item can answer for, by Binding name.

    The sources are a Monitoring Binding's subscription, a Keyed Binding's subscription, and a
    publishing Binding's publisher (which hangs on its Skill). An item that is not one of ours
    contributes nothing. The TF Binding is absent, as :mod:`opensmi.ros2.server.provenance` says:
    its endpoints belong to the Machine.
    """
    entries: dict[str, BindingQoS] = {}
    entries.update(getattr(machinery_item, "ros2_qos_provenance", None) or {})
    entries.update(getattr(machinery_item, "ros2_keyed_qos_provenance", None) or {})
    for skill in getattr(machinery_item, "skill_set", ()):
        published = getattr(skill, "ros2_qos_provenance", None)
        if published is not None and not isinstance(published, dict):
            entries[published.binding] = published
    return entries


def reports_of(machinery_item: Any, clock: WaitingClock) -> tuple[BindingReport, ...]:
    """Return one report row per Binding one machinery item declares.

    Presence is probed with :func:`~opensmi.ros2.server.required.satisfaction_of`, the same cascade
    the StartupSkill waits on, so the report and the StartupSkill cannot disagree.
    """
    entries = qos_entries_of(machinery_item)
    bindings: Sequence[Binding] = getattr(machinery_item, "ros2_bindings", ())
    rows: list[BindingReport] = []
    for binding in bindings:
        probe = satisfaction_of(machinery_item, binding)
        satisfied = None if probe is None else probe()
        key = waiting_key(machinery_item, binding.name)
        clock.note(key, satisfied=satisfied)
        rows.append(
            BindingReport(
                item=key[0],
                binding=binding.name,
                kind=type(binding).__name__,
                path=_path_of(machinery_item, binding),
                required=is_required(binding),
                satisfied=satisfied,
                waiting_since=clock.waiting_since(key),
                qos=entries.get(binding.name),
            )
        )
    return tuple(rows)


def _path_of(machinery_item: Any, binding: Any) -> str:
    """Return the ROS2 path a report row names; for the TF Binding, the Machine's two TF topics."""
    declared = getattr(binding, "path", None)
    if declared is not None:
        return str(declared)
    config = getattr(machinery_item, "ros2_config", None)
    topics: Sequence[str] = getattr(getattr(config, "transforms", None), "topics", ())
    return " + ".join(topics) or "(no ROS2 path)"


class ROS2ListingMethod(
    ParameterSetMixin[ParameterSet],
    FinalResultDataMixin[FinalResultData],
    ROS2GatedMethod,
):
    """A read-only Method answering with a list of lines, at the listing level and without the Lock.

    FinalResultData holds a ``Count`` and an ``Entries`` array of strings, and the subclass
    declares its arguments as Parameters in :meth:`_init`. The answers are strings because they
    are prose for a person reading an HMI panel. A subclass implements :meth:`answer`.
    """

    bypasses_the_lock: ClassVar[bool] = True

    def __init__(self, *, name: str, minimum_access_level: int = LISTING_ACCESS_LEVEL, **kwargs: Any) -> None:
        """Name the Method and give it the level its kind requires."""
        super().__init__(name=name, minimum_access_level=minimum_access_level, **kwargs)

    @property
    def component(self) -> Any:
        """Return the introspection Component."""
        return self.parent

    @property
    def runtime(self) -> RuntimeInterface:
        """Return the Machine's Runtime."""
        runtime: RuntimeInterface = self.component.runtime
        return runtime

    @override
    async def _init(self) -> None:
        """Let opensmi build the Method, then add the two result variables every one has."""
        await super()._init()
        await add_plan_variable(
            self.final_result_data,
            _COUNT,
            PlanVariable(name=_COUNT, source="", initial=0, variant_type=ua.VariantType.Int32),
            None,
        )
        await add_plan_variable(
            self.final_result_data,
            _ENTRIES,
            PlanVariable(name=_ENTRIES, source="", initial=[], variant_type=ua.VariantType.String),
            None,
        )

    async def _add_argument(self, name: str, initial: object, variant_type: ua.VariantType) -> None:
        """Create one writable argument. A listing's is writable without the Lock, a mutator's is not."""
        variable_type = ROS2ArgumentVariable if self.bypasses_the_lock else UaVariable
        variable: UaVariable[Any] = variable_type(
            ua.Variant(initial, variant_type),
            name=name,
            writable=True,
            minimum_access_level=self.minimum_access_level,
        )
        await self.parameter_set.add(variable)

    @override
    async def _condition_startup_completed(self, user: UserAuthorization) -> bool:
        """Answer yes: nothing here commands the robot, and one asks most when startup failed."""
        del user
        return True

    @override
    async def execute_method(self) -> None:
        """Answer the call, writing the count beside the lines."""
        await self._write_lines(list(await self.answer()))

    async def answer(self) -> Sequence[str]:
        """Do whatever this Method does, and return the lines it answers with.

        Implemented by each subclass.
        """
        raise NotImplementedError

    async def _write_lines(self, lines: Sequence[str]) -> None:
        """Write one answer into the two result variables."""
        await self.final_result_data[_COUNT].write(len(lines))
        await self.final_result_data[_ENTRIES].write(list(lines))

    async def _text_parameter(self, name: str) -> str:
        """Return one string Parameter, stripped."""
        value = await self.parameter_set[name].read()
        return str(value or "").strip()

    async def _refreshed_graph(self) -> GraphSnapshot:
        """Read the ROS2 graph once. No poll keeps running."""
        return await self.runtime.graph.refresh()

    async def _refuse(self, code: int, text: str) -> NoReturn:
        """Log the reason on the Notification, then refuse with a status code.

        ``Call`` can only answer a status code, so the reason travels as a Notification message.

        :raises UaStatusCodeError: Always.
        """
        await self.ua_log_error(text)
        raise ua.UaStatusCodeError(code)

    async def _require_an_argument(self, value: str, name: str, what: str) -> None:
        """Refuse a call whose text argument was left empty.

        :raises UaStatusCodeError: ``BadInvalidArgument``.
        """
        if value:
            return
        await self._refuse(StatusCodes.BadInvalidArgument, f"write {name} before calling: it is {what}")

    async def _one_type_on(self, path: str, names: Mapping[str, tuple[str, ...]], argument: str) -> str:
        """Return the one ROS2 type a graph name carries, or refuse to guess between several.

        :raises UaStatusCodeError: ``BadInvalidArgument`` when the name is not on the graph or
            carries more than one type.
        """
        types = names.get(path, ())
        if len(types) == 1:
            return types[0]
        found = "is not on the ROS2 graph" if not types else f"carries more than one type ({', '.join(types)})"
        return await self._refuse(
            StatusCodes.BadInvalidArgument,
            f"{path} {found}, so name the type in {argument}",
        )

    async def _answered(self, call: Awaitable[Any]) -> Any:
        """Await one ROS2 call, turning transport failures into UA status codes.

        :raises UaStatusCodeError: The code :data:`~opensmi.ros2.server.method.TRANSPORT_STATUS`
            gives, when the call did not happen. Declared service Methods use the same table.
        """
        try:
            return await call
        except tuple(TRANSPORT_STATUS) as error:
            code = next(each for kind, each in TRANSPORT_STATUS.items() if isinstance(error, kind))
            await self._refuse(code, str(error))


class ROS2MutatorMethod(ROS2ListingMethod):
    """A Method that changes something, and therefore needs the Lock as well as a level."""

    bypasses_the_lock: ClassVar[bool] = False

    def __init__(self, *, name: str, minimum_access_level: int, **kwargs: Any) -> None:  # pyright: ignore[reportIncompatibleMethodOverride]
        """Register the Method at the deployment's mutator level, which has no default."""
        super().__init__(name=name, minimum_access_level=minimum_access_level, **kwargs)


def _matching(names: Mapping[str, tuple[str, ...]], wanted: str) -> list[str]:
    """Return one graph vocabulary as sorted lines, narrowed by a substring."""
    return [f"{name}  {', '.join(names[name]) or 'no type'}" for name in sorted(names) if not wanted or wanted in name]


class ROS2GraphListing(ROS2ListingMethod):
    """One of the ROS2 graph's three vocabularies, narrowed by a substring.

    :cvar vocabulary: Which field of a graph snapshot this Method answers from.
    """

    vocabulary: ClassVar[str]

    @override
    async def _init(self) -> None:
        """Add the one argument: a substring to narrow the listing by."""
        await super()._init()
        await self._add_argument("Filter", "", ua.VariantType.String)

    @override
    async def answer(self) -> Sequence[str]:
        """Return every entry whose ROS2 name contains the filter, or every one when it is empty."""
        snapshot = await self._refreshed_graph()
        names: Mapping[str, tuple[str, ...]] = getattr(snapshot, self.vocabulary)
        return _matching(names, await self._text_parameter("Filter"))


class ListTopics(ROS2GraphListing):
    """Every topic on the ROS2 graph, with the types advertised on it."""

    vocabulary: ClassVar[str] = "topics"


class ListServices(ROS2GraphListing):
    """Every service on the ROS2 graph, with its types."""

    vocabulary: ClassVar[str] = "services"


class ListActions(ROS2GraphListing):
    """Every action on the ROS2 graph, with its types."""

    vocabulary: ClassVar[str] = "actions"


class ListROS2Parameters(ROS2ListingMethod):
    """Every ROS2 parameter one node declares, in the order the node declared them.

    This calls the node's own ``list_parameters`` service, so a node that is not running fails
    like any other transport failure.
    """

    @override
    async def _init(self) -> None:
        """Add the one argument: which ROS2 node to ask."""
        await super()._init()
        await self._add_argument("Node", "", ua.VariantType.String)

    @override
    async def answer(self) -> Sequence[str]:
        """Return every parameter name the named node declares.

        :raises UaStatusCodeError: ``BadInvalidArgument`` when no node was named, and the code
            the transport table gives when the node's parameter services did not answer.
        """
        node = await self._text_parameter("Node")
        await self._require_an_argument(node, "Node", "the ROS2 node whose parameters to list")
        return list(await self._answered(self.runtime.parameters.list(node=node)))


class DescribeROS2Parameter(ROS2ListingMethod):
    """What one ROS2 node says about one of its parameters (type, description, constraints, flags).

    It does not read the value. To read a value, declare the parameter on a Component.
    """

    @override
    async def _init(self) -> None:
        """Add the two arguments: which node, and which of its parameters."""
        await super()._init()
        await self._add_argument("Node", "", ua.VariantType.String)
        await self._add_argument("Name", "", ua.VariantType.String)

    @override
    async def answer(self) -> Sequence[str]:
        """Return the description as a handful of lines, or say the node has no such name.

        :raises UaStatusCodeError: ``BadInvalidArgument`` when either argument is missing, and
            the transport code when the node's parameter services did not answer.
        """
        node = await self._text_parameter("Node")
        name = await self._text_parameter("Name")
        await self._require_an_argument(node, "Node", "the ROS2 node the parameter lives on")
        await self._require_an_argument(name, "Name", "the ROS2 parameter to describe")
        described = await self._answered(self.runtime.parameters.describe(node=node, name=name))
        if not described.present:
            return [f"{node} does not declare {name!r}"]
        declared = "not set (the node declares it and it holds nothing)"
        if described.type is not None:
            declared = described.type.value
        return [
            f"name: {described.name}",
            f"type: {declared}",
            f"description: {described.description or '(none)'}",
            f"constraints: {described.additional_constraints or '(none)'}",
            f"read_only: {described.read_only}",
            f"dynamic_typing: {described.dynamic_typing}",
        ]


class ListBindings(ROS2ListingMethod):
    """Every Binding on this Machine, with its provenance and its state."""

    @override
    async def _init(self) -> None:
        """Add the one argument: a substring narrowing it to one item, Binding or ROS2 path."""
        await super()._init()
        await self._add_argument("Filter", "", ua.VariantType.String)

    @override
    async def answer(self) -> Sequence[str]:
        """Return one line per Binding, narrowed by a substring of its item, name or path."""
        wanted = await self._text_parameter("Filter")
        return [
            str(report)
            for report in self.component.binding_reports()
            if not wanted or wanted in report.item or wanted in report.binding or wanted in report.path
        ]


class ListRecentEvents(ROS2ListingMethod):
    """The last N BindingEvents this Machine fired, oldest first.

    An OPC UA event is a moment, so a client that was not subscribed at the time cannot learn it
    happened. The last :data:`~opensmi.ros2.server.events.RECENT_EVENTS` are kept, which makes
    conditions with no state variable (a QoS clash, a failed publish, a key collision, a cap
    reached) answerable.
    """

    @override
    async def _init(self) -> None:
        """Add the two arguments: how many to answer with, and a code to narrow them by."""
        await super()._init()
        # `Newest` rather than `Count`: every Method already answers with a `Count` in FinalResultData.
        await self._add_argument("Newest", 0, ua.VariantType.Int32)
        await self._add_argument("Code", "", ua.VariantType.String)

    @override
    async def answer(self) -> Sequence[str]:
        """Return the last N kept events, oldest first, narrowed by code where one was given."""
        wanted = await self._text_parameter("Code")
        count = int(await self.parameter_set["Newest"].read() or 0)
        kept: Sequence[BindingEventRecord] = self.component.recent_events()
        matching = [str(each) for each in kept if not wanted or each.code == wanted]
        # Zero means everything kept; the ring is bounded, so that cannot be unbounded.
        return matching if count <= 0 else matching[-count:]


class BindTopic(ROS2MutatorMethod):
    """Bind a ROS2 topic to this Component's Monitoring, ephemerally, to look at it.

    It creates an ordinary Monitoring Binding through the mutation chokepoint, so what an operator
    sees is what the written declaration would show. Nothing reaches disk and nothing is unbound:
    the Binding lives until the server restarts.
    """

    @override
    async def _init(self) -> None:
        """Add the three arguments: what to call it, which topic, and which type."""
        await super()._init()
        await self._add_argument("Name", "", ua.VariantType.String)
        await self._add_argument("Topic", "", ua.VariantType.String)
        await self._add_argument("MessageType", "", ua.VariantType.String)

    @override
    async def answer(self) -> Sequence[str]:
        """Bind the topic and say what was made, or say why nothing was.

        :raises UaStatusCodeError: ``BadInvalidArgument`` when a name or a topic is missing, when
            the type was neither given nor decidable from the graph, or when the name is taken;
            ``BadOutOfRange`` when a mutation cap refused the nodes.
        """
        name = await self._text_parameter("Name")
        topic = await self._text_parameter("Topic")
        declared = await self._text_parameter("MessageType")
        await self._require_an_argument(name, "Name", "what to call the Binding, which its variables are named for")
        await self._require_an_argument(topic, "Topic", "the ROS2 topic to bind")
        message_type = declared or await self._one_type_on(
            topic,
            (await self._refreshed_graph()).topics,
            "MessageType",
        )
        try:
            binding = MonitoringBinding(name=name, path=topic, message_type=message_type, depth=_AD_HOC_DEPTH)
            bound = await self.component.bind_ad_hoc(binding)
        except (ROS2RuntimeError, DeclarationError) as error:
            await self._refuse(StatusCodes.BadInvalidArgument, str(error))
        if not bound:
            await self._refuse(
                StatusCodes.BadOutOfRange,
                f"binding {topic} as {name!r} would take this Machine past a mutation cap, so "
                f"nothing was created; raise mutation_node_cap on this Component, or restart the "
                f"server to start from what was declared",
            )
        return [
            f"bound {topic} ({message_type}) as {name!r} under this Component's Monitoring",
            "it is ephemeral: it lives until this server restarts",
        ]


class CallService(ROS2MutatorMethod):
    """Call one ROS2 service once, ad hoc, without creating any nodes.

    The request and the response are JSON, keyed by the node path of each field joined with dots
    (``Mode.Mode`` for the variable ``Mode/Mode``), so ``{"Mode.Mode": 2, "Label": "wide"}`` is the
    request a declared Method's Parameters would have been. A field left out holds the type's own zero.
    """

    @override
    async def _init(self) -> None:
        """Add the three arguments: which service, which type, and the request as JSON."""
        await super()._init()
        await self._add_argument("Service", "", ua.VariantType.String)
        await self._add_argument("ServiceType", "", ua.VariantType.String)
        await self._add_argument("Request", "{}", ua.VariantType.String)

    @override
    async def answer(self) -> Sequence[str]:
        """Send one request and answer with the response as JSON.

        :raises UaStatusCodeError: ``BadInvalidArgument`` when the service or its type is missing
            or the request is not a JSON object this type can be built from; ``BadTypeMismatch``
            when a value the request carries is not one the ROS2 type can hold; and the code
            :data:`~opensmi.ros2.server.method.TRANSPORT_STATUS` gives when the call did not
            happen.
        """
        service = await self._text_parameter("Service")
        await self._require_an_argument(service, "Service", "the ROS2 service to call")
        service_type = await self._text_parameter("ServiceType") or await self._one_type_on(
            service,
            (await self._refreshed_graph()).services,
            "ServiceType",
        )
        values = await self._request_values(await self._text_parameter("Request") or "{}")
        request = await self._built(service_type, values)
        response = await self._answered(
            self.runtime.call_service(service=service, service_type=service_type, request=request)
        )
        read = default_response_to(response, self._context(type(response)))
        return [f"{service} answered {type(response).ros2.type_name}", _as_json(read)]

    async def _built(self, service_type: str, values: Mapping[str, object]) -> Message:
        """Build the request message with :func:`~opensmi.ros2.server.method.default_request_from`.

        It also checks that the message can be sent, so a value ROS2 would abort the process over
        becomes a status code.

        :raises UaStatusCodeError: ``BadInvalidArgument`` when the type cannot be resolved or the
            request cannot be built from what was given, and ``BadTypeMismatch`` when a value is
            not one the ROS2 type can hold.
        """
        try:
            types = self.runtime.describe_service(service_type)
            context = self._context(types.request)
            supplied = dict(_initials(shaped(types.request, context)))
            unknown = sorted(set(values) - set(supplied))
            if unknown:
                message = (
                    f"{service_type} has no field(s) {', '.join(unknown)}; a request names the "
                    f"node paths of a declared Method's Parameters, joined with dots, which for "
                    f"this type are: {', '.join(sorted(supplied))}"
                )
                raise DeclarationError(message)  # noqa: TRY301 -- refused below with the rest
            supplied.update(values)
            request = default_request_from(types.request, supplied, context)
        except (ROS2RuntimeError, DeclarationError) as error:
            await self._refuse(StatusCodes.BadInvalidArgument, str(error))
        try:
            require_sendable(request)
        except ROS2RuntimeError as error:
            await self._refuse(StatusCodes.BadTypeMismatch, str(error))
        return request

    async def _request_values(self, text: str) -> Mapping[str, object]:
        """Return the typed request as field paths to values.

        :raises UaStatusCodeError: ``BadInvalidArgument`` when the text is not a JSON object.
        """
        try:
            parsed = json.loads(text)
        except ValueError as error:
            await self._refuse(
                StatusCodes.BadInvalidArgument,
                f"Request is not JSON: {error}. It is an object keyed by the node paths of a "
                f"declared Method's Parameters, joined with dots, so "
                f'{{"Mode.Mode": 2}} -- and {{}} sends the type as the zeros a declared '
                f"Method's untouched Parameters would have sent",
            )
        if not isinstance(parsed, dict):
            await self._refuse(
                StatusCodes.BadInvalidArgument,
                f"Request is a JSON {type(parsed).__name__} and has to be an object: the field "
                f"path is what says which field a value is for",
            )
        return parsed

    def _context(self, message_type: type[Message]) -> ProjectionContext:
        """Return the Component's Projection context for one type, so ad-hoc calls render like declared ones."""
        return narrowed(self.component.ros2_projection_context, message_type, FieldSelection())


class ROS2IntrospectionComponent(ROS2ComponentMixin, BaseComponent):
    """The Component every Machine with a Runtime carries, and the ad-hoc Bindings it holds.

    It declares no Bindings of its own, so its Monitoring is empty at boot and fills with whatever
    an operator binds through :class:`BindTopic`. A Machine may replace it through
    :attr:`~opensmi.ros2.server.machine.ROS2MachineMixin.introspection_component_type`.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Start out holding no ad-hoc Bindings and nothing sampled."""
        kwargs.setdefault("name", INTROSPECTION_NAME)
        super().__init__(*args, **kwargs)
        self._ad_hoc: set[str] = set()
        self._clock = WaitingClock()
        self._sampler: asyncio.Task[None] | None = None

    @property
    def ad_hoc_bindings(self) -> frozenset[str]:
        """Return the names of the Bindings an operator created, rather than a deployment."""
        return frozenset(self._ad_hoc)

    @property
    def machine(self) -> Any:
        """Return the Machine this Component reports for, which is the one it hangs under."""
        return root_of(self)

    def binding_reports(self) -> tuple[BindingReport, ...]:
        """Return one report row per Binding on this Machine, sampling the clock as it goes."""
        return tuple(report for item in machinery_items(self.machine) for report in reports_of(item, self._clock))

    def recent_events(self) -> tuple[BindingEventRecord, ...]:
        """Return the BindingEvents this Machine has kept, oldest first."""
        kept: tuple[BindingEventRecord, ...] = getattr(self.machine, "ros2_recent_events", ())
        return kept

    async def bind_ad_hoc(self, binding: MonitoringBinding) -> bool:
        """Bind one topic on behalf of an operator, and remember that it was theirs.

        :return: Whether it happened; ``False`` means a mutation cap refused the nodes.
        :raises DuplicateBindingNameError: When the name is taken on this Component.
        :raises ROS2RuntimeError: When the type cannot be resolved or the subscription cannot be
            opened.
        """
        if not await self.bind_topic_now(binding, message_type=binding.message_type or ""):
            return False
        self._ad_hoc.add(binding.name)
        # Here `ros2_bindings` grows with the address space, so the report covers ad-hoc Bindings.
        self._resolved_bindings = (*self.ros2_bindings, binding)
        return True

    async def _write_identification(self) -> None:
        """Identify this Component as part of the Machine: its manufacturer, and its serial with the asset id."""
        identification: Any = getattr(self.machine, "identification", None)
        manufacturer: Any = ua.LocalizedText(INTROSPECTION_NAME)
        serial = ""
        if identification is not None and identification is not self.identification:
            manufacturer = await identification.Manufacturer.read()
            serial = str(await identification.SerialNumber.read() or "")
        await self.identification.Manufacturer.write(manufacturer)
        await self.identification.SerialNumber.write(f"{serial}/{INTROSPECTION_ID}" if serial else INTROSPECTION_ID)

    @override
    async def _init(self) -> None:
        """Build the mirror with no Bindings, add the nine Methods, then start sampling.

        The empty mirror still gives the Component its Runtime and Projection registry, which an
        ad-hoc bind needs. Nothing is subscribed yet.
        """
        await super()._init()
        await self._init_monitoring_mirror((), runtime=self.runtime, registry=self.ros2_projections)
        config = self.ros2_config
        for listing_type in (
            ListTopics,
            ListServices,
            ListActions,
            ListROS2Parameters,
            DescribeROS2Parameter,
            ListBindings,
            ListRecentEvents,
        ):
            await self.add_method(
                listing_type(name=listing_type.__name__, minimum_access_level=config.listing_access_level)
            )
        for mutator_type in (BindTopic, CallService):
            await self.add_method(
                mutator_type(name=mutator_type.__name__, minimum_access_level=config.introspection_access_level)
            )
        self._sampler = asyncio.create_task(self._sample(), name="ros2-introspection-waiting")

    @override
    async def _shutdown(self) -> None:
        """Stop sampling, then let the Component release its ad-hoc subscriptions."""
        sampler, self._sampler = self._sampler, None
        if sampler is not None:
            sampler.cancel()
            await asyncio.gather(sampler, return_exceptions=True)
        await super()._shutdown()

    async def _sample(self) -> None:
        """Sample every Binding's satisfaction on a slow timer. A failure is logged and the loop continues."""
        while True:
            await asyncio.sleep(LOSS_INTERVAL)
            try:
                self.binding_reports()
            except Exception:
                self.logger.exception("Could not sample what this Machine is bound to; the clock continues")


def _initials(plan: Sequence[PlanEntry], prefix: str = "") -> Iterator[tuple[str, object]]:
    """Yield the value each variable of a Projection's plan would have been created holding.

    A declared Method's Parameters start from these values, so an ad-hoc request that names only
    some fields sends the same message a declared Method would.
    """
    for entry in plan:
        name = flat_name(prefix, entry.full_name)
        if isinstance(entry, PlanObject):
            yield from _initials(entry.children, name)
            continue
        yield name, entry.initial


def _as_json(values: Mapping[str, object]) -> str:
    """Return one response's values as JSON, keyed by the flat field paths a request uses.

    Values that are not JSON types (a ``bytes`` payload above all) are rendered with ``str``.
    """
    return json.dumps(dict(values), default=str, sort_keys=True)
