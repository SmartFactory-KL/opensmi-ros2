# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Keyed Binding: one ROS2 path, many OPC UA objects, membership taken from the messages.

A Keyed Binding's message carries a repeated field, and a key inside each element names the object
it belongs to: ``/diagnostics`` keyed by ``status[]`` on ``hardware_id`` and ``name``. Membership
is the content-driven case. Each member is a variable group in the Binding's Monitoring that
carries the variables the element type's Projection shapes (a nested message is a complex
Variable), browses under its key with the characters of a RelativePath replaced
(:func:`member_browse_name`), shows the raw key as its DisplayName and holds it as a ``String[]``
variable. The Binding's name is the group above the members. Members are created
through the mutation chokepoint, so the caps and one model-change event per batch apply, and they
are never removed. Two keys that sanitise to one browse name are reported once as a BindingEvent
and the second gets no member.

An optional declared state field adds two things: a BindingEvent when a member's state changes
(at the severity the declaration maps it to), and one variable holding the highest state any
member holds. Nothing reads the aggregate back, so a foreign publisher cannot halt a Machine
. The mechanism is generic and has no diagnostics-specific code.
"""

from __future__ import annotations

import asyncio
import re
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Any, Final, TypeGuard

from asyncua import ua

from opensmi.ros2.runtime.messages import is_single_value, repeated_message
from opensmi.ros2.runtime.selection import resolve_selection
from opensmi.ros2.server.errors import DeclarationError, UnprojectableFieldError
from opensmi.ros2.server.events import Severity
from opensmi.ros2.server.mutation import NodeAddition, RenderedPlan, ROS2MutationMixin, ended, require_unused_name
from opensmi.ros2.server.projection import (
    EnumDataType,
    PlanVariable,
    ProjectionContext,
)
from opensmi.ros2.server.projections import DEFAULT_REGISTRY
from opensmi.ros2.server.provenance import QOS_INCOMPATIBLE, BindingQoS, qos_clash_text
from opensmi.ros2.server.status import status_code

if TYPE_CHECKING:
    from collections.abc import Iterable, Mapping, Sequence

    from opensmi.server.ua_variable_containers import UaVariableContainer

    from opensmi.ros2.runtime.interface import RuntimeInterface
    from opensmi.ros2.runtime.messages import Message
    from opensmi.ros2.runtime.subscription import Subscription
    from opensmi.ros2.server.bindings import KeyedBinding
    from opensmi.ros2.server.projection import PlanEntry, Projection, ProjectionRegistry

__all__ = [
    "KEY",
    "KEY_COLLISION",
    "MEMBER_STATE_CHANGED",
    "ROS2KeyedMirrorMixin",
    "member_browse_name",
    "member_key",
    "severity_of",
    "state_changed",
    "worst_state",
]

KEY: Final = "Key"
"""The browse name of the raw-key variable on every member (DisplayName ``key``).

A Binding whose element type already has a field that browses as this is refused at init.
"""

_KEY_DISPLAY: Final = "key"
"""The DisplayName of the raw-key variable."""

KEY_COLLISION: Final = "KEY_COLLISION"
"""The BindingEvent code a second key that sanitises onto an existing member carries."""

MEMBER_STATE_CHANGED: Final = "MEMBER_STATE_CHANGED"
"""The BindingEvent code a member's declared state field changing value carries."""

WORST: Final = "Worst"
"""What the aggregate variable's name is built from: ``Worst<State>``, such as ``WorstLevel``."""

_UNNAMED: Final = "unnamed"
"""What a key with nothing usable in it browses as, because refusing would drop the member."""

_RESERVED: Final = re.compile(r"[/.<>:#!&]")
"""The characters the OPC UA RelativePath text form reserves. Each in a member's key becomes one
underscore, so a generic client never has to escape a key."""


def _is_a_number(value: object) -> TypeGuard[int]:
    """Return whether a state value is a number to order and look labels or bands up by.

    ``bool`` is excluded: a state field is not a flag.
    """
    return isinstance(value, int) and not isinstance(value, bool)


def member_key(element: Message, key_fields: Sequence[str]) -> tuple[str, ...]:
    """Return the key one element of a repeated field carries, field by field.

    The fields are read in declaration order and rendered as text. Key fields are validated as
    scalar leaves when the Binding is built.

    :param element: One element of the Binding's repeated field, as it crossed the seam.
    :param key_fields: The ROS2 field names the declaration keys on.
    """
    return tuple(str(getattr(element, name)) for name in key_fields)


def member_browse_name(key: Sequence[str]) -> str:
    """Return the browse name a member is filed under: each reserved character of its key becomes ``_``.

    The parts of a multi-field key are joined with ``_``. ``"robot 1: left motor"`` browses as
    ``"robot 1_ left motor"`` and the raw key is the DisplayName. The mapping is not injective, and
    the caller refuses a collision.
    """
    return "_".join(_RESERVED.sub("_", part) for part in key) or _UNNAMED


def state_changed(previous: object, current: object) -> bool:
    """Return whether writing ``current`` over ``previous`` is a transition worth an event.

    A member is created holding the plan's initial (for a level, the first declared constant), so a
    healthy unit announces nothing when it first appears and a broken one does.
    """
    return previous != current


def worst_state(states: Iterable[object], *, initial: object) -> object:
    """Return the highest state any member holds, or ``initial`` when no member holds one.

    Highest by the ROS2 constants' numbering, which for ``DiagnosticStatus`` is the worst level.
    Non-numeric states have no order, so the aggregate stays at ``initial``.

    :param states: What each member's state variable currently reads.
    :param initial: What the aggregate was created holding, which is what an empty set reads.
    """
    numbers = [each for each in states if _is_a_number(each)]
    return max(numbers) if numbers else initial


def severity_of(state: object, severities: Mapping[int, Severity]) -> Severity:
    """Return the house band a transition into one state is reported at.

    The map comes from the declaration (see :class:`~opensmi.ros2.server.events.Severity`). With no
    map every transition is ``INFO``; with a map, a value outside it is a ``WARNING``.
    """
    if not severities:
        return Severity.INFO
    if _is_a_number(state) and state in severities:
        return severities[state]
    return Severity.WARNING


@dataclass(slots=True)
class _Member:
    """One member of a Keyed Binding: its key, the nodes it became, and what they read."""

    key: tuple[str, ...]
    browse: str
    rendered: RenderedPlan
    values: dict[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Start out holding whatever the plan created each variable with."""
        self.values = self.rendered.initial


@dataclass(slots=True)
class _Keyed:
    """One Keyed Binding, live: its subscription, the plan each member is built from, its members."""

    binding: KeyedBinding
    subscription: Subscription
    projection: Projection
    context: ProjectionContext
    element_type: type[Message]
    group: UaVariableContainer
    plan: tuple[PlanEntry, ...]
    state_name: str | None
    aggregate: PlanVariable | None
    aggregate_name: str | None
    aggregate_nodes: RenderedPlan | None = None
    members: dict[tuple[str, ...], _Member] = field(default_factory=dict)
    claimed: dict[str, tuple[str, ...]] = field(default_factory=dict)
    refused: set[tuple[str, ...]] = field(default_factory=set)
    aggregate_value: object = None
    writing: asyncio.Lock = field(default_factory=asyncio.Lock)
    task: asyncio.Task[None] | None = None
    status_task: asyncio.Task[None] | None = None
    qos_task: asyncio.Task[None] | None = None
    satisfied: bool = False
    sampled: bool = False
    capped: bool = False
    reported_unknown_name: bool = False

    def __post_init__(self) -> None:
        """Start out with the aggregate reading what its node was created holding."""
        self.aggregate_value = None if self.aggregate is None else self.aggregate.initial

    @property
    def status(self) -> ua.StatusCode:
        """Return the status code every variable of this Binding currently reads."""
        return status_code(satisfied=self.satisfied, sampled=self.sampled)

    def elements(self, message: Message) -> tuple[Message, ...]:
        """Return the repeated field of one message, which is the Binding's whole subject."""
        return tuple(getattr(message, self.binding.repeated_field))

    def state_of(self, member: _Member) -> object:
        """Return what one member's declared state variable reads, or ``None`` if there is none."""
        if self.state_name is None:
            return None
        return member.values.get(self.state_name)


class ROS2KeyedMirrorMixin(ROS2MutationMixin):
    """Keeps a machinery item's Keyed Bindings' members in sync with what arrives on their topics.

    Mixed in ahead of opensmi's class. The mutation chokepoint it extends creates the nodes; this
    mixin decides which nodes there should be. It runs loops parallel to the inward mirror's
    (stream, publisher count, QoS) on purpose, since sharing them would need a callback at every
    step.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Start out with no Keyed Bindings, and pass every argument to opensmi."""
        super().__init__(*args, **kwargs)
        self._keyed: list[_Keyed] = []
        self._keyed_runtime: RuntimeInterface | None = None
        self._keyed_registry: ProjectionRegistry = DEFAULT_REGISTRY

    @property
    def ros2_keyed_presence(self) -> Mapping[str, bool]:
        """Return, per Keyed Binding name, whether its topic has a publisher right now.

        Presence works as for a topic read into Monitoring and does not depend on membership: a
        Binding with a publisher but no member yet is a robot with nothing to report.
        """
        return {each.binding.name: each.satisfied for each in self._keyed}

    @property
    def ros2_members(self) -> Mapping[str, tuple[str, ...]]:
        """Return, per Keyed Binding name, the browse name of each member it has created.

        In creation order. Members are append-only, so this only grows.
        """
        return {each.binding.name: tuple(member.browse for member in each.members.values()) for each in self._keyed}

    @property
    def ros2_keyed_qos_provenance(self) -> Mapping[str, BindingQoS]:
        """Return, per Keyed Binding name, what its subscription asks for and where it came from.

        The counterpart of
        :attr:`~opensmi.ros2.server.mirror.ROS2InwardMirrorMixin.ros2_qos_provenance`.
        """
        return {
            each.binding.name: BindingQoS.of(
                each.binding.name,
                each.binding.path,
                each.subscription,
                each.subscription.qos_incompatibilities,
            )
            for each in self._keyed
        }

    async def _init_keyed_mirror(
        self,
        bindings: Iterable[KeyedBinding],
        *,
        runtime: RuntimeInterface,
        registry: ProjectionRegistry = DEFAULT_REGISTRY,
    ) -> None:
        """Open, shape and start every Keyed Binding, in declaration order.

        Fatal at init on a bad declaration: a field that does not repeat a message, a key field
        that is not a scalar leaf, a state field the Projection never renders, an element type
        that is not installed. What a message carries is never fatal. Only the
        aggregate is created here; members appear with messages.
        """
        self._keyed_runtime = runtime
        self._keyed_registry = registry
        try:
            for binding in bindings:
                self._keyed.append(await self._open_keyed(binding))
        except BaseException:
            await self._stop_keyed_mirror()
            raise
        for keyed in self._keyed:
            await self._start_following_keyed(keyed)

    async def _stop_keyed_mirror(self) -> None:
        """Stop following every Keyed Binding and close its subscription.

        Idempotent. Runs before the Runtime is stopped, so a Component torn down on its own
        still stops reading.
        """
        for keyed in self._keyed:
            for task in (keyed.task, keyed.status_task, keyed.qos_task):
                if task is not None:
                    task.cancel()
        for keyed in self._keyed:
            await ended(keyed.task)
            await ended(keyed.status_task)
            await ended(keyed.qos_task)
            keyed.task = keyed.status_task = keyed.qos_task = None
            await keyed.subscription.close()
        self._keyed = []

    async def _open_keyed(self, binding: KeyedBinding) -> _Keyed:
        """Subscribe, shape the member plan, and create one Keyed Binding's aggregate."""
        runtime = self._require_keyed_runtime()
        subscription = await runtime.subscribe(
            topic=binding.path,
            message_type=binding.message_type,
            depth=binding.depth,
            qos=binding.qos,
        )
        try:
            element_type = _repeated_element(subscription.message_type, binding)
            context = ProjectionContext(
                registry=self._keyed_registry,
                selected=resolve_selection(element_type, binding.selection),
                qualifier=self.ros2_projection_qualifier,
                byte_string_ceiling=self._byte_string_ceiling,
            )
            projection = self._keyed_registry.find(element_type.ros2.type_name)
            _require_scalar_key_fields(element_type, binding)
            # Looked up in the shaped plan, so naming the raw key as state field is refused.
            shaped = projection.shape(element_type, context)
            state = _state_entry(shaped, binding, element_type, context.qualifier)
            plan = _with_state(_member_plan(shaped, binding), state)
            parent = self.monitoring
            if binding.group is not None:
                parent = await self.machinery_group(parent, binding.group)
            require_unused_name(binding.name, (parent,), "this Monitoring")
            group = await parent.add_group(binding.name)
            aggregate = (
                None if state is None else replace(state, name=f"{WORST}{state.name}", display_name=None, above=())
            )
            keyed = _Keyed(
                binding=binding,
                subscription=subscription,
                projection=projection,
                context=context,
                element_type=element_type,
                group=group,
                plan=plan,
                state_name=None if state is None else state.full_name,
                aggregate=aggregate,
                aggregate_name=None if aggregate is None else aggregate.full_name,
            )
            await self._create_aggregate(keyed)
        except BaseException:
            await subscription.close()
            raise
        return keyed

    async def _create_aggregate(self, keyed: _Keyed) -> None:
        """Create the aggregate variable in the Binding's group, beside the members, at init."""
        if keyed.aggregate is None:
            return
        [keyed.aggregate_nodes] = await self.mutate_address_space(
            [
                NodeAddition(
                    binding=keyed.binding.name,
                    path=keyed.binding.path,
                    container=keyed.group,
                    plan=(keyed.aggregate,),
                    message_type=keyed.element_type,
                )
            ],
            at_init=True,
        )

    async def _start_following_keyed(self, keyed: _Keyed) -> None:
        """Put the first status on what a Keyed Binding already has, then start its tasks."""
        await self._write_keyed_status(keyed)
        name = keyed.binding.name
        keyed.task = asyncio.create_task(self._follow_keyed(keyed), name=f"ros2-keyed-{name}")
        keyed.status_task = asyncio.create_task(self._follow_keyed_publishers(keyed), name=f"ros2-keyed-pub-{name}")
        keyed.qos_task = asyncio.create_task(self._follow_keyed_qos(keyed), name=f"ros2-keyed-qos-{name}")

    async def _follow_keyed_qos(self, keyed: _Keyed) -> None:
        """Raise one BindingEvent for every QoS incompatibility reported on this Binding's topic.

        Like the inward mirror's ``_follow_qos``, with the same code.
        """
        async for incompatibility in keyed.subscription.follow_qos_incompatibilities():
            try:
                await self.emit_binding_event(
                    binding=keyed.binding.name,
                    path=keyed.binding.path,
                    code=QOS_INCOMPATIBLE,
                    severity=Severity.WARNING if incompatibility.adapted is not None else Severity.ERROR,
                    text=qos_clash_text(keyed.binding.path, incompatibility),
                )
            except Exception:
                # Never fatal; one failed event must not end the stream.
                self.logger.exception(
                    "Could not report a QoS incompatibility",
                    binding=keyed.binding.name,
                    topic=keyed.binding.path,
                )

    async def _follow_keyed(self, keyed: _Keyed) -> None:
        """Write every message of one Keyed Binding, until the subscription closes."""
        async for message in keyed.subscription:
            try:
                await self._write_keyed_message(keyed, message)
            except Exception:
                # Never fatal; a Binding that stopped reading would go quiet silently.
                self.logger.exception(
                    "Could not write a keyed message to its members",
                    binding=keyed.binding.name,
                    topic=keyed.binding.path,
                )

    async def _follow_keyed_publishers(self, keyed: _Keyed) -> None:
        """Write the status code every time the topic gains or loses its publishers."""
        async for count in keyed.subscription.publisher_counts():
            keyed.satisfied = count > 0
            try:
                await self._write_keyed_status(keyed)
            except Exception:
                self.logger.exception(
                    "Could not write a status code to a Keyed Binding's members",
                    binding=keyed.binding.name,
                    topic=keyed.binding.path,
                )

    async def _write_keyed_message(self, keyed: _Keyed, message: Message) -> None:
        """Give every element of one message its member, creating the ones that are new."""
        transitions: list[tuple[_Member, object, object]] = []
        async with keyed.writing:
            keyed.sampled = True
            elements = keyed.elements(message)
            await self._create_new_members(keyed, elements)
            for element in elements:
                member = keyed.members.get(member_key(element, keyed.binding.key))
                if member is None:
                    continue
                transition = await self._write_member(keyed, member, element)
                if transition is not None:
                    transitions.append(transition)
            await self._write_aggregate(keyed)
        # Outside the lock and after the values are written, so a woken client reads the new state.
        for member, previous, current in transitions:
            await self._report_transition(keyed, member, previous, current)

    async def _create_new_members(self, keyed: _Keyed, elements: Sequence[Message]) -> None:
        """Add every key this message names for the first time, in one mutation.

        One chokepoint call gives one model-change event for the whole batch. A key that cannot
        have a member (its browse name is taken by another key or by something else on this
        Monitoring, or a cap closed the Binding) is reported once and remembered as refused, so a
        10 Hz topic raises one event and not one per message.
        """
        binding = keyed.binding
        additions: list[NodeAddition] = []
        pending: list[tuple[tuple[str, ...], str]] = []
        for element in elements:
            key = member_key(element, binding.key)
            if key in keyed.members or key in keyed.refused or key in {each for each, _ in pending}:
                continue
            browse = member_browse_name(key)
            taken = keyed.claimed.get(browse) or next((each for each, name in pending if name == browse), None)
            if taken is not None:
                keyed.refused.add(key)
                await self._report_collision(keyed, key, taken, browse)
                continue
            if browse in keyed.group:
                # The name belongs to something that is not a member, e.g. the aggregate (an id
                # called `WorstLevel`). Asked before the chokepoint so one bad name does not
                # take the rest of the batch down.
                keyed.refused.add(key)
                await self._report_collision(keyed, key, (), browse)
                continue
            if keyed.capped:
                keyed.refused.add(key)
                continue
            pending.append((key, browse))
            additions.append(
                NodeAddition(
                    binding=binding.name,
                    path=binding.path,
                    container=keyed.group,
                    plan=_keyed_plan(keyed.plan, key),
                    message_type=keyed.element_type,
                    member=(browse, "_".join(key)),
                )
            )
        if not additions:
            return
        created = await self.mutate_address_space(additions)
        for (key, browse), nodes in zip(pending, created, strict=True):
            if nodes is None:
                # A cap refused it, and has already reported itself.
                keyed.capped = True
                keyed.refused.add(key)
                continue
            keyed.claimed[browse] = key
            keyed.members[key] = _Member(key=key, browse=browse, rendered=nodes)

    async def _write_member(
        self,
        keyed: _Keyed,
        member: _Member,
        element: Message,
    ) -> tuple[_Member, object, object] | None:
        """Decode one element onto its member's variables and return the state transition, if any.

        The caller reports it once the values are on their nodes.
        """
        previous = keyed.state_of(member)
        for name, value in keyed.projection.decode(element, keyed.context).items():
            if name not in member.rendered.entries:
                self._report_unknown_keyed_name(keyed, name)
                continue
            member.values[name] = value
        # Every variable, not just the decoded ones: the raw key is not in a decode and all
        # carry the status, which also gives a new member its first status.
        await member.rendered.write(member.values, keyed.status)
        current = keyed.state_of(member)
        if keyed.state_name is None or not state_changed(previous, current):
            return None
        return member, previous, current

    async def _write_aggregate(self, keyed: _Keyed) -> None:
        """Write the highest state any member holds, when this Binding has such a variable."""
        if keyed.aggregate is None or keyed.aggregate_name is None or keyed.aggregate_nodes is None:
            return
        keyed.aggregate_value = worst_state(
            (keyed.state_of(member) for member in keyed.members.values()),
            initial=keyed.aggregate.initial,
        )
        await keyed.aggregate_nodes.write({keyed.aggregate_name: keyed.aggregate_value}, keyed.status)

    async def _write_keyed_status(self, keyed: _Keyed) -> None:
        """Re-write everything this Binding owns under the status its publishers now imply.

        The value goes with it: a status written alone leaves the variant null.
        """
        async with keyed.writing:
            for member in keyed.members.values():
                await member.rendered.write(member.values, keyed.status)
            # Recomputed so the aggregate's value is decided in one place.
            await self._write_aggregate(keyed)

    async def _report_transition(self, keyed: _Keyed, member: _Member, previous: object, current: object) -> None:
        """Fire the BindingEvent for a member's state change."""
        await self.emit_binding_event(
            binding=keyed.binding.name,
            path=keyed.binding.path,
            code=MEMBER_STATE_CHANGED,
            severity=severity_of(current, keyed.binding.severities),
            text=(
                f"{member.browse} moved {keyed.binding.state_field} from "
                f"{_labelled(previous, keyed.binding.states)} to {_labelled(current, keyed.binding.states)}"
            ),
        )

    async def _report_collision(
        self,
        keyed: _Keyed,
        key: tuple[str, ...],
        claimed: tuple[str, ...],
        browse: str,
    ) -> None:
        """Say, as a BindingEvent, that a key has no member because its name is already taken.

        :param claimed: The key that holds the name, where one does. Empty when the name belongs
            to something that is not a member -- this Binding's own aggregate, say.
        """
        self.logger.error(
            "Refused a member whose key collides with a name already taken",
            binding=keyed.binding.name,
            key=list(key),
            browse_name=browse,
        )
        held_by = f"{list(claimed)} already holds" if claimed else "is already taken on this Monitoring"
        await self.emit_binding_event(
            binding=keyed.binding.name,
            path=keyed.binding.path,
            code=KEY_COLLISION,
            severity=Severity.ERROR,
            text=(
                f"the key {list(key)} browses as {browse!r}, which {held_by}; it has no member, "
                f"because sharing one would file two things under one name"
            ),
        )

    def _report_unknown_keyed_name(self, keyed: _Keyed, name: str) -> None:
        """Log once per Binding a decoded name its shape never created.

        Unreachable with the shipped Projections; a curated one whose halves disagree only shows
        up with a message.
        """
        if keyed.reported_unknown_name:
            return
        keyed.reported_unknown_name = True
        self.logger.error(
            "A Projection decoded a name its shape never created; those values are not written",
            binding=keyed.binding.name,
            projection=keyed.projection.type_name,
            name=name,
        )

    def _require_keyed_runtime(self) -> RuntimeInterface:
        """Return the Runtime this mirror was initialised with.

        :raises DeclarationError: When there is none, which is a wiring fault and never a
            ROS2 condition.
        """
        if self._keyed_runtime is None:  # pragma: no cover -- every path here runs after init
            message = f"{type(self).__name__} has no Runtime; its Keyed Bindings were never initialised"
            raise DeclarationError(message)
        return self._keyed_runtime


def _repeated_element(message_type: type[Message], binding: KeyedBinding) -> type[Message]:
    """Return the message type one Binding's repeated field carries.

    :raises UnprojectableFieldError: When the named field does not exist or does not repeat a
        message.
    """
    nested = repeated_message(message_type, binding.repeated_field)
    if nested is None:
        listed = ", ".join(each.name for each in message_type.ros2.fields) or "nothing"
        message = (
            f"the Keyed Binding {binding.name!r} keys on {binding.repeated_field!r} of "
            f"{message_type.ros2.type_name}, which is not a field that repeats a message; its "
            f"fields are {listed}, and a Keyed Binding renders one member per element of one of "
            f"them"
        )
        raise UnprojectableFieldError(message)
    return nested


def _require_scalar_key_fields(element_type: type[Message], binding: KeyedBinding) -> None:
    """Refuse a key field that cannot name a member.

    :raises DeclarationError: When a key field is absent from the element type, or is not a
        scalar leaf of it.
    """
    for name in binding.key:
        described = next((each for each in element_type.ros2.fields if each.name == name), None)
        if described is not None and is_single_value(described.field_type):
            continue
        listed = ", ".join(each.name for each in element_type.ros2.fields) or "nothing"
        message = (
            f"the Keyed Binding {binding.name!r} keys on {name!r}, which is not a single value of "
            f"{element_type.ros2.type_name}; its fields are {listed}, and a key names a member"
        )
        raise DeclarationError(message)


def _member_plan(shaped: tuple[PlanEntry, ...], binding: KeyedBinding) -> tuple[PlanEntry, ...]:
    """Return the plan one member is built from: the raw key, then whatever the Projection shaped.

    :raises DeclarationError: When the element type already renders something called :data:`KEY`.
    """
    if any(isinstance(entry, PlanVariable) and entry.name == KEY for entry in shaped):
        message = (
            f"the Keyed Binding {binding.name!r} renders a member with a {KEY!r} of its own, and "
            f"{KEY!r} is where every member carries its raw key; exclude that field from this "
            f"Binding, or curate the type with a Projection that renames it"
        )
        raise DeclarationError(message)
    return (
        PlanVariable(
            name=KEY,
            source=binding.repeated_field,
            initial=[],
            variant_type=ua.VariantType.String,
            display_name=_KEY_DISPLAY,
        ),
        *shaped,
    )


def _state_entry(
    plan: tuple[PlanEntry, ...],
    binding: KeyedBinding,
    element_type: type[Message],
    qualifier: str,
) -> PlanVariable | None:
    """Return the variable a Binding's declared state field renders as, with its labels named.

    The labels come from the declaration, not the Projection, so the member's state variable and
    the aggregate share one enumeration.

    :raises DeclarationError: When the element's Projection renders no variable of that name (the
        field was excluded or renamed).
    """
    if binding.state_field is None:
        return None
    found = next(
        (entry for entry in plan if isinstance(entry, PlanVariable) and entry.source == binding.state_field), None
    )
    if not isinstance(found, PlanVariable):
        listed = ", ".join(entry.full_name for entry in plan) or "nothing"
        message = (
            f"the Keyed Binding {binding.name!r} names {binding.state_field!r} as its state "
            f"field, and a member of it renders {listed}; an excluded or renamed field leaves "
            f"nothing to watch"
        )
        raise DeclarationError(message)
    if not binding.states:
        return found
    local = element_type.ros2.type_name.rpartition("/")[2]
    parts = (qualifier, local, binding.state_field)
    return replace(
        found,
        # An OPC UA enumeration is an Int32; the value written is the ROS2 constant.
        variant_type=ua.VariantType.Int32,
        datatype=EnumDataType(
            name="_".join(part for part in parts if part),
            values=tuple(sorted(binding.states.items())),
        ),
    )


def _with_state(plan: tuple[PlanEntry, ...], state: PlanVariable | None) -> tuple[PlanEntry, ...]:
    """Return the member plan with the state field's variable as the state entry describes it."""
    if state is None:
        return plan
    return tuple(state if entry.full_name == state.full_name else entry for entry in plan)


def _keyed_plan(plan: tuple[PlanEntry, ...], key: tuple[str, ...]) -> tuple[PlanEntry, ...]:
    """Return one member's plan, with its raw key as the initial value of the key variable.

    The node is created holding the key, so a client never reads it empty.
    """
    return tuple(replace(entry, initial=list(key)) if entry.name == KEY else entry for entry in plan)


def _labelled(state: object, states: Mapping[int, str]) -> str:
    """Return a state value as an event's text should read it: the label, and the number."""
    if _is_a_number(state) and state in states:
        return f"{states[state]} ({state})"
    return str(state)
