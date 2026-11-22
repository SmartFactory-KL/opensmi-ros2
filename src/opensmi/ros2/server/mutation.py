# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The mutation chokepoint: the one place a Binding's nodes are made.

Declared Bindings create their nodes once at init and never move them. The three cases where the
address space grows at run time (graph-driven, client-driven and content-driven) all go
through here. This module renders a Projection's plan under an opensmi container
(:meth:`ROS2MutationMixin.render_plan`) and announces each batch as one ``GeneralModelChangeEvent``
(nothing is announced at init). Nodes are created and added to, never removed.

Structure is nesting. A message becomes one complex Variable whose value is the whole
structure, with a child Variable per field and a nested complex Variable per nested message, and a
group (a Binding, a ROS2 parameter namespace, a Keyed member) is a variable group. A message is
written once, at the root of its complex Variable, and opensmi brings the children in step. A plan
with a leaf no structure field can carry (a DateTime, say) renders its levels as variable groups of
plain Variables instead. Growth is capped, per Binding (:attr:`ROS2MutationMixin.mutation_node_cap`)
and per Machine (:attr:`ROS2MutationMixin.machine_mutation_node_cap`). Reaching a cap refuses the
whole addition and reports it as a BindingEvent. Caps count mutations only, not what a declaration
asks for at init.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from datetime import UTC
from typing import TYPE_CHECKING, Any, Final

from asyncua import ua
from asyncua.common.structures104 import new_enum_field
from asyncua.ua.attribute_ids import AttributeIds
from asyncua.ua.object_ids import ObjectIds
from asyncua.ua.status_codes import StatusCodes
from asyncua.ua.uaerrors import BadNoMatch
from opensmi.core.nodesets import MachineryNodeIds
from opensmi.server import UaVariable
from opensmi.server.errors import TypeConflictError
from opensmi.server.mixins import MonitoringMixin
from opensmi.server.ua_variable_containers import UaVariableContainer

from opensmi.ros2.server.browse_names import pascal_case
from opensmi.ros2.server.errors import (
    DeclarationError,
    DuplicateProjectionError,
    DuplicateVariableNameError,
    UnprojectableFieldError,
)
from opensmi.ros2.server.events import ROS2EventsMixin, Severity
from opensmi.ros2.server.projection import (
    EnumDataType,
    NodeLevel,
    PlanObject,
    PlanVariable,
    ProjectDataType,
    StandardDataType,
    curated_naming,
    inner_levels,
    level_datatypes,
    level_value,
)
from opensmi.ros2.server.status import GOOD, status_value
from opensmi.ros2.server.structures import (
    DuplicateStructureError,
    structure_elements,
    structure_instance,
)

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable, Container, Iterable, Mapping, Sequence

    from asyncua.common.node import Node

    from opensmi.ros2.runtime.messages import Message
    from opensmi.ros2.server.projection import NodePath, PlanDataType, PlanEntry

__all__ = [
    "CAP_REACHED",
    "NodeAddition",
    "ROS2MutationMixin",
    "RenderedPlan",
    "add_plan_variable",
    "ended",
    "ensure_groups",
    "require_unused_name",
]

CAP_REACHED: Final = "NODE_CAP_REACHED"
"""The BindingEvent code a refused addition carries, for a client that wants to know why."""

_NODE_ADDED: Final = ua.Byte(1)
"""``ModelChangeStructureVerb.NodeAdded``, announced for every creation here."""

_UNKNOWN_LENGTH: Final = ua.UInt32(0)
"""The ArrayDimensions entry every array variable publishes: length unknown."""

_ENUM_NAMESPACE: Final = 1
"""The namespace enumeration DataTypes are minted into: the server's own."""

_STATUS: Final = "Status"
"""The Machinery Monitoring sub-structure that references the item's state machines (OPC 40001-1 section 16.1)."""

_MACHINERY_STATE: Final = ("MachineryItemState", "MachineryOperationMode")
"""The AddIns a ``Status`` group references, by their browse names below MachineryBuildingBlocks."""


@dataclass(frozen=True, slots=True, kw_only=True)
class NodeAddition:
    """One plan's nodes to create in one container, on behalf of one Binding.

    :ivar binding: The Binding these nodes belong to. The cap counts against it and a refusal's
        BindingEvent names it.
    :ivar path: The ROS2 topic, service, action or node path of that Binding.
    :ivar container: The opensmi container the nodes are added to.
    :ivar plan: What to create, as the Projection shaped it.
    :ivar message_type: The ROS2 type the plan was shaped from, which names the root's DataType.
    :ivar root: The name of the complex Variable the whole plan is rendered as, for a Binding's
        message. Without it the plan's top-level fields are the container's own children.
    :ivar member: A Keyed Binding's member as its browse name and DisplayName. The plan is then
        rendered inside a new group of that name, a child of the container.
    :ivar group: The Machinery Monitoring sub-structure the nodes go in, a FolderType Object in the
        container that the first addition naming it creates. ``None`` adds to the container.
    """

    binding: str
    path: str
    container: UaVariableContainer
    plan: tuple[PlanEntry, ...]
    message_type: type[Message]
    root: str | None = None
    member: tuple[str, str] | None = None
    group: str | None = None


def wire_value(entry: PlanVariable, value: object) -> object:
    """Return a plan value in the form its variable stores.

    An array of structures stores instances of the structure's dataclass. Other variables store
    the value as is.

    :raises ValueError: When the value for an array of structures is not a list.
    """
    if not isinstance(entry.datatype, type):
        return value
    structure = entry.datatype
    if not isinstance(value, (list, tuple)):
        message = f"{type(value).__name__} is not an array of {structure.__name__}"
        raise ValueError(message)  # noqa: TRY004
    return [structure_instance(structure, each) for each in value]  # pyright: ignore[reportArgumentType]


def plan_value(entry: PlanVariable, stored: object) -> object:
    """Return what a variable stores in the form a projection builds a message from.

    The inverse of :func:`wire_value`: an array of structures comes back as maps.

    :raises ValueError: When an array of structures holds something else, which the write check
        normally refuses first.
    """
    return structure_elements(entry.datatype, stored) if isinstance(entry.datatype, type) else stored


async def ensure_groups(container: UaVariableContainer, levels: Iterable[NodeLevel]) -> UaVariableContainer:
    """Return the variable group at the end of a chain of levels, creating those that do not exist yet.

    A level's name is its browse name and its display name the raw ROS2 name. A group two plans
    share (a ROS2 parameter namespace) is created by the first and found by the second.
    """
    for level in levels:
        if level.name in container.groups:
            container = container.groups[level.name]
        else:
            display = None if level.display_name == level.name else level.display_name
            container = await container.add_group(level.name, display_name=display)
    return container


@dataclass(slots=True)
class _Plain:
    """A plan entry rendered as a Variable of its own."""

    variable: UaVariable[Any]
    entry: PlanVariable


@dataclass(slots=True)
class _Complex:
    """The leaves below one path rendered as a complex Variable, which holds their whole structure."""

    variable: UaVariable[Any]
    leaves: list[PlanVariable]
    classes: Mapping[NodePath, type]
    path: NodePath


class RenderedPlan:
    """The nodes one plan became, written and read by the names the plan gave its entries.

    A complex Variable takes all the values below it in one write, a plain one takes its own. The
    values are the Projection's: a map from the plan's flat names (``PlanVariable.full_name``) to
    the value of that entry, whole, since the structure is built from all of them.

    :ivar entries: The plan entry behind each name.
    """

    def __init__(self, entries: Mapping[str, PlanVariable], parts: Sequence[_Plain | _Complex]) -> None:
        """Remember what was created."""
        self.entries = dict(entries)
        self._parts = list(parts)

    @property
    def initial(self) -> dict[str, object]:
        """Return the value each entry was created holding."""
        return {name: entry.initial for name, entry in self.entries.items()}

    async def write(self, values: Mapping[str, object], status: ua.StatusCode = GOOD) -> None:
        """Write every node of the plan in one call, each with the status, from a map of all the values.

        The value always travels with the status, because a status alone leaves the variant null.
        The call is the server's own write service, so each node still goes through its access
        control and write check, and all carry one SourceTimestamp. A node that refuses does not
        keep the others from being written; the first refusal is raised once the call is done.

        :raises KeyError: When an entry has no value.
        :raises UaStatusCodeError: When a node refuses its value, with the first refusal's status.
        """
        if not self._parts:
            return
        stamp = ua.DateTime.now(UTC)
        writes: list[ua.WriteValue] = []
        for part in self._parts:
            if isinstance(part, _Plain):
                value = wire_value(part.entry, values[part.entry.full_name])
                data = status_value(value, part.entry.variant_type, status)
            else:
                structure = level_value(part.leaves, part.classes, values, part.path)
                data = status_value(structure, ua.VariantType.ExtensionObject, status)
            data.SourceTimestamp = stamp
            write = ua.WriteValue()
            write.NodeId = part.variable.ua_node.nodeid
            write.AttributeId = AttributeIds.Value
            write.Value = data
            writes.append(write)
        results = await self._parts[0].variable.ua_node.session.write(ua.WriteParameters(writes))
        for result in results:
            result.check()

    async def read(self) -> dict[str, object]:
        """Return what the nodes hold, by the plan's names, in the form a projection builds a message from.

        :raises ValueError: When an array of structures holds something that is not one.
        """
        values: dict[str, object] = {}
        for part in self._parts:
            if isinstance(part, _Plain):
                values[part.entry.full_name] = plan_value(part.entry, await part.variable.read())
                continue
            structure = await part.variable.read()
            for leaf in part.leaves:
                value: Any = structure
                for field_name in leaf.display_names[len(part.path) :]:
                    value = getattr(value, field_name)
                values[leaf.full_name] = plan_value(leaf, value)
        return values


async def _add(container: UaVariableContainer, name: str, variable: UaVariable[Any]) -> None:
    """Add a variable to a container, leaving nothing registered when that fails.

    opensmi files a variable before creating its node, so a failed add would leave a node-less
    variable that breaks later iteration, and would replace what the name held. Both are undone.
    """
    # Not a dict.get: an opensmi container answers [] and `in`, and has no get.
    previous = container[name] if name in container else None  # noqa: SIM401
    try:
        await container.add(variable)
    except BaseException:
        if previous is None:
            delattr(container, name)
        else:
            setattr(container, name, previous)
        raise


async def add_plan_variable(
    container: UaVariableContainer,
    name: str,
    entry: PlanVariable,
    datatype: ua.NodeId | None,
    *,
    writable: bool = False,
    write_check: Callable[[Any], Awaitable[None]] | None = None,
    unit: str | None = None,
    value_range: tuple[float, float] | None = None,
) -> UaVariable[Any]:
    """Turn one plan entry into an opensmi ``UaVariable``, add it to a container, and type it.

    This is the one place DataType, ValueRank and ArrayDimensions of a plain Variable are decided.
    The variable is created with an explicit ``Variant`` (an empty sequence implies no type, and
    opensmi writes later values with the initial ``VariantType``) and with these attributes:

    * ValueRank ``OneDimension`` for an array, ``Scalar`` otherwise.
    * ArrayDimensions ``[0]`` for an array (length unknown), written after the add since opensmi
      has no parameter for it. A bounded ROS2 sequence publishes no bound.
    * DataType: the plan's own, else the built-in type of the variant type. opensmi refuses one
      on a variable with a unit (an ``AnalogUnitRangeType``), so that one is written after the add.

    An array of structures (plan DataType a structure class) is created empty, with the
    resolved DataType. Its write check refuses with ``BadTypeMismatch`` a value that is not an
    array of that structure, then runs the caller's ``write_check``.

    A failed add leaves nothing registered: the variable is removed from the container again and
    any variable it replaced is put back. The attribute writes pass no write check.

    :param container: The opensmi container to add the variable to.
    :param name: The variable's BrowseName.
    :param entry: The plan entry it is made from. Its DisplayName is the raw ROS2 name.
    :param datatype: The DataType the plan names, resolved on this server, or ``None``. Required
        for an array of structures; its namespace index is the structure's encoding namespace.
    :param writable: Whether a client may write it.
    :param write_check: An async check opensmi awaits before storing a written value.
    :param unit: The engineering unit's symbol, for a variable that is not a ROS2 field. opensmi
        requires a range with it.
    :param value_range: The bounds opensmi publishes as an ``EURange`` and refuses a write outside.

    :raises DeclarationError: When the plan names a structure for something that is not an array,
        or no resolved DataType comes with it.
    """
    structure = entry.datatype if isinstance(entry.datatype, type) else None
    builtin = ua.NodeId(ua.Int32(entry.variant_type.value))
    if structure is not None:
        if datatype is None or not entry.is_array:
            message = (
                f"the variable {name!r} names the structure {structure.__name__} and must be an array "
                f"of it with the DataType resolved; a repeated message is rendered that way"
            )
            raise DeclarationError(message)
        write_check = _structure_write_check(lambda: variable, name, structure, write_check)
    options: dict[str, Any] = {
        "name": name,
        "display_name": entry.display if entry.display not in ("", name) else None,
        "writable": writable,
        "write_check_callback": write_check,
        "unit": unit,
        "range": value_range,
        "ua_value_rank": ua.ValueRank.OneDimension if entry.is_array else ua.ValueRank.Scalar,
        "ua_data_type": None if unit is not None else datatype or builtin,
        # The ROS2 path, so a client can trace a node back to its field.
        "description": entry.source or name,
    }
    variable: UaVariable[Any]
    if structure is None:
        variable = UaVariable(ua.Variant(entry.initial, entry.variant_type), **options)
    else:
        variable = UaVariable([], variable_type=structure, **options)
    await _add(container, name, variable)
    node = variable.ua_node
    if entry.is_array:
        await _write_unbounded(node)
    if unit is not None:
        await node.write_attribute(
            AttributeIds.DataType,
            ua.DataValue(ua.Variant(datatype or builtin, ua.VariantType.NodeId)),
        )
    return variable


async def _write_unbounded(node: Node) -> None:
    """Publish an array's length as unknown. Through write_attribute, as asyncua types it as one int."""
    await node.write_attribute(
        AttributeIds.ArrayDimensions,
        ua.DataValue(ua.Variant([_UNKNOWN_LENGTH], ua.VariantType.UInt32)),
    )


async def _write_display_name(node: Node, text: str) -> None:
    """Write a node's DisplayName, which is where the raw ROS2 name, key or frame id goes."""
    await node.write_attribute(
        AttributeIds.DisplayName,
        ua.DataValue(ua.Variant(ua.LocalizedText(text), ua.VariantType.LocalizedText)),
    )


def _structure_write_check(
    variable: Callable[[], UaVariable[Any]],
    name: str,
    structure: type,
    then: Callable[[Any], Awaitable[None]] | None,
    *,
    array: bool = True,
) -> Callable[[Any], Awaitable[None]]:
    """Return the write check of a structure, or of an array of one: the codec's, then the caller's.

    It refuses with ``BadTypeMismatch`` a value that is not one (or an array of them), a null field,
    or a fixed-size array of the wrong length, which the encoding does not publish.

    :param variable: The variable the check belongs to, resolved lazily since the check is made
        first.
    :param array: Whether the variable holds an array of the structure and not one.
    :param then: The caller's check, awaited when the value passed.
    """

    async def check(value: object) -> None:
        try:
            structure_elements(structure, value if array else [value])
        except ValueError as error:
            # A status code is all the client gets, so the reason is logged.
            variable().logger.warning("Refused a write that is not the structure", variable=name, reason=str(error))
            raise ua.UaStatusCodeError(StatusCodes.BadTypeMismatch) from error
        if then is not None:
            await then(value)

    return check


class ROS2MutationMixin(MonitoringMixin, ROS2EventsMixin):
    """The one door through which a machinery item's Monitoring may grow.

    Mixed in ahead of opensmi's class. It brings opensmi's Monitoring container, where the
    variables go, and extends the events mixin because a refused mutation is a BindingEvent.

    :cvar mutation_node_cap: How many nodes one Binding may gain by mutation. A Component with a
        larger Binding overrides it.
    :cvar machine_mutation_node_cap: How many nodes one Machine may gain by mutation, across
        every machinery item on it.
    """

    mutation_node_cap: int = 200
    machine_mutation_node_cap: int = 2000

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Start out having mutated nothing, and pass every argument to opensmi."""
        super().__init__(*args, **kwargs)
        self._mutation_nodes: dict[str, int] = {}
        self._machine_mutation_nodes = 0
        # Read off the Machine: enumeration DataTypes live in the server's address space.
        self._ros2_enum_datatypes: dict[str, tuple[EnumDataType, ua.NodeId]] = {}

    @property
    def mutation_node_counts(self) -> Mapping[str, int]:
        """Return how many nodes each Binding on this item has gained by mutation."""
        return dict(self._mutation_nodes)

    @property
    def ros2_enum_datatypes(self) -> dict[str, tuple[EnumDataType, ua.NodeId]]:
        """Return the enumeration DataTypes minted on this Machine, by browse name.

        Only the Machine's is read, by every item on it. Beside each NodeId is the definition it
        was minted from, so one name asked for with two value sets is refused.
        """
        return self._ros2_enum_datatypes

    async def mint_structure(self, datatype: type) -> ua.NodeId:
        """Return the NodeId of a Structure DataType, creating it and what it names the first time.

        Dependencies first, into the ROS2 nodeset's namespace, through
        ``Server.ensure_structure`` of opensmi-server.

        :raises DuplicateStructureError: When another description of the type is already on the
            server. Nothing is created in that case.
        """
        try:
            return await self.server.ensure_structure(datatype, namespace_index=self.ros2_namespace_index)
        except TypeConflictError as conflict:
            message = f"{self.name!r} describes {datatype.__name__} differently from another Component: {conflict}"
            raise DuplicateStructureError(message) from conflict

    @property
    def machine_mutation_node_total(self) -> int:
        """Return how many nodes this Machine has gained by mutation, across every item.

        Only meaningful on a Machine, which is the item every other one reports to.
        """
        return self._machine_mutation_nodes

    @property
    def ros2_projection_qualifier(self) -> str:
        """Return what distinguishes this item's minted DataTypes from another item's.

        The Machine's name and the item's own, unique together in a server: two Machines in one
        process may each have a Component called ``camera``, and DataTypes are server-wide.
        """
        item = self.name
        machine = str(getattr(self.ros2_root_machine, "name", "") or "")
        return item if machine in ("", item) else f"{machine}_{item}"

    @property
    def _byte_string_ceiling(self) -> int:
        """Return how many bytes a bulk payload on this item may carry.

        The Machine's. Zero (no ceiling) outside a Machine.
        """
        return int(getattr(self.ros2_root_machine, "ros2_byte_string_ceiling", 0))

    def note_machine_mutation(self, nodes: int) -> None:
        """Add to this Machine's mutation tally. Called by every item on it, including itself."""
        self._machine_mutation_nodes += nodes

    async def mutate_address_space(
        self,
        additions: Iterable[NodeAddition],
        *,
        at_init: bool = False,
    ) -> list[RenderedPlan | None]:
        """Create nodes, keep their names unique, and announce the change once for all.

        :param additions: What to create.
        :param at_init: Whether the address space is being built rather than changed. At init
            nothing is announced and nothing counts against a cap.
        :return: What each addition became, in order; ``None`` for one a cap refused, which created
            nothing.
        :raises DuplicateVariableNameError: When a name is already taken in its container.
        :raises UnprojectableFieldError: When a plan carries an Object.
        """
        rendered: list[RenderedPlan | None] = []
        changes: list[ua.ModelChangeStructureDataType] = []
        for addition in additions:
            planned = _planned_nodes(addition)
            if not at_init and not await self._within_the_caps(
                binding=addition.binding,
                path=addition.path,
                planned=planned,
            ):
                rendered.append(None)
                continue
            rendered.append(await self._create_addition(addition, changes))
            if not at_init:
                self._count(addition.binding, planned)
        if not at_init:
            await self.ros2_root_machine.emit_model_change(changes)
        return rendered

    async def _create_addition(
        self,
        addition: NodeAddition,
        changes: list[ua.ModelChangeStructureDataType],
    ) -> RenderedPlan:
        """Create one addition: its Machinery group and member group if it has them, and the plan's nodes in it.

        A Machinery group this addition created is removed again when the plan fails, so an unused
        one never stays behind.
        """
        parent = addition.container
        fresh = addition.group if addition.group is not None and addition.group not in parent.groups else None
        if addition.group is not None:
            parent = await self.machinery_group(parent, addition.group, changes)
        container = parent
        if addition.member is not None:
            browse, display = addition.member
            require_unused_name(browse, (parent,), "this Monitoring")
            container = await parent.add_group(browse, display_name=display, runtime_keyed=True)
            changes.append(_node_added(container.ua_node, ObjectIds.BaseObjectType))
        try:
            return await self.render_plan(
                container,
                addition.plan,
                message_type=addition.message_type,
                qualifier=f"{self.ros2_projection_qualifier}.{addition.binding}",
                where="this Monitoring",
                root=addition.root,
                changes=changes,
            )
        except BaseException:
            if addition.member is not None:
                await parent.remove_group(addition.member[0])
            if fresh is not None:
                await self._drop_machinery_group(addition.container, fresh)
            raise

    async def machinery_group(
        self,
        container: UaVariableContainer,
        name: str,
        changes: list[ua.ModelChangeStructureDataType] | None = None,
    ) -> UaVariableContainer:
        """Return the Machinery Monitoring sub-structure of that name in a container, creating it the first time.

        It is a FolderType Object shared by every Binding that names it. A ``Status`` group
        references this item's MachineryItemState and MachineryOperationMode with HasComponent when
        the item provides them, as OPC 40001-1 section 16.1 asks. A group is never removed once a
        Binding is in it: nothing leaves the address space while the server runs.

        :param changes: Where to record the group when it is made, for the model-change event.
        """
        if name in container.groups:
            return container.groups[name]
        group = await container.add_group(name, object_type=ua.NodeId(ua.Int32(ObjectIds.FolderType)))
        if changes is not None:
            changes.append(_node_added(group.ua_node, ObjectIds.FolderType))
        if name == _STATUS:
            for add_in in await self._machinery_state():
                await group.ua_node.add_reference(add_in, ObjectIds.HasComponent)
        return group

    async def _drop_machinery_group(self, container: UaVariableContainer, name: str) -> None:
        """Remove a Machinery group a failed addition made, keeping the AddIns a Status group references.

        The removal is recursive and follows HasComponent, so the references go first.
        """
        if name == _STATUS:
            status = container.groups[name].ua_node
            for add_in in await self._machinery_state():
                await status.delete_reference(add_in, ObjectIds.HasComponent)
        await container.remove_group(name)

    async def _machinery_state(self) -> list[Node]:
        """Return the item's MachineryItemState and MachineryOperationMode, those it has.

        opensmi files both AddIns under MachineryBuildingBlocks. An item without them has none.
        """
        machinery = self.server.ua_get_namespace_index(MachineryNodeIds)
        try:
            blocks = await self.ua_node.get_child(f"{machinery}:MachineryBuildingBlocks")
        except BadNoMatch:
            return []
        found: list[Node] = []
        for add_in in _MACHINERY_STATE:
            try:
                found.append(await blocks.get_child(f"{machinery}:{add_in}"))
            except BadNoMatch:
                continue
        return found

    async def mutate_address_space_with(
        self,
        *,
        binding: str,
        path: str,
        nodes: int,
        build: Callable[[], Awaitable[Sequence[tuple[Node, ua.NodeId | int]]]],
    ) -> bool:
        """Create nodes a caller builds itself, under the same caps and announcement.

        Unlike :meth:`mutate_address_space`, the caller makes the nodes. This exists for OPC
        10000-210 Locations (opensmi's ``CartesianFrame``), which are not variables of a plan.

        :param binding: The Binding these nodes belong to, which the cap counts against.
        :param path: The ROS2 path of that Binding, for a refusal's BindingEvent.
        :param nodes: How many nodes the build will create. The caller states it, since only the
            caller knows.
        :param build: What to create, answering each node made and the type it instantiated (what
            a client filtering model changes by type reads).
        :return: Whether it happened. ``False`` means a cap refused it and nothing was created.
        """
        if not await self._within_the_caps(binding=binding, path=path, planned=nodes):
            return False
        created = await build()
        self._count(binding, nodes)
        await self.ros2_root_machine.emit_model_change(
            [_node_added(node, affected_type) for node, affected_type in created]
        )
        return True

    async def _within_the_caps(self, *, binding: str, path: str, planned: int) -> bool:
        """Return whether one addition fits, reporting it as a BindingEvent when it does not."""
        machine = self.ros2_root_machine
        on_binding = self._mutation_nodes.get(binding, 0) + planned
        on_machine = machine.machine_mutation_node_total + planned
        if on_binding <= self.mutation_node_cap and on_machine <= machine.machine_mutation_node_cap:
            return True
        limit = (
            f"the Binding's cap of {self.mutation_node_cap}"
            if on_binding > self.mutation_node_cap
            else f"the Machine's cap of {machine.machine_mutation_node_cap}"
        )
        text = f"{binding} would grow the address space by {planned} nodes, past {limit}; none of them were created"
        self.logger.error("Refused an address-space mutation at its cap", binding=binding, nodes=planned)
        await self.emit_binding_event(
            binding=binding,
            path=path,
            code=CAP_REACHED,
            severity=Severity.ERROR,
            text=text,
        )
        return False

    def _count(self, binding: str, nodes: int) -> None:
        """Add one addition's nodes to this Binding's tally and to the Machine's."""
        self._mutation_nodes[binding] = self._mutation_nodes.get(binding, 0) + nodes
        self.ros2_root_machine.note_machine_mutation(nodes)

    async def render_plan(
        self,
        container: UaVariableContainer,
        plan: Sequence[PlanEntry],
        *,
        message_type: type[Message],
        qualifier: str,
        where: str,
        root: str | None = None,
        writable: bool = False,
        changes: list[ua.ModelChangeStructureDataType] | None = None,
    ) -> RenderedPlan:
        """Create the nodes of one plan in a container, and return how to write them.

        With ``root`` the whole message is one complex Variable of that name. Without it, each
        top-level field is a child of the container, and a nested message a complex Variable. A
        complex Variable carries the structure of its level, minted here. A plan with a
        leaf that no structure field can carry (a DateTime) is rendered with variable groups for
        its levels and plain Variables for its leaves, so it keeps its names and loses only the
        atomic read. Every name is checked before any node is made.

        :param container: The opensmi container the nodes are added to.
        :param plan: What to create.
        :param message_type: The ROS2 type the plan was shaped from.
        :param qualifier: What names the structures of curated levels, such as ``Platform.Odom``.
        :param where: How a refusal names the container, from a reader's side.
        :param root: The name of the complex Variable the whole plan is rendered as.
        :param writable: Whether a client may write the Variables.
        :param changes: Where to record each node made, for the model-change event.
        :raises UnprojectableFieldError: When an entry is an Object or has no name.
        :raises DuplicateVariableNameError: When a name is already taken in the container.
        """
        leaves, plains, subtrees = _split(plan, root, where)
        naming = curated_naming(qualifier)
        names = [name for name, _ in plains] + list(subtrees)
        for name in names:
            require_unused_name(name, (container,), where)
        parts: list[_Plain | _Complex] = []
        for name, entry in plains:
            variable = await add_plan_variable(
                container,
                name,
                entry,
                await self.resolve_plan_datatype(entry.datatype),
                writable=writable,
            )
            parts.append(_Plain(variable, entry))
            _record(changes, variable)
        for name, below in subtrees.items():
            path: NodePath = () if root is not None else (name,)
            try:
                classes = level_datatypes(below, message_type, naming)
            except UnprojectableFieldError:
                parts += await self._render_groups(container, name, below, root=root, writable=writable)
                continue
            level = None if root is not None else below[0].above[0]
            parts.append(
                await self._render_structure(
                    container,
                    name,
                    below,
                    classes,
                    path,
                    display=name if level is None else level.display_name,
                    description=(message_type if level is None else level.message or message_type).ros2.type_name,
                    writable=writable,
                    changes=changes,
                )
            )
        return RenderedPlan({entry.full_name: entry for entry in leaves}, parts)

    async def _render_structure(
        self,
        container: UaVariableContainer,
        name: str,
        leaves: list[PlanVariable],
        classes: Mapping[NodePath, type],
        path: NodePath,
        *,
        display: str,
        description: str,
        writable: bool,
        changes: list[ua.ModelChangeStructureDataType] | None,
    ) -> _Complex:
        """Create the complex Variable of the structure at one path, holding its initial value."""
        _require_derived_names(name, leaves, path)
        structure = classes[path]
        datatype = await self.mint_structure(structure)
        initial = level_value(leaves, classes, {leaf.full_name: leaf.initial for leaf in leaves}, path)
        variable: UaVariable[Any] = UaVariable(
            initial,
            name=name,
            display_name=None if display == name else display,
            writable=writable,
            write_check_callback=_structure_write_check(lambda: variable, name, structure, None, array=False),
            ua_data_type=datatype,
            description=description,
        )
        await _add(container, name, variable)
        _record(changes, variable)
        for leaf in leaves:
            child = variable
            for part in leaf.node_path[len(path) :]:
                child = child[part]
            # What opensmi cannot know about a field: the ROS2 path, and a DataType finer than its variant type.
            if leaf.source:
                await child.write_description(leaf.source)
            if leaf.is_array:
                await _write_unbounded(child.ua_node)
            if leaf.datatype is not None and not isinstance(leaf.datatype, type):
                await child.ua_node.write_attribute(
                    AttributeIds.DataType,
                    ua.DataValue(
                        ua.Variant(await self.resolve_plan_datatype(leaf.datatype), ua.VariantType.NodeId),
                    ),
                )
        return _Complex(variable, leaves, classes, path)

    async def _render_groups(
        self,
        container: UaVariableContainer,
        name: str,
        leaves: list[PlanVariable],
        *,
        root: str | None,
        writable: bool,
    ) -> list[_Plain]:
        """Render a subtree no DataType can describe: groups for its levels, plain Variables for its leaves."""
        parts: list[_Plain] = []
        top = [] if root is None else [NodeLevel(name=name, display_name=name, group=True)]
        for leaf in leaves:
            group = await ensure_groups(container, [*top, *leaf.above])
            variable = await add_plan_variable(
                group,
                leaf.name,
                leaf,
                await self.resolve_plan_datatype(leaf.datatype),
                writable=writable,
            )
            parts.append(_Plain(variable, leaf))
        return parts

    async def resolve_plan_datatype(self, datatype: PlanDataType | None) -> ua.NodeId | None:
        """Return the NodeId a plan's DataType means on *this* server.

        A plan is a pure value. The NodeId of this package's types depends on the namespace index
        the server imported it at, and an enumeration has none until minted. Public because
        Skills create their nodes outside the chokepoint but need their DataTypes minted here.
        """
        match datatype:
            case None:
                return None
            case StandardDataType(identifier=identifier):
                return ua.NodeId(ua.Int32(identifier))
            case ProjectDataType(identifier=identifier):
                return self.server.ua_get_node_id(identifier)
            case EnumDataType():
                return await self._minted_enum(datatype)
            case type():
                return await self.mint_structure(datatype)

    async def _minted_enum(self, datatype: EnumDataType) -> ua.NodeId:
        """Return the NodeId of an enumeration DataType, minting it the first time it is asked for.

        Minted once per server and remembered on the Machine; the generated name is qualified by
        its Component for that reason. It uses asyncua directly and not opensmi's
        ``ensure_enum``, which needs an ``IntEnum`` numbered from zero, while ROS2 constants may
        start at 1 or skip numbers.
        """
        minted: dict[str, tuple[EnumDataType, ua.NodeId]] = self.ros2_root_machine.ros2_enum_datatypes
        known = minted.get(datatype.name)
        if known is not None:
            _require_one_definition_per_name(datatype, known[0])
            return known[1]
        definition = ua.EnumDefinition()
        for value, label in datatype.values:
            enum_field = new_enum_field(label)
            enum_field.Value = ua.Int64(value)
            definition.Fields.append(enum_field)
        node = await self.server.ua_server.nodes.enum_data_type.add_data_type(_ENUM_NAMESPACE, datatype.name)
        await node.write_data_type_definition(definition)
        minted[datatype.name] = (datatype, node.nodeid)
        return node.nodeid


def require_unused_name(name: str, taken: Iterable[Container[str]], where: str) -> None:
    """Refuse a variable name something on this container has already claimed.

    opensmi's container has no such check: it files a variable with ``setattr``, so a second one
    would replace the first. Shared by the chokepoint and by Skills and Methods.

    :param name: The browse name being claimed.
    :param taken: Every place a name may already be recorded, tested in order.
    :param where: What to call the container in the refusal, from a reader's point of view.
    :raises DuplicateVariableNameError: When the name is taken.
    """
    # a container answers `in` for its variables only, its groups are in `groups`
    if not any(name in each or name in getattr(each, "groups", ()) for each in taken):
        return
    message = f"the name {name!r} is already taken on {where}; the container would accept the collision silently"
    raise DuplicateVariableNameError(message)


async def ended(task: asyncio.Task[None] | None) -> None:
    """Cancel one task and wait for it, tolerating a task that has already finished."""
    if task is None:
        return
    task.cancel()
    await asyncio.gather(task, return_exceptions=True)


def _require_one_definition_per_name(wanted: EnumDataType, minted: EnumDataType) -> None:
    """Refuse two value sets minted under one DataType name.

    Reachable only when two ROS2 types with the same last name part are curated on one Component.
    Sharing the first definition would give a client another type's labels.

    :raises DuplicateProjectionError: When the two do not agree.
    """
    if wanted == minted:
        return
    message = (
        f"two Projections mint the DataType {wanted.name!r} with different values; curate one "
        f"of them on a Component of its own, or make their entries agree"
    )
    raise DuplicateProjectionError(message)


def _split(
    plan: Sequence[PlanEntry],
    root: str | None,
    where: str,
) -> tuple[list[PlanVariable], list[tuple[str, PlanVariable]], dict[str, list[PlanVariable]]]:
    """Sort a plan into its leaves, the entries that are Variables of their own, and the structures.

    The entries are listed with the name their Variable takes; a structure by its top-level name,
    which is ``root`` when there is one.

    :raises UnprojectableFieldError: When an entry is an Object or has no name.
    """
    leaves: list[PlanVariable] = []
    for entry in plan:
        if isinstance(entry, PlanObject) or not (entry.node_path or root):
            message = (
                f"{where} renders {entry.name!r} as an Object or without a name, and a message "
                f"nests as complex Variables, not as Objects"
            )
            raise UnprojectableFieldError(message)
        leaves.append(entry)
    plains: list[tuple[str, PlanVariable]] = []
    subtrees: dict[str, list[PlanVariable]] = {}
    if root is not None:
        if len(leaves) == 1 and not leaves[0].node_path:
            plains.append((root, leaves[0]))
        else:
            subtrees[root] = leaves
        return leaves, plains, subtrees
    for entry in leaves:
        if len(entry.node_path) == 1:
            plains.append((entry.name, entry))
        else:
            subtrees.setdefault(entry.node_path[0], []).append(entry)
    return leaves, plains, subtrees


def _require_derived_names(root: str, leaves: Sequence[PlanVariable], path: NodePath) -> None:
    """Refuse a plan whose names opensmi would not derive from its field names.

    The children of a complex Variable are named by opensmi from the field names of the structure,
    which are the DisplayNames: ``child_frame_id`` is ``ChildFrameId``. A name that is not already
    spelled that way (a Projection that names an entry ``data``) would browse as another name than
    the plan says, so it is refused instead of renamed silently.

    :raises DeclarationError: When a name on a node path differs from the one opensmi derives.
    """
    for leaf in leaves:
        for name, display in zip(leaf.node_path[len(path) :], leaf.display_names[len(path) :], strict=True):
            derived = pascal_case(display.split("_"))
            if derived != name:
                message = (
                    f"{root!r} renders {leaf.source or leaf.full_name!r} as {name!r}, and opensmi names the child "
                    f"Variable of the field {display!r} {derived!r}; name the entry {derived!r}, "
                    f"and keep {display!r} as its display name"
                )
                raise DeclarationError(message)


def _planned_nodes(addition: NodeAddition) -> int:
    """Return how many nodes one addition creates: its member group, its levels and its leaves."""
    plan = [entry for entry in addition.plan if isinstance(entry, PlanVariable)]
    levels = len(inner_levels(plan)) + (addition.root is not None) + (addition.member is not None)
    return len(addition.plan) + levels


def _record(changes: list[ua.ModelChangeStructureDataType] | None, variable: UaVariable[Any]) -> None:
    """Note a node made and the fields below it, for the model-change event."""
    if changes is None:
        return
    changes.append(_node_added(variable.ua_node, ObjectIds.BaseDataVariableType))
    for child in variable.children.values():
        _record(changes, child)


def _node_added(node: Node, affected_type: ua.NodeId | int) -> ua.ModelChangeStructureDataType:
    """Return the model change one created node is, with the type it was instantiated as."""
    return ua.ModelChangeStructureDataType(
        Affected=node.nodeid,
        AffectedType=affected_type if isinstance(affected_type, ua.NodeId) else ua.NodeId(ua.Int32(affected_type)),
        Verb=_NODE_ADDED,
    )
