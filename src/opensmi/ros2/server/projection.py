# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Projection engine: how one ROS2 type is rendered in OPC UA.

A Binding connects one ROS2 path to one OPC UA object. A Projection is about the type of what flows
through it, is reused by every Binding whose messages carry that type, and applies wherever the type
appears, at the root of a message or nested inside another. It has three pure functions:

* shape, run once at init, turns a message dataclass into a node plan of :class:`PlanObject` and
  :class:`PlanVariable`. The generic walk emits only variables, which nest as complex
  Variables; a repeated message becomes one variable holding an array of a structure derived from
  its type, and a plan that asks for an Object is refused where its nodes are made.
* decode, run per message, turns a payload into a map from plan name to value.
* encode, run per write on the publishing side, turns that map back into a payload. An entry that
  cannot run backwards has no :attr:`Projection.inverse`. That is derived from the recipe
  and asked as :meth:`Projection.inverts`, of a type, because for some recipes it
  depends on the message too.

The generic field-walk is an ordinary registry entry that matches last, so turning curation off for
one type means naming the generic entry for it. Authors write recipes
(:mod:`opensmi.ros2.server.recipes`), not these functions. No step sees an ``rclpy`` object.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Any, Final, TypeAlias, cast

from asyncua import ua
from asyncua.ua.object_ids import ObjectIds
from opensmi.core.ua import NodeIdDefinition

from opensmi.ros2.runtime.guard import INTEGER_RANGES
from opensmi.ros2.runtime.messages import (
    ArrayType,
    NestedType,
    PrimitiveType,
    SequenceType,
    StringType,
    element_of,
    is_byte_container,
)
from opensmi.ros2.runtime.selection import SEPARATOR, field_paths, resolve_selection
from opensmi.ros2.server.browse_names import browse_name, browse_names
from opensmi.ros2.server.errors import (
    DeclarationError,
    DuplicateProjectionError,
    NotInvertibleError,
    UnprojectableFieldError,
)
from opensmi.ros2.server.structures import NAME_SEPARATOR, StructureField, structure_class, structure_instance

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable, Iterator

    from opensmi.ros2.runtime.messages import (
        ElementType,
        FieldDescription,
        FieldType,
        Message,
        PrimitiveName,
    )
    from opensmi.ros2.runtime.selection import FieldSelection

__all__ = [
    "ANY_TYPE",
    "NAME_SEPARATOR",
    "DecodeFunction",
    "EncodeFunction",
    "EnumDataType",
    "NodeLevel",
    "NodeLevelNaming",
    "NodePath",
    "PlanDataType",
    "PlanEntry",
    "PlanObject",
    "PlanVariable",
    "ProjectDataType",
    "Projection",
    "ProjectionContext",
    "ProjectionRegistry",
    "ShapeFunction",
    "StandardDataType",
    "browse_name_of",
    "curated_naming",
    "element_structure",
    "flat_name",
    "generic_decode",
    "generic_encode",
    "generic_shape",
    "inner_levels",
    "join_path",
    "level_datatypes",
    "level_value",
    "message_class",
    "narrowed",
    "require_invertible",
    "ros2_value",
    "shaped",
    "ua_shape",
    "ua_value",
]

ANY_TYPE: Final = "*"
"""The type name of an entry that answers for every type; the generic walk carries it."""


NodePath: TypeAlias = tuple[str, ...]
"""The browse names from the top of a plan down to one node, such as ``("Pose", "Position", "X")``."""


def flat_name(*parts: str) -> str:
    """Return node path parts as the one name a plan entry has in the decode and encode maps.

    Empty parts are skipped: an entry named ``""`` takes the name of whatever contains it, so a
    message rendered as one variable browses as the field it sits in. The decode and encode maps
    are keyed the same way, so this is the one place that spells that name. It is a key and not a
    BrowseName: the nodes nest.
    """
    return NAME_SEPARATOR.join(part for part in parts if part)


def join_path(prefix: str, name: str) -> str:
    """Return one field path below another, spelled the way the selection spells it."""
    return f"{prefix}{SEPARATOR}{name}" if prefix else name


@dataclass(frozen=True, slots=True, kw_only=True)
class StandardDataType:
    """A DataType OPC UA itself defines, named by its numeric identifier in namespace 0.

    ``ObjectIds.ImagePNG`` is one. Naming it says what the bytes of a bulk payload are, without
    transforming them.
    """

    identifier: int


@dataclass(frozen=True, slots=True, kw_only=True)
class ProjectDataType:
    """A DataType this project's own nodeset defines, named by its generated NodeId definition.

    :data:`opensmi.ros2.server.events.SIGNED_BYTE_STRING_TYPE` is the only one so far. The
    definition is resolved to a NodeId where the nodes are made, because the namespace index is
    specific to one server.
    """

    identifier: NodeIdDefinition


@dataclass(frozen=True, slots=True, kw_only=True)
class EnumDataType:
    """An enumeration DataType to mint into the server, and the values it lists.

    :ivar name: The DataType's browse name, qualified per Component because the server's address
        space is shared while two Components may hold different value sets for one ROS2 field. The
        qualifier is generated.
    :ivar values: The ROS2 value of each label, in the order a client should see them. The ROS2
        value is kept, not the position, because a constant set need not start at zero or be
        contiguous.
    """

    name: str
    values: tuple[tuple[int, str], ...]


PlanDataType: TypeAlias = "StandardDataType | ProjectDataType | EnumDataType | type"
"""What a plan may say about a variable's DataType, resolved to a NodeId where the nodes are made.

A plan is a pure value, and the NodeId of one of our types depends on the server, while an
enumeration or a structure has no NodeId until it is minted (:mod:`opensmi.ros2.server.mutation`).
A ``type`` is the structure class that is the element type of an array of structures, which the
generic walk emits for a repeated message.
"""


@dataclass(frozen=True, slots=True, kw_only=True)
class NodeLevel:
    """One inner level of a node path: a structure, or a group, that holds the nodes below it.

    :ivar name: Its browse name.
    :ivar display_name: The raw ROS2 name, or the new name a ``rename`` gave it.
    :ivar message: The ROS2 message type of the structure; ``None`` for a group.
    :ivar group: Whether it only gathers the nodes below it, such as a Binding or a ROS2 parameter
        namespace. A group becomes a grouping Object, a structure a complex Variable.
    """

    name: str
    display_name: str
    message: type[Message] | None = None
    group: bool = False

    @property
    def ros2_type(self) -> str:
        """Return the name of the ROS2 message type of the structure, empty for a group."""
        return "" if self.message is None else self.message.ros2.type_name


@dataclass(frozen=True, slots=True, kw_only=True)
class PlanVariable:
    """One OPC UA Variable to create, and the ROS2 field it reads.

    Its place is a node path: the :attr:`above` levels, then :attr:`name`.

    :ivar name: The browse name of the leaf, relative to the Projection that emitted it. Empty
        means the node takes the name of whatever contains it.
    :ivar display_name: The raw ROS2 name of the leaf, or its new name from a ``rename``; ``None``
        when it is :attr:`name` itself.
    :ivar above: The levels this leaf sits in, outermost first. A parent adds its own field to the
        front of whatever its nested type emitted.
    :ivar source: The ROS2 field path this variable reads, so a node can be traced back.
    :ivar initial: The typed value written at boot. It is typed rather than null so that a status
        code written over it does not leave a client with an empty value.
    :ivar variant_type: The OPC UA type, explicit because an empty sequence cannot imply one and
        ``float32`` and ``float64`` are both Python ``float``.
    :ivar datatype: The semantic DataType where the variant type does not say enough (a signed
        bulk payload, an enumeration), else ``None``.
    """

    name: str
    source: str
    initial: object
    variant_type: ua.VariantType
    datatype: PlanDataType | None = None
    display_name: str | None = None
    above: tuple[NodeLevel, ...] = ()

    @property
    def node_path(self) -> NodePath:
        """Return the browse names from the top of the plan down to this leaf."""
        return (*(level.name for level in self.above), *((self.name,) if self.name else ()))

    @property
    def display_names(self) -> tuple[str, ...]:
        """Return the display name of each node on :attr:`node_path`."""
        leaf = (self.display_name or self.name,) if self.name else ()
        return (*(level.display_name for level in self.above), *leaf)

    @property
    def full_name(self) -> str:
        """Return the flat name this variable is rendered under, relative to the plan."""
        return flat_name(*self.node_path)

    @property
    def display(self) -> str:
        """Return the display name of the leaf node, or ``""`` when it takes its container's."""
        return self.display_names[-1] if self.display_names else ""

    @property
    def inner_levels(self) -> dict[NodePath, NodeLevel]:
        """Return the structures above this leaf by their node path.

        A message collapsed to one value is the leaf itself, not a level.
        """
        levels = self.above if self.name else self.above[:-1]
        return {
            tuple(level.name for level in self.above[: index + 1]): level
            for index, level in enumerate(levels)
            if not level.group
        }

    @property
    def is_array(self) -> bool:
        """Return whether this variable carries many values rather than one.

        Read off the boot value, which :func:`ua_shape` makes a list for any array.
        """
        return isinstance(self.initial, list)


@dataclass(frozen=True, slots=True, kw_only=True)
class PlanObject:
    """One OPC UA Object to create, with its own entries below it.

    The generic walk emits none, and a plan that carries one is refused where the nodes are made:
    a message nests as complex Variables.

    :ivar name: As :class:`PlanVariable`'s, empty included.
    :ivar children: What hangs below it, each named relative to it.
    :ivar display_name: The raw name of the Object, or ``None`` when it is :attr:`name` itself.
    :ivar object_type: The ObjectType to instantiate, for a recipe naming one out of a nodeset.
        A plain folder otherwise, which is what a structure rendered for browsing wants.
    """

    name: str
    children: tuple[PlanEntry, ...]
    display_name: str | None = None
    object_type: ua.NodeId | int = ObjectIds.FolderType

    @property
    def full_name(self) -> str:
        """Return the flat name of this Object, which is its :attr:`name`."""
        return self.name


PlanEntry: TypeAlias = "PlanVariable | PlanObject"
"""The two verbs, and there is no third."""


def inner_levels(plan: Iterable[PlanEntry]) -> dict[NodePath, NodeLevel]:
    """Return every structure a plan's variables sit in, by node path.

    These are the nodes that become complex Variables.
    """
    levels: dict[NodePath, NodeLevel] = {}
    for entry in plan:
        if isinstance(entry, PlanVariable):
            levels.update(entry.inner_levels)
    return levels


@dataclass(frozen=True, slots=True, kw_only=True)
class ProjectionContext:
    """What a Projection needs besides the message in front of it.

    :ivar registry: How a nested type finds its own Projection.
    :ivar selected: The ROS2 field paths that survived the Binding's selection, absolute from the
        root of the declared message. Selection is applied before projection, so a Projection is
        never asked about a field the declaration removed.
    :ivar path: The absolute ROS2 path of the message being shaped or decoded. Empty at the root.
    :ivar qualifier: What distinguishes this Component's minted DataTypes from another's. Empty
        outside a Component.
    :ivar rename: The new browse name of a field, by its absolute ROS2 field path, from the
        Binding's selection.
    :ivar raw_names: Whether fields keep their ROS2 names. The fields of a structure array's
        element do: they are field names of a DataType, not browse names.
    :ivar byte_string_ceiling: How many bytes a bulk payload may carry, or ``0`` for no ceiling.
        It is the server's own, published in its capability variable
        (:mod:`opensmi.ros2.server.limits`), and the Projection applies it because the send path
        has no check of its own.
    """

    registry: ProjectionRegistry
    selected: frozenset[str]
    path: str = ""
    qualifier: str = ""
    rename: Mapping[str, str] = field(default_factory=dict)
    raw_names: bool = False
    byte_string_ceiling: int = 0

    def child(self, field_name: str) -> ProjectionContext:
        """Return the context of one field of the message this context is on."""
        return replace(self, path=join_path(self.path, field_name))

    @property
    def included(self) -> bool:
        """Return whether the selection kept the path this context is on."""
        return self.path in self.selected

    def without(self, field_paths: Iterable[str]) -> ProjectionContext:
        """Return this context with some paths below it removed, subtrees and all.

        This is what the drop recipe uses. A dropped field is as invisible as an excluded one.
        """
        dropped: set[str] = set()
        for field_path in field_paths:
            full = join_path(self.path, field_path)
            dropped.update(path for path in self.selected if path == full or path.startswith(f"{full}{SEPARATOR}"))
        return replace(self, selected=self.selected - dropped)


ShapeFunction: TypeAlias = "Callable[[type[Message], ProjectionContext], tuple[PlanEntry, ...]]"
"""Run once at init: a message dataclass becomes a node plan."""

DecodeFunction: TypeAlias = "Callable[[Message, ProjectionContext], Mapping[str, object]]"
"""Run per message: a payload becomes a map from plan name to the value to write."""

EncodeFunction: TypeAlias = "Callable[[type[Message], Mapping[str, object], ProjectionContext], Message]"
"""Run per publish: that same map becomes a payload again, where the recipe has an inverse."""


@dataclass(frozen=True, slots=True)
class Projection(ABC):
    """How one ROS2 type is rendered, as the steps and the type they answer for.

    Abstract: an author writes one of the classes in :mod:`opensmi.ros2.server.recipes`.

    :ivar type_name: The ROS2 type this entry answers for, or :data:`ANY_TYPE` for the entry that
        answers for every one. Keying is by type name alone.
    :ivar override: Whether this entry may shadow one already registered for its type. A duplicate
        is fatal without it, so the address space never depends on registration order.
    :ivar type_hash: The RIHS01 hash of the definition this entry was authored against, or
        ``None``. It lets CI report that a type moved under a curated entry. It never selects an
        entry, since that would render different address spaces for different vendor builds.
    """

    type_name: str
    override: bool = field(default=False, kw_only=True)
    type_hash: str | None = field(default=None, kw_only=True)

    @abstractmethod
    def shape(self, message_type: type[Message], context: ProjectionContext) -> tuple[PlanEntry, ...]:
        """Return the nodes this type becomes, named relative to whatever contains them."""

    @abstractmethod
    def decode(self, message: Message, context: ProjectionContext) -> Mapping[str, object]:
        """Return what one payload writes, keyed by the names :meth:`shape` created."""

    @property
    def inverse(self) -> EncodeFunction | None:
        """Return the function that runs this entry backwards, or ``None`` when it has none.

        Each recipe derives it from what it is. ``None`` means nothing can be published through
        the entry, and a publishing Binding that reaches one fails at init.
        """
        return None

    def inverts(self, message_type: type[Message]) -> bool:
        """Return whether this entry runs backwards for one ROS2 type.

        It takes a type because an Object inverts only if its field plan is total, which
        depends on the message as well as the recipe. That lets the check run at init.
        """
        return self.inverse is not None

    def matches(self, type_name: str) -> bool:
        """Return whether this entry answers for a ROS2 type."""
        return self.type_name in (type_name, ANY_TYPE)

    def inverse_of(
        self,
        message_type: type[Message],
        values: Mapping[str, object],
        context: ProjectionContext,
    ) -> Message:
        """Run this entry backwards, refusing rather than inventing when it has no inverse.

        :raises NotInvertibleError: When this entry has no inverse, which is a declaration fault
            wherever it is reached.
        """
        encode = self.inverse
        if encode is None:
            message = (
                f"the {type(self).__name__} Projection of {self.type_name!r} cannot run "
                f"backwards, so nothing can be published through it"
            )
            raise NotInvertibleError(message)
        return encode(message_type, values, context)


@dataclass(frozen=True, slots=True, kw_only=True)
class ProjectionRegistry:
    """The ordered entries a Component renders its types through.

    The first entry that matches wins, so an entry placed ahead of another shadows it, and the
    generic walk sits last because it matches everything. A registry is grown with :meth:`extended`
    and its entries are never edited, so one Machine's default can start several Components. Only
    :attr:`element_structures`, a memo, changes.
    """

    entries: tuple[Projection, ...]

    element_structures: dict[tuple[type[Message], str], type] = field(
        default_factory=dict,
        init=False,
        compare=False,
        repr=False,
    )
    """The structure each repeated element type renders as, by type and qualifier.

    A memo, on the registry because the shipped registry is shared by several Components: a global
    cache keyed on the type alone would hand one Component another's enumeration names.
    """

    def find(self, type_name: str) -> Projection:
        """Return the first entry answering for a ROS2 type.

        :raises DeclarationError: When no entry does, which means the registry was
            assembled without the entry that matches last.
        """
        for entry in self.entries:
            if entry.matches(type_name):
                return entry
        message = (
            f"no Projection covers {type_name!r}; a registry has to end with an entry whose type "
            f"is {ANY_TYPE!r}, the generic field-walk being the one that does"
        )
        raise DeclarationError(message)

    def extended(self, entries: Iterable[Projection]) -> ProjectionRegistry:
        """Return this registry with entries ahead of it, refusing an unannounced duplicate.

        Ahead, so a Component's entry is found before the Machine's, and the Machine's before the
        shipped one.

        :raises DuplicateProjectionError: When an entry covers a type another already covers and
            does not say ``override=True``.
        """
        added: list[Projection] = []
        for entry in entries:
            _require_no_silent_shadowing(entry, (*added, *self.entries))
            added.append(entry)
        return ProjectionRegistry(entries=(*added, *self.entries))


def _require_no_silent_shadowing(entry: Projection, covered: Iterable[Projection]) -> None:
    """Refuse a second entry for one type unless it says it is replacing the first."""
    if entry.override:
        return
    shadowed = next((each for each in covered if each.type_name == entry.type_name), None)
    if shadowed is None:
        return
    message = (
        f"a {type(shadowed).__name__} Projection already covers {entry.type_name!r}, and this "
        f"{type(entry).__name__} one would shadow it; say override=True where you mean to "
        f"replace it, so that precedence is something a reader can see"
    )
    raise DuplicateProjectionError(message)


def require_invertible(message_type: type[Message], context: ProjectionContext) -> None:
    """Refuse a declaration that writes a type into ROS2 through a Projection with no inverse.

    A Binding that publishes is declarable only if every Projection reachable in its message type
    inverts. The error names the ROS2 type, the field path and the entry. An action's
    goal type is checked like a published topic's. Reachable means through the selection, so an
    excluded field is never asked.

    :raises NotInvertibleError: When any reachable entry cannot run backwards.
    """
    projection = context.registry.find(message_type.ros2.type_name)
    if not projection.inverts(message_type):
        message = (
            f"the {type(projection).__name__} Projection of {projection.type_name!r} cannot run "
            f"backwards, so nothing written at {context.path or 'the root of the message'!r} "
            f"could be published through it; exclude the field, or curate the type with an "
            f"entry that inverts"
        )
        raise NotInvertibleError(message)
    for described, child, nested in _selected_fields(message_type, context):
        if nested is None:
            continue
        if _repeats(described.field_type):
            require_invertible(nested, _element_context(nested, child))
            continue
        require_invertible(nested, child)


def narrowed(
    context: ProjectionContext,
    message_type: type[Message],
    selection: FieldSelection,
) -> ProjectionContext:
    """Return a Component's context narrowed to what one surface's selection kept.

    A Binding with several surfaces (an action's goal, feedback and result; a service's request and
    response) narrows its context once per surface.

    A type with no fields at all (a bare trigger goal, a ping request) skips the selector, since
    nothing was removed. A pattern written against such a type still reaches the selector and is
    refused by name.

    :raises FieldSelectionError: When a pattern matches nothing, or the two leave no field.
    """
    if not message_type.ros2.fields and not selection.patterns and not selection.rename:
        return replace(context, selected=frozenset())
    return replace(context, selected=resolve_selection(message_type, selection), rename=selection.rename)


def shaped(message_type: type[Message], context: ProjectionContext) -> tuple[PlanEntry, ...]:
    """Return the nodes one message type becomes, through the context's registry."""
    return context.registry.find(message_type.ros2.type_name).shape(message_type, context)


def generic_shape(
    message_type: type[Message],
    context: ProjectionContext,
    overrides: Mapping[str, PlanEntry] | None = None,
) -> tuple[PlanEntry, ...]:
    """Render a message type field by field, one flat variable per leaf.

    Recursion runs through the registry, so a nested type with a curated Projection is rendered by
    that entry and its names are carried up under this field's name. A field that repeats a message
    is one variable holding an array of a structure.

    :param overrides: What to emit for a field instead of walking it, by ROS2 field name. The blob
        and enum recipes are overrides and nothing else. An override of a field the selection
        removed is never reached.
    :raises UnprojectableFieldError: When a repeated message's element cannot render as a
        structure: its Projection emits an Object, or a field of it has a type no structure
        field can carry.
    """
    given = overrides or {}
    selected = list(_selected_fields(message_type, context))
    names = _siblings(context, [described.name for described, _, _ in selected])
    entries: list[PlanEntry] = []
    for described, child, nested in selected:
        name = names[described.name]
        display = _display_name(context, described.name)
        override = given.get(described.name)
        if override is not None:
            entries.append(replace(override, name=name, display_name=display))
        elif nested is None:
            entries.append(_plan_variable(described, child, name, display))
        elif _repeats(described.field_type):
            entries.append(_plan_structures(described, child, nested, name, display))
        else:
            level = NodeLevel(name=name, display_name=display, message=nested)
            projection = context.registry.find(nested.ros2.type_name)
            entries.extend(_below(level, entry) for entry in projection.shape(nested, child))
    return tuple(entries)


def generic_decode(
    message: Message,
    context: ProjectionContext,
    overrides: Mapping[str, object] | None = None,
) -> Mapping[str, object]:
    """Read one payload into the names :func:`generic_shape` created for its type.

    :param overrides: What a field's variable reads instead of the field itself, by ROS2 field
        name. It pairs with the shape override of the same name.
    """
    given = overrides or {}
    values: dict[str, object] = {}
    for described, child, nested in _selected_fields(type(message), context):
        name = described.name
        key = browse_name_of(context, name)
        if name in given:
            values[key] = given[name]
            continue
        value = getattr(message, name)
        if nested is None:
            values[key] = ua_value(value)
            continue
        if _repeats(described.field_type):
            values[key] = _decoded_elements(value, nested, child)
            continue
        projection = context.registry.find(nested.ros2.type_name)
        for below, decoded in projection.decode(value, child).items():
            values[flat_name(key, below)] = decoded
    return values


def _decoded_elements(
    value: object,
    element_type: type[Message],
    context: ProjectionContext,
) -> list[Mapping[str, object]]:
    """Return one repeated message field as the array of elements its variable carries.

    Each element is decoded by the Projection covering its type into a map from its structure's
    field names to values. An empty sequence decodes to an empty list, which is a value.

    The elements are plain maps, not ExtensionObjects: encoding one needs the server's namespace
    and belongs to the variable being written (:mod:`opensmi.ros2.server.mutation`).
    """
    projection = context.registry.find(element_type.ros2.type_name)
    element_context = _element_context(element_type, context)
    # A payload holds containers as tuples; anything else is not from the walk and decodes as empty.
    elements = value if isinstance(value, (list, tuple)) else ()
    return [projection.decode(element, element_context) for element in elements]


def generic_encode(
    message_type: type[Message],
    values: Mapping[str, object],
    context: ProjectionContext,
    overrides: Mapping[str, object] | None = None,
) -> Message:
    """Build one payload out of the names :func:`generic_shape` created, and nothing else.

    The inverse of :func:`generic_decode`. A field the selection removed is not filled from a
    default: a covariance of zeros means perfectly certain in ROS2, not unknown.

    :param overrides: The ROS2 value of a field the caller has already worked out, by field name.
    :raises NotInvertibleError: When a field of the message has no value, whether because the
        selection removed it or because the caller supplied none.
    """
    given = overrides or {}
    built: dict[str, object] = {}
    for described in message_type.ros2.fields:
        name = described.name
        child = context.child(name)
        if name in given:
            built[name] = given[name]
            continue
        nested = message_type.ros2_nested.get(name)
        if nested is not None and not nested.ros2.fields:
            # A nested type with no fields (`std_msgs/Empty`, the request of every no-argument
            # service) is never selected. There is no value to invent, so build it empty.
            built[name] = nested()
            continue
        if not child.included:
            _refuse_a_partial_payload(message_type, child.path, "the selection removed it")
        key = browse_name_of(context, name)
        if nested is None:
            if key not in values:
                _refuse_a_partial_payload(message_type, child.path, "no value was supplied for it")
            built[name] = ros2_value(values[key], described.field_type)
            continue
        if _repeats(described.field_type):
            if key not in values:
                _refuse_a_partial_payload(message_type, child.path, "no value was supplied for it")
            built[name] = _encoded_elements(values[key], nested, child)
            continue
        projection = context.registry.find(nested.ros2.type_name)
        built[name] = projection.inverse_of(nested, _below_values(values, key), child)
    return message_type(**built)


def _encoded_elements(
    value: object,
    element_type: type[Message],
    context: ProjectionContext,
) -> tuple[Message, ...]:
    """Return the ROS2 sequence that the array of elements of one repeated message field describes.

    Each element is a map from the names its Projection knows to values, rebuilt by that
    Projection's own inverse. An element that lacks a field is refused by name, and an
    empty list builds an empty sequence.

    :raises NotInvertibleError: When an element has no value for a field of its type.
    :raises ValueError: When the value is not a list, or an element of it is not a map. This is a
        client's write, not a declaration fault.
    """
    if not isinstance(value, (list, tuple)):
        message = (
            f"{context.path!r} was written as {type(value).__name__} rather than as an array; it "
            f"carries an array of {element_type.ros2.type_name}, of any length"
        )
        # ValueError, not TypeError: a client wrote this to a node, no caller passed a bad argument.
        raise ValueError(message)  # noqa: TRY004
    projection = context.registry.find(element_type.ros2.type_name)
    element_context = _element_context(element_type, context)
    elements: list[Message] = []
    for index, each in enumerate(value):
        if not isinstance(each, Mapping):
            message = (
                f"element {index} of {context.path!r} is {type(each).__name__} rather than a map "
                f"from field names to values; it carries an array of {element_type.ros2.type_name}"
            )
            raise ValueError(message)  # noqa: TRY004
        elements.append(projection.inverse_of(element_type, each, element_context))
    return tuple(elements)


def _refuse_a_partial_payload(message_type: type[Message], path: str, why: str) -> None:
    """Refuse to build a message with a hole in it, naming the field and the reason."""
    message = (
        f"{message_type.ros2.type_name} cannot be built without {path!r}, and {why}; a field "
        f"filled from a default is a value nobody sent"
    )
    raise NotInvertibleError(message)


def _below_values(values: Mapping[str, object], name: str) -> Mapping[str, object]:
    """Return the values belonging to one nested field, with its own name taken off the front."""
    prefix = f"{name}{NAME_SEPARATOR}"
    below = {key[len(prefix) :]: value for key, value in values.items() if key.startswith(prefix)}
    if name in values:
        # A collapsing recipe answers under the empty name, which up here is the field itself.
        below[""] = values[name]
    return below


def _plan_variable(described: FieldDescription, context: ProjectionContext, name: str, display: str) -> PlanVariable:
    """Return the variable one modelled ROS2 field becomes under the generic walk."""
    initial, variant_type = ua_shape(described.field_type)
    return PlanVariable(
        name=name,
        display_name=display,
        source=context.path,
        initial=initial,
        variant_type=variant_type,
    )


def browse_name_of(context: ProjectionContext, field_name: str) -> str:
    """Return the browse name of one field of the message the context is on.

    The key a field has in the decode and encode maps, for a recipe that works one field out itself.
    """
    if context.raw_names:
        return field_name
    return context.rename.get(join_path(context.path, field_name)) or browse_name(field_name)


def _display_name(context: ProjectionContext, field_name: str) -> str:
    """Return what a client is told the field is called: the new name, else the raw ROS2 name."""
    return context.rename.get(join_path(context.path, field_name), field_name)


def _siblings(context: ProjectionContext, field_names: list[str]) -> dict[str, str]:
    """Return the browse name of each of these sibling fields, refusing two that share one.

    :raises DuplicateVariableNameError: When two give one browse name, whether by conversion or by
        a rename. The error names both ROS2 names.
    """
    if context.raw_names:
        return {name: name for name in field_names}
    renamed = {name: new for name in field_names if (new := context.rename.get(join_path(context.path, name)))}
    return browse_names(field_names, renamed)


def _selected_fields(
    message_type: type[Message],
    context: ProjectionContext,
) -> Iterator[tuple[FieldDescription, ProjectionContext, type[Message] | None]]:
    """Yield each field the selection kept, with its context and the message type it nests.

    The generic walk's steps share this route and differ only in what they do at each stop.
    """
    for described in message_type.ros2.fields:
        child = context.child(described.name)
        if not child.included:
            continue
        yield described, child, message_type.ros2_nested.get(described.name)


def _below(level: NodeLevel, entry: PlanEntry) -> PlanEntry:
    """Carry one entry up under the field that produced it, which becomes a level above it."""
    if isinstance(entry, PlanObject):
        return replace(entry, name=flat_name(level.name, entry.name))
    return replace(entry, above=(level, *entry.above))


def _repeats(field_type: FieldType) -> bool:
    """Return whether a field holds many of its type rather than one."""
    return not isinstance(field_type, NestedType)


def _element_context(element_type: type[Message], context: ProjectionContext) -> ProjectionContext:
    """Return the context the elements of a repeated message field are rendered in.

    The path goes back to empty, because an element's leaves are named from its own root. The
    selection becomes the element type's whole field set: the selector treats a repeated message as
    one leaf and never walks inside it (:mod:`opensmi.ros2.runtime.selection`), so an element is
    never partial and can be rebuilt exactly. Fields keep their ROS2 names and no rename
    applies, because they are the field names of a DataType. The registry is kept, so curated
    Projections still apply to elements.
    """
    return replace(context, path="", selected=frozenset(field_paths(element_type)), rename={}, raw_names=True)


def _plan_structures(
    described: FieldDescription,
    context: ProjectionContext,
    element_type: type[Message],
    name: str,
    display: str,
) -> PlanVariable:
    """Return the variable a repeated message field becomes: one array of the element's structure.

    It boots as an empty array, a legal ROS2 value, so "nobody has written this yet" is told by the
    StatusCode, ``BadWaitingForInitialData`` (:mod:`opensmi.ros2.server.status`).

    :raises UnprojectableFieldError: When the element cannot render as a structure.
    """
    return PlanVariable(
        name=name,
        display_name=display,
        source=context.path,
        initial=[],
        # A structure travels in a Variant as an ExtensionObject; the DataType says which one.
        variant_type=ua.VariantType.ExtensionObject,
        datatype=element_structure(element_type, context),
    )


def element_structure(element_type: type[Message], context: ProjectionContext) -> type:
    """Return the structure one element of a repeated message renders as.

    Public because a recipe that repeats a message of its own needs it too. Its fields are the
    element's plan, nested like the message. Memoised on the registry
    (:attr:`ProjectionRegistry.element_structures`).

    :raises UnprojectableFieldError: When the element's Projection emits an Object, or a field of
        it is of a type that has no place in a structure.
    """
    known = context.registry.element_structures.get((element_type, context.qualifier))
    if known is not None:
        return known
    structure = _element_structure(element_type, context)
    context.registry.element_structures[element_type, context.qualifier] = structure
    return structure


def _element_structure(element_type: type[Message], context: ProjectionContext) -> type:
    """Work out the structure of one element, which :func:`element_structure` remembers."""
    shaped_element = shaped(element_type, _element_context(element_type, context))
    objects = [entry.name for entry in shaped_element if isinstance(entry, PlanObject)]
    if objects:
        listed = ", ".join(repr(name) for name in objects)
        message = (
            f"the Projection of {element_type.ros2.type_name} renders {listed} as a browsable "
            f"Object, and {context.path!r} repeats it, so its elements have to render as the "
            f"fields of one structure: there are no nodes inside an array. Register "
            f"Default({element_type.ros2.type_name!r}, override=True) for the Component that "
            f"repeats it, or exclude the repeated field"
        )
        raise UnprojectableFieldError(message)
    plan = [entry for entry in shaped_element if isinstance(entry, PlanVariable)]
    # A curated element keeps the name of its ROS2 type: a type curated differently in two
    # Components is refused where the DataType is made.
    return level_datatypes(plan, element_type, lambda _, message_type: message_type.ros2.type_name)[()]


NodeLevelNaming: TypeAlias = "Callable[[NodePath, type[Message]], str]"
"""How a level whose subtree is curated is named: from its node path and its ROS2 type."""


def curated_naming(qualifier: str) -> NodeLevelNaming:
    """Return the naming of curated levels: the qualifier and the node path, joined with dots.

    ``Platform.Odom`` names the root and ``Platform.Odom.Pose`` the level below it. The qualifier
    is the Component and the Binding, so a name does not depend on what else is declared.
    """
    return lambda path, _: flat_name(qualifier, *path)


_PACKABLE: Final = frozenset(
    getattr(ua.VariantType, name)
    for name in (
        "Boolean",
        "SByte",
        "Byte",
        "Int16",
        "UInt16",
        "Int32",
        "UInt32",
        "Int64",
        "UInt64",
        "Float",
        "Double",
        "String",
        "ByteString",
    )
)
"""The OPC UA types a field of a structure can carry."""


def _annotated(element: Any, initial: object) -> tuple[Any, int]:
    """Return the annotation of a field and the length of its fixed-size array, from its boot value."""
    if isinstance(initial, list):
        return list[element], len(initial)
    return element, 0


def _signature(message_type: type[Message]) -> tuple[StructureField, ...]:
    """Return the fields of the structure of a ROS2 message type, as its definition has them."""
    fields: list[StructureField] = []
    for described in message_type.ros2.fields:
        nested = message_type.ros2_nested.get(described.name)
        if nested is None:
            initial, variant_type = ua_shape(described.field_type)
            fields.append((described.name, *_annotated(getattr(ua, variant_type.name), initial)))
        else:
            length = described.field_type.length if isinstance(described.field_type, ArrayType) else 0
            element = message_class(nested)
            fields.append((described.name, list[element] if _repeats(described.field_type) else element, length))
    return tuple(fields)


def message_class(message_type: type[Message]) -> type:
    """Return the structure of a ROS2 message type, named by it, with the fields of its definition.

    It is the DataType of a level whose subtree is not curated. Equal for equal types.
    """
    return structure_class(message_type.ros2.type_name, _signature(message_type))


def level_datatypes(
    plan: Iterable[PlanVariable],
    root: type[Message],
    naming: NodeLevelNaming,
) -> dict[NodePath, type]:
    """Return the structure DataType of every level of a plan and of its root, by node path.

    A level whose rendered subtree is the one its ROS2 type has (no field excluded or renamed, no
    leaf of another type) gets the class of that type, :func:`message_class`. Any other gets a class
    of the rendered shape, named by ``naming``. A level is curated when anything below it is,
    since its fields differ then. The root is ``()``.

    :param plan: The variables of one message, as :func:`shaped` returns them.
    :param root: The ROS2 type of the message.
    :raises UnprojectableFieldError: When a leaf has a type no structure field can carry.
    """
    leaves = list(plan)
    tree: dict[str, Any] = {}
    for leaf in leaves:
        node = tree
        for name in leaf.node_path[:-1]:
            node = node.setdefault(name, {})
        node[leaf.node_path[-1]] = leaf
    levels = inner_levels(leaves)
    classes: dict[NodePath, type] = {}

    def build(node: dict[str, Any], path: NodePath, message_type: type[Message]) -> None:
        fields: list[StructureField] = []
        for name, child in node.items():
            if isinstance(child, dict):
                below = (*path, name)
                build(child, below, cast("type[Message]", levels[below].message))
                fields.append((levels[below].display_name, classes[below], 0))
            else:
                fields.append((child.display, *_leaf_annotation(child)))
        kept = tuple(fields)
        name = naming(path, message_type) if kept != _signature(message_type) else message_type.ros2.type_name
        classes[path] = structure_class(name, kept)

    build(tree, (), root)
    return classes


def _leaf_annotation(leaf: PlanVariable) -> tuple[Any, int]:
    """Return the field annotation of a leaf, and the length of its fixed-size array.

    Only a structure class and a plain value can be a field. Any other DataType (an enumeration, a
    type from the nodeset or one OPC UA defines) is left to the built-in type, so Components that
    curate such a field differently still share one structure.

    :raises UnprojectableFieldError: When the leaf is of a type no structure field can carry.
    """
    if isinstance(leaf.datatype, type):
        element = leaf.datatype
    elif leaf.variant_type in _PACKABLE:
        element = getattr(ua, leaf.variant_type.name)
    else:
        message = (
            f"{leaf.full_name!r} is a {leaf.variant_type.name}, which no field of a structure can "
            f"carry; curate its type so that the field is a plain value, or exclude it"
        )
        raise UnprojectableFieldError(message)
    return _annotated(element, leaf.initial)


def level_value(
    plan: Iterable[PlanVariable],
    classes: Mapping[NodePath, type],
    values: Mapping[str, object],
    path: NodePath = (),
) -> Any:
    """Return the instance of the structure of one level, from what a message decoded to.

    This is the value a level's complex Variable holds. ``values`` is what the Projection's decode
    returns for the message, keyed by flat name.

    :param classes: The result of :func:`level_datatypes` for the plan.
    :param path: The level to build. The root by default.
    :raises KeyError: When a leaf below the level has no value.
    """
    leaves = [leaf for leaf in plan if leaf.node_path[: len(path)] == path]
    return _instance(leaves, classes, values, path)


def _instance(
    leaves: list[PlanVariable],
    classes: Mapping[NodePath, type],
    values: Mapping[str, object],
    path: NodePath,
) -> Any:
    built: dict[str, object] = {}
    for name in dict.fromkeys(leaf.node_path[len(path)] for leaf in leaves):
        below = [leaf for leaf in leaves if leaf.node_path[len(path)] == name]
        here = (*path, name)
        if len(below) == 1 and below[0].node_path == here:
            leaf = below[0]
            value = values[leaf.full_name]
            if isinstance(leaf.datatype, type):
                value = [structure_instance(leaf.datatype, each) for each in value]  # pyright: ignore[reportGeneralTypeIssues]
            built[leaf.display] = value
        else:
            built[inner_levels(below)[here].display_name] = _instance(below, classes, values, here)
    return classes[path](**built)


def ua_value(value: object) -> object:
    """Return a decoded field value in the shape an OPC UA write wants.

    A payload holds a ``tuple`` and the library builds an array from a ``list``, so that is the one
    conversion.
    """
    return list(value) if isinstance(value, tuple) else value


def ros2_value(value: object, field_type: FieldType) -> object:
    """Return a value a client wrote in the shape a payload holds.

    The mirror image of :func:`ua_value`. It reads the modelled field, since ``[]`` alone does not
    say whether a tuple or a list is wanted.

    It also narrows a number to the exact Python type of its ROS2 field, because the guard that runs
    next (:mod:`opensmi.ros2.runtime.guard`) tests exact types, and OPC UA's scalar types subclass
    ``int``, ``float`` and ``str`` (a Double node written with ``1`` hands back an integer). The
    field decides the type. A value the field cannot hold is left as it came, for the guard to
    refuse by name.
    """
    if is_byte_container(field_type):
        return bytes(value) if isinstance(value, (bytes, bytearray, list)) else value
    if isinstance(field_type, (ArrayType, SequenceType)):
        if not isinstance(value, (list, tuple)):
            return value
        return tuple(_narrowed_scalar(each, field_type.element) for each in value)
    return _narrowed_scalar(value, field_type)


def _narrowed_scalar(value: object, element: ElementType) -> object:
    """Return one scalar as exactly the Python type its ROS2 element type is, where it can be."""
    match element:
        case PrimitiveType(name=name) if name in _INTEGERS:
            return int(value) if isinstance(value, int) and not isinstance(value, bool) else value  # type: ignore[arg-type]
        case PrimitiveType(name="bool"):
            return bool(value) if isinstance(value, (bool, int)) else value
        case PrimitiveType(name="float32" | "float64"):
            return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else value
        case PrimitiveType() | StringType():
            return str(value) if isinstance(value, str) else value
        case _:
            return value


_VARIANT_TYPES: Final[Mapping[PrimitiveName, ua.VariantType]] = {
    "bool": ua.VariantType.Boolean,
    "byte": ua.VariantType.Byte,
    # ROS2 `char` and `wchar` arrive as Python `str`.
    "char": ua.VariantType.String,
    "wchar": ua.VariantType.String,
    "float32": ua.VariantType.Float,
    "float64": ua.VariantType.Double,
    "int8": ua.VariantType.SByte,
    "int16": ua.VariantType.Int16,
    "int32": ua.VariantType.Int32,
    "int64": ua.VariantType.Int64,
    "uint8": ua.VariantType.Byte,
    "uint16": ua.VariantType.UInt16,
    "uint32": ua.VariantType.UInt32,
    "uint64": ua.VariantType.UInt64,
}

_INTEGERS: Final[frozenset[str]] = frozenset(INTEGER_RANGES)
"""The primitive names whose Python type is ``int``, taken from the guard's table."""

_ZEROS: Final[Mapping[ua.VariantType, object]] = {
    ua.VariantType.Boolean: False,
    ua.VariantType.String: "",
    ua.VariantType.Float: 0.0,
    ua.VariantType.Double: 0.0,
}


def ua_shape(field_type: FieldType) -> tuple[object, ua.VariantType]:
    """Return the value one modelled ROS2 field boots with, and the OPC UA type it carries.

    Public because a declared ROS2 parameter type is shaped here too
    (:attr:`opensmi.ros2.runtime.parameters.ParameterType.field_type`,
    :func:`opensmi.ros2.server.parameters.ua_parameter_shape`), so one table decides the mapping.
    """
    if is_byte_container(field_type):
        return b"", ua.VariantType.ByteString
    variant_type = _element_variant_type(element_of(field_type))
    zero = _ZEROS.get(variant_type, 0)
    match field_type:
        case ArrayType(length=length):
            # A fixed-size array arrives with its elements, so it boots with as many.
            return [zero] * length, variant_type
        case SequenceType():
            return [], variant_type
        case _:
            return zero, variant_type


def _element_variant_type(element: ElementType) -> ua.VariantType:
    """Return the OPC UA type of a scalar, or of what a container holds."""
    match element:
        case PrimitiveType(name=name):
            return _VARIANT_TYPES[name]
        case StringType():
            return ua.VariantType.String
        case NestedType():  # pragma: no cover - refused before it is reached
            message = f"a message field cannot render as a variable: {element.type_name}"
            raise UnprojectableFieldError(message)
