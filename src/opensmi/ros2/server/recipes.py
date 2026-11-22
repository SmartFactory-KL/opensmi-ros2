# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The recipes: curating a ROS2 type is writing data, not code.

A Projection is three pure functions (:mod:`opensmi.ros2.server.projection`). A curation is a plain
data literal from a closed set, so which Projections exist is a list somebody reads:

.. code-block:: python

    Default("nav_msgs/msg/Odometry")  # walk it; curate nothing
    Scalar("std_msgs/msg/Float64")  # one value, one variable
    Enum("sensor_msgs/msg/BatteryState", fields={"power_supply_status": {0: "Unknown"}})
    Blob("nav_msgs/msg/OccupancyGrid", field="data")  # one bulk byte string
    Drop("nav_msgs/msg/Odometry", fields=("pose/covariance",), override=True)

:class:`Functions` is the escape hatch for a type that fits none of them. There is no Object recipe:
a message is already a browsable complex Variable, and the mutation chokepoint refuses
the Object verb by name.

Three rules hold for every class. Entries are keyed by type name alone: ``type_hash=`` records the
RIHS01 the entry was authored against so CI can report that a type moved, but it never selects an
entry. Registration is explicit, through
:meth:`~opensmi.ros2.server.projection.ProjectionRegistry.extended`, and a second entry for one
type is fatal unless it says ``override=True``. There is no entry-point discovery. Invertibility is
derived from the recipe, never a keyword: the walk always inverts, and the escape hatch
inverts only if given ``encode=``. ``Default(type_name, override=True)`` turns curation off for
that type only; a Projection nested inside still applies.
"""

from __future__ import annotations

import struct
from collections.abc import Collection, Mapping
from dataclasses import dataclass
from typing import TYPE_CHECKING, Final

from asyncua import ua

from opensmi.ros2.runtime.guard import INTEGER_RANGES
from opensmi.ros2.runtime.messages import ArrayType, PrimitiveType, SequenceType, is_byte_container
from opensmi.ros2.runtime.selection import field_paths
from opensmi.ros2.server.errors import NotInvertibleError, UnprojectableFieldError
from opensmi.ros2.server.events import SIGNED_BYTE_STRING_TYPE
from opensmi.ros2.server.projection import (
    ANY_TYPE,
    EnumDataType,
    PlanEntry,
    PlanVariable,
    ProjectDataType,
    Projection,
    ProjectionContext,
    ProjectionRegistry,
    StandardDataType,
    browse_name_of,
    generic_decode,
    generic_encode,
    generic_shape,
    ros2_value,
    ua_value,
)

if TYPE_CHECKING:
    from opensmi.ros2.runtime.messages import FieldDescription, FieldType, Message
    from opensmi.ros2.server.projection import DecodeFunction, EncodeFunction, ShapeFunction

__all__ = ["GENERIC", "GENERIC_REGISTRY", "Blob", "Default", "Drop", "Enum", "Functions", "Scalar"]

_PACKED: Final[Mapping[str, str]] = {
    "bool": "?",
    "byte": "B",
    "uint8": "B",
    "int8": "b",
    "int16": "h",
    "uint16": "H",
    "int32": "i",
    "uint32": "I",
    "int64": "q",
    "uint64": "Q",
    "float32": "f",
    "float64": "d",
}
"""The ``struct`` code of each fixed-width ROS2 primitive.

``char`` and ``wchar`` are absent: they cross as Python ``str`` and have no packable width.
"""

_SIGNED: Final[frozenset[str]] = frozenset({"int8", "int16", "int32", "int64"})
"""Which element types make a packed buffer a ``SignedByteString`` rather than a ``ByteString``.

Two's-complement integers only. A float buffer keeps the plain ``ByteString``.
"""

_INTEGERS: Final = frozenset(INTEGER_RANGES)
"""Which field types can carry an enumeration: one whole number, of any ROS2 width."""

_LITTLE_ENDIAN: Final = "<"
"""Byte order, fixed because a client is not necessarily on this host."""


@dataclass(frozen=True, slots=True, kw_only=True)
class Default(Projection):
    """The generic field-walk, as an ordinary entry.

    Registered for one type, it disables curation for that type only. Registered for
    :data:`~opensmi.ros2.server.projection.ANY_TYPE` it is :data:`GENERIC`, the entry that matches
    last, so the engine needs no fallback branch.
    """

    def shape(self, message_type: type[Message], context: ProjectionContext) -> tuple[PlanEntry, ...]:
        """Render the type field by field, one flat variable per leaf."""
        return generic_shape(message_type, context)

    def decode(self, message: Message, context: ProjectionContext) -> Mapping[str, object]:
        """Read one payload into the names the walk created."""
        return generic_decode(message, context)

    @property
    def inverse(self) -> EncodeFunction | None:
        """Return the walk's inverse, which it always has."""
        return generic_encode


GENERIC: Final = Default(ANY_TYPE)
"""The generic field-walk, as the entry that matches last."""

GENERIC_REGISTRY: Final = ProjectionRegistry(entries=(GENERIC,))
"""A registry that curates nothing. :data:`opensmi.ros2.server.projections.DEFAULT_REGISTRY` extends it."""


@dataclass(frozen=True, slots=True, kw_only=True)
class Scalar(Projection):
    """A message that carries one value, rendered as that one variable.

    ``std_msgs/msg/Float64`` on ``/battery_voltage`` browses as ``battery_voltage`` instead of
    ``battery_voltage_data``. The message must have exactly one leaf field, and anything else is
    refused at init: a second field would have to be filled from a default on the way back, which
    is refused.

    :ivar variant_type: The OPC UA type to carry, where the field's own is not the one wanted.
    :ivar datatype: What the value is, as a DataType identifier in namespace 0, or ``None``.
    """

    variant_type: ua.VariantType | None = None
    datatype: int | None = None

    def shape(self, message_type: type[Message], context: ProjectionContext) -> tuple[PlanEntry, ...]:
        """Return the one variable this message is, named for whatever contains it.

        :raises UnprojectableFieldError: When the message does not carry exactly one value.
        """
        self._only_field(message_type)
        # Take the walk's variable and rename it, so the boot value and variant type stay in one place.
        walked = [entry for entry in generic_shape(message_type, context) if isinstance(entry, PlanVariable)]
        if not walked:  # pragma: no cover  (the parent skips a subtree the selection emptied)
            return ()
        (entry,) = walked
        return (
            PlanVariable(
                name="",
                source=entry.source,
                initial=entry.initial,
                variant_type=self.variant_type or entry.variant_type,
                datatype=None if self.datatype is None else StandardDataType(identifier=self.datatype),
            ),
        )

    def decode(self, message: Message, context: ProjectionContext) -> Mapping[str, object]:
        """Return the one value, under the empty name the shape gave it."""
        described = self._only_field(type(message))
        return {"": ua_value(getattr(message, described.name))}

    @property
    def inverse(self) -> EncodeFunction | None:
        """Return the inverse, which a one-field message always has."""
        return self._encode

    def _encode(
        self,
        message_type: type[Message],
        values: Mapping[str, object],
        context: ProjectionContext,
    ) -> Message:
        """Put the one value back into the one field."""
        described = self._only_field(message_type)
        value = ros2_value(values[""], described.field_type)
        return generic_encode(message_type, {}, context, {described.name: value})

    def _only_field(self, message_type: type[Message]) -> FieldDescription:
        """Return the message's single leaf field.

        :raises UnprojectableFieldError: When the message has more than one field, or when its
            one field is another message.
        """
        fields = message_type.ros2.fields
        nested = message_type.ros2_nested
        if len(fields) == 1 and fields[0].name not in nested:
            return fields[0]
        listed = ", ".join(described.name for described in fields) or "nothing"
        message = (
            f"Scalar renders a message that carries one value, and {message_type.ros2.type_name} "
            f"carries {listed}; curate it with Drop, or let the walk have it"
        )
        raise UnprojectableFieldError(message)


@dataclass(frozen=True, slots=True, kw_only=True)
class Enum(Projection):
    """A message walked as usual, with named integer fields rendered as enumerations.

    Naming the values of a ROS2 constant set mints a DataType the client can browse, instead of
    leaving it a bare number whose meaning lives in a ``.msg`` file. The rest of the message is the
    generic walk, and the number is what goes back on a write, so the recipe inverts.

    :ivar fields: For each ROS2 field name, the label of each value it may carry. The keys are the
        ROS2 constants themselves, so a set that starts at ``1`` or skips a number stays correct.
    """

    fields: Mapping[str, Mapping[int, str]]

    def shape(self, message_type: type[Message], context: ProjectionContext) -> tuple[PlanEntry, ...]:
        """Walk the message, with an enumeration variable where a field was named."""
        return generic_shape(message_type, context, self._overrides(message_type, context))

    def decode(self, message: Message, context: ProjectionContext) -> Mapping[str, object]:
        """Read the payload as usual: an enumeration's value is the integer that was sent."""
        return generic_decode(message, context)

    @property
    def inverse(self) -> EncodeFunction | None:
        """Return the walk's inverse, which naming values does not take away."""
        return generic_encode

    def _overrides(self, message_type: type[Message], context: ProjectionContext) -> Mapping[str, PlanEntry]:
        """Return the enumeration variable of each named field."""
        return {
            name: PlanVariable(
                name=name,
                source=context.child(name).path,
                initial=next(iter(values), 0),
                # An OPC UA enumeration is an Int32 whatever the ROS2 width is.
                variant_type=ua.VariantType.Int32,
                datatype=EnumDataType(
                    name=self._datatype_name(context, name),
                    values=tuple(values.items()),
                ),
            )
            for name, values in self.fields.items()
            if self._require_an_integer_field(message_type, name)
        }

    def _datatype_name(self, context: ProjectionContext, field_name: str) -> str:
        """Return the DataType's browse name, qualified by the Component.

        DataTypes live in the server-wide address space, while registries are per Component.
        """
        local = self.type_name.rpartition("/")[2]
        parts = (context.qualifier, local, field_name)
        return "_".join(part for part in parts if part)

    def _require_an_integer_field(self, message_type: type[Message], field_name: str) -> bool:
        """Return ``True`` for a field that can carry an enumeration, and refuse one that cannot.

        :raises UnprojectableFieldError: When the field is absent, or is not a single integer.
        """
        described = next((each for each in message_type.ros2.fields if each.name == field_name), None)
        if described is not None and _scalar_name(described.field_type) in _INTEGERS:
            return True
        message = (
            f"Enum names {field_name!r} on {message_type.ros2.type_name}, which is not a single "
            f"integer field; an enumeration renders one number as one label"
        )
        raise UnprojectableFieldError(message)


@dataclass(frozen=True, slots=True, kw_only=True)
class Blob(Projection):
    """One fixed-width primitive array carried as a single byte string.

    A million-cell grid as one UA array is a million Variants on the wire, and as one
    ``ByteString`` it is one buffer copy. The rest of the message is the generic walk. The element
    type comes from the ROS2 definition, so ``int8[]``, ``uint8[]`` and ``float32[]`` need no
    keyword. Signedness is published as a DataType, because a ``-1`` would otherwise reach a plain
    ``ByteString`` as ``0xFF``.

    The recipe never transforms the bytes. Naming a ``datatype`` says what they are (a
    ``CompressedImage``'s ``data`` really is a JPEG), and then the round trip can no longer be
    claimed for the author, so the Blob inverts only if an ``encode`` is supplied.

    :ivar field: The ROS2 field to carry as bytes.
    :ivar datatype: What the bytes are, as a DataType identifier in namespace 0
        (``ua.ObjectIds.ImagePNG``), or ``None`` to publish the signedness derived from the
        element type.
    :ivar encode: A codec whose round trip the author claims. Supplying it is that claim.
    """

    field: str
    datatype: int | None = None
    encode: EncodeFunction | None = None

    def shape(self, message_type: type[Message], context: ProjectionContext) -> tuple[PlanEntry, ...]:
        """Walk the message, with the named field rendered as one byte string."""
        described = self._packed_field(message_type)
        return generic_shape(
            message_type,
            context,
            {
                self.field: PlanVariable(
                    name=self.field,
                    source=context.child(self.field).path,
                    initial=b"",
                    variant_type=ua.VariantType.ByteString,
                    datatype=self._datatype_of(described.field_type),
                )
            },
        )

    def decode(self, message: Message, context: ProjectionContext) -> Mapping[str, object]:
        """Read the payload as usual, with the named field packed into bytes and capped."""
        described = self._packed_field(type(message))
        if not context.child(self.field).included:
            # Packing a buffer the selection removed would cost a copy per message for nothing.
            return generic_decode(message, context)
        packed = _pack(getattr(message, self.field), described.field_type)
        return generic_decode(message, context, {self.field: _capped(packed, context.byte_string_ceiling)})

    @property
    def inverse(self) -> EncodeFunction | None:
        """Return the supplied codec, else byte identity; ``None`` when a ``datatype`` is named without a codec."""
        if self.encode is not None:
            return self.encode
        return None if self.datatype is not None else self._encode

    def _encode(
        self,
        message_type: type[Message],
        values: Mapping[str, object],
        context: ProjectionContext,
    ) -> Message:
        """Unpack the byte string back into the elements the field holds."""
        described = self._packed_field(message_type)
        written = _written(values, browse_name_of(context, self.field), message_type)
        unpacked = _unpack(bytes(written) if isinstance(written, (bytes, bytearray)) else b"", described.field_type)
        return generic_encode(message_type, values, context, {self.field: unpacked})

    def _datatype_of(self, field_type: FieldType) -> StandardDataType | ProjectDataType | None:
        """Return what the variable's bytes say they are."""
        if self.datatype is not None:
            return StandardDataType(identifier=self.datatype)
        element = _element_name(field_type)
        return ProjectDataType(identifier=SIGNED_BYTE_STRING_TYPE) if element in _SIGNED else None

    def _packed_field(self, message_type: type[Message]) -> FieldDescription:
        """Return the named field, having checked that it is a fixed-width primitive array.

        :raises UnprojectableFieldError: When the field is absent, is not repeated, or holds
            something with no fixed width (a string, or another message).
        """
        described = next((each for each in message_type.ros2.fields if each.name == self.field), None)
        if described is not None and _element_name(described.field_type) in _PACKED:
            return described
        message = (
            f"Blob names {self.field!r} on {message_type.ros2.type_name}, which is not an array "
            f"of a fixed-width primitive; only those have a byte string to be carried as"
        )
        raise UnprojectableFieldError(message)


@dataclass(frozen=True, slots=True, kw_only=True)
class Drop(Projection):
    """The generic walk with some fields taken out of it.

    Dropping a path takes its whole subtree with it, as the declaration's own ``exclude`` does.

    A Drop never inverts: its only inverse would fill the dropped fields with defaults,
    and a covariance of zeros means perfectly certain in ROS2, not unknown.

    :ivar fields: The ROS2 field paths to remove, relative to this type.
    """

    fields: Collection[str]

    def shape(self, message_type: type[Message], context: ProjectionContext) -> tuple[PlanEntry, ...]:
        """Walk what is left of the message."""
        return generic_shape(message_type, self._without(message_type, context))

    def decode(self, message: Message, context: ProjectionContext) -> Mapping[str, object]:
        """Read what is left of the payload."""
        return generic_decode(message, self._without(type(message), context))

    def _without(self, message_type: type[Message], context: ProjectionContext) -> ProjectionContext:
        """Return the context with the dropped paths gone.

        :raises UnprojectableFieldError: When a dropped path is not a path of this type.
        """
        _require_known_paths("Drop", message_type, self.fields)
        return context.without(self.fields)


@dataclass(frozen=True, slots=True, kw_only=True)
class Functions(Projection):
    """The escape hatch: a Projection written as the functions themselves.

    For a type that fits no recipe. It is still keyed by type name and registered explicitly, and
    it inverts only if given ``encode``.

    :ivar shaped_by: What this type becomes at init.
    :ivar decoded_by: What one payload writes.
    :ivar encode: How a written value becomes a payload again, or ``None`` for a rendering that
        cannot be published through.
    """

    shaped_by: ShapeFunction
    decoded_by: DecodeFunction
    encode: EncodeFunction | None = None

    def shape(self, message_type: type[Message], context: ProjectionContext) -> tuple[PlanEntry, ...]:
        """Return what the supplied shape function makes of this type."""
        return self.shaped_by(message_type, context)

    def decode(self, message: Message, context: ProjectionContext) -> Mapping[str, object]:
        """Return what the supplied decode function makes of one payload."""
        return self.decoded_by(message, context)

    @property
    def inverse(self) -> EncodeFunction | None:
        """Return the inverse this entry was given."""
        return self.encode


def _require_known_paths(recipe: str, message_type: type[Message], paths: Collection[str]) -> None:
    """Refuse a named path a type does not have, which is a typo rather than a choice.

    :raises UnprojectableFieldError: When a named path is not a path of this type.
    """
    known = set(field_paths(message_type))
    unknown = [path for path in paths if path not in known]
    if not unknown:
        return
    listed = ", ".join(repr(path) for path in sorted(unknown))
    message = (
        f"{recipe} names {listed} on {message_type.ros2.type_name}, which has no such field; "
        f"its paths are {', '.join(sorted(known))}"
    )
    raise UnprojectableFieldError(message)


def _written(values: Mapping[str, object], name: str, message_type: type[Message]) -> object:
    """Return what a client wrote to one variable of a plan, refusing a payload with a hole.

    :raises NotInvertibleError: When the name carries no value, as in the walk: a field filled from
        a default is a value nobody sent.
    """
    if name not in values:
        message = (
            f"{message_type.ros2.type_name} cannot be built without a value for {name!r}; a "
            f"field filled from a default is a value nobody sent"
        )
        raise NotInvertibleError(message)
    return values[name]


def _element_name(field_type: FieldType) -> str:
    """Return the ``.msg`` spelling of what a container holds, or ``""`` for anything else."""
    if isinstance(field_type, (ArrayType, SequenceType)) and isinstance(field_type.element, PrimitiveType):
        return field_type.element.name
    return ""


def _scalar_name(field_type: FieldType) -> str:
    """Return the ``.msg`` spelling of a single primitive field, or ``""`` for anything else."""
    return field_type.name if isinstance(field_type, PrimitiveType) else ""


def _pack(value: object, field_type: FieldType) -> bytes:
    """Return one array field as the bytes its elements are.

    A byte container already crossed as ``bytes``, so packing it is the identity.
    """
    if isinstance(value, (bytes, bytearray)):
        return bytes(value)
    elements = tuple(value) if isinstance(value, (list, tuple)) else ()
    code = _PACKED[_element_name(field_type)]
    return struct.pack(f"{_LITTLE_ENDIAN}{len(elements)}{code}", *elements)


def _unpack(data: bytes, field_type: FieldType) -> object:
    """Return the elements a byte string holds, in the shape the payload wants them.

    :raises ValueError: When the byte string is not a whole number of elements. This is a client's
        write, so it is refused per write and never fatal at init.
    """
    if is_byte_container(field_type):
        return data
    code = _PACKED[_element_name(field_type)]
    width = struct.calcsize(code)
    if len(data) % width:
        message = (
            f"a byte string of {len(data)} bytes is not a whole number of "
            f"{_element_name(field_type)} elements, which are {width} bytes each"
        )
        raise ValueError(message)
    return struct.unpack(f"{_LITTLE_ENDIAN}{len(data) // width}{code}", data)


def _capped(data: bytes, ceiling: int) -> bytes:
    """Return a bulk payload cut down to the transport's ceiling.

    A byte string past the ceiling makes the write fail and the variable stop updating for every
    client, so it is cut here instead. A client can compare the length against the capability
    variable (:mod:`opensmi.ros2.server.limits`).
    """
    return data if ceiling <= 0 or len(data) <= ceiling else data[:ceiling]
