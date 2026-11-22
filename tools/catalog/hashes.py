# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""RIHS01: the type hash ROS2 computes, computed here from a description.

The ROS Interface Hashing Standard (REP-2011) hashes the REP-2016 *type description* of a type
together with every type it references. ROS2 puts the result on the wire -- a subscription reads
its publishers' ``topic_type_hash`` -- and this package already compares the two at bind time.
What it could not do until now is compute one from a definition, which is what the staleness
grader needs.

**It is a validation input, never a selector.** Nothing binds by hash, nothing selects a
Projection by hash, and an entry that records the hash it was authored against records it so that
drift is visible rather than so that anything switches on it.

**And it deliberately cannot see everything that matters.** ROS2 hashes field names and field
types; it excludes constants and field defaults, which is verifiable here and was verified before
it was relied on -- the hashable dictionary drops ``default_value`` before serialising, and
constants never enter the type description at all. So a vendor changing ``SMALL = 0`` to
``SMALL = 2`` moves no hash while breaking every Projection keyed on the value, which is precisely
why :mod:`tools.catalog.staleness` grades a *text* diff and uses this only to say how bad it is.

The serialisation is ROS2's own, reproduced rather than approximated: the field order is the
declaration order, referenced descriptions are sorted by name, and the JSON separators are the
ones ``rosidl_generator_type_description`` passes. A test checks the output against hashes ROS2
itself published, because "our hash" being a different number from ROS2's would make every
comparison meaningless in a way nothing else would catch.
"""

from __future__ import annotations

import hashlib
import json
from typing import TYPE_CHECKING, Final

from opensmi.ros2.runtime.messages import (
    ArrayType,
    NestedType,
    PrimitiveType,
    SequenceType,
    StringType,
    element_of,
)

if TYPE_CHECKING:
    from collections.abc import Callable, Mapping

    from opensmi.ros2.runtime.messages import FieldType, MessageDescription

__all__ = ["PREFIX", "individual_description", "rihs01", "type_description"]

PREFIX: Final = "RIHS01_"
"""The version prefix a hash string carries, per REP-2011."""

_SCALAR_TYPE_IDS: Final[Mapping[str, int]] = {
    "nested": 1,
    "int8": 2,
    "uint8": 3,
    "int16": 4,
    "uint16": 5,
    "int32": 6,
    "uint32": 7,
    "int64": 8,
    "uint64": 9,
    "float32": 10,
    "float64": 11,
    "char": 13,
    "wchar": 14,
    "bool": 15,
    "byte": 16,
    "string": 17,
    "wstring": 18,
    "bounded_string": 21,
    "bounded_wstring": 22,
}
"""``type_description_interfaces/msg/FieldType`` constants, for a field that is one value.

Written in this package's own spelling of a primitive rather than in IDL's, because that is what
a :class:`~opensmi.ros2.runtime.messages.PrimitiveType` carries.
"""

_EMPTY_MEMBER: Final = {
    "name": "structure_needs_at_least_one_member",
    "type": {"type_id": 3, "capacity": 0, "string_capacity": 0, "nested_type_name": ""},
}
"""The one field ROS2 hashes a message that has none.

IDL requires a struct to have at least one member, so the generator gives an empty ``.msg`` a
``uint8`` placeholder -- and the type description keeps it, while the generated Python class does
not report it as a field. Both halves of that are load-bearing here: leaving it out makes every
empty type's hash wrong, and putting it in the *description* would invent a field that no payload
carries and that both producers agree is not there.
"""

_ARRAY_OFFSET: Final = 48
"""What FieldType adds to a scalar's constant to say "a fixed number of these"."""

_BOUNDED_SEQUENCE_OFFSET: Final = 96
"""What it adds to say "at most this many of these"."""

_UNBOUNDED_SEQUENCE_OFFSET: Final = 144
"""What it adds to say "however many of these".

The three are arithmetic rather than a second table because FieldType is laid out that way: the
container is a stride over the same twenty-odd element constants, and writing out eighty-eight
names would be transcribing the same fact four times.
"""


def rihs01(type_name: str, resolve: Callable[[str], MessageDescription]) -> str:
    """Return the RIHS01 hash string of one ROS2 type.

    :param type_name: The fully qualified ``package/kind/Name``.
    :param resolve: How to get the description of that type and of every type it references.
        Both producers of a description satisfy it, which is the point: a hash computed from
        catalog text and a hash computed from an installed class are the same number or one of
        the two producers is wrong.
    """
    serialised = json.dumps(
        type_description(type_name, resolve),
        ensure_ascii=True,
        allow_nan=False,
        indent=None,
        separators=(", ", ": "),
        sort_keys=False,
    )
    return PREFIX + hashlib.sha256(serialised.encode("utf-8")).hexdigest()


def type_description(type_name: str, resolve: Callable[[str], MessageDescription]) -> dict[str, object]:
    """Return the REP-2016 type description ROS2 hashes, with defaults already excluded.

    Exposed beside :func:`rihs01` because it is the thing worth looking at when two hashes differ
    and nobody can see why.
    """
    referenced = sorted(_referenced(type_name, resolve))
    return {
        "type_description": individual_description(resolve(type_name)),
        "referenced_type_descriptions": [individual_description(resolve(name)) for name in referenced],
    }


def _referenced(type_name: str, resolve: Callable[[str], MessageDescription]) -> set[str]:
    """Return every type one type reaches through its fields, transitively and without itself."""
    found: set[str] = set()
    queue = list(_nested_names(resolve(type_name)))
    while queue:
        name = queue.pop()
        if name not in found:
            found.add(name)
            queue.extend(_nested_names(resolve(name)))
    return found


def _nested_names(description: MessageDescription) -> list[str]:
    """Return the types one description's fields name directly."""
    return [
        element.type_name
        for field in description.fields
        if isinstance(element := element_of(field.field_type), NestedType)
    ]


def individual_description(description: MessageDescription) -> dict[str, object]:
    """Serialise one type's own description, in ROS2's key order and without what it references.

    This is the unit the staleness grader compares, rather than the recursive hash: a file is
    graded against its own pinned text, and a change in a type it *references* arrives as a change
    to that type's own file. Grading recursively would report one vendor edit once per type that
    reaches it, which reads as a much larger change than the one that happened.
    """
    fields = [{"name": field.name, "type": _field_type(field.field_type)} for field in description.fields]
    return {"type_name": description.type_name, "fields": fields or [dict(_EMPTY_MEMBER)]}


def _field_type(field_type: FieldType) -> dict[str, object]:
    """Serialise one field's type, in ROS2's key order."""
    element = element_of(field_type)
    return {
        "type_id": _type_id(field_type),
        "capacity": _capacity(field_type),
        "string_capacity": _string_capacity(element),
        "nested_type_name": element.type_name if isinstance(element, NestedType) else "",
    }


def _type_id(field_type: FieldType) -> int:
    """Return the FieldType constant for one field, container and element together."""
    scalar = _scalar_type_id(element_of(field_type))
    match field_type:
        case ArrayType():
            return scalar + _ARRAY_OFFSET
        case SequenceType(maximum_length=None):
            return scalar + _UNBOUNDED_SEQUENCE_OFFSET
        case SequenceType():
            return scalar + _BOUNDED_SEQUENCE_OFFSET
        case _:
            return scalar


def _scalar_type_id(element: object) -> int:
    """Return the FieldType constant for what a field holds, ignoring how many of them."""
    match element:
        case NestedType():
            return _SCALAR_TYPE_IDS["nested"]
        case StringType(wide=wide, maximum_length=None):
            return _SCALAR_TYPE_IDS["wstring" if wide else "string"]
        case StringType(wide=wide):
            return _SCALAR_TYPE_IDS["bounded_wstring" if wide else "bounded_string"]
        case PrimitiveType(name=name):
            return _SCALAR_TYPE_IDS[name]
        case _:  # pragma: no cover -- the three above are every element type there is
            message = f"{element!r} is not a ROS2 element type"
            raise TypeError(message)


def _capacity(field_type: FieldType) -> int:
    """Return how many elements a field holds when that is declared, and zero when it is not."""
    match field_type:
        case ArrayType(length=length):
            return length
        case SequenceType(maximum_length=int() as bound):
            return bound
        case _:
            return 0


def _string_capacity(element: object) -> int:
    """Return a string field's declared bound, and zero for everything else."""
    if isinstance(element, StringType) and element.maximum_length is not None:
        return element.maximum_length
    return 0
