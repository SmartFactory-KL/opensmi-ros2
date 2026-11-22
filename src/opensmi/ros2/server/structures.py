# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The ROS2 side of the Structure DataTypes: a message level as a dataclass.

:func:`structure_class` makes a dataclass from a name and its fields, and ``opensmi.server.structures``
creates the DataType, its encoding and its definition from that class and encodes its instances.
What is left here is the mapping between the maps a projection works with and those instances, and
the checks a write needs that the encoding does not make.
"""

from __future__ import annotations

import dataclasses
import typing
from functools import cache
from typing import TYPE_CHECKING, Any, TypeAlias

from opensmi.ros2.server.errors import DeclarationError

if TYPE_CHECKING:
    from collections.abc import Mapping

__all__ = [
    "DuplicateStructureError",
    "StructureField",
    "structure_class",
    "structure_elements",
    "structure_instance",
]

NAME_SEPARATOR = "."
"""Joins the names of the levels above a field in the maps a projection works with.

It spells a key, not a BrowseName: the nodes nest.
"""

StructureField: TypeAlias = "tuple[str, Any, int]"
"""A field as its name, its annotation and the length of a fixed-size array (zero otherwise).

The annotation is a ua class, a structure class, or a list of one of them.
"""


class DuplicateStructureError(DeclarationError):
    """Two descriptions of one structure name disagree.

    Raised before any node is made.
    """


@cache
def structure_class(name: str, fields: tuple[StructureField, ...]) -> type:
    """Return the dataclass of a name and fields, the same one for equal arguments.

    The length of a fixed-size array is kept in the field's metadata, for the write check.
    """
    return dataclasses.make_dataclass(
        name,
        [(each, annotation, dataclasses.field(metadata={"length": length})) for each, annotation, length in fields],
    )


def structure_instance(cls: type, values: Mapping[str, object], prefix: str = "") -> Any:
    """Return the instance of a structure that a map of its values describes.

    :param values: The values by flat name, the field names of the levels above joined with dots.
        An array of structures holds one such map per element.
    :raises KeyError: When a field has no value.
    """
    built: dict[str, object] = {}
    for each in dataclasses.fields(cls):
        inner, array = _inner(each.type)
        key = f"{prefix}{each.name}"
        if inner is None:
            built[each.name] = values[key]
        elif array:
            built[each.name] = [structure_instance(inner, item) for item in values[key]]  # pyright: ignore[reportGeneralTypeIssues]
        else:
            built[each.name] = structure_instance(inner, values, f"{key}{NAME_SEPARATOR}")
    return cls(**built)


def structure_elements(cls: type, value: object) -> list[dict[str, object]]:
    """Return the elements of an array a client wrote as maps, or refuse the array as a whole.

    :raises ValueError: When the value is not an array of the structure, when a field is null, or when
        a fixed-size array has the wrong length. The message names the element by its position.
    """
    if not isinstance(value, (list, tuple)):
        message = f"{type(value).__name__} is not an array of {cls.__name__}"
        raise ValueError(message)  # noqa: TRY004
    elements: list[dict[str, object]] = []
    for index, each in enumerate(value):
        try:
            elements.append(_values(cls, each))
        except ValueError as refused:
            message = f"element {index}: {refused}"
            raise ValueError(message) from refused
    return elements


def _inner(annotation: Any) -> tuple[type | None, bool]:
    """Return the structure class an annotation names, or ``None``, and whether it is a list."""
    array = typing.get_origin(annotation) is list
    element = typing.get_args(annotation)[0] if array else annotation
    return (element if isinstance(element, type) and dataclasses.is_dataclass(element) else None), array


def _values(cls: type, element: object, prefix: str = "") -> dict[str, object]:
    if not isinstance(element, cls):
        message = f"an element of {cls.__name__} arrived as {type(element).__name__}"
        raise ValueError(message)  # noqa: TRY004
    values: dict[str, object] = {}
    for each in dataclasses.fields(cls):
        value = getattr(element, each.name)
        array = typing.get_origin(each.type) is list
        if value is None or (array and None in value):
            # A null is not an empty value.
            message = f"the field {each.name!r} is null"
            raise ValueError(message)
        length = each.metadata["length"]
        if length and len(value) != length:
            message = f"the field {each.name!r} is an array of {length} elements, and {len(value)} were given"
            raise ValueError(message)
        inner, _ = _inner(each.type)
        key = f"{prefix}{each.name}"
        if inner is None:
            values[key] = list(value) if array else value
        elif array:
            values[key] = [_values(inner, item) for item in value]
        else:
            values.update(_values(inner, value, f"{key}{NAME_SEPARATOR}"))
    return values
