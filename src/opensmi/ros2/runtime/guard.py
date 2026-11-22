# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The type-mismatch guard: what may be put into a ROS2 message, and what aborts the process.

A ROS2 message field given a value of the wrong Python type does not raise in ``rclpy``. The C
conversion that runs when the message is sent dies on ``Assertion 'PyFloat_Check(field)' failed``,
a ``SIGABRT`` that takes down every Component and the OPC UA endpoint (pinned by a Jazzy test).
The check therefore uses exact types, not ``isinstance``: ``1`` is not a ``float64`` and ``True``
is not an ``int32``.

A value out of its field's range is refused too: ``2**31`` into an ``int32`` goes onto the wire
wrong. Declared bounds (``string<256>``, ``sequence<double, 3>``, ``double[9]``) are enforced here
as well, since OPC UA advertises a bound and enforces none.

The guard is a pure function of a message dataclass. The Runtime calls it on every goal it sends
and every message it publishes; the server half calls it at declaration time on the initial
values, so defaults that could never be sent fail at init.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Final

from opensmi.ros2.runtime.errors import FieldTypeMismatchError, FieldValueOutOfRangeError
from opensmi.ros2.runtime.messages import (
    ArrayType,
    Message,
    NestedType,
    PrimitiveType,
    SequenceType,
    StringType,
    is_byte_container,
)
from opensmi.ros2.runtime.selection import SEPARATOR

if TYPE_CHECKING:
    from collections.abc import Mapping, Sized

    from opensmi.ros2.runtime.messages import ElementType, FieldType, PrimitiveName

__all__ = ["INTEGER_RANGES", "require_sendable"]

INTEGER_RANGES: Final[Mapping[str, tuple[int, int]]] = {
    "byte": (0, 255),
    "int8": (-(2**7), 2**7 - 1),
    "int16": (-(2**15), 2**15 - 1),
    "int32": (-(2**31), 2**31 - 1),
    "int64": (-(2**63), 2**63 - 1),
    "uint8": (0, 2**8 - 1),
    "uint16": (0, 2**16 - 1),
    "uint32": (0, 2**32 - 1),
    "uint64": (0, 2**64 - 1),
}
"""What each ROS2 integer type holds. ``rclpy`` checks none of these at the boundary that matters."""

# `char` and `wchar` are `str` because that is how the message dataclass annotates them.
_SCALAR_TYPES: Final[Mapping[str, type]] = {
    "bool": bool,
    "char": str,
    "wchar": str,
    "float32": float,
    "float64": float,
}


def require_sendable(message: Message, *, path: str = "") -> None:
    """Refuse a payload ``rclpy`` would abort the process over, or put wrong on the wire.

    :param message: The payload, as an instance of the dataclass built for its ROS2 type.
    :param path: The field path of this message, used in nested refusals. Empty at the root.
    :raises FieldTypeMismatchError: When a value is not the Python type its ROS2 field is.
    :raises FieldValueOutOfRangeError: When a value does not fit its integer width, or exceeds a
        length its ROS2 field declares.
    """
    description = message.ros2
    for field in description.fields:
        _require_field(
            getattr(message, field.name),
            field.field_type,
            message.ros2_nested.get(field.name),
            _joined(path, field.name),
            description.type_name,
        )


def _require_field(
    value: object,
    field_type: FieldType,
    nested: type[Message] | None,
    path: str,
    type_name: str,
) -> None:
    """Refuse one field's value, whatever shape the field has."""
    if isinstance(field_type, (ArrayType, SequenceType)) and is_byte_container(field_type):
        _require_type(value, (bytes, bytearray), "a byte container", path, type_name)
        _require_length(value, field_type, path, type_name)
        return
    if isinstance(field_type, (ArrayType, SequenceType)):
        _require_type(value, (tuple, list), f"a container of {_named(field_type.element)}", path, type_name)
        _require_length(value, field_type, path, type_name)
        for index, element in enumerate(value):  # type: ignore[union-attr]
            _require_element(element, field_type.element, nested, f"{path}[{index}]", type_name)
        return
    _require_element(value, field_type, nested, path, type_name)


def _require_element(
    value: object,
    element: ElementType,
    nested: type[Message] | None,
    path: str,
    type_name: str,
) -> None:
    """Refuse one scalar value: a number, a string, or a whole nested message."""
    match element:
        case NestedType():
            _require_message(value, element, nested, path, type_name)
        case StringType(maximum_length=bound):
            _require_type(value, (str,), _named(element), path, type_name)
            _require_within(len(value), bound, "characters", path, type_name)  # type: ignore[arg-type]
        case PrimitiveType(name=name):
            _require_primitive(value, name, path, type_name)


def _require_message(
    value: object,
    element: NestedType,
    nested: type[Message] | None,
    path: str,
    type_name: str,
) -> None:
    """Refuse a nested field that does not hold the message dataclass its type says it does."""
    if nested is None or not isinstance(value, nested):  # pragma: no branch
        _refuse_the_type(value, element.type_name, path, type_name)
    require_sendable(value, path=path)  # type: ignore[arg-type]


def _require_primitive(value: object, name: PrimitiveName, path: str, type_name: str) -> None:
    """Refuse a number, a boolean or a character that is not exactly what its field holds."""
    bounds = INTEGER_RANGES.get(name)
    if bounds is None:
        _require_type(value, (_SCALAR_TYPES[name],), name, path, type_name)
        return
    # Identity check: `bool` is a subclass of `int`, and `isinstance` would let `True` through.
    if type(value) is not int:
        _refuse_the_type(value, name, path, type_name)
    low, high = bounds
    if not low <= value <= high:  # type: ignore[operator]
        message = (
            f"{type_name} field {path!r} is a {name}, which holds {low} to {high}, and the value "
            f"is {value!r}; ROS2 takes a value past that outright and puts it on the wire wrong"
        )
        raise FieldValueOutOfRangeError(message)


def _require_type(value: object, expected: tuple[type, ...], named: str, path: str, type_name: str) -> None:
    """Refuse a value whose Python type is not exactly one of the ones a field accepts."""
    if type(value) not in expected:
        _refuse_the_type(value, named, path, type_name)


def _refuse_the_type(value: object, named: str, path: str, type_name: str) -> None:
    """Refuse one value, naming the field, what ROS2 wants there and what arrived."""
    message = (
        f"{type_name} field {path!r} is {named}, and the value is a {type(value).__name__} "
        f"({value!r}); ROS2 does not raise on this, it aborts the process when the message is sent"
    )
    raise FieldTypeMismatchError(message)


def _require_length(value: object, field_type: ArrayType | SequenceType, path: str, type_name: str) -> None:
    """Refuse a container that is not the length its ROS2 field declares."""
    sized: Sized = value  # type: ignore[assignment]
    if isinstance(field_type, ArrayType):
        if len(sized) != field_type.length:
            message = (
                f"{type_name} field {path!r} is a fixed-size array of {field_type.length}, and "
                f"the value has {len(sized)} elements"
            )
            raise FieldValueOutOfRangeError(message)
        return
    _require_within(len(sized), field_type.maximum_length, "elements", path, type_name)


def _require_within(length: int, bound: int | None, what: str, path: str, type_name: str) -> None:
    """Refuse a length past a bound the ROS2 definition declared, if it declared one."""
    if bound is None or length <= bound:
        return
    message = (
        f"{type_name} field {path!r} is bounded at {bound} {what}, and the value has {length}; "
        f"OPC UA advertises a bound and enforces none, so it is enforced here"
    )
    raise FieldValueOutOfRangeError(message)


def _named(element: ElementType) -> str:
    """Return how a refusal names the ROS2 type a value was supposed to be."""
    match element:
        case PrimitiveType(name=name):
            return name
        case StringType(wide=True):
            return "a wstring"
        case StringType():
            return "a string"
        case NestedType(type_name=type_name):
            return type_name


def _joined(path: str, name: str) -> str:
    """Return one field path below another, with the separator every path here uses."""
    return f"{path}{SEPARATOR}{name}" if path else name
