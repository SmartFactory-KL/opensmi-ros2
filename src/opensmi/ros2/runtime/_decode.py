# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Decoder: one live ROS2 message instance becomes one message dataclass.

It runs on the event loop. ``rclpy`` objects use ``numpy.ndarray`` and ``array.array`` containers
and nested generated objects, none of which may cross into the server half. Each becomes a plain
Python value:

* a container of bytes (``uint8`` or ``byte``, fixed or not) becomes ``bytes``;
* every other container becomes a ``tuple``;
* a nested message becomes its own message dataclass;
* a ``byte`` scalar, which ``rclpy`` hands over as a one-byte ``bytes``, becomes an ``int``.

``messages.is_byte_container`` decides the first case, so the dataclass annotation and the decoded
value agree. The per-field plan is cached per message dataclass.
"""

from __future__ import annotations

from functools import cache
from typing import TYPE_CHECKING, Any

from opensmi.ros2.runtime.messages import (
    ArrayType,
    NestedType,
    PrimitiveType,
    SequenceType,
    is_byte_container,
)

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

    from opensmi.ros2.runtime.messages import ElementType, FieldType, Message


def decode(message: Any, message_type: type[Message]) -> Message:
    """Convert one live ROS2 message instance into an instance of its message dataclass."""
    return message_type(**{name: decode_field(getattr(message, name)) for name, decode_field in _plan(message_type)})


@cache
def _plan(message_type: type[Message]) -> tuple[tuple[str, Callable[[Any], Any]], ...]:
    """Return the field-name and field-decoder pairs for a message dataclass, in field order."""
    return tuple(
        (field.name, _field_decoder(field.field_type, message_type.ros2_nested.get(field.name)))
        for field in message_type.ros2.fields
    )


def _field_decoder(field_type: FieldType, nested: type[Message] | None) -> Callable[[Any], Any]:
    """Return the function that turns one ``rclpy`` field value into its plain Python value."""
    if is_byte_container(field_type):
        return _as_bytes
    match field_type:
        case ArrayType(element=element) | SequenceType(element=element):
            decode_element = _element_decoder(element, nested)
            if decode_element is None:
                return tuple
            return lambda value: tuple(decode_element(item) for item in value)
        case _:
            return _element_decoder(field_type, nested) or _unchanged


def _element_decoder(element: ElementType, nested: type[Message] | None) -> Callable[[Any], Any] | None:
    """Return the decoder for a scalar, or ``None`` when the value already needs no work."""
    match element:
        case NestedType() if nested is not None:
            return lambda value: decode(value, nested)
        case PrimitiveType(name="byte"):
            return _as_int
        case _:
            return None


def _unchanged(value: Any) -> Any:
    """Return the value as it came (``rclpy`` already hands over a builtin)."""
    return value


def _as_int(value: Any) -> int:
    """Return a ``byte`` scalar (a one-byte ``bytes`` in ``rclpy``) as an integer."""
    if isinstance(value, (bytes, bytearray)):
        return value[0]
    return int(value)


def _as_bytes(value: Any) -> bytes:
    """Return a container of bytes as ``bytes``.

    ``rclpy`` hands over a ``numpy.ndarray`` for a fixed ``uint8`` array, ``array.array('B')`` for
    a ``uint8`` sequence and a list of one-byte ``bytes`` for an ``octet`` sequence.
    """
    if isinstance(value, (bytes, bytearray, memoryview)):
        return bytes(value)
    items: Sequence[Any] = value
    if len(items) and isinstance(items[0], (bytes, bytearray)):
        return b"".join(items)
    return bytes(items)
