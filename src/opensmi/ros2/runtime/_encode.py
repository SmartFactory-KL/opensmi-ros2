# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Encoder: one message dataclass becomes one live ROS2 message instance.

The mirror of :mod:`opensmi.ros2.runtime._decode`, so that no ``rclpy`` object crosses the seam.
It runs on the event loop. Each conversion inverts one the Decoder made, and a wrong shape
aborts the process instead of raising:

* a nested message becomes its generated interface class again;
* a ``tuple`` becomes a ``list``, which the generated constructors accept for every container;
* ``bytes`` stays ``bytes`` for a ``uint8`` container and becomes a list of one-byte values for
  an ``octet`` one;
* a ``byte`` scalar becomes a one-byte ``bytes`` again, because handing the int back dies on
  ``Assertion 'PyBytes_Check(field)' failed`` (pinned by a Jazzy test).

The Encoder does not validate. :mod:`opensmi.ros2.runtime.guard` does that beforehand, as a pure
function that needs no ROS2 installation to test.
"""

from __future__ import annotations

from functools import cache
from typing import TYPE_CHECKING, Any

from opensmi.ros2.runtime._decode import _unchanged
from opensmi.ros2.runtime._parser import import_interface_class
from opensmi.ros2.runtime.messages import (
    ArrayType,
    NestedType,
    PrimitiveType,
    SequenceType,
    is_byte_container,
)

if TYPE_CHECKING:
    from collections.abc import Callable

    from opensmi.ros2.runtime.messages import ElementType, FieldType, Message


def encode(message: Message) -> Any:
    """Convert one message dataclass into the ``rclpy`` object its ROS2 type is.

    :param message: The payload, already checked by
        :func:`opensmi.ros2.runtime.guard.require_sendable`.
    :raises MissingInterfacePackageError: When the type's interface package is not installed,
        which a declared type is refused for at init and an encoded one cannot reach.
    """
    interface_class, plan = _plan(type(message))
    return interface_class(**{name: encode_field(getattr(message, name)) for name, encode_field in plan})


@cache
def _plan(message_type: type[Message]) -> tuple[Any, tuple[tuple[str, Callable[[Any], Any]], ...]]:
    """Return the interface class and the field-name and field-encoder pairs, in field order."""
    return (
        import_interface_class(message_type.ros2.type_name),
        tuple(
            (field.name, _field_encoder(field.field_type, message_type.ros2_nested.get(field.name)))
            for field in message_type.ros2.fields
        ),
    )


def _field_encoder(field_type: FieldType, nested: type[Message] | None) -> Callable[[Any], Any]:
    """Return the function that turns one decoded field value back into what ``rclpy`` wants."""
    if is_byte_container(field_type):
        return _bytes_for(field_type)
    match field_type:
        case ArrayType(element=element) | SequenceType(element=element):
            encode_element = _element_encoder(element, nested)
            if encode_element is None:
                return list
            return lambda value: [encode_element(item) for item in value]
        case _:
            return _element_encoder(field_type, nested) or _unchanged


def _element_encoder(element: ElementType, nested: type[Message] | None) -> Callable[[Any], Any] | None:
    """Return the encoder for a scalar, or ``None`` when the value already needs no work."""
    match element:
        case NestedType() if nested is not None:
            return encode
        case PrimitiveType(name="byte"):
            return _as_one_byte
        case _:
            return None


def _as_one_byte(value: Any) -> bytes:
    """Return a ``byte`` scalar as the one-byte ``bytes`` ``rclpy`` uses for IDL ``octet``."""
    return value if isinstance(value, bytes) else bytes([value])


def _bytes_for(field_type: FieldType) -> Callable[[Any], Any]:
    """Return how a byte container is handed over: ``uint8`` as a buffer, ``byte`` as a list of ``bytes``."""
    element = field_type.element if isinstance(field_type, (ArrayType, SequenceType)) else field_type
    if isinstance(element, PrimitiveType) and element.name == "byte":
        return lambda value: [bytes([item]) for item in value]
    return bytes
