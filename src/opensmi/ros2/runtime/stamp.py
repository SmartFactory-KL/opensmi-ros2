# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Filling a message header's timestamp, so that no declaration has to hand-write a clock.

A client writing a pose into a Skill's Parameters says where, not when, so the adapter fills the
time. It fills a stamp only where it is still zero, which is ROS2's convention for unset (``tf2``
reads it as "latest available"); a stamp a client wrote is kept.

A stamp is a field named ``stamp`` whose message has exactly the fields ``sec`` and ``nanosec``.
The name is part of the test because ``builtin_interfaces/msg/Duration`` has the same two fields.
The clock belongs to the Runtime, since a ROS2 node may run on simulated time.
"""

from __future__ import annotations

from functools import cache
from typing import TYPE_CHECKING, Final

from opensmi.ros2.runtime.messages import Message, NestedType, PrimitiveType

if TYPE_CHECKING:
    from opensmi.ros2.runtime.messages import MessageDescription

__all__ = ["NANOSECONDS_FIELD", "SECONDS_FIELD", "STAMP_FIELD", "stamp_paths", "stamped"]

STAMP_FIELD: Final = "stamp"
"""The one field name this module fills."""

SECONDS_FIELD: Final = "sec"
"""The whole-seconds field of a ``builtin_interfaces/msg/Time``."""

NANOSECONDS_FIELD: Final = "nanosec"
"""The remainder field of a ``builtin_interfaces/msg/Time``. The TF facet reuses both names."""

_TIME_FIELDS: Final = {
    SECONDS_FIELD: PrimitiveType(name="int32"),
    NANOSECONDS_FIELD: PrimitiveType(name="uint32"),
}
"""The shape that identifies ``builtin_interfaces/msg/Time``."""


@cache
def stamp_paths(message_type: type[Message]) -> tuple[tuple[str, ...], ...]:
    """Return the field-name chain of every stamp reachable in one message type.

    A repeated message field is not descended into: a client that authored a list of waypoints
    authored their times too (:mod:`opensmi.ros2.runtime.tf` writes its own for a ``TFMessage``).
    """
    found: list[tuple[str, ...]] = []
    for field in message_type.ros2.fields:
        nested = message_type.ros2_nested.get(field.name)
        if nested is None or not isinstance(field.field_type, NestedType):
            continue
        if field.name == STAMP_FIELD and _is_a_time(nested.ros2):
            found.append((field.name,))
            continue
        found.extend((field.name, *below) for below in stamp_paths(nested))
    return tuple(found)


def stamped(message: Message, *, seconds: int, nanoseconds: int) -> Message:
    """Return the message with every stamp that is still zero filled from one clock reading.

    One clock reading serves the whole message. The same object is returned when nothing needed filling.

    :param seconds: The whole seconds of the ROS2 clock, for the ``sec`` field.
    :param nanoseconds: The remainder, for the ``nanosec`` field.
    """
    for path in stamp_paths(type(message)):
        message = _filled(message, path, seconds, nanoseconds)
    return message


def _filled(message: Message, path: tuple[str, ...], seconds: int, nanoseconds: int) -> Message:
    """Return the message with the stamp at one chain filled, or the message itself if it is set."""
    name, rest = path[0], path[1:]
    held = getattr(message, name)
    if rest:
        replaced = _filled(held, rest, seconds, nanoseconds)
    elif getattr(held, SECONDS_FIELD) or getattr(held, NANOSECONDS_FIELD):
        return message
    else:
        replaced = type(held)(**{SECONDS_FIELD: seconds, NANOSECONDS_FIELD: nanoseconds})
    if replaced is held:
        return message
    return _with(message, name, replaced)


def _with(message: Message, name: str, value: Message) -> Message:
    """Return a copy of one message with one field replaced; other fields are carried by reference."""
    fields = {field.name: getattr(message, field.name) for field in message.ros2.fields}
    fields[name] = value
    return type(message)(**fields)


def _is_a_time(described: MessageDescription) -> bool:
    """Return whether a message description is ``builtin_interfaces/msg/Time``'s shape."""
    return {field.name: field.field_type for field in described.fields} == _TIME_FIELDS
