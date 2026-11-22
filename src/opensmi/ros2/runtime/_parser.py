# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Parser: a ROS2 interface definition becomes a field structure and its types.

It reads the installed interface package by introspection over the generated class. The vendored
parser in ``tools/catalog/`` reads the other source and produces the same ``MessageDescription``;
a Jazzy CI job checks that the two agree. This is the one module that imports interface packages,
and it imports no ``rclpy``. Private: an interface class must not reach code above the seam.
"""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING, Any

from opensmi.ros2.runtime.errors import MissingInterfacePackageError, UnusableMessageTypeError
from opensmi.ros2.runtime.messages import (
    ACTION_MEMBERS,
    SERVICE_MEMBERS,
    FieldDescription,
    MessageDescription,
    canonical_type_name,
    parse_field_type,
)

if TYPE_CHECKING:
    from collections.abc import Iterable


def describe(type_name: str) -> MessageDescription:
    """Return the structure of a ROS2 message type, read off its installed interface class."""
    canonical = canonical_type_name(type_name)
    return _describe_class(canonical, import_interface_class(canonical))


def import_interface_class(type_name: str) -> Any:
    """Import and return the generated interface class for a ROS2 type name.

    :param type_name: ``package/kind/Name``, or ``package/Name`` for a message.
    :raises InvalidMessageTypeNameError: When the name has neither shape.
    :raises MissingInterfacePackageError: When the interface package is not installed. This is
        fatal at init: ``rclpy`` cannot subscribe to a type without its typesupport.
    :raises UnusableMessageTypeError: When the package is installed but has no such message.
    """
    package, kind, module, name = _import_package(type_name)
    interface_class = getattr(module, name, None) or _action_member(module, name)
    if interface_class is None or not hasattr(interface_class, "get_fields_and_field_types"):
        message = f"{package}.{kind} is installed but does not define the message {name!r}"
        raise UnusableMessageTypeError(message)
    return interface_class


def import_action_class(type_name: str) -> Any:
    """Import and return the generated class for a ROS2 action type (``package/action/Name``).

    :raises InvalidMessageTypeNameError: When the name has the wrong shape.
    :raises MissingInterfacePackageError: When the interface package is not installed.
    :raises UnusableMessageTypeError: When the package has no such action.
    """
    return _import_members(type_name, "action", ACTION_MEMBERS)


def import_service_class(type_name: str) -> Any:
    """Import and return the generated class for a ROS2 service type (``package/srv/Name``).

    :raises InvalidMessageTypeNameError: When the name has the wrong shape.
    :raises MissingInterfacePackageError: When the interface package is not installed.
    :raises UnusableMessageTypeError: When the package has no such service.
    """
    return _import_members(type_name, "service", SERVICE_MEMBERS)


def _import_members(type_name: str, what: str, members: Iterable[str]) -> Any:
    """Import an action or service class, which has no fields of its own and so skips :func:`import_interface_class`."""
    package, kind, module, name = _import_package(type_name)
    interface_class = getattr(module, name, None)
    if interface_class is None or not all(hasattr(interface_class, member) for member in members):
        message = f"{package}.{kind} is installed but does not define the {what} {name!r}"
        raise UnusableMessageTypeError(message)
    return interface_class


def _import_package(type_name: str) -> tuple[str, str, Any, str]:
    """Import the module one ROS2 type lives in, and return the parts of its name with it.

    :raises MissingInterfacePackageError: When the interface package is not installed.
    :raises UnusableMessageTypeError: When the package is installed but the module is not there.
    """
    package, kind, name = canonical_type_name(type_name).split("/")
    try:
        module = importlib.import_module(f"{package}.{kind}")
    except ModuleNotFoundError as error:
        if error.name == package:
            message = (
                f"the interface package {package!r} is not installed, so no Binding can receive "
                f"{type_name!r}; install the package or drop the declaration"
            )
            raise MissingInterfacePackageError(message) from error
        message = f"{package} is installed but {package}.{kind} could not be imported"
        raise UnusableMessageTypeError(message) from error
    return package, kind, module, name


def _action_member(module: Any, name: str) -> Any:
    """Return the generated class of an action's goal, feedback or result, or ``None``.

    The package exports only ``Fibonacci``, while ROS2 names the goal ``Fibonacci_Goal`` elsewhere,
    so that spelling is resolved as ``Fibonacci.Goal``. The suffix must be an action member name,
    so a message genuinely called ``Foo_Bar`` is not mistaken for a member of a missing ``Foo``.
    """
    parent, _, member = name.rpartition("_")
    if not parent or member not in ACTION_MEMBERS:
        return None
    return getattr(getattr(module, parent, None), member, None)


def _describe_class(type_name: str, interface_class: Any) -> MessageDescription:
    """Model every field of an interface class, in declaration order."""
    # Both come from the same member list in the same order. The strict zip raises if a future
    # generator breaks that, instead of pairing a field with another field's type.
    field_types: dict[str, str] = interface_class.get_fields_and_field_types()
    slots: tuple[object, ...] = interface_class.SLOT_TYPES
    fields = tuple(
        FieldDescription(
            name=name,
            field_type=parse_field_type(text, nested_type_name=_nested_type_name(slot)),
            ros2_text=text,
        )
        for (name, text), slot in zip(field_types.items(), slots, strict=True)
    )
    return MessageDescription(type_name=type_name, fields=fields)


def _nested_type_name(slot: object) -> str | None:
    """Return the fully qualified name of a slot's nested type, or ``None`` when it has none.

    The field-type string drops the ``msg``/``srv``/``action`` namespace, which only the slot keeps.
    """
    # An Array or a sequence wraps its element type; a scalar field's slot is the element type.
    element: Any = getattr(slot, "value_type", slot)
    namespaces: tuple[str, ...] | None = getattr(element, "namespaces", None)
    if namespaces is None:
        return None
    name: str = element.name
    return "/".join([*namespaces, name])
