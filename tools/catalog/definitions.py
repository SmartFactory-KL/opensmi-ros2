# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The catalog's Parser: definition text becomes the one normalised description.

This is the second producer of :class:`~opensmi.ros2.runtime.messages.MessageDescription`.
:mod:`opensmi.ros2.runtime._parser` is the first, reading an installed interface class; this one
reads the ``.msg``, ``.srv`` and ``.action`` text of a package nobody can install. **They produce
the same value**, field for field and string for string, which is what keeps a Projection's
``shape`` a function of one input type rather than of two that merely look alike.

Nothing here invents a type model. The vendored ``rosidl_adapter`` parser answers the *syntax*
question -- what does this line of ``.msg`` text say -- and its answer is rendered back into the
IDL spelling an interface class reports, then handed to
:func:`~opensmi.ros2.runtime.messages.parse_field_type`, which is the one owned answer to the
*type* question. Two syntaxes, one type model, and what joins them is a string both sides already
speak.

The rendering is not a convenience. ``.msg`` and IDL are different vocabularies -- ``.msg``
``bool`` is IDL ``boolean``, ``byte`` is ``octet``, ``float64`` is ``double``, and ``char`` is
``uint8`` rather than ``char`` -- so a producer that passed ``.msg`` spellings straight through
would disagree with the installed class about every one of them, and the disagreement would show
up as a wrong Projection rather than as an error.

**Constants and defaults are read, and are kept out of the description.** They are not fields, so
the description has no place for them and a message dataclass has no attribute for them. They are
returned beside it, because the staleness grader needs exactly what the type hash cannot see: a
vendor changing ``SMALL = 0`` to ``SMALL = 2`` moves no hash and breaks every enum Projection
keyed on the value.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import cache
from typing import TYPE_CHECKING, Any, Final

from opensmi.ros2.runtime.messages import (
    ACTION_MEMBERS,
    SERVICE_MEMBERS,
    FieldDescription,
    Message,
    MessageDescription,
    build_message_dataclass,
    canonical_type_name,
    parse_field_type,
)
from opensmi.ros2.runtime.messages import message_dataclass as installed_dataclass
from tools.catalog._vendor import rosidl_adapter_parser as vendored
from tools.catalog.sources import load_sources

if TYPE_CHECKING:
    from collections.abc import Mapping
    from pathlib import Path

__all__ = [
    "Definition",
    "UnknownCatalogTypeError",
    "catalog_packages",
    "catalog_types",
    "definition",
    "describe",
    "describe_text",
    "message_dataclass",
    "read_definition",
]

MSG_TYPE_TO_IDL: Final[dict[str, str]] = {
    "bool": "boolean",
    "byte": "octet",
    "char": "uint8",
    "int8": "int8",
    "uint8": "uint8",
    "int16": "int16",
    "uint16": "uint16",
    "int32": "int32",
    "uint32": "uint32",
    "int64": "int64",
    "uint64": "uint64",
    "float32": "float",
    "float64": "double",
    "string": "string",
    "wstring": "wstring",
}
"""``.msg`` primitive spellings to the IDL ones an interface class reports.

Copied from ``rosidl_adapter.msg.MSG_TYPE_TO_IDL`` rather than reasoned out, because one entry is
not a renaming: ``.msg`` ``char`` becomes IDL ``uint8``, so a ``char`` field and a ``uint8`` field
are the same field by the time either reaches Python. Guessing at ``char`` here would produce a
description no installed class could ever match.
"""


class UnknownCatalogTypeError(LookupError):
    """The catalog holds no definition for that ROS2 type name."""


@dataclass(frozen=True, slots=True, kw_only=True)
class Definition:
    """One ROS2 type as the catalog holds it: its fields, and what the type hash cannot see.

    :ivar description: The normalised description, identical to what introspection over an
        installed class produces for the same type.
    :ivar constants: The definition's constants, by name. Not fields, and deliberately not in
        the description -- but they are what an enum Projection is keyed on.
    :ivar defaults: The default value declared for a field, for the fields that declare one.
    """

    description: MessageDescription
    constants: Mapping[str, object]
    defaults: Mapping[str, object]


def catalog_packages() -> tuple[str, ...]:
    """Return the interface packages the catalog carries, in the order sources.toml records."""
    return tuple(source.name for source in load_sources().packages)


def catalog_types() -> tuple[str, ...]:
    """Return every ROS2 message type the catalog can describe, sorted.

    A service contributes its request and its response, an action its goal, feedback and result:
    those are ordinary message types, and they are the ones a Binding or a Projection names.
    """
    return tuple(sorted(_types_by_name()))


def definition(type_name: str) -> Definition:
    """Return one catalog type's description, constants and defaults.

    :param type_name: ``package/kind/Name``, or ``package/Name`` for a message.
    :raises UnknownCatalogTypeError: When the catalog holds no such type.
    """
    canonical = canonical_type_name(type_name)
    found = _types_by_name().get(canonical)
    if found is None:
        message = (
            f"the definition catalog holds no {canonical!r}; it carries "
            f"{', '.join(catalog_packages())} and nothing else, because every other package in "
            f"scope has a binary release and is better read off the installed package"
        )
        raise UnknownCatalogTypeError(message)
    return found


def describe(type_name: str) -> MessageDescription:
    """Return one catalog type's description.

    :raises UnknownCatalogTypeError: When the catalog holds no such type.
    """
    return definition(type_name).description


def describe_text(text: str, type_name: str) -> dict[str, Definition]:
    """Parse one interface definition's text, and return every type it defines.

    A pure function of its two arguments, which is what makes the whole producer testable with no
    ROS2 installation and no catalog on disk: a ``.msg`` defines one type, a ``.srv`` defines its
    request and its response, and an ``.action`` defines its goal, feedback and result.

    :param text: The definition source, verbatim.
    :param type_name: ``package/kind/Name``, naming the *file* rather than the types it defines.
    """
    package, kind, name = canonical_type_name(type_name).split("/")
    if kind == "srv":
        service = vendored.parse_service_string(package, name, text)
        members = dict(zip(SERVICE_MEMBERS, (service.request, service.response), strict=True))
    elif kind == "action":
        action = vendored.parse_action_string(package, name, text)
        members = dict(zip(ACTION_MEMBERS, (action.goal, action.feedback, action.result), strict=True))
    else:
        parsed = vendored.parse_message_string(package, name, text)
        return {f"{package}/msg/{name}": _definition_of(f"{package}/msg/{name}", parsed)}
    return {
        f"{package}/{kind}/{name}_{member}": _definition_of(f"{package}/{kind}/{name}_{member}", specification)
        for member, specification in members.items()
    }


def read_definition(path: Path) -> str:
    """Read one definition file as UTF-8, never as the platform's default encoding.

    A vendor file carrying a degree sign in a comment would otherwise decode differently on two
    machines and produce two descriptions of one type.
    """
    return path.read_text(encoding="utf-8")


def message_dataclass(type_name: str) -> type[Message]:
    """Build the message dataclass of a catalog type, resolving nesting through the catalog.

    This is the fabrication that makes a vendor Projection testable at all: ``decode`` takes a
    message dataclass and never an ``rclpy`` object, so filling one of these in is the whole of
    what running a Projection against an uninstallable type requires.

    A nested type the catalog does not carry -- ``std_msgs/msg/Header``, say, which has a binary
    release and is therefore deliberately absent -- falls through to the installed package, and
    raises :class:`~opensmi.ros2.runtime.errors.MissingInterfacePackageError` where nothing is
    installed. That is the honest answer: the catalog covers what nobody can install, not what
    nobody happened to install.
    """
    return _fabricate(canonical_type_name(type_name))


@cache
def _fabricate(canonical: str) -> type[Message]:
    """Build and memoise one fabricated dataclass, so that nesting shares one identity."""

    def resolve(nested: str) -> type[Message]:
        canonical_nested = canonical_type_name(nested)
        if canonical_nested in _types_by_name():
            return _fabricate(canonical_nested)
        return installed_dataclass(canonical_nested)

    return build_message_dataclass(describe(canonical), resolve=resolve)


@cache
def _types_by_name() -> dict[str, Definition]:
    """Read every interface file the catalog carries, once."""
    found: dict[str, Definition] = {}
    for source in load_sources().packages:
        for path in source.interface_files():
            type_name = f"{source.name}/{path.parent.name}/{path.stem}"
            found.update(describe_text(read_definition(path), type_name))
    return found


def _definition_of(type_name: str, specification: Any) -> Definition:
    """Turn one parsed specification into the normalised description plus what it excludes."""
    fields = tuple(_field(each) for each in specification.fields)
    return Definition(
        description=MessageDescription(type_name=type_name, fields=fields),
        constants={constant.name: constant.value for constant in specification.constants},
        defaults={each.name: each.default_value for each in specification.fields if each.default_value is not None},
    )


def _field(parsed: Any) -> FieldDescription:
    """Model one parsed field, through the same type string an interface class would report."""
    text = _idl_text(parsed.type)
    nested = None if parsed.type.is_primitive_type() else f"{parsed.type.pkg_name}/msg/{parsed.type.type}"
    field_type = parse_field_type(text, nested_type_name=nested)
    return FieldDescription(name=parsed.name, field_type=field_type, ros2_text=text)


def _idl_text(parsed: Any) -> str:
    """Render one parsed ``.msg`` type as the string an interface class reports for it."""
    element = _idl_element(parsed)
    if not parsed.is_array:
        return element
    if parsed.array_size is None:
        return f"sequence<{element}>"
    if parsed.is_upper_bound:
        return f"sequence<{element}, {parsed.array_size}>"
    return f"{element}[{parsed.array_size}]"


def _idl_element(parsed: Any) -> str:
    """Render the scalar half of a parsed type: a primitive, a string, or another message."""
    if not parsed.is_primitive_type():
        return f"{parsed.pkg_name}/{parsed.type}"
    idl = MSG_TYPE_TO_IDL[parsed.type]
    if parsed.string_upper_bound:
        return f"{idl}<{parsed.string_upper_bound}>"
    return idl
