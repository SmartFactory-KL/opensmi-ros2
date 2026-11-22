# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Message dataclasses: the payloads that cross the seam, and the ROS2 facts they carry.

A ROS2 message reaches the rest of this package as an instance of a frozen dataclass built here,
never as an ``rclpy`` object. There is one dataclass per ROS2 message type, built once and cached.
It carries its :class:`MessageDescription` on the ``ros2`` class variable, so Projections, field
selection and the publish-time guard read ROS2 facts from a plain value.

The structure is taken by introspection over the imported interface class, not by parsing
definition text, which gets four things wrong: fixed-size arrays (``double[9]`` is nine values),
fields named like a constant (``points`` beside ``POINTS``), string bounds (``string<256>``) and
float widths (``float32`` against ``float64``).

Dataclasses built at run time have no static field set, so the types come from stubs beside this
module. ``<package>.py`` and ``<package>.pyi`` are generated from the definition catalog for
interface packages without a binary release. ``from opensmi.ros2.runtime.messages.neo_msgs2
import USBoardV2`` type-checks against the stub and resolves through :func:`message_dataclass`
at run time, to the dataclass a Binding of that type holds.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, make_dataclass
from functools import cache
from typing import TYPE_CHECKING, ClassVar, Final, Literal, TypeAlias

from opensmi.ros2.runtime.errors import InvalidMessageTypeNameError, MessageTypeError

if TYPE_CHECKING:
    from collections.abc import Callable, Mapping

__all__ = [
    "ACTION_MEMBERS",
    "SERVICE_MEMBERS",
    "ActionTypes",
    "ArrayType",
    "ElementType",
    "FieldDescription",
    "FieldType",
    "Message",
    "MessageDescription",
    "NestedType",
    "PrimitiveName",
    "PrimitiveType",
    "SequenceType",
    "ServiceTypes",
    "StringType",
    "action_types",
    "build_message_dataclass",
    "canonical_type_name",
    "is_byte_container",
    "is_single_value",
    "message_dataclass",
    "parse_field_type",
    "repeated_message",
    "service_types",
]

_KINDS = ("msg", "srv", "action")
_TWO_PART = 2
_THREE_PART = 3

ACTION_MEMBERS: Final = ("Goal", "Feedback", "Result")
"""The three message types a ROS2 action is made of, as the generated class spells them.

They are ordinary message types (``example_interfaces/action/Fibonacci_Goal``).
"""

SERVICE_MEMBERS: Final = ("Request", "Response")
"""The two message types a ROS2 service is made of, as the generated class spells them.

A service package exports its members (``std_srvs.srv.SetBool_Request``), while an action's goal
is only reachable as ``Fibonacci.Goal``. The ``_Event`` member Jazzy adds is not listed.
"""

PrimitiveName: TypeAlias = Literal[
    "bool",
    "byte",
    "char",
    "wchar",
    "float32",
    "float64",
    "int8",
    "int16",
    "int32",
    "int64",
    "uint8",
    "uint16",
    "uint32",
    "uint64",
]
"""The primitive ROS2 field types this Runtime models, spelled as a ``.msg`` file spells them."""

# The interface class reports IDL names (`octet`, `boolean`, `float`, `double`, ...), which are
# translated to the `.msg` vocabulary. IDL `long double` is absent on purpose: mapping it onto
# `float64` would narrow a value, so it raises `MessageTypeError`.
_IDL_PRIMITIVES: Final[Mapping[str, PrimitiveName]] = {
    "boolean": "bool",
    "octet": "byte",
    "char": "char",
    "wchar": "wchar",
    "float": "float32",
    "double": "float64",
    "int8": "int8",
    "int16": "int16",
    "int32": "int32",
    "int64": "int64",
    "uint8": "uint8",
    "uint16": "uint16",
    "uint32": "uint32",
    "uint64": "uint64",
    "short": "int16",
    "unsigned short": "uint16",
    "long": "int32",
    "unsigned long": "uint32",
    "long long": "int64",
    "unsigned long long": "uint64",
}

_PYTHON_TYPES: Final[Mapping[PrimitiveName, type]] = {
    "bool": bool,
    "byte": int,
    "char": str,
    "wchar": str,
    "float32": float,
    "float64": float,
    "int8": int,
    "int16": int,
    "int32": int,
    "int64": int,
    "uint8": int,
    "uint16": int,
    "uint32": int,
    "uint64": int,
}

_BYTE_LIKE: Final[frozenset[PrimitiveName]] = frozenset({"byte", "uint8"})

_SEQUENCE = re.compile(r"^sequence<(?P<element>.+?)(?:, (?P<bound>\d+))?>$")
_ARRAY = re.compile(r"^(?P<element>.+)\[(?P<length>\d+)\]$")
_STRING = re.compile(r"^(?P<wide>w?)string(?:<(?P<bound>\d+)>)?$")
_NESTED = re.compile(r"^(?P<package>[A-Za-z_]\w*)/(?P<name>[A-Za-z_]\w*)$")


@dataclass(frozen=True, slots=True, kw_only=True)
class PrimitiveType:
    """A single number, boolean or character.

    :ivar name: The ``.msg`` spelling, so that ``float32`` and ``float64`` stay distinguishable
        after both have become Python ``float``.
    """

    name: PrimitiveName


@dataclass(frozen=True, slots=True, kw_only=True)
class StringType:
    """A string field, with the bound the interface declared for it.

    :ivar wide: Whether this is a ``wstring``.
    :ivar maximum_length: The declared bound, or ``None`` when the string is unbounded.
    """

    wide: bool = False
    maximum_length: int | None = None


@dataclass(frozen=True, slots=True, kw_only=True)
class NestedType:
    """A field whose type is another ROS2 message.

    :ivar type_name: The fully qualified ``package/kind/Name``, resolved from the interface
        class rather than from the truncated ``package/Name`` the type mapping reports.
    """

    type_name: str


ElementType: TypeAlias = "PrimitiveType | StringType | NestedType"
"""What may appear inside an array or a sequence, and on its own as a scalar field."""


@dataclass(frozen=True, slots=True, kw_only=True)
class ArrayType:
    """A fixed-size array: exactly :attr:`length` elements, always.

    Distinct from :class:`SequenceType`; collapsing the two would turn ``double[9]`` into a scalar.
    """

    element: ElementType
    length: int


@dataclass(frozen=True, slots=True, kw_only=True)
class SequenceType:
    """A variable-length sequence, bounded or not.

    :ivar maximum_length: The declared bound, or ``None`` when unbounded.
    """

    element: ElementType
    maximum_length: int | None = None


FieldType: TypeAlias = "ElementType | ArrayType | SequenceType"
"""Every shape a ROS2 message field can have."""


@dataclass(frozen=True, slots=True, kw_only=True)
class FieldDescription:
    """One field of a ROS2 message.

    :ivar name: The ROS2 field name, which is also the dataclass attribute name.
    :ivar field_type: The modelled shape of the field.
    :ivar ros2_text: The interface class's own type string, kept verbatim.
    """

    name: str
    field_type: FieldType
    ros2_text: str


@dataclass(frozen=True, slots=True, kw_only=True)
class MessageDescription:
    """Everything the rest of the package needs to know about one ROS2 message type.

    :ivar type_name: The fully qualified ``package/kind/Name``.
    :ivar fields: The fields, in declaration order. Constants are not fields and are not here.
    """

    type_name: str
    fields: tuple[FieldDescription, ...]


class Message:
    """Base of every message dataclass built by :func:`message_dataclass`.

    Carries no data. It makes "a payload that crossed the seam" a type that can be annotated and
    checked with ``isinstance`` without touching ``rclpy``.

    :cvar ros2: The ROS2 facts the subclass was built from.
    :cvar ros2_nested: For each field whose type is another message, the dataclass that message
        was built as, resolved once at build time.
    """

    __slots__ = ()

    ros2: ClassVar[MessageDescription]
    ros2_nested: ClassVar[Mapping[str, type[Message]]]


def parse_field_type(text: str, *, nested_type_name: str | None = None) -> FieldType:
    """Model one entry of an interface class's field-type mapping.

    A pure function, so the type model needs no ROS2 installation to test.

    :param text: The type string, as the interface class reports it (``double``, ``double[9]``,
        ``string<256>``, ``sequence<uint8>``, ``sequence<double, 3>``, ``geometry_msgs/Point``).
    :param nested_type_name: The fully qualified name to give a nested type. Without it the
        ``package/Name`` form is read as a message type, which is wrong only for the generated
        wrapper types of a service or an action.
    :raises MessageTypeError: When the string is a form this Runtime does not model.
    """
    sequence = _SEQUENCE.match(text)
    if sequence:
        bound = sequence["bound"]
        return SequenceType(
            element=_parse_element(sequence["element"], nested_type_name),
            maximum_length=int(bound) if bound else None,
        )
    array = _ARRAY.match(text)
    if array:
        return ArrayType(
            element=_parse_element(array["element"], nested_type_name),
            length=int(array["length"]),
        )
    return _parse_element(text, nested_type_name)


def _parse_element(text: str, nested_type_name: str | None) -> ElementType:
    """Model the scalar type inside an array, inside a sequence, or standing on its own."""
    primitive = _IDL_PRIMITIVES.get(text)
    if primitive is not None:
        return PrimitiveType(name=primitive)
    string = _STRING.match(text)
    if string:
        bound = string["bound"]
        return StringType(wide=bool(string["wide"]), maximum_length=int(bound) if bound else None)
    nested = _NESTED.match(text)
    if nested:
        return NestedType(type_name=nested_type_name or f"{nested['package']}/msg/{nested['name']}")
    message = f"{text!r} is not a ROS2 field type this Runtime models"
    raise MessageTypeError(message)


def canonical_type_name(type_name: str) -> str:
    """Return ``package/kind/Name``, filling in ``msg`` for the two-part ROS2 shorthand.

    :raises InvalidMessageTypeNameError: When the name has neither shape, or names a kind that is
        not one of ``msg``, ``srv`` or ``action``.
    """
    parts = type_name.split("/")
    if len(parts) == _TWO_PART:
        return f"{parts[0]}/msg/{parts[1]}"
    if len(parts) == _THREE_PART and parts[1] in _KINDS:
        return type_name
    message = f"{type_name!r} is not a ROS2 type name; expected 'package/Name' or 'package/kind/Name'"
    raise InvalidMessageTypeNameError(message)


@dataclass(frozen=True, slots=True, kw_only=True)
class ActionTypes:
    """The three message types one ROS2 action carries, resolved together.

    A declaration names one action and gets the goal, feedback and result surfaces from it.

    :ivar type_name: The fully qualified ``package/action/Name``.
    :ivar goal: What a client's Parameters are built from, and what is sent.
    :ivar feedback: What arrives while the goal runs.
    :ivar result: What the server answers with when the goal ends.
    """

    type_name: str
    goal: type[Message]
    feedback: type[Message]
    result: type[Message]


def action_types(type_name: str) -> ActionTypes:
    """Return the goal, feedback and result message dataclasses of a ROS2 action type.

    Synchronous and free of the Runtime, like :func:`message_dataclass`.

    :param type_name: ``package/action/Name``. The ``msg`` shorthand is not accepted.
    :raises InvalidMessageTypeNameError: When the name is not a three-part action name.
    :raises MissingInterfacePackageError: When the interface package is not installed.
    :raises UnusableMessageTypeError: When the package has no such action.
    """
    canonical = canonical_type_name(type_name)
    if canonical.split("/")[1] != "action":
        message = f"{type_name!r} is not a ROS2 action name; expected 'package/action/Name'"
        raise InvalidMessageTypeNameError(message)
    goal, feedback, result = (message_dataclass(f"{canonical}_{member}") for member in ACTION_MEMBERS)
    return ActionTypes(type_name=canonical, goal=goal, feedback=feedback, result=result)


@dataclass(frozen=True, slots=True, kw_only=True)
class ServiceTypes:
    """The two message types one ROS2 service carries, resolved together.

    The counterpart of :class:`ActionTypes`; a service has no feedback. A custom lifecycle Driver
    gets one from :meth:`RuntimeInterface.describe_service` and passes it to
    :meth:`RuntimeInterface.call_service`.

    :ivar type_name: The fully qualified ``package/srv/Name``.
    :ivar request: What a Method's Parameters are built from, and what is sent.
    :ivar response: What the server answers with.
    """

    type_name: str
    request: type[Message]
    response: type[Message]


def service_types(type_name: str) -> ServiceTypes:
    """Return the request and response message dataclasses of a ROS2 service type.

    Synchronous and free of the Runtime, like :func:`action_types`.

    :param type_name: ``package/srv/Name``. The ``msg`` shorthand is not accepted.
    :raises InvalidMessageTypeNameError: When the name is not a three-part service name.
    :raises MissingInterfacePackageError: When the interface package is not installed.
    :raises UnusableMessageTypeError: When the package has no such service.
    """
    canonical = canonical_type_name(type_name)
    if canonical.split("/")[1] != "srv":
        message = f"{type_name!r} is not a ROS2 service name; expected 'package/srv/Name'"
        raise InvalidMessageTypeNameError(message)
    request, response = (message_dataclass(f"{canonical}_{member}") for member in SERVICE_MEMBERS)
    return ServiceTypes(type_name=canonical, request=request, response=response)


def message_dataclass(type_name: str) -> type[Message]:
    """Return the message dataclass for a ROS2 message type, importing the interface package.

    Synchronous and free of the Runtime: a Binding declares its type while a Component is being
    constructed, before any event loop exists, and a missing interface package must fail there.

    The result is cached, and two Bindings declaring the same type get the same dataclass, so
    ``isinstance`` checks between them hold.

    :param type_name: ``package/kind/Name``, or ``package/Name`` for a message.
    :raises InvalidMessageTypeNameError: When the name has the wrong shape.
    :raises MissingInterfacePackageError: When the interface package is not installed.
    :raises UnusableMessageTypeError: When the package has no such message.
    :raises MessageTypeError: When a field has a type this Runtime does not model.
    """
    # Canonicalised before the cache so `std_msgs/String` and `std_msgs/msg/String` share one dataclass.
    return _build_from_interface(canonical_type_name(type_name))


@cache
def _build_from_interface(canonical: str) -> type[Message]:
    """Build and memoise the dataclass for one canonical ROS2 type name."""
    # Imported here so this module stays importable without ROS2.
    from opensmi.ros2.runtime import _parser

    return build_message_dataclass(_parser.describe(canonical))


def build_message_dataclass(
    description: MessageDescription,
    *,
    resolve: Callable[[str], type[Message]] | None = None,
) -> type[Message]:
    """Build the frozen dataclass a :class:`MessageDescription` describes.

    A pure function of a value, so a test can hand it a description of a type nobody has installed.

    :param resolve: How to turn a nested field's type name into its dataclass. Defaults to
        :func:`message_dataclass`.
    """
    resolver = resolve or message_dataclass
    nested = {
        field.name: resolver(element.type_name)
        for field in description.fields
        if isinstance(element := element_of(field.field_type), NestedType)
    }
    fields = [(field.name, _annotation(field.field_type, nested.get(field.name))) for field in description.fields]
    name = description.type_name.rpartition("/")[2]
    built = make_dataclass(
        name,
        fields,
        bases=(Message,),
        frozen=True,
        slots=True,
        # Keyword-only: a definition that gains a field would silently reorder positional payloads.
        kw_only=True,
        namespace={
            "ros2": description,
            "ros2_nested": nested,
            "__doc__": f"The ROS2 message {description.type_name}, as it crosses the seam.",
        },
    )
    built.__module__ = __name__
    built.__qualname__ = name
    return built


def element_of(field_type: FieldType) -> ElementType:
    """Return the scalar type a field holds, unwrapping an array or a sequence."""
    if isinstance(field_type, (ArrayType, SequenceType)):
        return field_type.element
    return field_type


def repeated_message(message_type: type[Message], field_name: str) -> type[Message] | None:
    """Return the message type one field repeats, or ``None`` when it repeats no message.

    The field must exist, be an array or a sequence, and hold a message. The Keyed Binding and
    the recipe that turns such a field into a standard value both ask this.
    """
    described = next((each for each in message_type.ros2.fields if each.name == field_name), None)
    if described is None or not isinstance(described.field_type, (ArrayType, SequenceType)):
        return None
    return message_type.ros2_nested.get(field_name)


def is_single_value(field_type: FieldType) -> bool:
    """Return whether a field is one value: a primitive or a string, neither nested nor repeated.

    What a key may be read from, and what a pair's two halves have to be.
    """
    return isinstance(field_type, (PrimitiveType, StringType))


def is_byte_container(field_type: FieldType) -> bool:
    """Return whether a field is an array or sequence of bytes, and so renders as ``bytes``.

    Shared by the annotation and the Decoder, which must agree on which fields these are.
    """
    return isinstance(field_type, (ArrayType, SequenceType)) and (
        isinstance(field_type.element, PrimitiveType) and field_type.element.name in _BYTE_LIKE
    )


def _annotation(field_type: FieldType, nested: type[Message] | None) -> object:
    """Return the Python type a modelled field is annotated and decoded as."""
    if is_byte_container(field_type):
        return bytes
    match field_type:
        case PrimitiveType(name=name):
            return _PYTHON_TYPES[name]
        case StringType():
            return str
        case NestedType():
            return nested
        case ArrayType(element=element) | SequenceType(element=element):
            return tuple[_annotation(element, nested), ...]
