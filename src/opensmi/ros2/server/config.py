# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Configuration: a generic dataclass loader, and the ROS2 settings an application injects.

ROS2 settings are their own TOML file and dataclass, not a ``[ros2]`` table in the opensmi server
configuration, because that tree is closed and refuses keys it does not know. The application loads
:class:`ROS2Config` itself and hands it to the Runtime and the Machine mixin.

A setting lives here and not on a declaration when it is true of the Machine (one ROS2 Domain ID)
and not of one object on it. :class:`TransformConfig` is the case: the transform topics and the TF
poll period look like Binding keywords, but one Domain ID has one frame network and one poll. Such a
value declared per object is refused and named here instead.

The two settings dataclasses raise :class:`~opensmi.ros2.server.errors.DeclarationError` on
contradictory values. The loader raises ``ValueError`` and ``TypeError`` only.

The loader is one function over any dataclass and also reads the Binding TOML of
:mod:`opensmi.ros2.server.bindings`. It behaves like ``opensmi.core.config.load_dataclass``, including
``ValueError`` on an unknown key, with two differences:

* It converts a TOML array into a ``tuple`` when the field asks for one, since the declarations are
  frozen and a list would make them unhashable.
* A TOML file can name no Python class. A field whose type is not a string, number, boolean, time or a
  container of those (enums and nested dataclasses are built from them) is refused a value, so an
  editable file cannot run code. Custom implementation classes and callables are Python-side keywords
  only. The machine file (:mod:`opensmi.ros2.server.loader`) reaches a few by importing a named path
  and handing the object in through ``supplied``.
"""

from __future__ import annotations

import collections.abc as abc
import datetime as dt
import inspect
import sys
import tomllib
from dataclasses import dataclass, fields, is_dataclass
from enum import Enum, IntEnum
from pathlib import Path
from types import UnionType
from typing import TYPE_CHECKING, Any, Final, TypeVar, Union, get_args, get_origin, get_type_hints

from opensmi.ros2.runtime.tf import TF_DEPTH, TF_STATIC_TOPIC, TF_TOPIC
from opensmi.ros2.server.access import LISTING_ACCESS_LEVEL, MUTATOR_ACCESS_LEVEL, SKILL_ACCESS_LEVEL
from opensmi.ros2.server.errors import DeclarationError

if TYPE_CHECKING:
    from collections.abc import Mapping

__all__ = [
    "TF_INTERVAL",
    "TF_QUIET_AFTER",
    "FieldError",
    "ROS2Config",
    "TransformConfig",
    "declarable_keys",
    "from_dict",
    "load_toml",
    "python_keys",
]

_T = TypeVar("_T")

_TOML_SCALARS: Final = (str, int, float, bool, dt.datetime, dt.date, dt.time)
"""Every Python type a TOML document can produce a value of."""


TF_INTERVAL: Final = 1.0
"""How many seconds pass between polls of the frame network, unless a deployment says otherwise."""

TF_QUIET_AFTER: Final = 5.0
"""How many seconds a frame may go unpublished before it is quiet, by default.

This has to clear the slowest publisher on the robot: a diagnostic frame republished every two
seconds is healthy. A deployment that knows its publishers can tighten it.
"""


class FieldError(ValueError, TypeError):
    """A TOML value does not fit the field it was written under.

    Both a :class:`ValueError` and a :class:`TypeError`, since the conversion raises either. It adds
    which field, so a caller reading a whole file can report the place and carry on.

    :ivar field: The field name, dotted when the value sat inside a nested table.
    :ivar reason: What was wrong with the value, without the field name.
    """

    def __init__(self, field: str, reason: str) -> None:
        """Remember the field and the reason, and say both."""
        super().__init__(f"{field}: {reason}")
        self.field = field
        self.reason = reason


@dataclass(frozen=True, slots=True, kw_only=True)
class TransformConfig:
    """Where this Machine reads TF from, and how often the frame network is sampled.

    One Runtime is one Domain ID and one frame network, and one Machine runs one poll over the
    ownership partition of every declared head. So these values are declared once here, not per
    object. A :class:`~opensmi.ros2.server.bindings.TFBinding` keeps what is true of its object: head
    frame, exclusions, alternatives. Reached as :attr:`ROS2Config.transforms`; in the settings file::

        [transforms]
        path = "/robot/tf"
        static_path = "/robot/tf_static"
        interval = 0.5
        quiet_after = 2.0

    :ivar path: The transform topic, ``/tf`` unless remapped.
    :ivar static_path: The static transform topic, always read as well. When this Machine publishes a
        frame of its own there, it uses :data:`~opensmi.ros2.runtime.tf.TF_STATIC_QOS`. What it
        requests on both topics is discovered from the publishers.
    :ivar depth: How many messages each of the two subscriptions buffers.
    :ivar quiet_after: Seconds a frame may go unpublished before its Location reads
        ``BadNoCommunication`` (see :func:`~opensmi.ros2.server.status.transform_status`). A static
        frame is exempt, so this only has to clear the slowest moving publisher.
    :ivar interval: Seconds between polls of the frame network. Inward is a poll, not a write per
        message, because ``/tf`` carries one message per publisher per control cycle.
    """

    path: str = TF_TOPIC
    static_path: str = TF_STATIC_TOPIC
    depth: int = TF_DEPTH
    interval: float = TF_INTERVAL
    quiet_after: float = TF_QUIET_AFTER

    def __post_init__(self) -> None:
        """Refuse settings that contradict themselves.

        :raises DeclarationError: When either topic is unnamed, both name one topic, the
            subscriptions buffer nothing, or the poll period or the staleness window is not positive.
        """
        for name, topic in (("path", self.path), ("static_path", self.static_path)):
            if topic:
                continue
            message = (
                f"the transform settings give {name}={topic!r}, which names no ROS2 topic; both "
                f"transform topics are read and always, because a system whose frames are all "
                f"static publishes every one of them on the static one and none on the other"
            )
            raise DeclarationError(message)
        if self.path == self.static_path:
            message = (
                f"the transform settings read both kinds of transform from {self.path!r}; they "
                f"are two topics because a message on the static one marks its frames constant, "
                f"so reading one topic as both would say a frame that moves never will"
            )
            raise DeclarationError(message)
        if self.depth < 1:
            message = (
                f"the transform settings give depth={self.depth!r}; it is how many messages each "
                f"transform subscription buffers, so it holds at least one message and a "
                f"subscription that buffered none would be one that reads nothing"
            )
            raise DeclarationError(message)
        if self.interval <= 0:
            message = (
                f"the transform settings give interval={self.interval!r}; it is how many seconds "
                f"pass between polls of the frame network, so it is a positive number and there "
                f"is no way to spell a poll that has already happened"
            )
            raise DeclarationError(message)
        if self.quiet_after <= 0:
            message = (
                f"the transform settings give quiet_after={self.quiet_after!r}; it is how long a "
                f"frame may go unpublished before its Location reads BadNoCommunication, so it is "
                f"a positive number and a window of zero would call every frame stale the instant "
                f"it arrived"
            )
            raise DeclarationError(message)

    @property
    def topics(self) -> tuple[str, str]:
        """Return both transform topics, which is what a reader of these settings subscribes to."""
        return (self.path, self.static_path)


@dataclass(frozen=True, slots=True, kw_only=True)
class ROS2Config:
    """The ROS2 settings of one Machine, which is the unit of one Domain ID.

    The first four fields are the Runtime's constructor keywords, so a Runtime can be built from a
    loaded config field by field. The Machine mixin takes the whole object.

    :ivar domain_id: The ROS2 Domain ID this Machine's Runtime is confined to.
    :ivar node_name: The name of the single ``rclpy`` Node the Runtime owns.
    :ivar namespace: The ROS2 namespace for that node, or ``None`` for the root.
    :ivar stop_timeout: How long, in seconds, stopping waits for the executor thread.
    :ivar startup_timeout: Seconds a required Binding has to become satisfied before its StartupSkill
        gives up on it. A declaration can override it with its own ``startup_timeout``; a Binding that
        is not required has no deadline. A Machine's StartupSkill also allows each Component this much
        slack beyond that Component's own deadlines.
    :ivar transforms: Where the TF Bindings read the frame network from and how often it is polled;
        the ``[transforms]`` table in the settings file. See :class:`TransformConfig`.
    :ivar listing_access_level: Access level the introspection Component's read-only listings
        require. They answer without the Lock whatever this is. Default
        :data:`~opensmi.ros2.server.access.LISTING_ACCESS_LEVEL`.
    :ivar skill_access_level: Access level a Skill requires, on top of the Lock. Default
        :data:`~opensmi.ros2.server.access.SKILL_ACCESS_LEVEL`.
    :ivar introspection_access_level: Access level the introspection mutators (``BindTopic``,
        ``CallService``) require, on top of the Lock. Default
        :data:`~opensmi.ros2.server.access.MUTATOR_ACCESS_LEVEL`. All three defaults follow opensmi's
        default user table; a site with a different table sets its own.
    :ivar debug_goal_outcome_booleans: Whether an action Skill also exposes ``succeeded``,
        ``aborted`` and ``canceled`` booleans in its FinalResultData. A debugging aid, off by default:
        the goal outcome drives the Skill's state machine, and the booleans beside ``CurrentState``
        can disagree with it.
    """

    domain_id: int = 0
    node_name: str = "opensmi_ros2"
    namespace: str | None = None
    stop_timeout: float = 5.0
    startup_timeout: float = 30.0
    transforms: TransformConfig = TransformConfig()
    listing_access_level: int = LISTING_ACCESS_LEVEL
    skill_access_level: int = SKILL_ACCESS_LEVEL
    introspection_access_level: int = MUTATOR_ACCESS_LEVEL
    debug_goal_outcome_booleans: bool = False

    @classmethod
    def from_toml(cls, path: str | Path) -> ROS2Config:
        """Read these settings from a TOML file; a key the file omits keeps its default.

        :raises ValueError: When the file carries a key this class does not declare.
        :raises DeclarationError: When the access levels contradict each other.
        """
        return from_dict(cls, load_toml(path))

    def __post_init__(self) -> None:
        """Refuse an introspection level below the Skill level.

        :raises DeclarationError: When ``introspection_access_level`` is below ``skill_access_level``.
            A mutator changes a live robot, so it asks at least as much as starting a Skill.
        """
        if self.introspection_access_level >= self.skill_access_level:
            return
        message = (
            f"the ROS2 settings give introspection_access_level="
            f"{self.introspection_access_level!r}; binding a topic and calling a service ad hoc "
            f"change what a live robot does, so they ask at least the level a Skill asks, which "
            f"is skill_access_level={self.skill_access_level!r}"
        )
        raise DeclarationError(message)


def load_toml(path: str | Path) -> dict[str, Any]:
    """Load a TOML file into a dictionary."""
    with Path(path).open("rb") as source:
        return tomllib.load(source)


def from_dict(data_cls: type[_T], data: Mapping[str, Any], *, supplied: Mapping[str, Any] | None = None) -> _T:
    """Build a dataclass instance from a TOML table.

    :param supplied: Values for fields a TOML table cannot produce a value of (a class or a
        callable), which the caller has obtained some other way. They are passed to the
        constructor as they are. A name in both ``data`` and here is the caller's mistake.

    :raises TypeError: When the target is not a dataclass, or a table was expected and something
        else arrived.
    :raises FieldError: When a value does not fit its field: the wrong TOML type, an enum member
        that does not exist, a nested table that is not one. A :class:`ValueError` and a
        :class:`TypeError`, naming the field.
    :raises ValueError: When the table carries a key the dataclass does not declare, or a key whose
        field a TOML file cannot produce a value for (a string would otherwise sit where a callable
        belongs).
    """
    if not is_dataclass(data_cls):
        message = f"{data_cls.__name__} is not a dataclass"
        raise TypeError(message)

    hints = _hints(data_cls)
    unknown = set(data) - {field.name for field in fields(data_cls)}
    if unknown:
        listed = ", ".join(sorted(unknown))
        message = f"unknown configuration option(s) for {data_cls.__name__}: {listed}"
        raise ValueError(message)

    for name in data:
        _require_expressible(data_cls, name, hints[name])

    # An omitted key is not passed as None, so the dataclass default decides.
    converted: dict[str, Any] = {}
    for name, value in data.items():
        try:
            converted[name] = _convert(value, hints[name])
        except FieldError as error:
            nested = f"{name}.{error.field}"
            raise FieldError(nested, error.reason) from error
        except (ValueError, TypeError) as error:
            raise FieldError(name, str(error)) from error
    return data_cls(**converted, **(supplied or {}))


class _Unresolved:
    """The annotation of a field that cannot be evaluated at run time, so no TOML value fits it."""


def _hints(data_cls: type) -> dict[str, Any]:
    """Return the type hints of a dataclass, field by field where they cannot be had at once.

    A field annotated with a type-checker-only alias maps to :class:`_Unresolved`, a Python-side keyword.
    """
    try:
        return get_type_hints(data_cls)
    except (NameError, TypeError):
        pass
    hints: dict[str, Any] = {}
    for base in reversed(data_cls.__mro__):
        module = sys.modules.get(base.__module__)
        scope = dict(vars(module)) if module else {}
        for name, annotation in inspect.get_annotations(base).items():
            try:
                hints[name] = eval(annotation, scope) if isinstance(annotation, str) else annotation
            except (NameError, TypeError):
                hints[name] = _Unresolved
    return hints


def declarable_keys(data_cls: type) -> tuple[str, ...]:
    """Return the fields of a dataclass that a TOML table can set, in declaration order.

    The complement stays Python-side: a callable or a class has no TOML spelling.
    """
    hints = _hints(data_cls)
    return tuple(field.name for field in fields(data_cls) if _expressible(hints[field.name]))


def python_keys(data_cls: type) -> tuple[str, ...]:
    """Return the fields of a dataclass that a TOML table cannot set: the complement of :func:`declarable_keys`.

    A caller that lets a file name one by import path passes the imported object to :func:`from_dict`
    as ``supplied``.
    """
    hints = _hints(data_cls)
    return tuple(field.name for field in fields(data_cls) if not _expressible(hints[field.name]))


def _require_expressible(data_cls: type, name: str, target: Any) -> None:
    """Refuse a key whose field is not something a TOML document can produce a value for.

    :raises ValueError: When the field's type is not a TOML scalar, an enum, a dataclass, or a
        container of those (a callable or class would arrive as a string).
    """
    if _expressible(target):
        return
    message = (
        f"{data_cls.__name__}.{name} is a {target!r}, which a TOML file cannot produce a value "
        f"of; a file declares names, paths, patterns and numbers, and anything that has to be a "
        f"Python object is a Python-side keyword"
    )
    raise ValueError(message)


def _file_members(union: Any) -> list[Any]:
    """Return the members of a union a file can write: not ``None``, and not ``type`` beside a name.

    A class alternative (``str | type``) is the Python spelling of what the file writes as a name.
    """
    members = [each for each in get_args(union) if each is not type(None)]
    named = [each for each in members if each is not type]
    return named or members


def _expressible(target: Any) -> bool:
    """Return whether a TOML document can produce a value of one field's declared type.

    ``object`` takes any TOML value as it was read, for a field that checks its own shape later.
    """
    if target is object:
        return True
    origin = get_origin(target)
    if origin in (Union, UnionType):
        return all(_expressible(each) for each in _file_members(target))
    if origin in (list, tuple, dict, set, frozenset, abc.Mapping, abc.Collection, abc.Sequence):
        return all(_expressible(each) for each in get_args(target) if each is not Ellipsis)
    if not isinstance(target, type):
        return False
    return is_dataclass(target) or issubclass(target, (*_TOML_SCALARS, Enum))


def _convert(value: Any, target: Any) -> Any:
    """Convert one TOML value into the type its field declares."""
    # TOML has no null, so this only ever fires for a caller building the table by hand.
    if value is None:
        return None

    origin = get_origin(target)

    if origin in (Union, UnionType):
        rest = _file_members(target)
        if len(rest) == 1:
            return _convert(value, rest[0])

    if isinstance(target, type) and issubclass(target, Enum):
        return _as_enum(value, target)

    if isinstance(target, type) and is_dataclass(target):
        if not isinstance(value, dict):
            message = f"expected a TOML table for {target.__name__}, got {type(value).__name__}"
            raise TypeError(message)
        return from_dict(target, value)

    if origin is list:
        (item,) = get_args(target)
        return [_convert(each, item) for each in value]

    if origin is tuple:
        return _as_tuple(value, target)

    if origin in (abc.Collection, abc.Sequence, dict, abc.Mapping):
        return _as_container(value, origin, get_args(target))

    return _require_scalar(value, target)


def _as_container(value: Any, origin: Any, arguments: tuple[Any, ...]) -> Any:
    """Return a TOML array or table as the read-only container a frozen declaration annotates.

    An array becomes a tuple and a table a dict, with its keys converted to the key type.
    """
    if origin in (abc.Collection, abc.Sequence):
        if not isinstance(value, list):
            message = f"expected an array, got {type(value).__name__} {value!r}"
            raise TypeError(message)
        return tuple(_convert(each, arguments[0]) for each in value)
    if not isinstance(value, dict):
        message = f"expected a table, got {type(value).__name__} {value!r}"
        raise TypeError(message)
    key_type, value_type = arguments
    return {_convert_key(key, key_type): _convert(each, value_type) for key, each in value.items()}


def _convert_key(key: Any, target: Any) -> Any:
    """Convert a table key, which TOML always spells as a string, into the type the field wants."""
    if target is int and isinstance(key, str):
        try:
            return int(key)
        except ValueError as error:
            message = f"the key {key!r} is not an integer"
            raise ValueError(message) from error
    return _convert(key, target)


def _require_scalar(value: Any, target: Any) -> Any:
    """Return a value after checking it is the TOML scalar its field declares.

    Only bool, int, float and str are checked. An int may stand where a float is wanted, and a bool is
    never a number.
    """
    if target is bool:
        accepted = isinstance(value, bool)
    elif target is int:
        accepted = isinstance(value, int) and not isinstance(value, bool)
    elif target is float:
        accepted = isinstance(value, (int, float)) and not isinstance(value, bool)
    elif target is str:
        accepted = isinstance(value, str)
    else:
        return value
    if not accepted:
        message = f"expected {target.__name__}, got {type(value).__name__} {value!r}"
        raise TypeError(message)
    return value


def _as_enum(value: Any, target: type[Enum]) -> Enum:
    """Return the enum member a TOML value names, or say which ones exist."""
    try:
        return target(value)
    except ValueError as error:
        if isinstance(value, str) and issubclass(target, IntEnum) and value in target.__members__:
            return target[value]  # an int enum may be spelled by member name
        valid = ", ".join(repr(member.value) for member in target)
        message = f"invalid value {value!r} for {target.__name__}; expected one of: {valid}"
        raise ValueError(message) from error


def _as_tuple(value: Any, target: Any) -> tuple[Any, ...]:
    """Return a TOML array as the tuple a frozen declaration holds.

    ``tuple[str, ...]`` converts element by element, a fixed-length tuple position by position.
    """
    arguments = get_args(target)
    variadic = 2  # a variadic tuple is spelled with exactly two arguments: an element type and an ellipsis
    if len(arguments) == variadic and arguments[1] is Ellipsis:
        return tuple(_convert(each, arguments[0]) for each in value)
    return tuple(_convert(each, argument) for each, argument in zip(value, arguments, strict=True))
