# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Start a Machine from one TOML file, with no code of the deployer's.

.. code-block:: toml

    [machine]
    name = "Robot"

    [machine.identification]
    serial_number = "example-0001"
    manufacturer = "Example Manufacturer"
    product_instance_uri = "urn:example:opensmi-ros2:robot"

    [ros2]
    domain_id = 42

    [[component]]
    name = "Platform"

    [[component.monitoring]]
    name = "Odom"
    path = "/odom"
    message_type = "nav_msgs/msg/Odometry"

    [[component]]
    name = "Arm"
    parent = "Platform"

The function :func:`load_machine` reads the file and returns a Machine ready for opensmi's
``Server.add_machine``.

The whole file is checked before anything is built. Every mistake is collected with the path to its
place (``component[1].monitoring[2].message_type``) and raised together as one
:class:`~opensmi.ros2.server.errors.FileDeclarationError`. The checks reuse the Binding declarations' own
validation and the loader of :mod:`opensmi.ros2.server.config`, so a new rule on a Binding applies to the
file at once. The Runtime comes from ``runtime_factory`` and is made only once the file has passed.

An optional ``[server] config = "server.toml"`` is resolved against the file's directory, must exist,
and is carried as ``ConfiguredMachine.server_config`` for the caller that builds the ``Server``, as
:mod:`opensmi.ros2.cli` does. Without it, opensmi's defaults apply.

A Component names its parent with ``parent`` (default: the Machine), and a parent comes before its
children in the file. A Binding file can be included with ``bindings_toml``.

``[[component.projection]]`` and ``[[machine.projection]]`` hold one Projection entry each: ``recipe``
(one of :data:`RECIPES`), ``type``, and the recipe's own parameters. The keys of a recipe are derived
from its dataclass, minus fields with no TOML spelling (callables) and minus the ``functions`` recipe,
which is only callables. Entries register through
:meth:`~opensmi.ros2.server.projection.ProjectionRegistry.extended` in the same layers as in Python
(Component, then Machine, then the shipped catalogue), so a second entry for one type is an error
unless it says ``override = true``. Every recipe takes ``override`` and ``type_hash``. ``scalar`` also
takes ``variant_type`` (a name such as ``Float``, or its number) and ``datatype`` (a namespace 0
DataType number), ``blob`` takes ``field`` and ``datatype``, ``enum`` takes ``fields`` (a table per
field name of number to label) and ``drop`` takes ``fields`` (a list of field paths):

.. code-block:: toml

    [[component.projection]]
    recipe = "drop"
    type = "nav_msgs/msg/Odometry"
    fields = ["pose/covariance", "twist/covariance"]

    [[component.projection]]
    recipe = "enum"
    type = "sensor_msgs/msg/BatteryState"

    [component.projection.fields.power_supply_status]
    0 = "Unknown"
    1 = "Charging"

    [[component.projection]]
    recipe = "scalar"
    type = "std_msgs/msg/Float64"
    variant_type = "Float"

A file may name the deployer's own classes by import path (``package.module.Name`` or
``package.module:Name``, the colon reaching a nested name): ``machine_class`` and ``component_class``,
``startup_skill_type``, ``driver_type`` inside a lifecycle table, and the callables of a projection
table (``recipe = "functions"``). Without a key the stock class applies. Importing runs that module's
code, so a machine file is trusted configuration. Nothing is imported until the whole file has passed
the structural checks. Import errors and classes of the wrong kind are reported at their key.

.. code-block:: toml

    [machine]
    name = "Robot"
    machine_class = "my_robot.cell:Cell"
    startup_skill_type = "my_robot.cell:CellStartup"

    [[component]]
    name = "Platform"
    component_class = "my_robot.cell:Platform"

    [[component.lifecycle]]
    name = "Navigation"
    path = "/lifecycle_manager"
    driver_type = "my_robot.cell:NavigationDriver"

    [[component.projection]]
    recipe = "functions"
    type = "my_msgs/msg/Pose"
    shaped_by = "my_robot.curation:pose_shape"
    decoded_by = "my_robot.curation:pose_decode"
    encode = "my_robot.curation:pose_encode"

``machine_class`` must subclass :class:`ConfiguredMachine` and ``component_class``
:class:`ConfiguredComponent`; a class with its own ``__init__`` passes unknown keywords on to the base.
``startup_skill_type`` must subclass the stock StartupSkill (the lifecycle one on a Component with
lifecycle Bindings) and ``driver_type`` ``LifecycleDriver``. The ``functions`` recipe takes one key per
callable (``shaped_by``, ``decoded_by`` and the optional ``encode``); the ``encode`` of a ``blob`` is
named the same way.

Anything beyond Bindings, Projections, identification and these classes stays Python, such as
the Skill, Method or goal classes and the message functions of a Binding, and logic between declarations
(adding Components in a loop), which belongs in a ``machine_class``.

From Python:

.. code-block:: python

    async with Server() as server:
        await server.add_machine(load_machine("machine.toml"))
        await server.start(blocking=True)
"""

from __future__ import annotations

import importlib
import re
import tomllib
from dataclasses import MISSING, dataclass, field, fields, replace
from pathlib import Path
from typing import TYPE_CHECKING, Any, Final, NamedTuple, TypeVar

from asyncua import ua
from opensmi.server import BaseComponent, BaseMachine

from opensmi.ros2.server.bindings import TOML_KINDS, Binding, LifecycleBinding, load_bindings, resolve_bindings
from opensmi.ros2.server.component import ROS2ComponentMixin
from opensmi.ros2.server.config import (
    FieldError,
    ROS2Config,
    TransformConfig,
    declarable_keys,
    from_dict,
    python_keys,
)
from opensmi.ros2.server.drivers import LifecycleDriver
from opensmi.ros2.server.errors import DeclarationError, DuplicateProjectionError, FileDeclarationError
from opensmi.ros2.server.introspection import INTROSPECTION_NAME
from opensmi.ros2.server.machine import ROS2MachineMixin
from opensmi.ros2.server.projection import Projection, ProjectionRegistry
from opensmi.ros2.server.projections import DEFAULT_REGISTRY
from opensmi.ros2.server.recipes import Blob, Default, Drop, Enum, Functions, Scalar
from opensmi.ros2.server.required import ROS2StartupSkill
from opensmi.ros2.server.startup import ROS2LifecycleStartupSkill

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable, Mapping, Sequence

    from opensmi.ros2.runtime.interface import RuntimeInterface

_T = TypeVar("_T")

__all__ = ["RECIPES", "ConfiguredComponent", "ConfiguredMachine", "Identification", "load_machine"]

_TOP_KEYS: Final = frozenset({"machine", "ros2", "server", "component"})
_MACHINE_KEYS: Final = frozenset({"name", "identification", "tf", "projection", "machine_class", "startup_skill_type"})
_COMPONENT_KEYS: Final = frozenset(
    {
        "name",
        "parent",
        "identification",
        "bindings_toml",
        "projection",
        "component_class",
        "startup_skill_type",
        *TOML_KINDS,
    }
)
_MACHINE_IDENTIFICATION: Final = ("serial_number", "manufacturer", "product_instance_uri")
_COMPONENT_IDENTIFICATION: Final = ("serial_number", "manufacturer")
"""A Component's identification block has no product URI; only a Machine's does."""
RECIPES: Final[Mapping[str, type[Projection]]] = {
    "default": Default,
    "scalar": Scalar,
    "enum": Enum,
    "blob": Blob,
    "drop": Drop,
    "functions": Functions,
}
"""The recipes a projection table can name. A test fails when a recipe of
:mod:`opensmi.ros2.server.recipes` is missing from it."""
_TYPE_KEY: Final = "type"
"""The key a table spells a recipe's ``type_name`` with."""
_PATH: Final = re.compile(r"[^\W\d]\w*(\.[^\W\d]\w*)*(:[^\W\d]\w*(\.[^\W\d]\w*)*)?")
"""An import path: dotted names, with an optional ``:`` before the attribute (which may be dotted)."""
_LOCALE: Final = "en-US"
"""The locale of the manufacturer's text; a file carries one string."""


@dataclass(frozen=True, slots=True, kw_only=True)
class Identification:
    """The identification block of a Machine or Component, as a file gives it.

    :ivar serial_number: The serial number, or ``None`` to leave it as opensmi has it.
    :ivar manufacturer: The manufacturer's name, written with the locale ``en-US``.
    :ivar product_instance_uri: The product instance URI.
    """

    serial_number: str | None = None
    manufacturer: str | None = None
    product_instance_uri: str | None = None


async def _write_identification(item: Any, given: Identification) -> None:
    """Write what the file gave into the item's identification block."""
    block = item.identification
    if given.serial_number is not None:
        await block.SerialNumber.write(given.serial_number)
    if given.manufacturer is not None:
        await block.Manufacturer.write(ua.LocalizedText(given.manufacturer, _LOCALE))
    if given.product_instance_uri is not None:
        await block.ProductInstanceUri.write(given.product_instance_uri)


class ConfiguredComponent(ROS2ComponentMixin, BaseComponent):
    """The stock Component a machine file declares: Bindings, identification and children.

    Built by :func:`load_machine`, not by hand.
    """

    def __init__(
        self,
        *,
        bindings: Sequence[Binding] = (),
        identification: Identification | None = None,
        children: Sequence[ConfiguredComponent] = (),
        projections: Sequence[Projection] = (),
        startup_skill_type: type[ROS2StartupSkill] | None = None,
        **kwargs: Any,
    ) -> None:
        """Take the declarations this Component serves and the Components it contains.

        A subclass named by ``component_class`` that defines ``__init__`` passes the keywords it does
        not know on to this one.

        :param bindings: Every Binding, inline and included, already resolved and unique by name.
        :param projections: The Projections the file curates this Component's types with, ahead of
            the Machine's.
        :param identification: What to write into the identification block.
        :param children: The Components registered under this one, in the order they start.
        :param startup_skill_type: The StartupSkill class the file names, or ``None`` to leave the
            class's own ``startup_skill_type`` as it is.
        """
        super().__init__(**kwargs)
        if startup_skill_type is not None:
            self.startup_skill_type = startup_skill_type
        self.bindings = tuple(bindings)
        self.projections = tuple(projections)
        self._given = identification or Identification()
        self._children = tuple(children)

    async def _write_identification(self) -> None:
        await _write_identification(self, self._given)

    async def _init(self) -> None:
        """Let the mixin build this Component, then register its children in file order."""
        await super()._init()
        for child in self._children:
            await self.add_component(child)


class ConfiguredMachine(ROS2MachineMixin, BaseMachine):
    """The stock Machine a machine file declares: identification, TF Bindings and Components."""

    def __init__(
        self,
        *,
        identification: Identification,
        components: Sequence[ConfiguredComponent] = (),
        bindings: Sequence[Binding] = (),
        projections: Sequence[Projection] = (),
        server_config: Path | None = None,
        startup_skill_type: type[ROS2StartupSkill] | None = None,
        **kwargs: Any,
    ) -> None:
        """Take the identification, the top-level Components and the Machine's own TF Bindings.

        A subclass named by ``machine_class`` that defines ``__init__`` passes the keywords it does
        not know on to this one.

        :param startup_skill_type: The StartupSkill class the file names, or ``None`` to leave the
            class's own ``startup_skill_type`` as it is.

        :param server_config: The opensmi server configuration the file points at, as an absolute
            path, or ``None`` when it names none. The Machine only carries it; the caller that
            builds the ``Server`` decides whether to use it.
        """
        super().__init__(**kwargs)
        if startup_skill_type is not None:
            self.startup_skill_type = startup_skill_type
        self.server_config = server_config
        self.bindings = tuple(bindings)
        self.projections = tuple(projections)
        self._given = identification
        self._top_level = tuple(components)

    async def _write_identification(self) -> None:
        await _write_identification(self, self._given)

    async def _init(self) -> None:
        """Let the mixin build the Machine, then register its Components in file order."""
        await super()._init()
        for component in self._top_level:
            await self.add_component(component)


@dataclass(slots=True)
class _Spec:
    """One Component as the file declares it, before anything is built."""

    name: str
    parent: str | None
    identification: Identification
    bindings: tuple[Binding, ...]
    projections: tuple[Projection, ...]
    component_class: _Ref | None = None
    startup_skill_type: _Ref | None = None
    children: list[_Spec] = field(default_factory=list)


@dataclass(slots=True)
class _Model:
    """A file that has passed every check."""

    name: str
    identification: Identification
    bindings: tuple[Binding, ...]
    projections: tuple[Projection, ...]
    ros2: ROS2Config
    server_config: Path | None
    roots: list[_Spec]
    machine_class: _Ref | None
    startup_skill_type: _Ref | None
    swaps: dict[int, Any]
    """What replaces each stand-in declaration (by ``id``) once its classes and callables are imported."""


@dataclass(slots=True)
class _Ref:
    """A class or callable the file names by import path, resolved only after the structure passed."""

    path: str
    """Where the file names it, such as ``component[1].component_class``."""
    text: str
    complaint: Callable[[Any], str | None]
    """Returns what is wrong with the imported object, or ``None`` when it will do."""
    value: Any = None


@dataclass(slots=True)
class _Swap:
    """A frozen declaration built with stand-ins, rebuilt with the imported objects.

    The stand-in takes part in the structural checks without needing the import.
    """

    stand_in: Any
    refs: dict[str, _Ref]
    path: str


class _Problems:
    """The mistakes found so far, in the order the file reaches them, and what waits for an import."""

    def __init__(self) -> None:
        self.found: list[tuple[str, str]] = []
        self.refs: list[_Ref] = []
        self.swaps: list[_Swap] = []

    def add(self, path: str, message: str) -> None:
        self.found.append((path, message))


def _join(path: str, key: str | int) -> str:
    if isinstance(key, int):
        return f"{path}[{key}]"
    return f"{path}.{key}" if path else key


def _table(value: Any, path: str, problems: _Problems) -> dict[str, Any] | None:
    """Return the value if it is a table, else report it at the path."""
    if isinstance(value, dict):
        return value
    problems.add(path, f"expected a table, got {type(value).__name__}")
    return None


def _check_keys(table: Mapping[str, Any], allowed: Iterable[str], path: str, problems: _Problems) -> None:
    """Report each key the table has that the model does not know, at the key."""
    known = sorted(allowed)
    for key in table:
        if key not in known:
            problems.add(_join(path, key), f"unknown key; this table takes: {', '.join(known)}")


def _string(table: Mapping[str, Any], key: str, path: str, problems: _Problems) -> str | None:
    """Return a non-empty string value, or report a wrong one at its path (an absent key is silent)."""
    value = table.get(key)
    if isinstance(value, str) and value:
        return value
    if key in table:
        problems.add(_join(path, key), f"expected a non-empty string, got {value!r}")
    return None


def _reference(
    table: Mapping[str, Any], key: str, path: str, problems: _Problems, complaint: Callable[[Any], str | None]
) -> _Ref | None:
    """Read a key that names a class or callable by import path, checking only its shape."""
    text = _string(table, key, path, problems)
    if text is None:
        return None
    here = _join(path, key)
    if _PATH.fullmatch(text) is None or (":" not in text and "." not in text):
        problems.add(here, f"{text!r} is not an import path; write package.module.Name or package.module:Name")
        return None
    ref = _Ref(here, text, complaint)
    problems.refs.append(ref)
    return ref


def _subclass_of(base: type, why: str) -> Callable[[Any], str | None]:
    """Return the check that an imported object is a class derived from ``base``."""

    def complaint(value: Any) -> str | None:
        if isinstance(value, type) and issubclass(value, base):
            return None
        shown = getattr(value, "__name__", repr(value))
        return f"{shown} is not a subclass of {base.__name__}; {why}"

    return complaint


def _callable(value: Any) -> str | None:
    """Return what is wrong with an imported object that has to be called, if anything."""
    return None if callable(value) else f"{value!r} is not callable"


def _import(text: str) -> Any:
    """Import the object an import path names.

    :raises LookupError: With what went wrong, when the module does not import or lacks the attribute.
    """
    module_name, _, attribute = text.partition(":")
    if not attribute:
        module_name, _, attribute = text.rpartition(".")
    try:
        found = importlib.import_module(module_name)
    except Exception as error:  # the file names arbitrary code; whatever it raises is a mistake of the file
        message = f"cannot import module {module_name!r}: {type(error).__name__}: {error}"
        raise LookupError(message) from error
    for part in attribute.split("."):
        try:
            found = getattr(found, part)
        except AttributeError as error:
            message = f"module {module_name!r} has no attribute {attribute!r}"
            raise LookupError(message) from error
    return found


def _resolve(problems: _Problems) -> dict[int, Any]:
    """Import everything the file names, report what fails, and rebuild the stand-in declarations.

    :return: The real declaration for each stand-in, by the stand-in's ``id``.
    """
    for ref in problems.refs:
        try:
            value = _import(ref.text)
        except LookupError as error:
            problems.add(ref.path, str(error))
            continue
        complaint = ref.complaint(value)
        if complaint is not None:
            problems.add(ref.path, complaint)
            continue
        ref.value = value
    real: dict[int, Any] = {}
    for swap in problems.swaps:
        if any(each.value is None for each in swap.refs.values()):
            continue
        try:
            real[id(swap.stand_in)] = replace(swap.stand_in, **{key: each.value for key, each in swap.refs.items()})
        except (DeclarationError, ValueError, TypeError) as error:
            problems.add(swap.path, str(error))
    return real


def _identification(raw: Any, path: str, problems: _Problems, *, machine: bool) -> Identification:
    """Read an identification table. opensmi refuses to start an item that leaves an entry empty."""
    keys = _MACHINE_IDENTIFICATION if machine else _COMPONENT_IDENTIFICATION
    table = _table(raw if raw is not None else {}, path, problems)
    if table is None:
        return Identification()
    _check_keys(table, keys, path, problems)
    values = {key: _string(table, key, path, problems) for key in keys}
    for key in keys:
        if key not in table:
            problems.add(_join(path, key), "missing; opensmi needs it to start")
    return Identification(**values)


def _declare(kind: type[Any], entry: Any, path: str, problems: _Problems) -> Binding | None:
    """Build one Binding from its table through the declaration's own checks."""
    table = _table(entry, path, problems)
    if table is None:
        return None
    driver: _Ref | None = None
    if kind is LifecycleBinding and "driver_type" in table:
        driver = _reference(
            table,
            "driver_type",
            path,
            problems,
            _subclass_of(LifecycleDriver, "a Driver knows one coordinator flavour and holds no state machine"),
        )
        if driver is None:
            return None
        table = {key: value for key, value in table.items() if key != "driver_type"}
    declared = [each.name for each in fields(kind)]
    unknown = [key for key in table if key not in declared]
    for key in unknown:
        problems.add(_join(path, key), f"unknown key; {kind.__name__} takes: {', '.join(sorted(declared))}")
    if unknown:
        return None
    try:
        binding = from_dict(kind, table)
    except FieldError as error:
        problems.add(_join(path, error.field), error.reason)
        return None
    except (DeclarationError, ValueError, TypeError) as error:
        problems.add(path, str(error))
        return None
    if driver is not None:
        problems.swaps.append(_Swap(binding, {"driver": driver}, driver.path))
    return binding


def _declare_all(tables: Mapping[str, Any], kinds: Iterable[str], path: str, problems: _Problems) -> list[Binding]:
    """Build the Bindings the kind tables of one component or machine declare, in file order."""
    wanted = set(kinds)
    declared: list[Binding] = []
    for name, entries in tables.items():
        if name not in wanted:
            continue
        here = _join(path, name)
        if not isinstance(entries, list):
            problems.add(here, f"expected an array of tables, written [[{name}]] once per entry")
            continue
        for index, entry in enumerate(entries):
            binding = _declare(TOML_KINDS[name], entry, _join(here, index), problems)
            if binding is not None:
                declared.append(binding)
    return declared


def _unique(declared: list[Binding], path: str, problems: _Problems) -> tuple[Binding, ...]:
    """Check no two Bindings share a name, with the rule a Component applies at init."""
    try:
        return resolve_bindings(declared)
    except DeclarationError as error:
        problems.add(path, str(error))
        return tuple(declared)


def _recipe_keys(recipe: type[Projection]) -> tuple[str, ...]:
    """Return the keys a recipe's table takes; a callable field is spelled as an import path."""
    keys = (*declarable_keys(recipe), *python_keys(recipe))
    return tuple(_TYPE_KEY if key == "type_name" else key for key in keys)


def _required_keys(recipe: type[Projection]) -> tuple[str, ...]:
    """Return the keys a recipe's table must carry: its fields that have no default."""
    return tuple(
        _TYPE_KEY if each.name == "type_name" else each.name
        for each in fields(recipe)
        if each.default is MISSING and each.default_factory is MISSING
    )


def _recipe_table_fits(table: Mapping[str, Any], name: str, path: str, problems: _Problems) -> bool:
    """Report each key the recipe does not take and each it needs and lacks, and say whether none."""
    recipe = RECIPES[name]
    keys = _recipe_keys(recipe)
    before = len(problems.found)
    for key in table:
        if key != "recipe" and key not in keys:
            problems.add(_join(path, key), f"unknown key; the {name} recipe takes: recipe, {', '.join(sorted(keys))}")
    for key in _required_keys(recipe):
        if key not in table:
            problems.add(_join(path, key), f"missing; the {name} recipe needs it")
    return len(problems.found) == before


def _projection(raw: Any, path: str, problems: _Problems) -> Projection | None:
    """Build one Projection from its table, through the recipe's own dataclass."""
    table = _table(raw, path, problems)
    if table is None:
        return None
    name = table.get("recipe")
    if name not in RECIPES:
        known = ", ".join(sorted(RECIPES))
        said = f"unknown recipe {name!r}" if "recipe" in table else "missing"
        problems.add(_join(path, "recipe"), f"{said}; the recipes are: {known}")
        return None
    recipe = RECIPES[name]
    if not _recipe_table_fits(table, name, path, problems) or _string(table, _TYPE_KEY, path, problems) is None:
        return None
    callables = {key: _reference(table, key, path, problems, _callable) for key in python_keys(recipe) if key in table}
    if None in callables.values():
        return None
    given = {
        ("type_name" if key == _TYPE_KEY else key): value
        for key, value in table.items()
        if key != "recipe" and key not in callables
    }
    # The stand-in lets the entry be registered and checked for duplicates before the real callable is imported.
    try:
        projection = from_dict(recipe, given, supplied=dict.fromkeys(callables, _stand_in))
    except FieldError as error:
        key = _TYPE_KEY if error.field == "type_name" else error.field.replace("type_name", _TYPE_KEY, 1)
        problems.add(_join(path, key), error.reason)
        return None
    except (DeclarationError, ValueError, TypeError) as error:
        problems.add(path, str(error))
        return None
    refs = {key: ref for key, ref in callables.items() if ref is not None}
    if refs:
        problems.swaps.append(_Swap(projection, refs, path))
    return projection


def _stand_in(*_args: object, **_kwargs: object) -> Any:
    """Stand for a callable a file names until it is imported."""
    message = "a callable named in a machine file was used before it was imported"
    raise RuntimeError(message)


def _layered(
    raw: Mapping[str, Any], path: str, below: ProjectionRegistry, problems: _Problems
) -> tuple[tuple[Projection, ...], ProjectionRegistry]:
    """Read the ``projection`` tables of a Machine or Component and register them over a registry.

    :return: The entries that registered, and the registry they extend.
    """
    entries = raw.get("projection")
    if entries is None:
        return (), below
    here = _join(path, "projection")
    if not isinstance(entries, list):
        problems.add(here, "expected an array of tables, written [[projection]] once per entry")
        return (), below
    kept: list[Projection] = []
    registry = below
    for index, entry in enumerate(entries):
        where = _join(here, index)
        projection = _projection(entry, where, problems)
        if projection is None:
            continue
        try:
            registry = registry.extended([projection])
        except DuplicateProjectionError as error:
            problems.add(_join(where, _TYPE_KEY), str(error))
            continue
        kept.append(projection)
    return tuple(kept), registry


class _MachineTable(NamedTuple):
    """What the ``[machine]`` table declares."""

    name: str
    identification: Identification
    bindings: tuple[Binding, ...]
    projections: tuple[Projection, ...]
    registry: ProjectionRegistry
    """The shipped catalogue extended by the Machine's entries; Components register over it."""
    machine_class: _Ref | None = None
    startup_skill_type: _Ref | None = None


def _machine(raw: Any, problems: _Problems) -> _MachineTable:
    """Read the ``[machine]`` table."""
    table = _table(raw, "machine", problems) if raw is not None else None
    if table is None:
        if raw is None:
            problems.add("machine", "missing; a file declares the Machine it serves")
        return _MachineTable("", Identification(), (), (), DEFAULT_REGISTRY)
    _check_keys(table, _MACHINE_KEYS, "machine", problems)
    name = _string(table, "name", "machine", problems)
    if name is None and "name" not in table:
        problems.add("machine.name", "missing; the Machine needs a name")
    identification = _identification(table.get("identification"), "machine.identification", problems, machine=True)
    bindings = _unique(_declare_all(table, ("tf",), "machine", problems), "machine", problems)
    projections, registry = _layered(table, "machine", DEFAULT_REGISTRY, problems)
    machine_class = _reference(
        table,
        "machine_class",
        "machine",
        problems,
        _subclass_of(ConfiguredMachine, "it receives the file's declarations through ConfiguredMachine's constructor"),
    )
    skill = _reference(
        table,
        "startup_skill_type",
        "machine",
        problems,
        _subclass_of(ROS2StartupSkill, "a custom class adds behaviour to the stock StartupSkill"),
    )
    return _MachineTable(name or "", identification, bindings, projections, registry, machine_class, skill)


def _ros2(raw: Any, problems: _Problems) -> ROS2Config:
    """Read the ``[ros2]`` table into the settings the Runtime and the Machine share."""
    table = _table(raw, "ros2", problems) if raw is not None else {}
    if table is None:
        return ROS2Config()
    known = [each.name for each in fields(ROS2Config)]
    _check_keys(table, known, "ros2", problems)
    nested = table.get("transforms")
    if isinstance(nested, dict):
        _check_keys(nested, [each.name for each in fields(TransformConfig)], "ros2.transforms", problems)
    if any(path.startswith("ros2") for path, _ in problems.found):
        return ROS2Config()
    try:
        return from_dict(ROS2Config, table)
    except FieldError as error:
        problems.add(_join("ros2", error.field), error.reason)
    except (DeclarationError, ValueError, TypeError) as error:
        problems.add("ros2", str(error))
    return ROS2Config()


def _server(raw: Any, base: Path, problems: _Problems) -> Path | None:
    """Read the ``[server]`` table: the opensmi server configuration the file points at, which must exist."""
    if raw is None:
        return None
    table = _table(raw, "server", problems)
    if table is None:
        return None
    _check_keys(table, ("config",), "server", problems)
    given = _string(table, "config", "server", problems)
    if given is None:
        return None
    resolved = base / given
    if not resolved.is_file():
        problems.add("server.config", f"{given!r} is not a file (looked for {str(resolved)!r})")
        return None
    return resolved.resolve()


def _component(index: int, raw: Any, base: Path, machine: ProjectionRegistry, problems: _Problems) -> _Spec | None:
    """Read one ``[[component]]`` table. Where it sits in the tree is decided afterwards."""
    path = _join("component", index)
    table = _table(raw, path, problems)
    if table is None:
        return None
    _check_keys(table, _COMPONENT_KEYS, path, problems)
    name = _string(table, "name", path, problems)
    if name is None and "name" not in table:
        problems.add(_join(path, "name"), "missing; every Component needs a name")
    parent = _string(table, "parent", path, problems)
    identification = _identification(
        table.get("identification"), _join(path, "identification"), problems, machine=False
    )
    declared = _declare_all(table, TOML_KINDS, path, problems)
    declared.extend(_include(table, path, base, problems))
    bindings = _unique(declared, path, problems)
    projections, _ = _layered(table, path, machine, problems)
    component_class = _reference(
        table,
        "component_class",
        path,
        problems,
        _subclass_of(
            ConfiguredComponent, "it receives the file's declarations through ConfiguredComponent's constructor"
        ),
    )
    stock = (
        ROS2LifecycleStartupSkill if any(isinstance(each, LifecycleBinding) for each in bindings) else ROS2StartupSkill
    )
    skill = _reference(
        table,
        "startup_skill_type",
        path,
        problems,
        _subclass_of(stock, "a custom class adds behaviour to the stock StartupSkill its Bindings ask for"),
    )
    if name is None:
        return None
    return _Spec(
        name=name,
        parent=parent,
        identification=identification,
        bindings=bindings,
        projections=projections,
        component_class=component_class,
        startup_skill_type=skill,
    )


def _include(table: Mapping[str, Any], path: str, base: Path, problems: _Problems) -> list[Binding]:
    """Read the Binding file a component includes, relative to the machine file."""
    given = _string(table, "bindings_toml", path, problems)
    if given is None:
        return []
    here = _join(path, "bindings_toml")
    try:
        return list(load_bindings(base / given))
    except OSError as error:
        problems.add(here, f"cannot read {given!r}: {error.strerror or error}")
    except tomllib.TOMLDecodeError as error:
        problems.add(here, f"{given!r} is not valid TOML: {error}")
    except (DeclarationError, ValueError, TypeError) as error:  # includes FieldError
        problems.add(here, f"{given!r}: {error}")
    return []


def _misplaced(spec: _Spec, index_of: Mapping[str, int]) -> str:
    """Say why a Component's parent is not a place it can go."""
    if spec.parent == spec.name:
        return "a Component cannot be its own parent"
    if spec.parent is not None and spec.parent in index_of:
        return (
            f"{spec.parent!r} is declared after its child (component[{index_of[spec.parent]}]); "
            f"a parent comes before the Components under it"
        )
    return f"{spec.parent!r} is not a Component of this file; leave parent out to put it under the Machine"


def _arrange(specs: list[_Spec | None], problems: _Problems) -> list[_Spec]:
    """Place each Component under its parent, or report why it cannot be."""
    index_of: dict[str, int] = {}
    for index, spec in enumerate(specs):
        if spec is not None:
            index_of.setdefault(spec.name, index)
    placed: dict[str, _Spec] = {}
    refused: set[str] = set()
    roots: list[_Spec] = []
    for index, spec in enumerate(specs):
        if spec is None:
            continue
        path = _join("component", index)
        if spec.name == INTROSPECTION_NAME:
            problems.add(
                _join(path, "name"), f"{INTROSPECTION_NAME!r} is the introspection Component every Machine carries"
            )
            refused.add(spec.name)
            continue
        if index_of[spec.name] != index:
            problems.add(_join(path, "name"), f"{spec.name!r} is already the name of component[{index_of[spec.name]}]")
            continue
        placed[spec.name] = spec
        if spec.parent is None:
            roots.append(spec)
            continue
        if spec.parent in refused:
            continue  # the mistake is the parent's, already reported
        if spec.parent in placed and spec.parent != spec.name:
            placed[spec.parent].children.append(spec)
        else:
            problems.add(_join(path, "parent"), _misplaced(spec, index_of))
    return roots


def _validate(source: Path) -> _Model:
    """Read and check the whole file, and raise every mistake in it at once."""
    problems = _Problems()
    try:
        with source.open("rb") as handle:
            data = tomllib.load(handle)
    except tomllib.TOMLDecodeError as error:
        raise FileDeclarationError([("", f"not valid TOML: {error}")], source=str(source)) from error
    _check_keys(data, _TOP_KEYS, "", problems)
    machine = _machine(data.get("machine"), problems)
    ros2 = _ros2(data.get("ros2"), problems)
    server_config = _server(data.get("server"), source.parent, problems)
    raw = data.get("component", [])
    roots: list[_Spec] = []
    if isinstance(raw, list):
        specs = [_component(index, each, source.parent, machine.registry, problems) for index, each in enumerate(raw)]
        roots = _arrange(specs, problems)
    else:
        problems.add("component", "expected an array of tables, written [[component]]")
    if problems.found:
        raise FileDeclarationError(problems.found, source=str(source))
    swaps = _resolve(problems)
    if problems.found:
        raise FileDeclarationError(problems.found, source=str(source))
    return _Model(
        name=machine.name,
        identification=machine.identification,
        bindings=machine.bindings,
        projections=machine.projections,
        ros2=ros2,
        server_config=server_config,
        roots=roots,
        machine_class=machine.machine_class,
        startup_skill_type=machine.startup_skill_type,
        swaps=swaps,
    )


def _swapped(items: Iterable[_T], swaps: Mapping[int, Any]) -> tuple[_T, ...]:
    """Replace each stand-in declaration by its real one."""
    return tuple(swaps.get(id(each), each) for each in items)


def _build(spec: _Spec, swaps: Mapping[int, Any]) -> ConfiguredComponent:
    """Construct a Component with its children, which are constructed first."""
    component_class = spec.component_class.value if spec.component_class is not None else ConfiguredComponent
    return component_class(
        name=spec.name,
        bindings=_swapped(spec.bindings, swaps),
        projections=_swapped(spec.projections, swaps),
        identification=spec.identification,
        children=[_build(each, swaps) for each in spec.children],
        startup_skill_type=spec.startup_skill_type.value if spec.startup_skill_type is not None else None,
    )


def _real_runtime(config: ROS2Config) -> RuntimeInterface:
    """Build the real Runtime. Imported here, as ``rclpy`` is needed to import it."""
    from opensmi.ros2.runtime import Runtime

    return Runtime(
        domain_id=config.domain_id,
        node_name=config.node_name,
        namespace=config.namespace,
        stop_timeout=config.stop_timeout,
    )


def load_machine(
    path: str | Path,
    *,
    runtime_factory: Callable[[ROS2Config], RuntimeInterface] | None = None,
) -> ConfiguredMachine:
    """Read a machine file and return the Machine it declares, ready for ``Server.add_machine``.

    :param path: The TOML file. A relative ``bindings_toml`` or ``[server] config`` in it is
        relative to this file.
    :param runtime_factory: How to make the Runtime from the file's ``[ros2]`` settings, called
        once and only after the file has passed. ``None`` builds the real Runtime, which needs
        ``rclpy``; a test passes a function returning its fake.
    :raises FileDeclarationError: When the file has any mistake, with all of them and the path to
        each. Nothing has been constructed by then.
    :raises OSError: When the file cannot be read.
    """
    model = _validate(Path(path))
    runtime = (runtime_factory or _real_runtime)(model.ros2)
    machine_class = model.machine_class.value if model.machine_class is not None else ConfiguredMachine
    return machine_class(
        name=model.name,
        runtime=runtime,
        ros2_config=model.ros2,
        identification=model.identification,
        bindings=model.bindings,
        projections=_swapped(model.projections, model.swaps),
        components=[_build(each, model.swaps) for each in model.roots],
        server_config=model.server_config,
        startup_skill_type=model.startup_skill_type.value if model.startup_skill_type is not None else None,
    )
