# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Every Binding kind, the TOML kinds table and the loaders."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, Final, TypeAlias

from opensmi.ros2.server.bindings.action import ActionBinding
from opensmi.ros2.server.bindings.keyed import KeyedBinding
from opensmi.ros2.server.bindings.lifecycle import LifecycleBinding
from opensmi.ros2.server.bindings.method import MethodBinding
from opensmi.ros2.server.bindings.monitoring import MonitoringBinding
from opensmi.ros2.server.bindings.parameter import ROS2ParameterBinding
from opensmi.ros2.server.bindings.publisher import PublisherBinding
from opensmi.ros2.server.bindings.tf import TFBinding
from opensmi.ros2.server.config import from_dict, load_toml
from opensmi.ros2.server.errors import DuplicateBindingNameError

if TYPE_CHECKING:
    from collections.abc import Iterable
    from pathlib import Path

Binding: TypeAlias = (
    MonitoringBinding
    | KeyedBinding
    | MethodBinding
    | ROS2ParameterBinding
    | ActionBinding
    | TFBinding
    | PublisherBinding
    | LifecycleBinding
)
"""Every declaration kind. Later kinds widen the alias rather than the mechanism."""

TOML_KINDS: Final[Mapping[str, type[Any]]] = {
    "monitoring": MonitoringBinding,
    "keyed": KeyedBinding,
    "method": MethodBinding,
    # Qualified because bare ``parameter`` is opensmi vocabulary for the OPC UA variable.
    "ros2_parameter": ROS2ParameterBinding,
    "action": ActionBinding,
    "tf": TFBinding,
    "publisher": PublisherBinding,
    "lifecycle": LifecycleBinding,
}
"""Which table of a Binding file declares which kind. A file chooses among these and cannot add one."""


def is_required(binding: Binding) -> bool:
    """Return whether a declaration is a prerequisite its StartupSkill waits for.

    Works for every kind, including :class:`PublisherBinding`, which has no ``required`` field.
    """
    return bool(getattr(binding, "required", False))


def load_bindings(path: str | Path) -> tuple[Binding, ...]:
    """Read a Binding file, in the order its tables appear.

    :raises ValueError: When a table carries a key its kind does not declare, or names a kind that does
        not exist.
    """
    declared: list[Binding] = []
    for table, entries in load_toml(path).items():
        kind = TOML_KINDS.get(table)
        if kind is None:
            listed = ", ".join(sorted(TOML_KINDS))
            message = f"{table!r} is not a Binding kind; a Binding file declares one of: {listed}"
            raise ValueError(message)
        declared.extend(from_dict(kind, entry) for entry in entries)
    return tuple(declared)


def resolve_bindings(declared: Iterable[Binding], toml: str | Path | None = None) -> tuple[Binding, ...]:
    """Union what a Component declared in Python with what a deployment declared in TOML.

    The Python declarations come first and the file's follow. Neither source may overwrite the other.

    :raises DuplicateBindingNameError: When one name appears twice, in either source or across both.
    """
    resolved = (*declared, *(load_bindings(toml) if toml is not None else ()))
    seen: set[str] = set()
    for binding in resolved:
        if binding.name in seen:
            message = (
                f"the Binding name {binding.name!r} is declared twice; a name is what its "
                f"variables are called, so the second declaration would take the first's nodes"
            )
            raise DuplicateBindingNameError(message)
        seen.add(binding.name)
    return resolved
