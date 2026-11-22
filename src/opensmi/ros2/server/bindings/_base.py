# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""What every Binding kind repeats: the name, the wait, and the checks on both."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, ClassVar, Final

from opensmi.core.paths import check_name

from opensmi.ros2.server.browse_names import suggested_name
from opensmi.ros2.server.errors import (
    BindingNameError,
    DeclarationError,
    ImplementationClassError,
    RequiredDeclarationError,
)

if TYPE_CHECKING:
    from collections.abc import Mapping

__all__ = ["AwaitedBinding", "NamedBinding"]

_MACHINERY_MONITORING: Final = ("Status", "Health", "Process", "Consumption")
"""The sub-structures OPC 40001-1 gives a machinery item's Monitoring (section 16.2), kept free for them."""


@dataclass(frozen=True, slots=True, kw_only=True)
class NamedBinding:
    """A declaration with a name, which becomes a browse name.

    :cvar kind: What the messages call this declaration, such as ``"Monitoring Binding"``.
    :ivar name: The Binding's name.
    """

    kind: ClassVar[str] = "Binding"

    name: str

    def _require_a_name_that_can_be_browsed(self) -> None:
        """Refuse an empty name or one that breaks the naming rule, which opensmi would only refuse at start.

        :raises BindingNameError: When the name is empty or breaks the rule. The message suggests a name.
        """
        if not self.name:
            message = f"the {self.kind} has an empty name; give it one, because it is what the Binding is browsed as"
            raise BindingNameError(message)
        try:
            check_name(self.name)
        except ValueError:
            message = f"Name {self.name!r} breaks the naming rule"
            if (suggestion := suggested_name(self.name)) is not None:
                message = f"{message}; try {suggestion!r}"
            raise BindingNameError(message) from None

    def _require_a_name_machinery_monitoring_leaves_free(self) -> None:
        """Refuse a name OPC 40001-1 keeps for a sub-structure of a Machine's or Component's Monitoring.

        For the kinds whose name is a top-level node of that Monitoring. The match is exact, so
        ``StatusCode`` is free, and a name below the top level is not affected.

        :raises DeclarationError: When the name is ``Status``, ``Health``, ``Process`` or ``Consumption``.
        """
        if self.name not in _MACHINERY_MONITORING:
            return
        message = (
            f"the {self.kind} {self.name!r} takes a name reserved in a Machine's or Component's Monitoring: "
            f"OPC 40001-1 keeps {', '.join(_MACHINERY_MONITORING)} for the Machinery Monitoring "
            f"sub-structures; choose another name"
        )
        raise DeclarationError(message)

    def _require_a_machinery_group(self, group: str | None) -> None:
        """Refuse a ``group`` that is not one of the Machinery Monitoring sub-structures.

        :raises DeclarationError: When ``group`` is given and is not ``Status``, ``Health``, ``Process`` or
            ``Consumption``.
        """
        if group is None or group in _MACHINERY_MONITORING:
            return
        message = (
            f"the {self.kind} {self.name!r} names the group {group!r}; a group is one of the Machinery "
            f"Monitoring sub-structures {', '.join(_MACHINERY_MONITORING)} (OPC 40001-1), or left out"
        )
        raise DeclarationError(message)

    def _require_a_subclass(self, given: object, stock: type, why: str) -> None:
        """Refuse an implementation class that is not a subclass of the stock one.

        :raises ImplementationClassError: When ``given`` is not a class derived from ``stock``.
        """
        if isinstance(given, type) and issubclass(given, stock):
            return
        named = getattr(given, "__name__", repr(given))
        message = (
            f"the Binding {self.name!r} names {named} as its implementation class, and it is "
            f"not a subclass of {stock.__name__}; {why}"
        )
        raise ImplementationClassError(message)

    def _require_new_names_that_follow_the_rule(self, option: str, rename: Mapping[str, str]) -> None:
        """Refuse a ``rename`` value that breaks the naming rule. It is taken as is, never converted.

        :raises BindingNameError: When a new name is not a valid browse name.
        """
        for path, new_name in rename.items():
            try:
                check_name(new_name)
            except ValueError:
                message = (
                    f"the {self.kind} {self.name!r} gives {path!r} the new name {new_name!r} in {option}, "
                    f"which breaks the naming rule"
                )
                if (suggestion := suggested_name(new_name)) is not None:
                    message = f"{message}; try {suggestion!r}"
                raise BindingNameError(message) from None


@dataclass(frozen=True, slots=True, kw_only=True)
class AwaitedBinding(NamedBinding):
    """A declaration the StartupSkill may wait for.

    :ivar required: Whether the StartupSkill waits for it before ``Running``.
    :ivar startup_timeout: Seconds that wait lasts, or ``None`` for the deployment's default.
    """

    required: bool = False
    startup_timeout: float | None = None

    def _require_a_deadline_that_means_something(self) -> None:
        """Refuse a ``startup_timeout`` on a Binding that is not required, or one that is not positive.

        :raises RequiredDeclarationError: On either contradiction.
        """
        timeout = self.startup_timeout
        if timeout is None:
            return
        if not self.required:
            message = (
                f"the {self.kind} {self.name!r} declares startup_timeout={timeout!r} and is not required; a "
                f"deadline is how long a StartupSkill waits for a required Binding, so one on an "
                f"optional Binding is a guard nobody is standing"
            )
            raise RequiredDeclarationError(message)
        if timeout <= 0:
            message = (
                f"the {self.kind} {self.name!r} declares startup_timeout={timeout!r}; it is how many seconds a "
                f"StartupSkill waits for this Binding, so it is a positive number of seconds and "
                f"there is no way to spell a deadline that has already passed"
            )
            raise RequiredDeclarationError(message)
