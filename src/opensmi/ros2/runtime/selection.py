# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The one field-selection mechanism: include and exclude globs over ROS2 field paths.

Every Binding that narrows a message uses it; it is a pure function of a message dataclass.

* A pattern is tested against a leaf and against each of its ancestors, so ``exclude=("pose",)``
  removes every field under ``pose``.
* Exclude wins. Include narrows first, exclude removes after.
* A pattern that matches nothing is fatal, as is a selection that leaves no field.

Patterns are ``fnmatch`` globs matched against the whole path, with ``/`` an ordinary character:
``*`` crosses levels, so ``*/position/*`` reaches a position at any depth. A repeated message
field (``sequence<PoseStamped>``) is a leaf: it can be selected by name, but nothing inside it can.
"""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field as dataclass_field
from fnmatch import fnmatchcase
from typing import TYPE_CHECKING

from opensmi.ros2.runtime.errors import FieldSelectionError
from opensmi.ros2.runtime.messages import NestedType

if TYPE_CHECKING:
    from collections.abc import Iterable, Iterator, Mapping, Sequence

    from opensmi.ros2.runtime.messages import Message

__all__ = ["FieldSelection", "field_paths", "resolve_selection"]

SEPARATOR = "/"
"""What joins a field path, and what a UA variable name is joined with."""


@dataclass(frozen=True, slots=True, kw_only=True)
class FieldSelection:
    """Which fields of a message type a Binding exposes.

    :ivar include: Patterns narrowing the message to what they match. Empty means every field,
        which is the ordinary case: a Binding that wants the whole message says nothing.
    :ivar exclude: Patterns removing what they match, after ``include`` has narrowed.
    :ivar rename: New names by exact field path. The Binding checks a name against the naming
        rule; :func:`resolve_selection` checks the path is rendered.
    """

    include: tuple[str, ...] = ()
    exclude: tuple[str, ...] = ()
    rename: Mapping[str, str] = dataclass_field(default_factory=dict)

    @property
    def patterns(self) -> tuple[str, ...]:
        """Return every declared pattern (include, then exclude)."""
        return (*self.include, *self.exclude)


def field_paths(message_type: type[Message]) -> tuple[str, ...]:
    """Return every field path a message type reaches, in ROS2 declaration order.

    Interior paths are included, so ``pose`` appears beside ``pose/position/x``.
    """
    return tuple(path for path, _ in _walk(message_type, ""))


def resolve_selection(message_type: type[Message], selection: FieldSelection) -> frozenset[str]:
    """Return the field paths that survive a selection, interior paths included.

    :raises FieldSelectionError: When a declared pattern matches no path of the type.
    :raises FieldSelectionError: When nothing survives.
    :raises FieldSelectionError: When ``rename`` names a path that is not rendered: unknown or removed.
    """
    walked = tuple(_walk(message_type, ""))
    _require_every_pattern_matches(message_type, selection, [path for path, _ in walked])

    surviving: set[str] = set()
    for path, leaf in walked:
        if not leaf:
            continue
        if selection.include and not _matched(path, selection.include):
            continue
        if _matched(path, selection.exclude):
            continue
        surviving.update(_ancestors_or_self(path))
    if not surviving and any(leaf for _, leaf in walked):
        # Only where the type had leaves: `std_msgs/Empty` has none, and an empty result is right for it.
        message = (
            f"the include and exclude patterns leave {message_type.ros2.type_name} with no field "
            f"at all; a Binding with no variables is a declaration nobody meant"
        )
        raise FieldSelectionError(message)
    _require_renames_to_be_rendered(message_type, selection, surviving)
    return frozenset(surviving)


def _require_renames_to_be_rendered(
    message_type: type[Message],
    selection: FieldSelection,
    rendered: set[str],
) -> None:
    """Refuse a rename of a path the Binding does not render."""
    missing = sorted(path for path in selection.rename if path not in rendered)
    if not missing:
        return
    listed = ", ".join(repr(path) for path in missing)
    message = (
        f"rename names {listed}, which {message_type.ros2.type_name} does not render with this selection; "
        f"rendered fields are {', '.join(sorted(rendered))}"
    )
    raise FieldSelectionError(message)


def _walk(message_type: type[Message], prefix: str) -> Iterator[tuple[str, bool]]:
    """Yield every path below a message type, paired with whether it is a leaf."""
    for field in message_type.ros2.fields:
        path = f"{prefix}{field.name}"
        nested = message_type.ros2_nested.get(field.name)
        if nested is not None and isinstance(field.field_type, NestedType):
            yield path, False
            yield from _walk(nested, f"{path}{SEPARATOR}")
        else:
            yield path, True


def _require_every_pattern_matches(
    message_type: type[Message],
    selection: FieldSelection,
    paths: Sequence[str],
) -> None:
    """Refuse a declaration carrying a pattern that matches no path of this message type."""
    unmatched = [pattern for pattern in selection.patterns if not any(fnmatchcase(path, pattern) for path in paths)]
    if not unmatched:
        return
    listed = ", ".join(repr(pattern) for pattern in sorted(set(unmatched)))
    message = f"{listed} matches no field of {message_type.ros2.type_name}; its fields are {', '.join(paths)}"
    raise FieldSelectionError(message)


def _matched(path: str, patterns: Iterable[str]) -> bool:
    """Return whether a pattern matches this path or any path it hangs below."""
    return any(fnmatchcase(each, pattern) for each in _ancestors_or_self(path) for pattern in patterns)


def _ancestors_or_self(path: str) -> tuple[str, ...]:
    """Return ``a``, ``a/b``, ``a/b/c`` for ``a/b/c``: the path and everything carrying it."""
    parts = path.split(SEPARATOR)
    return tuple(SEPARATOR.join(parts[: index + 1]) for index in range(len(parts)))
