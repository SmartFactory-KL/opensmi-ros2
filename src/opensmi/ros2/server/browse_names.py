# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Turning names from ROS2 into browse names.

A message field or a parameter segment such as ``child_frame_id`` is browsed as ``ChildFrameId``,
which follows the opensmi naming rule. The raw ROS2 name stays as the DisplayName. A name that does
not convert is refused, never fixed silently.
"""

from __future__ import annotations

import re
from collections import defaultdict
from functools import cache
from typing import TYPE_CHECKING

from opensmi.core.paths import check_name

from opensmi.ros2.server.errors import BindingNameError, DuplicateVariableNameError

if TYPE_CHECKING:
    from collections.abc import Iterable, Mapping

__all__ = ["browse_name", "browse_names", "suggested_name"]

_CONVERTIBLE = re.compile(r"[A-Za-z0-9]+(_[A-Za-z0-9]+)*")
_NOT_NAME_CHARACTERS = re.compile(r"[^A-Za-z0-9]+")


def pascal_case(parts: Iterable[str]) -> str:
    """Return name parts capitalised and joined, which is how opensmi derives a browse name from a field name."""
    return "".join(part[:1].upper() + part[1:] for part in parts)


def suggested_name(name: str) -> str | None:
    """Return the name that follows the rule and is closest to ``name``, or ``None`` if there is none."""
    suggestion = pascal_case(_NOT_NAME_CHARACTERS.split(name))
    try:
        check_name(suggestion)
    except ValueError:
        return None
    return suggestion


@cache
def browse_name(ros2_name: str) -> str:
    """Return the browse name for a ROS2 name: split on ``_``, capitalise each part, join.

    ``child_frame_id`` gives ``ChildFrameId`` and ``FollowPath`` stays as it is. Use ``ros2_name``
    itself as the DisplayName.

    :raises BindingNameError: When the name has a character other than letters, digits and ``_``,
        an empty part, or does not give a valid browse name (it starts with a digit, or is ``None``,
        ``True`` or ``False``). The message suggests a name where one can be derived.
    """
    result = pascal_case(ros2_name.split("_"))
    try:
        if not _CONVERTIBLE.fullmatch(ros2_name):
            raise ValueError(ros2_name)  # noqa: TRY301
        check_name(result)
    except ValueError:
        suggestion = suggested_name(ros2_name)
        message = f"the ROS2 name {ros2_name!r} cannot be a browse name"
        if suggestion is not None:
            message = f"{message}; try {suggestion!r}"
        raise BindingNameError(message) from None
    return result


def browse_names(ros2_names: Iterable[str], renamed: Mapping[str, str] | None = None) -> dict[str, str]:
    """Return the browse name of each ROS2 name among siblings, keyed by the ROS2 name.

    A name in ``renamed`` takes its new name as it is, which already follows the rule.

    :raises BindingNameError: When one name does not convert, see :func:`browse_name`.
    :raises DuplicateVariableNameError: When two names give one browse name, such as ``x_1`` and
        ``x1``. The error names both ROS2 names; the author fixes it with the ``rename`` option of the Binding.
    """
    new_names = renamed or {}
    converted = {name: new_names.get(name) or browse_name(name) for name in ros2_names}
    by_browse_name: defaultdict[str, list[str]] = defaultdict(list)
    for name, browse in converted.items():
        by_browse_name[browse].append(name)
    for browse, names in by_browse_name.items():
        if len(names) > 1:
            quoted = " and ".join(repr(name) for name in names)
            message = (
                f"the ROS2 names {quoted} both become the browse name {browse!r}; "
                f"give one of them a new name with the rename option"
            )
            raise DuplicateVariableNameError(message)
    return converted
