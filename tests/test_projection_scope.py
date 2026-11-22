# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Every ROS2 type in scope is classified, and the record is true.

Ring 1 checks the record against itself and the shipped catalogue. Ring 3 checks it against an
installed Jazzy and fails on unclassified types.
"""

from __future__ import annotations

import importlib
from typing import Any

import pytest

from opensmi.ros2.runtime.messages import message_dataclass, repeated_message
from opensmi.ros2.runtime.selection import FieldSelection, resolve_selection
from opensmi.ros2.server.projection import ANY_TYPE, ProjectionContext
from opensmi.ros2.server.projections import DEFAULT_REGISTRY, SHIPPED
from opensmi.ros2.server.projections.scope import CLASSIFIED, CORE, CURATED, EMPTY, KEYED, SCOPE, WALK

VERDICTS = (WALK, CURATED, KEYED, EMPTY)


def installed_types(package: str) -> set[str] | None:
    """Return every message type one interface package installs, or None when it is absent."""
    try:
        module = importlib.import_module(f"{package}.msg")
    except ModuleNotFoundError:
        return None
    return {
        f"{package}/msg/{name}"
        for name in dir(module)
        if not name.startswith("_") and hasattr(getattr(module, name), "get_fields_and_field_types")
    }


def recorded_types(package: str) -> set[str]:
    """Return every type of one package the record classifies."""
    return {name for name in CLASSIFIED if name.startswith(f"{package}/")}


def rendered(type_name: str) -> Any:
    """Shape one installed type through the shipped registry, as a Binding over it would."""
    message_type = message_dataclass(type_name)
    context = ProjectionContext(
        registry=DEFAULT_REGISTRY,
        selected=resolve_selection(message_type, FieldSelection()),
    )
    return DEFAULT_REGISTRY.find(type_name).shape(message_type, context)


def repeats_a_message(message_type: Any) -> bool:
    """Return whether a type repeats a message, at its own level or anywhere inside it."""
    for described in message_type.ros2.fields:
        if repeated_message(message_type, described.name) is not None:
            return True
    return any(repeats_a_message(nested) for nested in message_type.ros2_nested.values())


# -- ring 1: the record, against itself ---------------------------------------------------------


def test_every_classification_is_one_of_the_four_verdicts() -> None:
    assert set(CLASSIFIED.values()) <= set(VERDICTS)


def test_every_classified_type_is_in_a_package_in_scope() -> None:
    packages = {name.split("/")[0] for name in CLASSIFIED}
    kinds = {name.split("/")[1] for name in CLASSIFIED}

    assert packages == set(SCOPE)
    assert kinds == {"msg"}


def test_the_core_packages_are_in_scope() -> None:
    assert set(CORE) <= set(SCOPE)


def test_a_type_recorded_as_curated_is_one_the_catalogue_actually_ships() -> None:
    curated = {name for name, verdict in CLASSIFIED.items() if verdict == CURATED}

    assert curated == {entry.type_name for entry in SHIPPED}


def test_nothing_shipped_is_missing_from_the_record() -> None:
    for entry in SHIPPED:
        assert entry.type_name != ANY_TYPE
        assert entry.type_name in CLASSIFIED


# -- ring 3: the record, against a live ROS2 Jazzy installation ---------------------------------


@pytest.mark.jazzy
def test_the_completeness_check_has_something_to_check() -> None:
    # Without this the job passes silently when no scope is installed.
    assert [package for package in CORE if installed_types(package) is None] == []


@pytest.mark.jazzy
@pytest.mark.parametrize("package", SCOPE)
def test_every_installed_type_in_scope_is_classified(package: str) -> None:
    installed = installed_types(package)
    if installed is None:
        pytest.skip(f"{package} is not installed")

    unclassified = sorted(installed - recorded_types(package))
    stale = sorted(recorded_types(package) - installed)

    assert unclassified == [], "classify these in opensmi.ros2.server.projections.scope"
    assert stale == [], "these are recorded but no longer installed"


@pytest.mark.jazzy
@pytest.mark.parametrize("package", SCOPE)
def test_the_registry_renders_each_type_the_way_the_record_says(package: str) -> None:
    if installed_types(package) is None:
        pytest.skip(f"{package} is not installed")

    for type_name in sorted(recorded_types(package)):
        verdict = CLASSIFIED[type_name]
        if verdict == KEYED:
            assert repeats_a_message(message_dataclass(type_name)), (
                f"{type_name} is recorded as repeating a message and repeats none"
            )
        elif verdict == EMPTY:
            assert rendered(type_name) == (), type_name
        else:
            assert rendered(type_name), type_name
            curated = DEFAULT_REGISTRY.find(type_name).type_name != ANY_TYPE
            assert curated is (verdict == CURATED), type_name
