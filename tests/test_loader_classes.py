# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A machine file names the deployer's own classes by import path, and mistakes in those keys are reported."""

from __future__ import annotations

import sys
from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua
from opensmi.server.base_startup_skill import BaseStartupSkill

from opensmi.ros2.server import load_machine
from opensmi.ros2.server.errors import FileDeclarationError
from tests import fixture_custom, fixture_lifecycle, fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.served import connected, serve

if TYPE_CHECKING:
    from collections.abc import AsyncIterator
    from pathlib import Path

    from asyncua.common.node import Node
    from opensmi.server import Server

    from opensmi.ros2.server.config import ROS2Config

CUSTOM = "tests.fixture_custom"
UNIMPORTED = "tests.fixture_unimported"

MACHINE = """
[machine]
name = "Cell"
{machine}
[machine.identification]
serial_number = "cell-1"
manufacturer = "Acme"
product_instance_uri = "urn:acme:cell"

[[component]]
name = "Base"
{base}
[component.identification]
serial_number = "base-1"
manufacturer = "Acme"
{base_tables}
[[component]]
name = "Arm"
{arm}
[component.identification]
serial_number = "arm-1"
manufacturer = "Acme"
"""

LIFECYCLE = """
[[component.lifecycle]]
name = "Talker"
path = "/lc_talker"
{lifecycle}
"""

REQUIRED = """
[[component.monitoring]]
name = "M"
path = "/m"
message_type = "fixture_msgs/msg/Voltage"
required = true
"""


def text_of(*, machine: str = "", base: str = "", base_tables: str = "", arm: str = "") -> str:
    return MACHINE.format(machine=machine, base=base, base_tables=base_tables, arm=arm)


def write(directory: Path, text: str) -> Path:
    path = directory / "machine.toml"
    path.write_text(text, encoding="utf-8")
    return path


def runtime_for(_config: ROS2Config) -> FakeRuntime:
    return FakeRuntime(
        message_types={**fixture_lifecycle.MESSAGE_TYPES, "fixture_msgs/msg/Voltage": fixture_messages.Voltage},
        service_types=fixture_lifecycle.SERVICE_TYPES,
    )


class Served:
    """A server started from a file, and the Machine the file made."""

    def __init__(self, server: Server) -> None:
        self.server = server
        self.machine: Any = server.machines[0]  # pyright: ignore[reportAttributeAccessIssue]

    def component(self, name: str) -> Any:
        return self.machine.components[name]

    async def serial_number_of(self, *path: str) -> object:
        """Read the SerialNumber a client sees on the Machine (no path) or on a Component."""
        async with connected(self.server, "visitor") as client:
            node: Node = client.nodes.objects
            below = [step for name in path for step in ("Components", name)]
            for name in ("Machines", "Cell", *below, "Identification", "SerialNumber"):
                node = await child(node, name)
            value = await node.read_value()
            return value.Text if isinstance(value, ua.LocalizedText) else value


async def child(node: Node, name: str) -> Node:
    for each in await node.get_children():
        if (await each.read_browse_name()).Name == name:
            return each
    pytest.fail(f"{name!r} is not below {node}")


async def start(tmp_path: Path, text: str) -> Served:
    return Served(await serve(load_machine(write(tmp_path, text), runtime_factory=runtime_for)))


@pytest.fixture
async def stopping() -> AsyncIterator[list[Served]]:
    started: list[Served] = []
    yield started
    for each in started:
        await each.server.shutdown()


# -- each key starts a server in which the named class is in use ----------------------------------------


@pytest.mark.parametrize("spelling", [f"{CUSTOM}.CustomMachine", f"{CUSTOM}:CustomMachine"])
async def test_machine_class_is_the_class_of_the_served_machine(
    tmp_path: Path, stopping: list[Served], spelling: str
) -> None:
    served = await start(tmp_path, text_of(machine=f'machine_class = "{spelling}"'))
    stopping.append(served)

    assert type(served.machine) is fixture_custom.CustomMachine
    assert await served.serial_number_of() == "custom-machine"
    assert type(served.component("Base")).__name__ == "ConfiguredComponent"


async def test_component_class_is_the_class_of_that_component_only(tmp_path: Path, stopping: list[Served]) -> None:
    served = await start(tmp_path, text_of(arm=f'component_class = "{CUSTOM}:CustomComponent"'))
    stopping.append(served)

    assert type(served.component("Arm")) is fixture_custom.CustomComponent
    assert await served.serial_number_of("Arm") == "custom-Arm"
    assert await served.serial_number_of("Base") == "base-1"
    assert type(served.machine).__name__ == "ConfiguredMachine"


async def test_a_component_class_is_built_with_its_children(tmp_path: Path, stopping: list[Served]) -> None:
    text = text_of(base=f'component_class = "{CUSTOM}:CustomComponent"', arm='parent = "Base"')
    served = await start(tmp_path, text)
    stopping.append(served)

    assert await served.serial_number_of("Base", "Arm") == "arm-1"


async def test_the_machines_startup_skill_type_is_the_class_of_its_startup_skill(
    tmp_path: Path, stopping: list[Served]
) -> None:
    served = await start(tmp_path, text_of(machine=f'startup_skill_type = "{CUSTOM}:CustomStartupSkill"'))
    stopping.append(served)

    assert type(served.machine.skill_set[BaseStartupSkill.NAME]) is fixture_custom.CustomStartupSkill
    assert BaseStartupSkill.NAME not in served.component("Base").skill_set


async def test_a_components_startup_skill_type_is_the_class_of_its_startup_skill(
    tmp_path: Path, stopping: list[Served]
) -> None:
    text = text_of(base=f'startup_skill_type = "{CUSTOM}:CustomStartupSkill"', base_tables=REQUIRED)
    served = await start(tmp_path, text)
    stopping.append(served)

    assert type(served.component("Base").skill_set[BaseStartupSkill.NAME]) is fixture_custom.CustomStartupSkill


async def test_a_component_with_lifecycle_bindings_takes_a_lifecycle_startup_skill(
    tmp_path: Path, stopping: list[Served]
) -> None:
    text = text_of(
        base=f'startup_skill_type = "{CUSTOM}:CustomLifecycleStartupSkill"',
        base_tables=LIFECYCLE.format(lifecycle=""),
    )
    served = await start(tmp_path, text)
    stopping.append(served)

    assert type(served.component("Base").skill_set[BaseStartupSkill.NAME]) is fixture_custom.CustomLifecycleStartupSkill


async def test_driver_type_is_the_driver_of_that_lifecycle_binding(tmp_path: Path, stopping: list[Served]) -> None:
    fixture_custom.BUILT.clear()
    tables = LIFECYCLE.format(lifecycle=f'driver_type = "{CUSTOM}:CustomDriver"')
    served = await start(tmp_path, text_of(base_tables=tables))
    stopping.append(served)

    (binding,) = served.component("Base").bindings
    assert binding.driver is fixture_custom.CustomDriver
    assert binding.driver_class is fixture_custom.CustomDriver
    assert fixture_custom.BUILT == ["driver"]


async def test_without_a_key_the_stock_classes_apply(tmp_path: Path, stopping: list[Served]) -> None:
    served = await start(tmp_path, text_of(base_tables=LIFECYCLE.format(lifecycle="")))
    stopping.append(served)

    assert type(served.machine).__name__ == "ConfiguredMachine"
    assert type(served.component("Base")).__name__ == "ConfiguredComponent"
    (binding,) = served.component("Base").bindings
    assert binding.driver == "lifecycle"


# -- mistakes in the keys ---------------------------------------------------------------------------------


def problems_of(tmp_path: Path, text: str) -> dict[str, str]:
    calls: list[ROS2Config] = []

    def factory(config: ROS2Config) -> FakeRuntime:
        calls.append(config)
        return runtime_for(config)

    with pytest.raises(FileDeclarationError) as raised:
        load_machine(write(tmp_path, text), runtime_factory=factory)
    assert calls == []
    return dict(raised.value.problems)


def keyed(key: str, value: str, *, where: str) -> str:
    """Return a file that sets one key on the machine, the first component or the second."""
    given = f'{key} = "{value}"'
    if where == "machine":
        return text_of(machine=given)
    if where == "lifecycle":
        return text_of(base_tables=LIFECYCLE.format(lifecycle=given))
    return text_of(arm=given)


PLACES = [
    ("machine", "machine_class", "machine.machine_class"),
    ("component", "component_class", "component[1].component_class"),
    ("machine", "startup_skill_type", "machine.startup_skill_type"),
    ("component", "startup_skill_type", "component[1].startup_skill_type"),
    ("lifecycle", "driver_type", "component[0].lifecycle[0].driver_type"),
]


@pytest.mark.parametrize(("where", "key", "path"), PLACES)
def test_a_missing_module_is_reported_at_the_key(tmp_path: Path, where: str, key: str, path: str) -> None:
    problems = problems_of(tmp_path, keyed(key, "tests.no_such_module:Thing", where=where))

    assert set(problems) == {path}
    assert "cannot import module 'tests.no_such_module'" in problems[path]


@pytest.mark.parametrize(("where", "key", "path"), PLACES)
def test_a_missing_attribute_is_reported_at_the_key(tmp_path: Path, where: str, key: str, path: str) -> None:
    problems = problems_of(tmp_path, keyed(key, f"{CUSTOM}.NoSuchClass", where=where))

    assert set(problems) == {path}
    assert f"module '{CUSTOM}' has no attribute 'NoSuchClass'" in problems[path]


@pytest.mark.parametrize(("where", "key", "path"), PLACES)
@pytest.mark.parametrize("wrong", ["Plain", "NOT_A_CLASS"])
def test_a_class_of_the_wrong_kind_is_reported_at_the_key(
    tmp_path: Path, where: str, key: str, path: str, wrong: str
) -> None:
    problems = problems_of(tmp_path, keyed(key, f"{CUSTOM}:{wrong}", where=where))

    assert set(problems) == {path}
    assert "is not a subclass of" in problems[path]


def test_a_machine_class_must_be_a_configured_machine_and_a_component_class_a_configured_component(
    tmp_path: Path,
) -> None:
    swapped = text_of(
        machine=f'machine_class = "{CUSTOM}:CustomComponent"', arm=f'component_class = "{CUSTOM}:CustomMachine"'
    )

    problems = problems_of(tmp_path, swapped)

    assert "is not a subclass of ConfiguredMachine" in problems["machine.machine_class"]
    assert "is not a subclass of ConfiguredComponent" in problems["component[1].component_class"]


def test_a_component_with_lifecycle_bindings_refuses_a_startup_skill_that_is_not_a_lifecycle_one(
    tmp_path: Path,
) -> None:
    text = text_of(
        base=f'startup_skill_type = "{CUSTOM}:CustomStartupSkill"', base_tables=LIFECYCLE.format(lifecycle="")
    )

    problems = problems_of(tmp_path, text)

    assert "is not a subclass of ROS2LifecycleStartupSkill" in problems["component[0].startup_skill_type"]


def test_a_module_that_fails_while_importing_is_a_mistake_of_the_file(tmp_path: Path) -> None:
    problems = problems_of(tmp_path, keyed("machine_class", "tests.fixture_broken:Thing", where="machine"))

    assert "RuntimeError: this module fails when it is imported" in problems["machine.machine_class"]


def test_every_path_that_fails_is_reported_in_one_go(tmp_path: Path) -> None:
    text = text_of(
        machine=f'machine_class = "{CUSTOM}:Plain"\nstartup_skill_type = "tests.nope:Skill"',
        base=f'component_class = "{CUSTOM}:Missing"',
        arm=f'startup_skill_type = "{CUSTOM}:CustomLifecycleStartupSkill"\ncomponent_class = "{CUSTOM}:Plain"',
    )

    problems = problems_of(tmp_path, text)

    assert set(problems) == {
        "machine.machine_class",
        "machine.startup_skill_type",
        "component[0].component_class",
        "component[1].component_class",
    }


@pytest.mark.parametrize("text", ["Thing", "a b.C", "tests..x", ".x", "pkg.mod:", "1pkg.mod.X", "pkg:mod:X"])
def test_something_that_is_not_an_import_path_is_reported_without_importing(tmp_path: Path, text: str) -> None:
    problems = problems_of(tmp_path, keyed("machine_class", text, where="machine"))

    assert "is not an import path" in problems["machine.machine_class"]


def test_the_key_must_be_a_string(tmp_path: Path) -> None:
    problems = problems_of(tmp_path, text_of(machine="machine_class = 5"))

    assert "expected a non-empty string" in problems["machine.machine_class"]


# -- what is imported, and when ---------------------------------------------------------------------------


@pytest.fixture
def unimported() -> None:
    sys.modules.pop(UNIMPORTED, None)
    fixture_custom.IMPORTED.clear()


def test_a_file_with_structural_mistakes_imports_nothing(tmp_path: Path, unimported: None) -> None:
    text = text_of(
        machine=f'machine_class = "{UNIMPORTED}:Anything"\ncolour = "red"',
        base=f'component_class = "{UNIMPORTED}:Anything"',
        arm='parent = "Nobody"',
    )

    problems = problems_of(tmp_path, text)

    assert set(problems) == {"machine.colour", "component[1].parent"}
    assert fixture_custom.IMPORTED == []
    assert UNIMPORTED not in sys.modules


def test_a_structural_mistake_in_a_binding_or_a_projection_also_stops_the_import(
    tmp_path: Path, unimported: None
) -> None:
    tables = LIFECYCLE.format(lifecycle=f'driver_type = "{UNIMPORTED}:Anything"\nflavour = "x"')
    projection = f'[[component.projection]]\nrecipe = "functions"\ntype = "a/msg/B"\nshaped_by = "{UNIMPORTED}:f"\n'

    problems = problems_of(
        tmp_path, text_of(machine=f'machine_class = "{UNIMPORTED}:Anything"', base_tables=tables + projection)
    )

    assert "component[0].lifecycle[0].flavour" in problems
    assert "component[0].projection[0].decoded_by" in problems
    assert fixture_custom.IMPORTED == []


def test_a_file_that_is_otherwise_valid_is_imported_and_its_path_is_judged(tmp_path: Path, unimported: None) -> None:
    problems = problems_of(tmp_path, keyed("machine_class", f"{UNIMPORTED}:Anything", where="machine"))

    assert "is not a subclass of ConfiguredMachine" in problems["machine.machine_class"]
    assert fixture_custom.IMPORTED == [UNIMPORTED]
