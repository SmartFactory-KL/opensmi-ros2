# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Projections declared as tables in a machine file: equivalence with the Python literal, layering, and
error reports."""

from __future__ import annotations

import contextlib
from typing import TYPE_CHECKING, Any, NamedTuple

import pytest
from asyncua import ua
from asyncua.ua.object_ids import ObjectIds
from opensmi.server import BaseComponent

from opensmi.ros2.runtime.messages import PrimitiveType, SequenceType
from opensmi.ros2.server import load_machine
from opensmi.ros2.server import recipes as recipes_module
from opensmi.ros2.server.bindings import MonitoringBinding
from opensmi.ros2.server.config import declarable_keys
from opensmi.ros2.server.errors import FileDeclarationError
from opensmi.ros2.server.loader import RECIPES
from opensmi.ros2.server.projection import Projection
from opensmi.ros2.server.recipes import Blob, Default, Drop, Enum, Functions, Scalar
from tests import fixture_custom, fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.fixture_messages import UINT32, build, describe
from tests.served import TestComponent, TestMachine, connected, monitoring_nodes, serve

if TYPE_CHECKING:
    from pathlib import Path

    from opensmi.server import Server

    from opensmi.ros2.server.config import ROS2Config

OCCUPANCY_GRID = build(
    describe(
        "nav_msgs/msg/OccupancyGrid",
        width=UINT32,
        data=(SequenceType(element=PrimitiveType(name="int8")), "sequence<int8>"),
    )
)

MESSAGE_TYPES: dict[str, Any] = {
    "fixture_msgs/msg/Grid": fixture_messages.Grid,
    "fixture_msgs/msg/Mode": fixture_messages.Mode,
    "fixture_msgs/msg/Pose": fixture_messages.Pose,
    "fixture_msgs/msg/Point": fixture_messages.Point,
    "fixture_msgs/msg/Quaternion": fixture_messages.Quaternion,
    "fixture_msgs/msg/Voltage": fixture_messages.Voltage,
    "nav_msgs/msg/OccupancyGrid": OCCUPANCY_GRID,
}

HEAD = """
[machine]
name = "Cell"
{machine}
[machine.identification]
serial_number = "cell-1"
manufacturer = "Acme"
product_instance_uri = "urn:acme:cell"
"""

COMPONENT = """
[[component]]
name = "{name}"

[component.identification]
serial_number = "{name}-1"
manufacturer = "Acme"

[[component.monitoring]]
name = "M"
path = "/m"
message_type = "{message_type}"
{tables}
"""


def machine_file(*components: str, machine: str = "") -> str:
    return HEAD.format(machine=machine) + "".join(components)


def component(tables: str = "", *, name: str = "C", message_type: str = "fixture_msgs/msg/Pose") -> str:
    return COMPONENT.format(name=name, message_type=message_type, tables=tables)


def write(directory: Path, text: str) -> Path:
    path = directory / "machine.toml"
    path.write_text(text, encoding="utf-8")
    return path


def runtime_for(_config: ROS2Config) -> FakeRuntime:
    return FakeRuntime(message_types=MESSAGE_TYPES)


class Rendering(NamedTuple):
    """What a client sees of one Component's Monitoring: per variable, its type and its shape."""

    datatype: str
    value_rank: int
    dimensions: list[int] | None
    labels: list[tuple[int, str]] | None


async def rendering_of(server: Server, item: Any) -> dict[str, Rendering]:
    seen: dict[str, Rendering] = {}
    async with connected(server, "visitor") as client:
        for name, nodes in sorted(monitoring_nodes(item).items()):
            node = client.get_node(nodes.nodeid)
            datatype = client.get_node(await node.read_data_type())
            definition = None
            with contextlib.suppress(ua.UaError):
                definition = await datatype.read_data_type_definition()
            labels = (
                [(field.Value, field.Name) for field in definition.Fields]
                if isinstance(definition, ua.EnumDefinition)
                else None
            )
            seen[name] = Rendering(
                (await datatype.read_browse_name()).Name,
                await node.read_value_rank(),
                await node.read_array_dimensions(),
                labels,
            )
    return seen


# -- each recipe, as a table and as its literal ----------------------------------------------------


class Case(NamedTuple):
    table: str
    literal: Projection
    message_type: str


CASES = {
    "default": Case(
        'recipe = "default"\ntype = "nav_msgs/msg/OccupancyGrid"\noverride = true',
        Default("nav_msgs/msg/OccupancyGrid", override=True),
        "nav_msgs/msg/OccupancyGrid",
    ),
    "scalar": Case(
        'recipe = "scalar"\ntype = "fixture_msgs/msg/Voltage"\nvariant_type = "Float"\ntype_hash = "RIHS01_0000"',
        Scalar("fixture_msgs/msg/Voltage", variant_type=ua.VariantType.Float, type_hash="RIHS01_0000"),
        "fixture_msgs/msg/Voltage",
    ),
    "enum": Case(
        'recipe = "enum"\ntype = "fixture_msgs/msg/Mode"\n[component.projection.fields.mode]\n0 = "Idle"\n7 = "Run"',
        Enum("fixture_msgs/msg/Mode", fields={"mode": {0: "Idle", 7: "Run"}}),
        "fixture_msgs/msg/Mode",
    ),
    "blob": Case(
        f'recipe = "blob"\ntype = "fixture_msgs/msg/Grid"\nfield = "data"\ndatatype = {ObjectIds.ImagePNG}',
        Blob("fixture_msgs/msg/Grid", field="data", datatype=ObjectIds.ImagePNG),
        "fixture_msgs/msg/Grid",
    ),
    "drop": Case(
        'recipe = "drop"\ntype = "fixture_msgs/msg/Pose"\nfields = ["orientation"]',
        Drop("fixture_msgs/msg/Pose", fields=("orientation",)),
        "fixture_msgs/msg/Pose",
    ),
    "functions": Case(
        'recipe = "functions"\ntype = "fixture_msgs/msg/Voltage"\n'
        'shaped_by = "tests.fixture_custom:voltage_shape"\ndecoded_by = "tests.fixture_custom.voltage_decode"',
        Functions(
            "fixture_msgs/msg/Voltage",
            shaped_by=fixture_custom.voltage_shape,
            decoded_by=fixture_custom.voltage_decode,
        ),
        "fixture_msgs/msg/Voltage",
    ),
}


def python_component(case: Case, *, curated: bool) -> BaseComponent:
    """The Component the Python route writes: the same Binding, with the literal or without it."""

    class Written(TestComponent):
        bindings = (MonitoringBinding(name="M", path="/m", message_type=case.message_type),)
        projections = (case.literal,) if curated else ()

    return Written(name="C")


def machine_of(item: BaseComponent) -> TestMachine:
    return TestMachine(item, name="Cell", runtime=FakeRuntime(message_types=MESSAGE_TYPES))


@pytest.mark.parametrize("recipe", sorted(CASES))
async def test_a_table_renders_the_address_space_its_python_literal_does(tmp_path: Path, recipe: str) -> None:
    case = CASES[recipe]
    from_file = await serve(
        load_machine(
            write(tmp_path, machine_file(entries(case.table, message_type=case.message_type))),
            runtime_factory=runtime_for,
        )
    )
    from_python = await serve(machine_of(python_component(case, curated=True)))
    uncurated = await serve(machine_of(python_component(case, curated=False)))
    try:
        loaded: Any = from_file.machines[0].components["C"]  # pyright: ignore[reportAttributeAccessIssue]
        declared: Any = from_python.machines[0].components["C"]  # pyright: ignore[reportAttributeAccessIssue]
        plain: Any = uncurated.machines[0].components["C"]  # pyright: ignore[reportAttributeAccessIssue]

        assert loaded.projections == (case.literal,)
        seen = await rendering_of(from_file, loaded)
        assert seen == await rendering_of(from_python, declared)
        assert seen != await rendering_of(uncurated, plain)
    finally:
        await from_file.shutdown()
        await from_python.shutdown()
        await uncurated.shutdown()


# -- the recipes' keys come from their dataclasses --------------------------------------------------


def test_every_recipe_is_one_a_table_can_name() -> None:
    shipped = {
        each
        for each in vars(recipes_module).values()
        if isinstance(each, type) and issubclass(each, Projection) and each is not Projection
    }

    assert shipped == set(RECIPES.values())


def test_a_recipe_field_a_file_cannot_spell_is_a_known_callable() -> None:
    # A callable has no TOML spelling: a new such field must fail here.
    left_out = {
        (name, field)
        for name, recipe in RECIPES.items()
        for field in recipe.__dataclass_fields__
        if field not in declarable_keys(recipe)
    }

    assert left_out == {
        ("blob", "encode"),
        ("functions", "shaped_by"),
        ("functions", "decoded_by"),
        ("functions", "encode"),
    }


@pytest.mark.parametrize(
    ("recipe", "keys"),
    [
        ("default", {"type", "override", "type_hash"}),
        ("scalar", {"type", "override", "type_hash", "variant_type", "datatype"}),
        ("enum", {"type", "override", "type_hash", "fields"}),
        ("blob", {"type", "override", "type_hash", "field", "datatype"}),
        ("drop", {"type", "override", "type_hash", "fields"}),
    ],
)
def test_a_table_takes_the_fields_of_its_recipe(recipe: str, keys: set[str]) -> None:
    assert {"type" if key == "type_name" else key for key in declarable_keys(RECIPES[recipe])} == keys


# -- layering ---------------------------------------------------------------------------------------


def names_below(item: Any) -> list[str]:
    return sorted(monitoring_nodes(item))


async def test_a_machine_entry_reaches_every_component_and_a_components_own_goes_ahead_of_it(tmp_path: Path) -> None:
    machine_drop = '[[machine.projection]]\nrecipe = "drop"\ntype = "fixture_msgs/msg/Pose"\nfields = ["orientation"]\n'
    own = '[[component.projection]]\nrecipe = "drop"\ntype = "fixture_msgs/msg/Pose"\nfields = ["position"]\n'
    own += "override = true\n"
    text = machine_file(component(name="Plain"), component(own, name="Own"), machine=machine_drop)
    server = await serve(load_machine(write(tmp_path, text), runtime_factory=runtime_for))
    try:
        components: Any = server.machines[0].components  # pyright: ignore[reportAttributeAccessIssue]

        assert names_below(components["Plain"]) == ["M.Position.X", "M.Position.Y", "M.Position.Z"]
        assert names_below(components["Own"]) == [
            "M.Orientation.W",
            "M.Orientation.X",
            "M.Orientation.Y",
            "M.Orientation.Z",
        ]
    finally:
        await server.shutdown()


async def test_the_machine_ones_are_ahead_of_the_shipped_catalogue(tmp_path: Path) -> None:
    off = '[[machine.projection]]\nrecipe = "default"\ntype = "nav_msgs/msg/OccupancyGrid"\noverride = true\n'
    text = machine_file(component(message_type="nav_msgs/msg/OccupancyGrid"), machine=off)
    server = await serve(load_machine(write(tmp_path, text), runtime_factory=runtime_for))
    try:
        loaded: Any = server.machines[0].components["C"]  # pyright: ignore[reportAttributeAccessIssue]
        assert names_below(loaded) == ["M.Data", "M.Width"]
        assert (await rendering_of(server, loaded))["M.Data"].value_rank == 1
    finally:
        await server.shutdown()


# -- mistakes ---------------------------------------------------------------------------------------


def problems_of(tmp_path: Path, text: str) -> dict[str, str]:
    calls: list[ROS2Config] = []

    def factory(config: ROS2Config) -> FakeRuntime:
        calls.append(config)
        return runtime_for(config)

    with pytest.raises(FileDeclarationError) as raised:
        load_machine(write(tmp_path, text), runtime_factory=factory)
    assert calls == []
    return dict(raised.value.problems)


def entries(*tables: str, message_type: str = "fixture_msgs/msg/Pose") -> str:
    given = "".join(f"\n[[component.projection]]\n{each}\n" for each in tables)
    return component(given, message_type=message_type)


def test_an_unknown_recipe_is_reported_at_the_recipe(tmp_path: Path) -> None:
    found = problems_of(
        tmp_path,
        machine_file(entries('recipe = "drop"\ntype = "a/msg/B"\nfields = []', 'recipe = "curate"\ntype = "a/msg/B"')),
    )

    assert "unknown recipe 'curate'" in found["component[0].projection[1].recipe"]
    assert "blob, default, drop, enum, functions, scalar" in found["component[0].projection[1].recipe"]


def test_a_missing_recipe_is_reported(tmp_path: Path) -> None:
    found = problems_of(tmp_path, machine_file(entries('type = "a/msg/B"')))

    assert "missing" in found["component[0].projection[0].recipe"]


def test_a_key_the_recipe_does_not_take_is_reported_at_the_key(tmp_path: Path) -> None:
    found = problems_of(
        tmp_path, machine_file(entries('recipe = "drop"\ntype = "a/msg/B"\nfields = []\nfield = "data"'))
    )

    assert "unknown key; the drop recipe takes" in found["component[0].projection[0].field"]


def test_a_callable_is_named_by_an_import_path_and_not_by_a_bare_word(tmp_path: Path) -> None:
    found = problems_of(tmp_path, machine_file(entries('recipe = "blob"\ntype = "a/msg/B"\nfield = "d"\nencode = "x"')))

    assert "not an import path" in found["component[0].projection[0].encode"]


@pytest.mark.parametrize(
    ("table", "missing"),
    [
        ('recipe = "enum"\ntype = "a/msg/B"', "fields"),
        ('recipe = "drop"\ntype = "a/msg/B"', "fields"),
        ('recipe = "blob"\ntype = "a/msg/B"', "field"),
        ('recipe = "scalar"', "type"),
    ],
)
def test_a_missing_parameter_is_reported_at_its_key(tmp_path: Path, table: str, missing: str) -> None:
    found = problems_of(tmp_path, machine_file(entries(table)))

    assert "missing" in found[f"component[0].projection[0].{missing}"]


@pytest.mark.parametrize(
    ("table", "key"),
    [
        ('recipe = "scalar"\ntype = "a/msg/B"\noverride = "yes"', "override"),
        ('recipe = "scalar"\ntype = 5', "type"),
        ('recipe = "scalar"\ntype = "a/msg/B"\nvariant_type = "Nope"', "variant_type"),
        ('recipe = "drop"\ntype = "a/msg/B"\nfields = "pose"', "fields"),
        ('recipe = "enum"\ntype = "a/msg/B"\n[component.projection.fields]\nmode = 3', "fields"),
        ('recipe = "enum"\ntype = "a/msg/B"\n[component.projection.fields.mode]\nzero = "Idle"', "fields"),
        ('recipe = "blob"\ntype = "a/msg/B"\nfield = "d"\ndatatype = "ImagePNG"', "datatype"),
    ],
)
def test_a_value_of_the_wrong_type_is_reported_at_its_key(tmp_path: Path, table: str, key: str) -> None:
    found = problems_of(tmp_path, machine_file(entries(table)))

    assert f"component[0].projection[0].{key}" in found


def test_a_second_entry_for_one_type_is_reported_unless_it_overrides(tmp_path: Path) -> None:
    first = 'recipe = "drop"\ntype = "a/msg/B"\nfields = ["x"]'
    found = problems_of(tmp_path, machine_file(entries(first, first, first + "\noverride = true")))

    assert list(found) == ["component[0].projection[1].type"]
    assert "override=True" in found["component[0].projection[1].type"]


def test_an_entry_against_the_shipped_catalogue_needs_override_too(tmp_path: Path) -> None:
    found = problems_of(
        tmp_path, machine_file(entries('recipe = "drop"\ntype = "nav_msgs/msg/OccupancyGrid"\nfields = []'))
    )

    assert "Blob" in found["component[0].projection[0].type"]


def test_machine_entries_are_checked_like_a_components(tmp_path: Path) -> None:
    twice = '[[machine.projection]]\nrecipe = "drop"\ntype = "a/msg/B"\nfields = []\n' * 2
    unknown = '[[machine.projection]]\nrecipe = "curate"\ntype = "a/msg/B"\n'

    found = problems_of(tmp_path, machine_file(component(), machine=twice + unknown))

    assert sorted(found) == ["machine.projection[1].type", "machine.projection[2].recipe"]


def test_a_components_entry_against_the_machines_needs_override(tmp_path: Path) -> None:
    one = 'recipe = "drop"\ntype = "a/msg/B"\nfields = ["x"]'
    machine = "[[machine.projection]]\n" + one + "\n"

    found = problems_of(tmp_path, machine_file(entries(one), machine=machine))

    assert list(found) == ["component[0].projection[0].type"]


def test_the_mistakes_of_the_tables_come_with_the_other_mistakes_of_the_file(tmp_path: Path) -> None:
    text = machine_file(entries('recipe = "curate"\ntype = "a/msg/B"')).replace('name = "M"', 'nam = "m"', 1)

    found = problems_of(tmp_path, text)

    assert {"component[0].projection[0].recipe", "component[0].monitoring[0].nam"} <= set(found)


def test_a_projection_that_is_not_an_array_of_tables_is_reported(tmp_path: Path) -> None:
    text = machine_file(component().replace('name = "C"\n', 'name = "C"\nprojection = "x"\n', 1))

    assert "array of tables" in problems_of(tmp_path, text)["component[0].projection"]


# -- the callables a table names --------------------------------------------------------------------

FUNCTIONS = 'recipe = "functions"\ntype = "fixture_msgs/msg/Voltage"\n'
SHAPE = 'shaped_by = "tests.fixture_custom:voltage_shape"\n'
DECODE = 'decoded_by = "tests.fixture_custom:voltage_decode"\n'


def test_a_functions_table_needs_both_of_its_required_callables(tmp_path: Path) -> None:
    found = problems_of(tmp_path, machine_file(entries(FUNCTIONS + SHAPE)))

    assert set(found) == {"component[0].projection[0].decoded_by"}
    assert "missing" in found["component[0].projection[0].decoded_by"]


def test_a_callable_that_is_not_there_is_reported_at_its_key(tmp_path: Path) -> None:
    table = FUNCTIONS + SHAPE + 'decoded_by = "tests.fixture_custom:nothing"\nencode = "tests.nope:encode"\n'

    found = problems_of(tmp_path, machine_file(entries(table)))

    assert set(found) == {"component[0].projection[0].decoded_by", "component[0].projection[0].encode"}
    assert "has no attribute 'nothing'" in found["component[0].projection[0].decoded_by"]
    assert "cannot import module 'tests.nope'" in found["component[0].projection[0].encode"]


def test_something_that_cannot_be_called_is_reported_at_its_key(tmp_path: Path) -> None:
    table = FUNCTIONS + SHAPE + 'decoded_by = "tests.fixture_custom:NOT_A_CLASS"'

    found = problems_of(tmp_path, machine_file(entries(table)))

    assert "not callable" in found["component[0].projection[0].decoded_by"]


def test_a_second_functions_entry_for_one_type_is_a_duplicate_found_before_any_import(tmp_path: Path) -> None:
    both = FUNCTIONS + SHAPE + DECODE
    table = machine_file(entries(both, both))

    found = problems_of(tmp_path, table)

    assert set(found) == {"component[0].projection[1].type"}


async def test_a_blobs_encode_is_named_like_the_functions(tmp_path: Path) -> None:
    table = (
        'recipe = "blob"\ntype = "fixture_msgs/msg/Grid"\nfield = "data"\n'
        'encode = "tests.fixture_custom:voltage_decode"'
    )
    server = await serve(load_machine(write(tmp_path, machine_file(entries(table))), runtime_factory=runtime_for))
    try:
        loaded: Any = server.machines[0].components["C"]  # pyright: ignore[reportAttributeAccessIssue]
        assert loaded.projections[0].encode is fixture_custom.voltage_decode
    finally:
        await server.shutdown()
