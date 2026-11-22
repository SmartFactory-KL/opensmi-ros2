# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A server comes up from one TOML file, and a file with mistakes is refused whole with every error
reported."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua

from opensmi.ros2.server import ConfiguredMachine, load_machine
from opensmi.ros2.server.errors import DeclarationError, FileDeclarationError
from tests import fixture_example
from tests.fake_runtime import FakeRuntime
from tests.served import connected, serve

if TYPE_CHECKING:
    from collections.abc import AsyncIterator
    from pathlib import Path

    from asyncua.client.client import Client
    from asyncua.common.node import Node
    from opensmi.server import Server

    from opensmi.ros2.server.config import ROS2Config

MACHINE = """
[machine]
name = "Cell"

[machine.identification]
serial_number = "cell-1"
manufacturer = "Acme"
product_instance_uri = "urn:acme:cell"

[ros2]
domain_id = 7
node_name = "cell_node"

[[component]]
name = "Base"

[component.identification]
serial_number = "base-1"
manufacturer = "Acme"

[[component.monitoring]]
name = "Odom"
path = "/odom"
message_type = "nav_msgs/msg/Odometry"

[[component]]
name = "Arm"
parent = "Base"

[component.identification]
serial_number = "arm-1"
manufacturer = "Acme"

[[component.monitoring]]
name = "State"
path = "/arm/state"
"""


IDENTIFIED = """
[component.identification]
serial_number = "x"
manufacturer = "Acme"
"""


def write(directory: Path, text: str, name: str = "machine.toml") -> Path:
    path = directory / name
    path.write_text(text, encoding="utf-8")
    return path


class Factory:
    """A Runtime factory that remembers what it was asked for, and whether it was asked at all."""

    def __init__(self) -> None:
        self.configs: list[ROS2Config] = []
        self.runtime = FakeRuntime(domain_id=7, message_types=fixture_example.MESSAGE_TYPES)

    def __call__(self, config: ROS2Config) -> FakeRuntime:
        self.configs.append(config)
        return self.runtime


async def names_of(node: Node) -> list[str]:
    return [(await each.read_browse_name()).Name for each in await node.get_children()]


async def below(node: Node, *names: str) -> Node:
    for name in names:
        for each in await node.get_children():
            if (await each.read_browse_name()).Name == name:
                node = each
                break
        else:
            pytest.fail(f"{name!r} is not below {node}; there are {await names_of(node)}")
    return node


async def read(node: Node) -> object:
    value = await node.read_value()
    return value.Text if isinstance(value, ua.LocalizedText) else value


# -- the file starts a server -------------------------------------------------------------------


@pytest.fixture
def factory() -> Factory:
    return Factory()


@pytest.fixture
async def served(tmp_path: Path, factory: Factory) -> AsyncIterator[Server]:
    machine = load_machine(write(tmp_path, MACHINE), runtime_factory=factory)
    server = await serve(machine)
    try:
        yield server
    finally:
        await server.shutdown()


async def cell_of(client: Client) -> Node:
    return await below(client.nodes.objects, "Machines", "Cell")


async def test_the_runtime_is_made_from_the_files_settings(served: Server, factory: Factory) -> None:
    assert [(each.domain_id, each.node_name) for each in factory.configs] == [(7, "cell_node")]
    assert served.machines[0].runtime is factory.runtime  # pyright: ignore[reportAttributeAccessIssue]


async def test_a_client_sees_the_declared_machine_and_its_identification(served: Server) -> None:
    async with connected(served) as client:
        cell = await cell_of(client)
        identification = await below(cell, "Identification")

        assert await read(await below(identification, "SerialNumber")) == "cell-1"
        assert await read(await below(identification, "Manufacturer")) == "Acme"
        assert await read(await below(identification, "ProductInstanceUri")) == "urn:acme:cell"


async def test_a_component_without_a_parent_sits_under_the_machine_after_the_introspection_component(
    served: Server,
) -> None:
    async with connected(served) as client:
        components = await below(await cell_of(client), "Components")

        assert await names_of(components) == ["ROS2Introspection", "Base"]


async def test_a_component_with_a_parent_sits_under_it(served: Server) -> None:
    async with connected(served) as client:
        base = await below(await cell_of(client), "Components", "Base")

        assert await names_of(await below(base, "Components")) == ["Arm"]
        arm_identification = await below(base, "Components", "Arm", "Identification")
        assert await read(await below(arm_identification, "SerialNumber")) == "arm-1"


async def test_bindings_under_a_component_become_its_nodes(served: Server) -> None:
    async with connected(served) as client:
        base = await below(await cell_of(client), "Components", "Base")
        arm = await below(base, "Components", "Arm")

        assert "Odom" in await names_of(await below(base, "Monitoring"))
        assert "Pose" in await names_of(await below(base, "Monitoring", "Odom"))
        assert "State" not in await names_of(await below(base, "Monitoring"))
        assert await names_of(await below(arm, "Monitoring")) == []


async def test_the_machine_is_a_configured_machine(tmp_path: Path, factory: Factory) -> None:
    machine = load_machine(write(tmp_path, MACHINE), runtime_factory=factory)

    assert isinstance(machine, ConfiguredMachine)
    assert machine.name == "Cell"
    assert machine.ros2_config.domain_id == 7


async def test_an_included_binding_file_is_read_relative_to_the_machine_file(tmp_path: Path, factory: Factory) -> None:
    write(
        tmp_path,
        '[[monitoring]]\nname = "Scan"\npath = "/scan"\nmessage_type = "geometry_msgs/msg/Twist"\n',
        "bindings.toml",
    )
    text = MACHINE.replace('name = "Base"\n', 'name = "Base"\nbindings_toml = "bindings.toml"\n', 1)
    server = await serve(load_machine(write(tmp_path, text), runtime_factory=factory))
    try:
        base: Any = server.machines[0].components["Base"]  # pyright: ignore[reportAttributeAccessIssue]
        assert sorted(each.name for each in base.ros2_bindings) == ["Odom", "Scan"]
    finally:
        await server.shutdown()


async def test_a_machine_level_tf_binding_is_declared_like_a_components(tmp_path: Path, factory: Factory) -> None:
    text = MACHINE.replace("[ros2]", '[[machine.tf]]\nname = "frames"\nhead = "base_link"\n\n[ros2]', 1)

    machine = load_machine(write(tmp_path, text), runtime_factory=factory)

    assert [each.name for each in machine.ros2_bindings] == ["frames"]


# -- a file with mistakes is refused whole --------------------------------------------------------


def problems_of(tmp_path: Path, text: str) -> dict[str, str]:
    """Load a file that must fail, check nothing was built, and return its problems by path."""
    factory = Factory()
    with pytest.raises(FileDeclarationError) as raised:
        load_machine(write(tmp_path, text), runtime_factory=factory)
    assert factory.configs == []
    return dict(raised.value.problems)


def test_a_file_error_is_a_declaration_error(tmp_path: Path) -> None:
    with pytest.raises(DeclarationError):
        load_machine(write(tmp_path, "[machine]\n"), runtime_factory=Factory())


def test_two_mistakes_in_one_file_are_both_reported_with_their_paths(tmp_path: Path) -> None:
    text = MACHINE.replace('message_type = "nav_msgs/msg/Odometry"', "message_type = 5").replace(
        'path = "/arm/state"', 'path = "/arm/state"\ncolour = "red"'
    )

    problems = problems_of(tmp_path, text)

    assert set(problems) == {"component[0].monitoring[0].message_type", "component[1].monitoring[0].colour"}
    assert "expected str" in problems["component[0].monitoring[0].message_type"]


def test_the_error_message_lists_every_problem(tmp_path: Path) -> None:
    text = MACHINE.replace('parent = "Base"', 'parent = "Nobody"').replace("domain_id = 7", "domain_id = true")
    with pytest.raises(FileDeclarationError) as raised:
        load_machine(write(tmp_path, text), runtime_factory=Factory())

    shown = str(raised.value)
    assert "2 problems" in shown
    assert "component[1].parent" in shown
    assert "ros2.domain_id" in shown


def test_an_unknown_key_is_an_error_at_the_key(tmp_path: Path) -> None:
    text = MACHINE.replace('name = "Base"', 'name = "Base"\nflavour = "x"').replace(
        'name = "Cell"', 'name = "Cell"\ncolour = "red"'
    )

    problems = problems_of(tmp_path, text)

    assert set(problems) == {"component[0].flavour", "machine.colour"}
    assert "unknown key" in problems["machine.colour"]


def test_an_unknown_top_level_table_and_an_unknown_ros2_key_are_errors(tmp_path: Path) -> None:
    text = MACHINE.replace("[ros2]", "[bogus]\nconfig = 'x'\n\n[ros2]\nfoo = 1", 1)

    assert set(problems_of(tmp_path, text)) == {"bogus", "ros2.foo"}


def test_a_duplicate_component_name_is_an_error_at_the_second(tmp_path: Path) -> None:
    problems = problems_of(tmp_path, MACHINE.replace('name = "Arm"\nparent = "Base"', 'name = "Base"'))

    assert list(problems) == ["component[1].name"]
    assert "component[0]" in problems["component[1].name"]


def test_an_unknown_parent_is_an_error(tmp_path: Path) -> None:
    problems = problems_of(tmp_path, MACHINE.replace('parent = "Base"', 'parent = "Elsewhere"'))

    assert list(problems) == ["component[1].parent"]


def test_a_parent_declared_after_its_child_is_an_error(tmp_path: Path) -> None:
    text = MACHINE + '\n[[component]]\nname = "Late"\n' + IDENTIFIED
    text = text.replace('parent = "Base"', 'parent = "Late"')

    problems = problems_of(tmp_path, text)

    assert list(problems) == ["component[1].parent"]
    assert "after its child" in problems["component[1].parent"]


def test_a_component_cannot_be_its_own_parent(tmp_path: Path) -> None:
    assert list(problems_of(tmp_path, MACHINE.replace('parent = "Base"', 'parent = "Arm"'))) == ["component[1].parent"]


def test_the_introspection_component_name_is_taken(tmp_path: Path) -> None:
    problems = problems_of(
        tmp_path,
        MACHINE.replace('name = "Base"', 'name = "ROS2Introspection"').replace(
            'parent = "Base"', 'parent = "ROS2Introspection"'
        ),
    )

    assert list(problems) == ["component[0].name"]


def test_a_missing_machine_name_and_identification_are_reported_together(tmp_path: Path) -> None:
    text = MACHINE.replace('name = "Cell"\n', "").replace('serial_number = "cell-1"\n', "")

    assert set(problems_of(tmp_path, text)) == {"machine.name", "machine.identification.serial_number"}


def test_a_missing_machine_table_is_an_error(tmp_path: Path) -> None:
    assert "machine" in problems_of(tmp_path, '[[component]]\nname = "A"\n')


def test_a_component_without_a_name_is_an_error(tmp_path: Path) -> None:
    assert list(problems_of(tmp_path, MACHINE + "\n[[component]]\n" + IDENTIFIED)) == ["component[2].name"]


def test_a_binding_value_the_declaration_refuses_is_an_error_at_the_binding(tmp_path: Path) -> None:
    text = MACHINE.replace('path = "/odom"', 'path = "/odom"\nmax_rate = -1.0')

    problems = problems_of(tmp_path, text)

    assert list(problems) == ["component[0].monitoring[0]"]
    assert "max_rate" in problems["component[0].monitoring[0]"]


def publishing(keys: str, *, continuous: str = "true") -> str:
    """Return MACHINE with a publishing Binding on its first Component, carrying extra keys."""
    table = (
        f'\n[[component.publisher]]\nname = "Drive"\npath = "/cmd_vel"\n'
        f'message_type = "geometry_msgs/msg/Twist"\ncontinuous = {continuous}\n{keys}\n'
    )
    return MACHINE.replace(
        'message_type = "nav_msgs/msg/Odometry"\n', f'message_type = "nav_msgs/msg/Odometry"\n{table}'
    )


@pytest.mark.parametrize("seconds", ["0", "-0.5", "inf"])
def test_a_stale_after_that_is_not_a_positive_number_of_seconds_is_an_error_at_the_binding(
    tmp_path: Path, seconds: str
) -> None:
    problems = problems_of(tmp_path, publishing(f"rate = 20.0\nstale_after = {seconds}"))

    assert list(problems) == ["component[0].publisher[0]"]
    assert "stale_after" in problems["component[0].publisher[0]"]


def test_a_stale_after_on_a_binding_without_a_rate_is_an_error_at_the_binding(tmp_path: Path) -> None:
    problems = problems_of(tmp_path, publishing("stale_after = 0.5", continuous="false"))

    assert list(problems) == ["component[0].publisher[0]"]
    assert "stale_after" in problems["component[0].publisher[0]"]


def test_a_stale_after_that_is_not_a_number_is_an_error_at_its_field(tmp_path: Path) -> None:
    problems = problems_of(tmp_path, publishing('rate = 20.0\nstale_after = "soon"'))

    assert list(problems) == ["component[0].publisher[0].stale_after"]


def test_a_halt_message_on_a_binding_without_a_rate_is_an_error_at_the_binding(tmp_path: Path) -> None:
    problems = problems_of(tmp_path, publishing("halt_message = { linear = { x = 0.0 } }", continuous="false"))

    assert list(problems) == ["component[0].publisher[0]"]
    assert "halt_message" in problems["component[0].publisher[0]"]


def test_a_halt_message_that_is_not_a_table_is_an_error_at_its_field(tmp_path: Path) -> None:
    problems = problems_of(tmp_path, publishing("rate = 20.0\nhalt_message = 0.0"))

    assert list(problems) == ["component[0].publisher[0].halt_message"]


def test_an_enum_value_that_does_not_exist_is_an_error_at_its_field(tmp_path: Path) -> None:
    text = MACHINE.replace(
        'message_type = "nav_msgs/msg/Odometry"',
        'message_type = "nav_msgs/msg/Odometry"\n\n[component.monitoring.qos]\nreliability = "maybe"',
    )

    assert list(problems_of(tmp_path, text)) == ["component[0].monitoring[0].qos.reliability"]


def test_a_binding_name_used_twice_on_a_component_is_an_error_at_the_component(tmp_path: Path) -> None:
    text = MACHINE.replace(
        'name = "State"\npath = "/arm/state"',
        'name = "State"\npath = "/arm/state"\n\n[[component.monitoring]]\nname = "State"\npath = "/x"',
    )

    assert list(problems_of(tmp_path, text)) == ["component[1]"]


def test_a_kind_table_that_is_not_an_array_is_an_error(tmp_path: Path) -> None:
    text = MACHINE.replace('[[component.monitoring]]\nname = "State"', '[component.method]\nname = "State"')

    assert list(problems_of(tmp_path, text)) == ["component[1].method"]


def test_a_missing_include_is_an_error_at_the_key(tmp_path: Path) -> None:
    text = MACHINE.replace('name = "Base"', 'name = "Base"\nbindings_toml = "nope.toml"', 1)

    assert list(problems_of(tmp_path, text)) == ["component[0].bindings_toml"]


def test_a_machine_level_binding_of_another_kind_is_an_error(tmp_path: Path) -> None:
    text = MACHINE.replace("[ros2]", '[[machine.monitoring]]\nname = "x"\npath = "/x"\n\n[ros2]', 1)

    assert list(problems_of(tmp_path, text)) == ["machine.monitoring"]


def test_toml_that_does_not_parse_is_one_problem_for_the_file(tmp_path: Path) -> None:
    problems = problems_of(tmp_path, "[machine\n")

    assert list(problems) == [""]


def test_a_file_that_is_not_there_is_an_os_error(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        load_machine(tmp_path / "missing.toml", runtime_factory=Factory())


def test_a_product_uri_is_a_machines_and_not_a_components(tmp_path: Path) -> None:
    text = MACHINE.replace('serial_number = "base-1"', 'serial_number = "base-1"\nproduct_instance_uri = "urn:x"')

    assert list(problems_of(tmp_path, text)) == ["component[0].identification.product_instance_uri"]


def test_a_component_without_identification_is_told_what_is_missing(tmp_path: Path) -> None:
    text = MACHINE.replace('[component.identification]\nserial_number = "base-1"\nmanufacturer = "Acme"\n', "")

    assert set(problems_of(tmp_path, text)) == {
        "component[0].identification.serial_number",
        "component[0].identification.manufacturer",
    }


def test_a_binding_name_that_breaks_the_rule_is_an_error_at_its_name_with_the_suggestion(tmp_path: Path) -> None:
    text = MACHINE.replace('name = "Odom"', 'name = "joint_states"')

    problems = problems_of(tmp_path, text)

    assert "Name 'joint_states' breaks the naming rule; try 'JointStates'" in problems["component[0].monitoring[0]"]


def test_a_group_outside_the_machinery_four_is_an_error_at_its_binding(tmp_path: Path) -> None:
    text = MACHINE.replace('path = "/odom"', 'path = "/odom"\ngroup = "Energy"')

    problems = problems_of(tmp_path, text)

    assert set(problems) == {"component[0].monitoring[0]"}
    assert "names the group 'Energy'" in problems["component[0].monitoring[0]"]
    assert "Status, Health, Process, Consumption" in problems["component[0].monitoring[0]"]


def test_a_group_on_a_skill_is_an_unknown_key(tmp_path: Path) -> None:
    text = MACHINE.replace(
        '[[component]]\nname = "Arm"',
        '[[component.action]]\nname = "Navigate"\npath = "/navigate"\naction_type = "nav2_msgs/action/NavigateToPose"\n'
        'group = "Process"\n\n[[component]]\nname = "Arm"',
    )

    problems = problems_of(tmp_path, text)

    assert set(problems) == {"component[0].action[0].group"}
    assert "unknown key" in problems["component[0].action[0].group"]
