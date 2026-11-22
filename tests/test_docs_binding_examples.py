# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The examples of docs/bindings.md load with the project's own loader and constructors.

Every ``toml`` block of the page is preceded by one marker line, an HTML comment:

* ``toml: component``: complete once a stock Machine and Component header is put in front of it. It is
  loaded as a machine file, and again as a Binding file (the ``component.`` prefix removed).
* ``toml: machine``: a complete machine file.
* ``toml: bindings``: a Binding file, as ``bindings_toml`` points to.
* ``toml: fragment``: only has to be valid TOML.

Every ``python`` block runs as it stands. In the section of one Binding kind, the first ``component``
block and the first ``python`` block declare the same Binding: the variable ``binding`` of the Python
block equals what the TOML block loads. A Python block marked ``python: only`` runs and is not compared.
"""

from __future__ import annotations

import re
import tomllib
from pathlib import Path
from typing import TYPE_CHECKING, NamedTuple

import pytest

from opensmi.ros2.server import load_machine
from opensmi.ros2.server.bindings import TOML_KINDS, load_bindings
from tests.fake_runtime import FakeRuntime

if TYPE_CHECKING:
    from opensmi.ros2.server.config import ROS2Config

PAGE = Path(__file__).resolve().parent.parent / "docs" / "bindings.md"

HEADER = """\
[machine]
name = "Cell"

[machine.identification]
serial_number = "cell-0001"
manufacturer = "Example Manufacturer"
product_instance_uri = "urn:example:cell"

[[component]]
name = "Base"

[component.identification]
serial_number = "base-0001"
manufacturer = "Example Manufacturer"

"""

_BLOCK = re.compile(
    r"^(?:<!-- (?P<marker>(?:toml|python): \w+) -->\n)?```(?P<language>toml|python)\n(?P<body>.*?)^```$",
    re.DOTALL | re.MULTILINE,
)


class Block(NamedTuple):
    """One fenced block of the page."""

    section: str
    language: str
    marker: str | None
    body: str


def blocks() -> list[Block]:
    """Return every ``toml`` and ``python`` block of the page, with the section it sits in."""
    found: list[Block] = []
    text = PAGE.read_text(encoding="utf-8")
    for section in re.split(r"^## ", text, flags=re.MULTILINE)[1:]:
        title = section.partition("\n")[0].strip()
        found.extend(
            Block(title, match["language"], match["marker"], match["body"]) for match in _BLOCK.finditer(section)
        )
    return found


def of(language: str, marker: str | None = None) -> list[Block]:
    """Return the blocks of one language, optionally of one marker."""
    return [each for each in blocks() if each.language == language and (marker is None or each.marker == marker)]


def factory(_config: ROS2Config) -> FakeRuntime:
    """Return the Runtime double, so loading needs no ROS 2."""
    return FakeRuntime()


def strip_component_prefix(text: str) -> str:
    """Return a ``component`` snippet as the Binding file the Component could include instead."""
    return re.sub(r"^(\[\[?)component\.", r"\1", text, flags=re.MULTILINE)


def test_the_page_has_examples() -> None:
    assert len(of("toml")) >= 10
    assert len(of("python")) >= 8


def test_every_toml_block_has_a_known_marker() -> None:
    markers = {each.marker for each in of("toml")}

    assert markers <= {"toml: component", "toml: machine", "toml: bindings", "toml: fragment"}
    assert None not in markers


def test_every_python_block_is_unmarked_or_marked_only() -> None:
    assert {each.marker for each in of("python")} <= {None, "python: only"}


def test_every_binding_kind_has_a_section_with_both_spellings() -> None:
    sections = {each.section for each in of("toml", "toml: component")} & {each.section for each in of("python")}

    assert sections >= {
        "MonitoringBinding",
        "KeyedBinding",
        "MethodBinding",
        "ActionBinding",
        "PublisherBinding",
        "LifecycleBinding",
        "ROS2ParameterBinding",
        "TFBinding",
    }


def test_the_toml_keys_of_the_page_are_the_loader_keys() -> None:
    text = PAGE.read_text(encoding="utf-8")

    for key in TOML_KINDS:
        assert f"`{key}`" in text


@pytest.mark.parametrize("block", of("toml"), ids=lambda each: f"{each.section}-{each.marker}")
def test_a_toml_block_is_valid_toml(block: Block) -> None:
    text = HEADER + block.body if block.marker == "toml: component" else block.body

    tomllib.loads(text)


@pytest.mark.parametrize("block", of("toml", "toml: component"), ids=lambda each: each.section)
def test_a_component_block_loads_in_a_machine_file(block: Block, tmp_path: Path) -> None:
    path = tmp_path / "machine.toml"
    path.write_text(HEADER + block.body, encoding="utf-8")

    load_machine(path, runtime_factory=factory)


@pytest.mark.parametrize("block", of("toml", "toml: component"), ids=lambda each: each.section)
def test_a_component_block_loads_as_a_binding_file(block: Block, tmp_path: Path) -> None:
    path = tmp_path / "bindings.toml"
    path.write_text(strip_component_prefix(block.body), encoding="utf-8")

    assert load_bindings(path)


@pytest.mark.parametrize("block", of("toml", "toml: machine"), ids=lambda each: each.section)
def test_a_machine_block_loads(block: Block, tmp_path: Path) -> None:
    path = tmp_path / "machine.toml"
    path.write_text(block.body, encoding="utf-8")

    machine = load_machine(path, runtime_factory=factory)

    assert machine.ros2_bindings


@pytest.mark.parametrize("block", of("toml", "toml: bindings"), ids=lambda each: each.section)
def test_a_bindings_block_loads(block: Block, tmp_path: Path) -> None:
    path = tmp_path / "bindings.toml"
    path.write_text(block.body, encoding="utf-8")

    assert load_bindings(path)


@pytest.mark.parametrize("block", of("python"), ids=lambda each: f"{each.section}-{each.marker}")
def test_a_python_block_runs(block: Block) -> None:
    exec(compile(block.body, f"docs/bindings.md ({block.section})", "exec"), {"__name__": "docs_example"})


@pytest.mark.parametrize("kind", sorted({each.section for each in of("python", None)}))
def test_the_python_example_declares_what_the_toml_example_does(kind: str, tmp_path: Path) -> None:
    toml = next(each for each in of("toml", "toml: component") if each.section == kind)
    python = next(each for each in of("python", None) if each.section == kind)
    namespace: dict[str, object] = {"__name__": "docs_example"}
    exec(compile(python.body, f"docs/bindings.md ({kind})", "exec"), namespace)
    path = tmp_path / "bindings.toml"
    path.write_text(strip_component_prefix(toml.body), encoding="utf-8")

    assert load_bindings(path) == (namespace["binding"],)
