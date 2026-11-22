# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The seam, checked over the syntax tree: only one module imports rclpy, and the fake covers the real
Runtime's surface."""

from __future__ import annotations

import ast
import inspect
from pathlib import Path

import pytest

import opensmi.ros2
import opensmi.ros2.runtime
from opensmi.ros2.runtime.interface import RuntimeInterface
from tests import fake_runtime

PACKAGE = Path(opensmi.ros2.__file__).resolve().parent
TESTS = Path(inspect.getfile(fake_runtime)).resolve().parent
# The only two test modules allowed to reach ROS2 (marked jazzy, deselected by default).
THE_JAZZY_TESTS = {
    "test_jazzy_environment.py",
    "test_runtime_jazzy.py",
    "test_performance_jazzy.py",
    "test_structured_skills_methods_jazzy.py",
}
# The one module allowed to import a ROS2 package.
THE_RCLPY_IMPORTER = PACKAGE / "runtime" / "runtime.py"
# Distributions from a sourced ROS2 install, not PyPI; none may be imported by a second module.
THE_ROS2_ROOTS = {"rclpy", "tf2_ros"}

# The levers the fake may have beyond the real Runtime.
FAKE_ONLY = {
    "deliver",
    "note_type_hash_mismatch",
    "note_publishers",
    "set_publisher_qos",
    "note_qos_incompatible",
    "set_graph",
    "serve_action",
    "deliver_feedback",
    "settle_goal",
    "cancel_requests",
    "sent_goals",
    "serve_service",
    "requests",
    "published",
    "refuse_publishing",
    "set_clock",
    "serve_parameters",
    "stop_parameters",
    "refuse_parameter",
    "refuse_parameter_value",
    "describe_parameter",
    "deliver_parameter_event",
    "hold_parameter_writes",
    "parameter_reads",
    "parameter_writes",
    "serve_transform",
    "subscribed_topics",
    "advertised_topics",
}


def imported_roots(source: Path) -> set[str]:
    """Return the top-level package name of every module a file imports."""
    roots: set[str] = set()
    for node in ast.walk(ast.parse(source.read_text(encoding="utf-8"), filename=str(source))):
        if isinstance(node, ast.Import):
            roots.update(alias.name.partition(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and not node.level and node.module:
            roots.add(node.module.partition(".")[0])
    return roots


def members(source: Path, class_name: str) -> dict[str, str]:
    """Return each public member of a class as a name and a rendered signature."""
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    found = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef) or node.name != class_name:
            continue
        for member in node.body:
            if not isinstance(member, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            if member.name.startswith("_") and member.name not in {"__init__", "__repr__"}:
                continue
            decorators = ",".join(sorted(ast.unparse(each) for each in member.decorator_list))
            returns = ast.unparse(member.returns) if member.returns else ""
            found[member.name] = f"{decorators}|{ast.unparse(member.args)}|{returns}"
    return found


@pytest.mark.parametrize("root", sorted(THE_ROS2_ROOTS))
def test_exactly_one_module_in_the_package_imports_a_ros2_package(root) -> None:
    importers = sorted(
        source.relative_to(PACKAGE).as_posix() for source in PACKAGE.rglob("*.py") if root in imported_roots(source)
    )

    assert importers == [THE_RCLPY_IMPORTER.relative_to(PACKAGE).as_posix()]


def test_the_runtime_half_imports_without_a_ros2_installation() -> None:
    assert not THE_ROS2_ROOTS & imported_roots(PACKAGE / "runtime" / "__init__.py")
    assert "Runtime" in opensmi.ros2.runtime.__all__


# __init__ is compared separately: the fake's adds a lever, so it is a superset.
@pytest.mark.parametrize("member", sorted(set(members(THE_RCLPY_IMPORTER, "Runtime")) - {"__init__"}))
def test_the_fake_runtime_covers_the_real_surface(member) -> None:
    real = members(THE_RCLPY_IMPORTER, "Runtime")
    fake = members(Path(inspect.getfile(fake_runtime)), "FakeRuntime")

    assert member in fake, f"the fake Runtime is missing {member}"
    assert fake[member] == real[member], f"the fake Runtime's {member} does not match the real one"


def test_the_fakes_constructor_accepts_everything_the_real_one_does() -> None:
    signature = inspect.signature(fake_runtime.FakeRuntime.__init__)
    real = ast.parse(THE_RCLPY_IMPORTER.read_text(encoding="utf-8"))
    # The Runtime's own __init__; the module also holds a private helper with a second one.
    (runtime,) = [node for node in ast.walk(real) if isinstance(node, ast.ClassDef) and node.name == "Runtime"]
    (init,) = [
        node
        for node in runtime.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == "__init__"
    ]

    expected = {argument.arg for argument in init.args.kwonlyargs}
    assert expected <= set(signature.parameters)


def test_the_fake_runtime_has_no_surface_of_its_own_beyond_its_levers() -> None:
    real = set(members(THE_RCLPY_IMPORTER, "Runtime"))
    fake = set(members(Path(inspect.getfile(fake_runtime)), "FakeRuntime"))

    assert fake - real <= FAKE_ONLY


def test_no_test_but_the_jazzy_ones_imports_a_ros2_package() -> None:
    importers = {
        source.name for source in TESTS.glob("*.py") if THE_ROS2_ROOTS & imported_roots(source)
    } - THE_JAZZY_TESTS

    assert importers == set()


def test_the_fake_runtime_satisfies_the_runtime_interface() -> None:
    runtime: RuntimeInterface = fake_runtime.FakeRuntime()

    assert isinstance(runtime, RuntimeInterface)


def test_the_fake_runtime_is_not_part_of_the_distribution() -> None:
    # Packaging only finds src/, which keeps consumers from importing the double.
    source = Path(inspect.getfile(fake_runtime)).resolve()

    assert PACKAGE not in source.parents
    assert source.parent.name == "tests"
