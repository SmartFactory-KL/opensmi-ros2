# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The runtime half may import only the standard library, rclpy, tf2_ros, structlog, typing-extensions
and itself.

A static check parses every runtime module; a dynamic one imports them in a fresh interpreter and
inspects `sys.modules`. Each has a positive control.
"""

from __future__ import annotations

import ast
import json
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

import opensmi.ros2.runtime

RUNTIME = "opensmi.ros2.runtime"
# rclpy and tf2_ros come from the sourced ROS2 installation; interface packages load by type name.
ALLOWED_THIRD_PARTY = frozenset({"rclpy", "structlog", "tf2_ros", "typing_extensions"})


def _runtime_dir() -> Path:
    paths = list(opensmi.ros2.runtime.__path__)
    assert len(paths) == 1, paths
    return Path(paths[0])


def _runtime_modules() -> dict[str, Path]:
    """Map the dotted name of every module in the runtime package to its file."""
    root = _runtime_dir()
    modules: dict[str, Path] = {}
    for path in sorted(root.rglob("*.py")):
        parts = [RUNTIME, *path.relative_to(root).with_suffix("").parts]
        if parts[-1] == "__init__":
            parts.pop()
        modules[".".join(parts)] = path
    return modules


def _in_runtime(name: str) -> bool:
    return name == RUNTIME or name.startswith(RUNTIME + ".")


def _is_allowed(name: str) -> bool:
    top = name.partition(".")[0]
    return top in sys.stdlib_module_names or top in ALLOWED_THIRD_PARTY or _in_runtime(name)


def forbidden_imports(source: str, module: str, *, is_package: bool) -> list[str]:
    """Return every import in ``source`` that the runtime half may not make.

    ``module`` is the dotted name of the module the source belongs to; it resolves relative imports.
    """
    package = module if is_package else module.rpartition(".")[0]
    found: list[str] = []
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            targets = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                parts = package.split(".")
                keep = len(parts) - (node.level - 1)
                base = ".".join([*parts[: max(keep, 0)], *([node.module] if node.module else [])])
            else:
                base = node.module or ""
            # `from opensmi.ros2 import server` imports a module named by the alias.
            if not base or RUNTIME.startswith(base + "."):
                targets = [f"{base}.{alias.name}" for alias in node.names]
            else:
                targets = [base]
        else:
            continue
        found.extend(f"line {node.lineno}: {target}" for target in targets if not _is_allowed(target))
    return found


def test_the_runtime_package_is_found() -> None:
    assert RUNTIME in _runtime_modules()


def test_runtime_modules_import_only_the_standard_library_ros2_structlog_and_typing_extensions() -> None:
    violations = {
        name: problems
        for name, path in _runtime_modules().items()
        if (
            problems := forbidden_imports(path.read_text(encoding="utf-8"), name, is_package=path.name == "__init__.py")
        )
    }

    assert violations == {}


@pytest.mark.parametrize(
    "source",
    [
        "import asyncua",
        "import asyncua.ua as ua",
        "from asyncua import ua",
        "import opensmi.server",
        "from opensmi.core import x",
        "from opensmi import server",
        "from opensmi.ros2 import server",
        "from opensmi.ros2.server import component",
        "from ..server import component",
        "from .. import server",
        "import numpy",
        "def f():\n    import asyncua",
        "from typing import TYPE_CHECKING\nif TYPE_CHECKING:\n    from opensmi.server import x",
    ],
)
def test_the_static_check_reports_a_forbidden_import(source: str) -> None:
    assert forbidden_imports(source, f"{RUNTIME}.module", is_package=False)


@pytest.mark.parametrize(
    "source",
    [
        "from __future__ import annotations",
        "import asyncio",
        "import collections.abc",
        "from dataclasses import dataclass",
        "import rclpy",
        "from rclpy.node import Node",
        "import tf2_ros",
        "import structlog",
        "from typing_extensions import override",
        "from . import sibling",
        "from .sibling import name",
        "from .sub.module import name",
        "from opensmi.ros2.runtime import name",
        "from opensmi.ros2 import runtime",
    ],
)
def test_the_static_check_allows_a_permitted_import(source: str) -> None:
    assert forbidden_imports(source, f"{RUNTIME}.module", is_package=False) == []


def test_relative_imports_resolve_from_a_package_init() -> None:
    assert forbidden_imports("from . import sub", RUNTIME, is_package=True) == []
    assert forbidden_imports("from .. import server", RUNTIME, is_package=True)


# Imports each module named on the command line and prints sys.modules; a missing rclpy is reported as skipped.
IMPORT_SCRIPT = textwrap.dedent("""\
    import importlib, json, sys

    skipped = []
    for name in sys.argv[1:]:
        try:
            importlib.import_module(name)
        except ModuleNotFoundError as error:
            if (error.name or "").partition(".")[0] != "rclpy":
                raise
            skipped.append(name)
    print(json.dumps({"modules": sorted(sys.modules), "skipped": skipped}))
""")


def _import_in_fresh_interpreter(names: list[str], cwd: Path) -> dict[str, list[str]]:
    completed = subprocess.run(
        [sys.executable, "-c", IMPORT_SCRIPT, *names],
        capture_output=True,
        text=True,
        cwd=cwd,
        check=False,
        timeout=120,
    )
    assert completed.returncode == 0, completed.stderr
    return json.loads(completed.stdout)


def _leaked(modules: list[str]) -> list[str]:
    return [
        name
        for name in modules
        if name == "asyncua"
        or name.startswith("asyncua.")
        or (name.startswith("opensmi.") and name != "opensmi.ros2" and not _in_runtime(name))
    ]


def test_importing_the_runtime_loads_neither_opensmi_nor_asyncua(tmp_path: Path) -> None:
    result = _import_in_fresh_interpreter(list(_runtime_modules()), tmp_path)

    assert RUNTIME in result["modules"]
    assert RUNTIME not in result["skipped"]
    assert _leaked(result["modules"]) == []


def test_the_dynamic_check_sees_opensmi_and_asyncua(tmp_path: Path) -> None:
    result = _import_in_fresh_interpreter([RUNTIME, "opensmi.core", "asyncua"], tmp_path)

    leaked = _leaked(result["modules"])
    assert "opensmi.core" in leaked
    assert "asyncua" in leaked


def test_opensmi_server_imports_next_to_opensmi_ros2(tmp_path: Path) -> None:
    result = _import_in_fresh_interpreter(["opensmi.ros2", RUNTIME, "opensmi.server"], tmp_path)

    assert "opensmi.server" in _leaked(result["modules"])
    assert "opensmi.ros2" in result["modules"]
