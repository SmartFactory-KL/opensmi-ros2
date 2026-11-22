# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The exact `__all__` of each package, and that every public name imports and is documented where it
is defined.

Changing the surface means changing the expected lists below in the same commit (CONTRIBUTING.md).
"""

from __future__ import annotations

import ast
import importlib
import importlib.util
import inspect
from pathlib import Path

import pytest

import opensmi.ros2.runtime
import opensmi.ros2.server
import opensmi.ros2.server.errors
from opensmi.ros2.server.errors import DeclarationError

SERVER = frozenset(
    {
        "ActionBinding",
        "Binding",
        "BindingQoS",
        "ConfiguredComponent",
        "ConfiguredMachine",
        "DRIVERS",
        "DecodeFunction",
        "EncodeFunction",
        "EnumDataType",
        "GoalFunction",
        "KeyedBinding",
        "LISTING_ACCESS_LEVEL",
        "LifecycleBinding",
        "LifecycleDriver",
        "LifecycleNodeState",
        "LifecycleNodeTransition",
        "MUTATOR_ACCESS_LEVEL",
        "MessageFunction",
        "MethodBinding",
        "MonitoringBinding",
        "Nav2ManagerDriver",
        "NodeTransition",
        "PlanDataType",
        "PlanEntry",
        "PlanObject",
        "PlanVariable",
        "ProjectDataType",
        "Projection",
        "ProjectionContext",
        "ProjectionRegistry",
        "PublishedFrame",
        "PublisherBinding",
        "ROS2ComponentMixin",
        "ROS2Config",
        "ROS2MachineMixin",
        "ROS2ParameterBinding",
        "RequestFunction",
        "ResponseFunction",
        "ResultFunction",
        "SKILL_ACCESS_LEVEL",
        "Severity",
        "ShapeFunction",
        "SmartFactoryROS2NodeIds",
        "StandardDataType",
        "StandardLifecycleDriver",
        "TFBinding",
        "TOML_KINDS",
        "TransformConfig",
        "load_bindings",
        "load_machine",
        "recipes",
        "resolve_bindings",
    }
)
RUNTIME = frozenset(
    {
        "Durability",
        "Liveliness",
        "Message",
        "ParameterType",
        "Provenance",
        "QoSIncompatibility",
        "QoSProfile",
        "Reliability",
        "Runtime",
        "RuntimeInterface",
        "ServiceTypes",
        "Subscription",
        "TypeHashMismatch",
    }
)
SERVER_ERRORS = frozenset(
    {
        "BindingNameError",
        "DeclarationError",
        "DuplicateBindingNameError",
        "DuplicateProjectionError",
        "DuplicateVariableNameError",
        "FileDeclarationError",
        "FrameDirectionError",
        "ImplementationClassError",
        "NotInvertibleError",
        "ParameterDeclarationError",
        "RequiredDeclarationError",
        "SuccessFieldError",
        "UnprojectableFieldError",
    }
)

HAS_RCLPY = importlib.util.find_spec("rclpy") is not None


def _documented(module: str, name: str) -> bool:
    """Return whether ``name`` carries a docstring where ``module`` defines it."""
    spec = importlib.util.find_spec(module)
    assert spec is not None
    assert spec.origin is not None
    path = Path(spec.origin)
    body = ast.parse(path.read_text(encoding="utf-8")).body
    for index, node in enumerate(body):
        if isinstance(node, ast.ClassDef | ast.FunctionDef | ast.AsyncFunctionDef) and node.name == name:
            return bool(ast.get_docstring(node))
        targets = [node.target] if isinstance(node, ast.AnnAssign) else getattr(node, "targets", [])
        if any(isinstance(each, ast.Name) and each.id == name for each in targets):
            following = body[index + 1] if index + 1 < len(body) else None
            return (
                isinstance(following, ast.Expr)
                and isinstance(following.value, ast.Constant)
                and isinstance(following.value.value, str)
                and bool(following.value.value.strip())
            )
        if isinstance(node, ast.ImportFrom) and node.module and any((a.asname or a.name) == name for a in node.names):
            return _documented(node.module, name)
    return False


def _defining_modules(package: str) -> dict[str, str]:
    """Map each name a package imports to the module it comes from, read from its __init__."""
    spec = importlib.util.find_spec(package)
    assert spec is not None
    assert spec.origin is not None
    init = Path(spec.origin)
    sources: dict[str, str] = {}
    for node in ast.walk(ast.parse(init.read_text(encoding="utf-8"))):
        if isinstance(node, ast.ImportFrom) and node.module:
            for alias in node.names:
                sources[alias.asname or alias.name] = node.module
    sources["Runtime"] = "opensmi.ros2.runtime.runtime"
    return sources


def test_the_server_package_exports_exactly_the_public_surface() -> None:
    assert set(opensmi.ros2.server.__all__) == SERVER
    assert len(opensmi.ros2.server.__all__) == len(SERVER)


def test_the_runtime_package_exports_exactly_the_public_surface() -> None:
    assert set(opensmi.ros2.runtime.__all__) == RUNTIME
    assert len(opensmi.ros2.runtime.__all__) == len(RUNTIME)


def test_the_server_errors_module_exports_exactly_the_declaration_errors() -> None:
    assert set(opensmi.ros2.server.errors.__all__) == SERVER_ERRORS


def test_the_runtime_errors_module_exports_every_error_it_defines() -> None:
    errors = importlib.import_module("opensmi.ros2.runtime.errors")
    defined = {
        name for name, value in vars(errors).items() if inspect.isclass(value) and value.__module__ == errors.__name__
    }

    assert set(errors.__all__) == defined
    assert "ROS2RuntimeError" in defined


def test_every_server_error_is_a_declaration_error_and_none_is_at_the_package_top_level() -> None:
    for name in SERVER_ERRORS:
        assert issubclass(getattr(opensmi.ros2.server.errors, name), DeclarationError)
    assert not SERVER_ERRORS & set(opensmi.ros2.server.__all__)
    assert not {name for name in opensmi.ros2.runtime.__all__ if name.endswith("Error")}


@pytest.mark.parametrize("name", sorted(SERVER))
def test_a_public_server_name_imports_and_is_documented(name: str) -> None:
    assert getattr(opensmi.ros2.server, name) is not None
    if name == "recipes":
        assert opensmi.ros2.server.recipes.__doc__
        return
    assert _documented(_defining_modules("opensmi.ros2.server")[name], name), name


@pytest.mark.parametrize("name", sorted(RUNTIME))
def test_a_public_runtime_name_imports_and_is_documented(name: str) -> None:
    if name != "Runtime" or HAS_RCLPY:
        assert getattr(opensmi.ros2.runtime, name) is not None
    assert _documented(_defining_modules("opensmi.ros2.runtime")[name], name), name


@pytest.mark.parametrize("name", sorted(SERVER_ERRORS))
def test_a_public_server_error_is_documented(name: str) -> None:
    assert _documented("opensmi.ros2.server.errors", name), name


def test_the_recipes_are_a_namespace_so_that_enum_does_not_shadow_the_standard_library() -> None:
    recipes = opensmi.ros2.server.recipes

    assert {"Blob", "Default", "Drop", "Enum", "Functions", "Scalar"} <= set(recipes.__all__)
    assert "Enum" not in opensmi.ros2.server.__all__
