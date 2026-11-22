# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Binding kinds and the transforms keep their import paths, and the kinds share one name check."""

from __future__ import annotations

import importlib
import inspect
from typing import Any

import pytest

from opensmi.ros2.runtime.parameters import ParameterType
from opensmi.ros2.server import bindings, transforms
from opensmi.ros2.server.bindings import (
    ActionBinding,
    KeyedBinding,
    LifecycleBinding,
    MethodBinding,
    MonitoringBinding,
    PublisherBinding,
    ROS2ParameterBinding,
    TFBinding,
)
from opensmi.ros2.server.errors import BindingNameError, RequiredDeclarationError

KINDS: dict[str, Any] = {
    "monitoring": lambda **kw: MonitoringBinding(path="/a", **kw),
    "keyed": lambda **kw: KeyedBinding(
        path="/a", message_type="std_msgs/msg/String", repeated_field="data", key=("data",), **kw
    ),
    "method": lambda **kw: MethodBinding(path="/a", service_type="std_srvs/srv/Trigger", **kw),
    "action": lambda **kw: ActionBinding(path="/a", action_type="nav2_msgs/action/NavigateToPose", **kw),
    "parameter": lambda **kw: ROS2ParameterBinding(path="/a", parameters={"x": ParameterType.DOUBLE}, **kw),
    "lifecycle": lambda **kw: LifecycleBinding(path="/a", **kw),
    "publisher": lambda **kw: PublisherBinding(path="/a", message_type="std_msgs/msg/String", continuous=False, **kw),
}
WAITING = [kind for kind in KINDS if kind != "publisher"]


@pytest.mark.parametrize("name", [*bindings.__all__])
def test_every_binding_name_is_importable_from_the_package(name: str) -> None:
    assert hasattr(bindings, name)


@pytest.mark.parametrize("name", [*transforms.__all__])
def test_every_transform_name_is_importable_from_the_package(name: str) -> None:
    assert hasattr(transforms, name)


@pytest.mark.parametrize("kind", KINDS)
def test_every_kind_refuses_an_empty_name_and_a_slash(kind: str) -> None:
    with pytest.raises(BindingNameError, match="empty name"):
        KINDS[kind](name="")
    with pytest.raises(BindingNameError, match="breaks the naming rule"):
        KINDS[kind](name="a/b")


@pytest.mark.parametrize("kind", WAITING)
def test_every_waiting_kind_refuses_a_deadline_without_required(kind: str) -> None:
    with pytest.raises(RequiredDeclarationError, match="not required"):
        KINDS[kind](name="N", startup_timeout=5.0)
    assert KINDS[kind](name="N", required=True, startup_timeout=5.0).startup_timeout == 5.0


def test_the_tf_binding_still_refuses_a_deadline_without_required() -> None:
    with pytest.raises(RequiredDeclarationError):
        TFBinding(name="Tf", head="base_link", startup_timeout=5.0)


@pytest.mark.parametrize(
    "cls",
    [
        MonitoringBinding,
        KeyedBinding,
        MethodBinding,
        ActionBinding,
        ROS2ParameterBinding,
        LifecycleBinding,
        PublisherBinding,
        TFBinding,
    ],
)
def test_the_shared_fields_are_declared_in_one_place(cls: type) -> None:
    own = inspect.get_annotations(cls)
    assert "name" not in own
    assert not {"required", "startup_timeout"} & own.keys()


@pytest.mark.parametrize(
    "module",
    ["monitoring", "keyed", "publisher", "action", "method", "parameter", "tf", "lifecycle"],
)
def test_each_kind_has_its_own_module(module: str) -> None:
    importlib.import_module(f"opensmi.ros2.server.bindings.{module}")
