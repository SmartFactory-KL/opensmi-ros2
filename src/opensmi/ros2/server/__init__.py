# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The OPC UA half: a thin layer on opensmi.server that renders a Runtime as a Machine.

UA-facing classes inherit from opensmi's base classes with this project's mixins in front. The
Runtime is injected through the Machine's constructor. Everything exported here is what a server
author touches when declaring or starting a server; anything else is importable from its module but
not part of the public surface (see CONTRIBUTING.md).

Public names:

* Binding declarations and the callback types their fields take, the Machine and Component mixins.
* ``ROS2Config`` and ``TransformConfig``.
* The TOML loader: ``load_bindings``, ``resolve_bindings``, ``TOML_KINDS``, ``load_machine``.
* The Projection engine with its plan types. The recipes that curate a Component's types are the
  namespace ``recipes`` (``recipes.Enum``), since ``Enum`` would shadow ``enum.Enum``.
* The three access-level constants, ``Severity`` and ``SmartFactoryROS2NodeIds``.
* The lifecycle drivers with ``LifecycleNodeState`` and ``LifecycleNodeTransition``. A custom driver
  subclasses ``LifecycleDriver`` and is given as a Binding's ``driver``.

Errors raised for a bad declaration are in :mod:`opensmi.ros2.server.errors`. The command line is
public as a command (``opensmi-ros2 serve``, ``--server-config``, the exit status), not as a module.
``load_toml`` and ``from_dict`` share names with ``opensmi.core.config`` but behave differently and
are internal.

What a client sees:

* A write to a ROS2 parameter is answered by the owning node. A refused write comes back as a Bad
  status code and the variable keeps the node's value.
* Skills and Methods need the Lock, which times out after ten minutes without activity, as in
  opensmi. Introspection listings do not need it.
* A message is a complex variable with one child variable per field, named from the ROS2 field in
  PascalCase (``Odom/Pose/Pose/Position/X``). A repeated field is one array variable whose element is
  a structure type named after the ROS2 type.
* The address space grows but never shrinks while the server runs.
"""

from __future__ import annotations

from opensmi.ros2.server import recipes
from opensmi.ros2.server.access import LISTING_ACCESS_LEVEL, MUTATOR_ACCESS_LEVEL, SKILL_ACCESS_LEVEL
from opensmi.ros2.server.bindings import (
    TOML_KINDS,
    ActionBinding,
    Binding,
    GoalFunction,
    KeyedBinding,
    LifecycleBinding,
    MessageFunction,
    MethodBinding,
    MonitoringBinding,
    PublishedFrame,
    PublisherBinding,
    RequestFunction,
    ResponseFunction,
    ResultFunction,
    ROS2ParameterBinding,
    TFBinding,
    load_bindings,
    resolve_bindings,
)
from opensmi.ros2.server.component import ROS2ComponentMixin
from opensmi.ros2.server.config import ROS2Config, TransformConfig
from opensmi.ros2.server.drivers import (
    DRIVERS,
    LifecycleDriver,
    Nav2ManagerDriver,
    NodeTransition,
    StandardLifecycleDriver,
)
from opensmi.ros2.server.events import Severity
from opensmi.ros2.server.lifecycle import LifecycleNodeState, LifecycleNodeTransition
from opensmi.ros2.server.loader import ConfiguredComponent, ConfiguredMachine, load_machine
from opensmi.ros2.server.machine import ROS2MachineMixin
from opensmi.ros2.server.nodesets import SmartFactoryROS2NodeIds
from opensmi.ros2.server.projection import (
    DecodeFunction,
    EncodeFunction,
    EnumDataType,
    PlanDataType,
    PlanEntry,
    PlanObject,
    PlanVariable,
    ProjectDataType,
    Projection,
    ProjectionContext,
    ProjectionRegistry,
    ShapeFunction,
    StandardDataType,
)
from opensmi.ros2.server.provenance import BindingQoS

__all__ = [
    "DRIVERS",
    "LISTING_ACCESS_LEVEL",
    "MUTATOR_ACCESS_LEVEL",
    "SKILL_ACCESS_LEVEL",
    "TOML_KINDS",
    "ActionBinding",
    "Binding",
    "BindingQoS",
    "ConfiguredComponent",
    "ConfiguredMachine",
    "DecodeFunction",
    "EncodeFunction",
    "EnumDataType",
    "GoalFunction",
    "KeyedBinding",
    "LifecycleBinding",
    "LifecycleDriver",
    "LifecycleNodeState",
    "LifecycleNodeTransition",
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
    "Severity",
    "ShapeFunction",
    "SmartFactoryROS2NodeIds",
    "StandardDataType",
    "StandardLifecycleDriver",
    "TFBinding",
    "TransformConfig",
    "load_bindings",
    "load_machine",
    "recipes",
    "resolve_bindings",
]
