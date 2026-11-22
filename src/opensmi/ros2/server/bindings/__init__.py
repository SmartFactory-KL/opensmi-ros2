# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Binding declarations: what a Component exposes, written down rather than assembled.

A Binding connects one ROS2 noun (a topic, service, action or node path) to one OPC UA object. It is a
frozen dataclass listed on a Component's ``bindings`` class attribute; no lifecycle hook is overridden
to add a variable. The class attribute is the instance default, so a constructor may compute the list
from injected configuration. Every kind that names a ROS2 path says ``path=``. A TOML source unions
with the class attribute and a name collision is fatal; neither source may change a Binding the other
wrote.

Kinds are named for their OPC UA target, or for the ROS2 side where two kinds share one:
:class:`MonitoringBinding` (topic read into Monitoring variables), :class:`KeyedBinding` (one member per
key of a repeated field), :class:`MethodBinding` (service), :class:`ActionBinding` (action as a finite
Skill), :class:`PublisherBinding` (topic published from a Skill), :class:`ROS2ParameterBinding` (node
parameters), :class:`LifecycleBinding` (lifecycle node behind the StartupSkill) and :class:`TFBinding`
(TF tree as RSL Locations).

A TOML Binding file has one table per kind: ``monitoring``, ``keyed``, ``method``, ``ros2_parameter``,
``action``, ``tf``, ``publisher`` and ``lifecycle``. A Component picks the file up through
``bindings_toml``. A name may be declared only once across Python and TOML.

.. code-block:: toml

    [[monitoring]]
    name = "Chatter"
    path = "/chatter"
    message_type = "std_msgs/msg/String"

    [[method]]
    name = "AddTwoInts"
    path = "/add_two_ints"
    service_type = "example_interfaces/srv/AddTwoInts"

A Binding's ``name`` becomes a browse name, so an empty name or one that breaks opensmi's
browse-name rule ``^[A-Z][A-Za-z0-9]*$`` is refused with
:class:`~opensmi.ros2.server.errors.BindingNameError` (a ``/`` in a field path stays legal).

Keywords that name Python objects (implementation classes, callables) are Python-only. The loader
refuses a TOML value for any field a TOML file cannot produce
(:func:`opensmi.ros2.server.config.from_dict`).

``required=True`` makes the StartupSkill of the declaring item wait until the Binding is satisfied, and
halt if its deadline passes first. Every Binding is optional otherwise.

A topic's ``qos=`` is a contract. Declared, it is honoured exactly for the life of the endpoint, even
when provably incompatible with a publisher (reported, never fatal). Left out on a
:class:`MonitoringBinding`, a profile is discovered: the most permissive one compatible with every
publisher. Depth is a separate keyword (:mod:`opensmi.ros2.runtime.qos`). A Monitoring Binding without
``message_type`` is the graph-driven case.
"""

from opensmi.ros2.server.bindings.action import ActionBinding, GoalFunction, ResultFunction
from opensmi.ros2.server.bindings.keyed import KeyedBinding
from opensmi.ros2.server.bindings.lifecycle import LifecycleBinding
from opensmi.ros2.server.bindings.method import MethodBinding, RequestFunction, ResponseFunction
from opensmi.ros2.server.bindings.monitoring import MonitoringBinding
from opensmi.ros2.server.bindings.parameter import WRITE_TIMEOUT, ROS2ParameterBinding
from opensmi.ros2.server.bindings.publisher import MessageFunction, PublisherBinding
from opensmi.ros2.server.bindings.registry import (
    TOML_KINDS,
    Binding,
    is_required,
    load_bindings,
    resolve_bindings,
)
from opensmi.ros2.server.bindings.tf import PublishedFrame, TFBinding

__all__ = [
    "TOML_KINDS",
    "WRITE_TIMEOUT",
    "ActionBinding",
    "Binding",
    "GoalFunction",
    "KeyedBinding",
    "LifecycleBinding",
    "MessageFunction",
    "MethodBinding",
    "MonitoringBinding",
    "PublishedFrame",
    "PublisherBinding",
    "ROS2ParameterBinding",
    "RequestFunction",
    "ResponseFunction",
    "ResultFunction",
    "TFBinding",
    "is_required",
    "load_bindings",
    "resolve_bindings",
]
