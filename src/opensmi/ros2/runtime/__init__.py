# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The portable half: the Runtime, the sole owner of ROS2 access for one Machine.

This is the only part of the project that imports ``rclpy``. It speaks ROS2 vocabulary, knows
nothing about OPC UA and imports nothing from :mod:`opensmi.ros2.server` (enforced by
``tests/test_import_boundary.py``).

The package exports what a server author touches: the :class:`Runtime` handed to a Machine, the
:class:`RuntimeInterface` it implements, the :class:`Message` base that Binding callbacks receive,
the :class:`ParameterType` of a parameter Binding and the QoS types. A custom lifecycle driver
also needs the :class:`Subscription` and :class:`ServiceTypes` returned by
:meth:`RuntimeInterface.subscribe` and :meth:`RuntimeInterface.describe_service`, the
:class:`QoSIncompatibility` and :class:`Provenance` records of a Subscription's QoS, and the
:class:`TypeHashMismatch` records. Everything else is internal plumbing (see CONTRIBUTING.md).

Only ``Runtime`` is exported lazily. The rest imports without a ROS2 installation, which the
server half and its tests rely on. Accessing ``Runtime`` without ROS2 raises ``ImportError``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from opensmi.ros2.runtime.interface import RuntimeInterface
from opensmi.ros2.runtime.messages import Message, ServiceTypes
from opensmi.ros2.runtime.parameters import ParameterType
from opensmi.ros2.runtime.qos import (
    Durability,
    Liveliness,
    Provenance,
    QoSIncompatibility,
    QoSProfile,
    Reliability,
)
from opensmi.ros2.runtime.subscription import Subscription, TypeHashMismatch

if TYPE_CHECKING:
    from opensmi.ros2.runtime.runtime import Runtime

__all__ = [
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
]


def __getattr__(name: str) -> object:
    """Resolve :class:`Runtime` on first use, so that importing this package needs no ROS2."""
    if name == "Runtime":
        from opensmi.ros2.runtime.runtime import Runtime

        return Runtime
    message = f"module {__name__!r} has no attribute {name!r}"
    raise AttributeError(message)
