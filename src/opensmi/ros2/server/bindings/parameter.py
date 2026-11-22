# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The parameters of a ROS2 node."""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar, Final

from opensmi.ros2.runtime.parameters import ParameterType
from opensmi.ros2.server.bindings._base import AwaitedBinding
from opensmi.ros2.server.browse_names import browse_name
from opensmi.ros2.server.errors import (
    ParameterDeclarationError,
)

WRITE_TIMEOUT: Final = 3.0
"""Seconds a client's write of a ROS2 parameter waits for the node, by default.

Below asyncua's client timeout of four seconds, so the client sees this server's ``BadTimeout`` and not
a timeout of its own.
"""


@dataclass(frozen=True, slots=True, kw_only=True)
class ROS2ParameterBinding(AwaitedBinding):
    """The ROS2 parameters of one node, exposed as Parameters on the declaring Component.

    One declaration per ROS2 node: its parameter services share one path and its changes arrive on
    ``/parameter_events`` under one node name. A Component that tunes two nodes declares two. The ROS2
    qualifier is in the name because bare Parameter is opensmi vocabulary for the OPC UA variable.

    Names, types and read-only marks are declared, never discovered. The nine-way
    ``rcl_interfaces/msg/ParameterValue`` union never reaches OPC UA: the declaration names one typed
    field and the Parameter is a scalar of that type. A UA Parameter's access level is fixed when the node
    is created, so read-only cannot be learned from the robot later. A parameter the node marks
    read-only itself still has every write refused with ``BadNotWritable``.

    A client's write is checked against the node's descriptor and sent with ``set_parameters``. The client
    is answered ``Good`` and the value stored only when the node took it. Declaring a
    parameter here is the only way to make it writable; the introspection Component only lists and
    describes them.

    :ivar name: The Binding's name, the group its Parameters sit in. A ROS2 name such as
        ``FollowPath.max_vel_x`` is one group per namespace level and then the variable:
        ``<name>/FollowPath/MaxVelX``, with each segment converted to a browse name.
    :ivar path: The ROS2 node, fully qualified. A missing leading ``/`` is added.
    :ivar parameters: Map from ROS2 parameter name to type. An empty map is fatal at init. This is the
        one Binding with a mapping field and so the one that is not hashable; nothing here hashes a
        Binding.
    :ivar read_only: Names among ``parameters`` that a client may not write. A name not declared in
        ``parameters`` is fatal at init.
    :ivar retry_interval: Seconds before a parameter not yet confirmed on the node is read again. It lifts
        ``BadNoCommunication`` from a node that was off at boot, which ``/parameter_events`` would never
        announce. Nothing is read while every name is confirmed.
    :ivar write_timeout: Seconds a client's write waits for the node before ``BadTimeout``. The default,
        :data:`WRITE_TIMEOUT`, is below the client's own timeout.
    :ivar required: Whether the StartupSkill waits for this Binding before ``Running``. Unlike other
        kinds it is satisfied only when every declared name is confirmed on the node, so a typo leaves it
        unsatisfied.
    :ivar startup_timeout: Seconds that wait lasts, or ``None`` for the deployment's default. Refused when the
        Binding is constructed without ``required``.
    """

    kind: ClassVar[str] = "ROS2-parameter Binding"

    path: str
    parameters: dict[str, ParameterType]
    read_only: tuple[str, ...] = ()
    retry_interval: float = 5.0
    write_timeout: float = WRITE_TIMEOUT

    def __post_init__(self) -> None:
        """Refuse a contradictory declaration.

        :raises ParameterDeclarationError: When the declaration exposes nothing, marks a name read-only
            that it does not expose, or has a retry interval or write timeout that is not positive.
        :raises RequiredDeclarationError: When ``startup_timeout`` is declared without ``required``, or
            is not positive.
        """
        self._require_a_name_that_can_be_browsed()
        self._require_a_deadline_that_means_something()
        for parameter in self.parameters:
            self.node_path(parameter)
        if not self.parameters:
            message = (
                f"the ROS2-parameter Binding {self.name!r} declares no parameters; the names and "
                f"their types are declared and never discovered, so a Binding with an empty map "
                f"would expose nothing at all"
            )
            raise ParameterDeclarationError(message)
        undeclared = sorted(set(self.read_only) - set(self.parameters))
        if undeclared:
            listed = ", ".join(repr(each) for each in undeclared)
            them = "them" if len(undeclared) > 1 else "it"
            message = (
                f"the ROS2-parameter Binding {self.name!r} marks {listed} read-only and does not "
                f"declare {them}; a read-only mark on a name this Binding does not expose is a "
                f"Parameter somebody believes is protected"
            )
            raise ParameterDeclarationError(message)
        for setting, seconds, meaning in (
            ("retry_interval", self.retry_interval, "pass before an unconfirmed name is read again"),
            ("write_timeout", self.write_timeout, "a client's write waits for the node"),
        ):
            if seconds > 0:
                continue
            message = (
                f"the ROS2-parameter Binding {self.name!r} declares {setting}={seconds!r}; it is "
                f"how many seconds {meaning}, so it is a positive number of seconds"
            )
            raise ParameterDeclarationError(message)

    def node_path(self, parameter: str) -> tuple[str, ...]:
        """Return the browse names of a parameter's namespace groups, then the variable itself.

        :raises BindingNameError: When a segment does not convert, see :func:`browse_name`.
        """
        return tuple(browse_name(segment) for segment in parameter.split("."))

    def writable(self, parameter: str) -> bool:
        """Return whether a client may write one of this Binding's declared parameters."""
        return parameter not in self.read_only

    @property
    def needs_the_graph(self) -> bool:
        """Return ``False``: names and types are declared.

        Whether the node is there is answered by reading a parameter, not the graph.
        """
        return False
