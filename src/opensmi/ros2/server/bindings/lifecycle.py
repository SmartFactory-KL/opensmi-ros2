# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A ROS2 lifecycle node behind the StartupSkill."""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

from opensmi.ros2.server.bindings._base import AwaitedBinding
from opensmi.ros2.server.errors import (
    DeclarationError,
    ImplementationClassError,
)


@dataclass(frozen=True, slots=True, kw_only=True)
class LifecycleBinding(AwaitedBinding):
    """A ROS2 lifecycle node driving the declaring Component's StartupSkill.

    A managed node's state machine and the continuous-skill state machine are the same, so this lands on
    the StartupSkill opensmi already has. ``Halted``, ``Ready`` and ``Running`` are
    ``unconfigured``, ``inactive`` and ``active``. The triggers ``reset``, ``start`` and ``halt`` send
    ``configure``, ``activate`` and ``deactivate`` then ``cleanup``. ``halt`` is never ``shutdown``, because
    ``finalized`` is terminal and the Skill could not be ``Reset``. There is no ``suspend``, since the
    StartupSkill is not suspendable.

    The Binding is two-way. Transitions somebody else caused move the Skill, downward only, because
    starting is a client-driven contract. Several of these on one Component drive the one
    StartupSkill, with a lowest-wins aggregate: ``Running`` means every node is active.

    The coordinator flavour is a :class:`~opensmi.ros2.server.drivers.LifecycleDriver` (the standard
    ``lifecycle_msgs`` services, or a nav2-style manager), and the OPC UA surface is the same for each.

    :ivar name: The Binding's name. It is not a Skill name (the Skill is the Component's StartupSkill);
        it keeps declarations apart, prefixes this Binding's variables and names the ``BindingEvent``
        source.
    :ivar path: The ROS2 node this declaration commands through: the managed node itself for the standard
        flavour, the manager for a nav2-style one.
    :ivar nodes: The lifecycle nodes observed and gated on, when ``path`` is a coordinator. Empty for the
        standard flavour; required (fatal at init if empty) for a flavour that coordinates a set.
    :ivar driver: The coordinator flavour: a name in :data:`~opensmi.ros2.server.drivers.DRIVERS`
        (``lifecycle``, the ROS2 standard, or ``nav2``) or a subclass of
        :class:`~opensmi.ros2.server.drivers.LifecycleDriver`. A Binding file gives a name; a machine
        file can give a class as ``driver_type``, by import path. Fatal at init when it is neither.
    :ivar depth: The ROS2 queue depth of the ``transition_event`` subscription of each node.
    :ivar server_timeout: Seconds a question or command waits for the node's services before giving up on
        that attempt. A node that is off is a runtime condition and the next appearance looks again.
    :ivar call_timeout: Seconds to wait for an answer once the request has gone.
    :ivar required: Whether the StartupSkill waits for its nodes to be on the graph before ``Running``.
        A managed node is present when it publishes ``transition_event``, so a ``finalized``
        node still counts.
    :ivar startup_timeout: Seconds that wait lasts, or ``None`` for the deployment's default. Refused when the
        Binding is constructed without ``required``.
    """

    kind: ClassVar[str] = "lifecycle Binding"

    path: str
    nodes: tuple[str, ...] = ()
    driver: str | type = "lifecycle"
    depth: int = 10
    server_timeout: float = 10.0
    call_timeout: float = 30.0

    def __post_init__(self) -> None:
        """Refuse a declaration that names a coordinator that cannot be built.

        :raises ImplementationClassError: When ``driver`` is a class that is not a LifecycleDriver subclass.
        :raises DeclarationError: When ``driver`` names no registered flavour, or a flavour whose ``path``
            is a coordinator is given no nodes to drive.
        :raises RequiredDeclarationError: When ``startup_timeout`` is declared without ``required``, or
            is not positive.
        """
        self._require_a_name_that_can_be_browsed()
        self._require_a_deadline_that_means_something()
        # Imported here to keep this module cheap to import.
        from opensmi.ros2.server.drivers import DRIVERS, LifecycleDriver

        driver = self.driver
        if isinstance(driver, str):
            if driver not in DRIVERS:
                listed = ", ".join(sorted(DRIVERS))
                message = (
                    f"the lifecycle Binding {self.name!r} names the coordinator flavour "
                    f"{driver!r}, and the flavours that ship are: {listed}; a third one is a "
                    f"LifecycleDriver subclass given as driver="
                )
                raise DeclarationError(message)
        elif isinstance(driver, type):
            if not issubclass(driver, LifecycleDriver):
                message = (
                    f"the lifecycle Binding {self.name!r} names {driver.__name__} as its Driver, and it is "
                    f"not a subclass of LifecycleDriver; a Driver knows one coordinator flavour "
                    f"and holds no state machine"
                )
                raise ImplementationClassError(message)
        else:
            message = (
                f"the lifecycle Binding {self.name!r} gives {driver!r} as its driver, and it is "
                f"neither a name nor a LifecycleDriver subclass"
            )
            raise DeclarationError(message)
        # Read off the resolved class, so a custom Driver answers it too.
        resolved = self.driver_class
        if resolved.coordinates_a_set and not self.nodes:
            message = (
                f"the lifecycle Binding {self.name!r} is driven by {resolved.__name__}, whose "
                f"path is the coordinator rather than a managed node, and it lists no nodes; a "
                f"coordinator has no lifecycle state of its own to follow"
            )
            raise DeclarationError(message)

    @property
    def driver_class(self) -> type:
        """Return the Driver class ``driver`` selects, looking a name up in the registry."""
        from opensmi.ros2.server.drivers import DRIVERS

        return DRIVERS[self.driver] if isinstance(self.driver, str) else self.driver

    @property
    def observed(self) -> tuple[str, ...]:
        """Return the lifecycle nodes this Binding follows: ``nodes``, or ``path`` when that is empty."""
        return self.nodes or (self.path,)

    @property
    def needs_the_graph(self) -> bool:
        """Return ``False``; whether a node is there comes from ``transition_event`` match events."""
        return False
