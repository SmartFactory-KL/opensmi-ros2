# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The seam, written down: what a Runtime is, to everything that is not one.

The Runtime's interface as a type, so the server half can name what it was injected without
importing :class:`opensmi.ros2.runtime.runtime.Runtime` (the only module that imports ``rclpy``).
A ``Protocol``, so a test double satisfies it by shape; tests compare the fake's surface against
the real Runtime's signatures.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, runtime_checkable

if TYPE_CHECKING:
    from opensmi.ros2.runtime.goal import Goal
    from opensmi.ros2.runtime.graph import GraphFacet
    from opensmi.ros2.runtime.messages import ActionTypes, Message, ServiceTypes
    from opensmi.ros2.runtime.outcomes import StopOutcome
    from opensmi.ros2.runtime.parameters import ParameterFacet
    from opensmi.ros2.runtime.publisher import Publisher
    from opensmi.ros2.runtime.qos import EndpointInfo, QoSProfile
    from opensmi.ros2.runtime.subscription import Subscription
    from opensmi.ros2.runtime.tf import TFFacet

__all__ = ["RuntimeInterface"]


@runtime_checkable
class RuntimeInterface(Protocol):
    """All ROS2 access for one Machine, as the rest of the package sees it.

    The docstrings of :class:`~opensmi.ros2.runtime.runtime.Runtime` describe the behaviour.
    """

    @property
    def domain_id(self) -> int:
        """Return the ROS2 Domain ID this Runtime is confined to."""
        ...

    @property
    def node_name(self) -> str:
        """Return the name of the single ``rclpy`` Node this Runtime owns."""
        ...

    @property
    def started(self) -> bool:
        """Return whether the Context, Node and executor thread are up."""
        ...

    @property
    def graph(self) -> GraphFacet:
        """Return the facet answering what is on the ROS2 graph."""
        ...

    @property
    def parameters(self) -> ParameterFacet:
        """Return the facet answering what a ROS2 node's parameters are, and writing them."""
        ...

    @property
    def tf(self) -> TFFacet:
        """Return the facet answering what the TF frame network looks like, and where a frame sits."""
        ...

    async def start(self) -> None:
        """Bring up the Context, the Node and the executor thread."""
        ...

    async def stop(self) -> StopOutcome:
        """Close every open subscription, then tear the ROS2 objects down."""
        ...

    async def subscribe(
        self,
        *,
        topic: str,
        message_type: str | type[Message],
        depth: int = 10,
        qos: QoSProfile | None = None,
    ) -> Subscription:
        """Subscribe to a ROS2 topic, and return the handle that owns the subscription."""
        ...

    async def advertise(
        self,
        *,
        topic: str,
        message_type: str | type[Message],
        depth: int = 10,
        qos: QoSProfile | None = None,
    ) -> Publisher:
        """Advertise a ROS2 topic, and return the handle that owns the publisher."""
        ...

    def publishers_on(self, topic: str) -> tuple[EndpointInfo, ...]:
        """Return what every publisher on a topic offers, as the QoS resolver takes it."""
        ...

    def describe_message(self, message_type: str) -> type[Message]:
        """Return the message dataclass of a ROS2 message type."""
        ...

    def default_message(self, message_type: type[Message]) -> Message:
        """Return a message of a type as its generated ROS2 class constructs it, with no arguments."""
        ...

    def describe_action(self, action_type: str) -> ActionTypes:
        """Return the goal, feedback and result message dataclasses of a ROS2 action type."""
        ...

    def describe_service(self, service_type: str) -> ServiceTypes:
        """Return the request and response message dataclasses of a ROS2 service type."""
        ...

    async def call_service(
        self,
        *,
        service: str,
        service_type: str | ServiceTypes,
        request: Message,
        server_timeout: float = 10.0,
        call_timeout: float = 30.0,
    ) -> Message:
        """Call one ROS2 service and return the response it answered with."""
        ...

    async def send_goal(
        self,
        *,
        action: str,
        action_type: str | ActionTypes,
        goal: Message,
        depth: int = 10,
        server_timeout: float = 10.0,
    ) -> Goal:
        """Send one goal to a ROS2 action server, and return the handle that owns it."""
        ...
