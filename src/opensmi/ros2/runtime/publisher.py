# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Publisher handle: one topic's outward lifetime, and the checks in front of it.

The mirror of :class:`~opensmi.ros2.runtime.subscription.Subscription`, without a buffer: a publish
is push-only with no acknowledgement. Nothing here imports ``rclpy``.

:meth:`Publisher.publish` checks the payload type, fills unset header stamps from the Runtime's
ROS2 clock (:mod:`opensmi.ros2.runtime.stamp`), and runs the type-mismatch guard
(:mod:`opensmi.ros2.runtime.guard`) on the stamped message, so what reaches ``rclpy`` is what was
checked. The QoS profile is declared or defaulted, never discovered or adapted.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, final

from opensmi.ros2.runtime.errors import ROS2RuntimeError
from opensmi.ros2.runtime.guard import require_sendable
from opensmi.ros2.runtime.qos import DEFAULT_PROFILE, Provenance, QoSProfile
from opensmi.ros2.runtime.stamp import stamped

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from opensmi.ros2.runtime.messages import Message

__all__ = ["Publisher"]


@final
class Publisher:
    """A live publisher on one ROS2 topic, from :meth:`~opensmi.ros2.runtime.runtime.Runtime.advertise`."""

    def __init__(
        self,
        *,
        topic: str,
        message_type: type[Message],
        clock: Callable[[], tuple[int, int]],
        send: Callable[[Message], Awaitable[None]],
        release: Callable[[], Awaitable[None]],
        qos: QoSProfile = DEFAULT_PROFILE,
        provenance: Provenance = Provenance.DEFAULTED,
    ) -> None:
        """Bind a handle to the clock, the send step and the teardown the Runtime prepared."""
        self._topic = topic
        self._message_type = message_type
        self._clock = clock
        self._send = send
        self._release = release
        self._qos = qos
        self._provenance = provenance
        self._published = 0
        self._closed = False

    def __repr__(self) -> str:
        """Return a representation naming the topic and the ROS2 type."""
        state = "closed" if self._closed else "open"
        return f"<Publisher {self._topic} {self._message_type.ros2.type_name} ({state})>"

    @property
    def topic(self) -> str:
        """Return the ROS2 topic this publisher is on."""
        return self._topic

    @property
    def message_type(self) -> type[Message]:
        """Return the message dataclass every payload handed to this publisher must be."""
        return self._message_type

    @property
    def closed(self) -> bool:
        """Return whether this publisher's lifetime has ended."""
        return self._closed

    @property
    def qos(self) -> QoSProfile:
        """Return the QoS profile this publisher offers (fixed once advertised)."""
        return self._qos

    @property
    def qos_provenance(self) -> Provenance:
        """Return where that profile came from: ``DECLARED`` or ``DEFAULTED``, never discovered."""
        return self._provenance

    @property
    def published(self) -> int:
        """Return how many sends did not raise (ROS2 gives no acknowledgement, so not messages delivered)."""
        return self._published

    async def publish(self, message: Message) -> None:
        """Fill the header stamp, check the payload and put one message on the topic.

        :param message: The payload, as an instance of this publisher's message dataclass.
        :raises ROS2RuntimeError: When this handle's lifetime has already ended.
        :raises TypeError: When the payload is not an instance of the type advertised.
        :raises FieldTypeMismatchError: When a field holds a value of the wrong Python type.
            Refused here because ``rclpy`` aborts the process instead of raising.
        :raises FieldValueOutOfRangeError: When a field is past its width or its declared bound.
        :raises PublishFailedError: When ``rclpy`` refused to send it.
        """
        if self._closed:
            text = f"the publisher on {self._topic} is closed and will send nothing further"
            raise ROS2RuntimeError(text)
        if not isinstance(message, self._message_type):
            text = (
                f"the message for {self._topic} is a {type(message).__name__}, and this publisher "
                f"carries {self._message_type.ros2.type_name}"
            )
            raise TypeError(text)
        seconds, nanoseconds = self._clock()
        payload = stamped(message, seconds=seconds, nanoseconds=nanoseconds)
        require_sendable(payload)
        await self._send(payload)
        self._published += 1

    async def close(self) -> None:
        """End this publisher, releasing its ROS2 endpoint. Idempotent."""
        if self._closed:
            return
        self._closed = True
        await self._release()
