# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""What the Runtime raises when the thing asked for did not happen.

An error is raised when something did not happen; an outcome is returned when it happened and went
badly (a stop whose executor thread would not join is a ``StopOutcome``). All errors derive from
:class:`ROS2RuntimeError`, which carries the ROS2 prefix so it does not collide with ``RuntimeError``.
"""

from __future__ import annotations

__all__ = [
    "ActionServerUnavailableError",
    "FieldSelectionError",
    "FieldTypeMismatchError",
    "FieldValueError",
    "FieldValueOutOfRangeError",
    "GoalClosedError",
    "InvalidMessageTypeNameError",
    "LoopAffinityError",
    "MessageTypeError",
    "MissingInterfacePackageError",
    "PublishFailedError",
    "ROS2RuntimeError",
    "RuntimeAlreadyStartedError",
    "RuntimeNotStartedError",
    "ServiceCallTimeoutError",
    "ServiceUnavailableError",
    "SubscriptionClosedError",
    "UnusableMessageTypeError",
]


class ROS2RuntimeError(Exception):
    """Base of every error the Runtime raises: something it was asked to do did not happen."""


class RuntimeNotStartedError(ROS2RuntimeError):
    """An operation needing the ROS2 node ran before :meth:`Runtime.start`, or after stop."""


class RuntimeAlreadyStartedError(ROS2RuntimeError):
    """:meth:`Runtime.start` ran twice, which would leak the first Context, Node and executor thread."""


class LoopAffinityError(ROS2RuntimeError):
    """A Runtime operation was awaited on an event loop other than the one that started it."""


class SubscriptionClosedError(ROS2RuntimeError):
    """A message, delta or update was awaited from a Subscription or a watch whose lifetime has ended."""


class MessageTypeError(ROS2RuntimeError):
    """A declared ROS2 message type cannot be resolved, or a field has a form this Runtime does not model.

    All are fatal at init: the adapter never guesses a field shape, because a wrong one reaches an
    OPC UA client as a plausible, wrong value.
    """


class InvalidMessageTypeNameError(MessageTypeError):
    """A declared type name is not ``package/Name`` or ``package/kind/Name``."""


class MissingInterfacePackageError(MessageTypeError):
    """A declared type's interface package is not installed.

    ``rclpy`` cannot subscribe without the typesupport, so there is no degraded mode.
    """


class UnusableMessageTypeError(MessageTypeError):
    """The interface package is installed but the named type is absent or is not a message."""


class ActionServerUnavailableError(ROS2RuntimeError):
    """No action server answered on a path a goal was sent to.

    A goal that was rejected, aborted or cancelled comes back as a
    :class:`~opensmi.ros2.runtime.outcomes.GoalOutcome`. A goal nobody received raises this.
    """


class ServiceUnavailableError(ROS2RuntimeError):
    """No service server answered on a path a request was sent to.

    A service that answered comes back as the response message, whatever it answered.
    """


class ServiceCallTimeoutError(ROS2RuntimeError):
    """A service server took the request and never answered within the call timeout.

    Distinct from :class:`ServiceUnavailableError` (nothing there against something stuck).
    """


class PublishFailedError(ROS2RuntimeError):
    """``rclpy`` refused to put a message on a topic.

    It wraps the fault ``rclpy`` raised. A field of the wrong Python type is a
    :class:`FieldValueError` instead and never reaches ``rclpy``.
    """


class GoalClosedError(ROS2RuntimeError):
    """Feedback or a result was awaited from a Goal whose lifetime has ended."""


class FieldValueError(ROS2RuntimeError):
    """Base of what the Runtime refuses to put into a ROS2 message.

    ``rclpy`` accepts a field of the wrong Python type, but the conversion to C aborts the whole
    process on send (pinned by a Jazzy test). The guard therefore checks at declaration time, and
    again on send for values a client writes into a Parameter.
    """


class FieldTypeMismatchError(FieldValueError):
    """A field value is not the Python type its ROS2 field is.

    The check uses exact types: ``1`` is not a ``float64`` and ``True`` is not an ``int32``.
    """


class FieldValueOutOfRangeError(FieldValueError):
    """A field value does not fit the ROS2 integer or the bound its field declares.

    ``2**31`` into an ``int32`` does not abort but goes onto the wire wrong. Bounds on strings
    and sequences are advertised by OPC UA and enforced nowhere else.
    """


class FieldSelectionError(ROS2RuntimeError):
    """A Binding's include and exclude patterns match no field, or leave the message with none.

    Fatal at init, so a mistyped pattern cannot silently produce fewer variables than declared.
    """
