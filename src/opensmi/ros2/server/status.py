# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Satisfied, as a status code: telling a robot being off from this server being wrong.

A Binding is Satisfied when its ROS2 counterpart is present on the graph (a publisher on the
topic, a discovered service or action server), whether or not it has carried data. Every variable
the Binding owns reads that as a status code:

* ``BadNoCommunication``: nothing publishes on the topic. The robot is off or the topic does not
  exist on it.
* ``BadWaitingForInitialData``: something publishes and no message has arrived yet.
* ``Good``: a sample has arrived, and the value beside the status is that sample.

A bad status is written together with the typed value (:func:`status_value`), so recovery is one
write and nothing stale is left. OPC UA Part 4 7.7.1 says a server receiving an error status
sets the value to null, and asyncua does that, so a client reading a Bad variable sees a null
value however it was written. :func:`transform_status` applies the same three codes to a TF frame.
"""

from __future__ import annotations

from typing import Final

from asyncua import ua
from asyncua.ua.status_codes import StatusCodes

__all__ = [
    "GOOD",
    "NO_COMMUNICATION",
    "WAITING_FOR_INITIAL_DATA",
    "status_code",
    "status_value",
    "transform_status",
]

GOOD: Final = ua.StatusCode(ua.UInt32(StatusCodes.Good))
"""What a variable reads once a sample has arrived on a topic that has a publisher."""

NO_COMMUNICATION: Final = ua.StatusCode(ua.UInt32(StatusCodes.BadNoCommunication))
"""What a variable reads while nothing publishes on its topic: the Binding is not satisfied."""

WAITING_FOR_INITIAL_DATA: Final = ua.StatusCode(ua.UInt32(StatusCodes.BadWaitingForInitialData))
"""What a variable reads while its topic has a publisher and no message has arrived yet."""


def status_code(*, satisfied: bool, sampled: bool) -> ua.StatusCode:
    """Return the status code one variable reads, given its Binding's two facts.

    :param satisfied: Whether the Binding's ROS2 counterpart is present on the graph.
    :param sampled: Whether a message has ever arrived on it.
    """
    # Presence outranks history: a vanished publisher is the robot being off, whatever arrived before.
    if not satisfied:
        return NO_COMMUNICATION
    return GOOD if sampled else WAITING_FOR_INITIAL_DATA


def transform_status(
    *,
    present: bool,
    static: bool,
    rooted: bool,
    seen: float | None,
    now: float,
    within: float,
) -> ua.StatusCode:
    """Return the status code one TF frame's Location reads, out of the same three codes.

    A frame is satisfied when that frame was in a recent message and something is publishing on
    the transform topics. Topic presence alone is too coarse: a dead arm driver leaves the base
    publishing ``/tf``. Two exemptions from freshness keep the status worth reading:

    * A static frame is published once and is quiet by definition. Silence says nothing, because
      its publisher is transient-local and may exit, and its pose already carries the ``Constant``
      access-level bit. Absence still counts: a static frame that never arrived reads
      bad.
    * The root of the rendered tree is nobody's child, so no message carries it. It reads the
      presence of the transform topics only, which still turns bad when the robot is off.
      An all-static system whose publisher exits reads bad here,
      while its static frames stay good.

    :param present: Whether anything else is publishing on the transform topics, with our own
        broadcast discounted.
    :param static: Whether the last message to carry this frame arrived on the static topic.
    :param rooted: Whether this frame is the root of the rendered tree.
    :param seen: When this frame was last in a message, or ``None`` if it never has been.
    :param now: The current time, off the same clock ``seen`` was stamped from.
    :param within: How many seconds a frame may go unpublished and still speak for itself.
    """
    if rooted:
        return GOOD if present else NO_COMMUNICATION
    if seen is None:
        # Before the static exemption: never having arrived is absence, which static does not excuse.
        return status_code(satisfied=present, sampled=False)
    if static:
        return GOOD
    return status_code(satisfied=present and now - seen <= within, sampled=True)


def status_value(value: object, variant_type: ua.VariantType, status: ua.StatusCode) -> ua.DataValue:
    """Return the DataValue one write puts on a variable: the typed value and its status.

    Writing a status alone would leave the variant null.
    """
    return ua.DataValue(Value=ua.Variant(value, variant_type), StatusCode=status)
