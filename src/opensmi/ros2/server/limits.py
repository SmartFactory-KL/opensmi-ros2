# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The byte-string ceiling: one number, published once and applied once.

A bulk payload (an occupancy grid, an image) is as large as its source, not as large as this package
chooses. OPC UA says how much a server can carry in
``Server/ServerCapabilities/MaxByteStringLength`` (OPC UA Part 5). The effective ceiling, derived
from the transport limits the server actually runs with, is written there at init, and our own
outbound values are capped at it inside the Projection that produces the bytes
(:mod:`opensmi.ros2.server.recipes`). The send path has no second check.

The ceiling is smaller than the message size, because a byte string travels inside a DataValue, a
Variant, a response and a chunked message. :data:`ENVELOPE_MARGIN` holds back an allowance for that
envelope.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Final

from asyncua import ua
from asyncua.ua.object_ids import ObjectIds

if TYPE_CHECKING:
    from asyncua.server.server import Server

__all__ = ["ENVELOPE_MARGIN", "byte_string_ceiling", "publish_byte_string_ceiling"]

ENVELOPE_MARGIN: Final = 65_536
"""What is held back from the message budget for everything wrapped around the value itself."""

NO_CEILING: Final = 0
"""What OPC UA writes in a capability variable to say a limit does not apply."""


def byte_string_ceiling(*, max_message_size: int) -> int:
    """Return the largest byte string this server can carry, given its message budget.

    A pure function of one number. A small budget gets a small ceiling and never zero: the envelope
    allowance is capped at half the budget, because zero in this capability variable means that no
    limit applies.

    :param max_message_size: The transport's limit in bytes, or ``0`` for no limit.
    """
    if max_message_size <= NO_CEILING:
        return NO_CEILING
    margin = min(ENVELOPE_MARGIN, max_message_size // 2)
    return max(max_message_size - margin, 1)


async def publish_byte_string_ceiling(server: Server) -> int:
    """Write this server's effective ceiling into its capability variable, and return it.

    Idempotent. The limits are read off the server, not off our configuration, so a client reads the
    number the transport will accept.

    :param server: The asyncua server the Machine was built on.
    """
    limits: Any = server.limits
    ceiling = byte_string_ceiling(max_message_size=int(limits.max_message_size))
    identifier = ua.Int32(ObjectIds.Server_ServerCapabilities_MaxByteStringLength)
    node = server.get_node(ua.NodeId(identifier))
    await node.write_value(ua.Variant(ua.UInt32(ceiling), ua.VariantType.UInt32))
    return ceiling
