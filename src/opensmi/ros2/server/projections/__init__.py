# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The shipped catalogue: the curation this package brings, as data.

A list of recipe literals (:mod:`opensmi.ros2.server.recipes`) ahead of the generic walk. The record
of which ROS2 types in scope each one answers for is in :mod:`opensmi.ros2.server.projections.scope`
and is not re-exported here.

The list is short on purpose. A ``uint8[]`` field already reaches a client as a ``ByteString`` under
the walk, so a blob over one would change nothing. The walk gets a signed bulk buffer wrong (a
million-element array of ``SByte``), and a plain ``ByteString`` would misreport it as large positive
numbers. That is the occupancy grid, the one shipped entry. A diagnostic status needs no entry
either: the walk renders its key-value list as an array of a derived structure.
Anything else a deployment wants is one literal in its own Component.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Final

from opensmi.ros2.server.recipes import GENERIC_REGISTRY, Blob

if TYPE_CHECKING:
    from opensmi.ros2.server.projection import Projection

__all__ = ["DEFAULT_REGISTRY", "SHIPPED"]

SHIPPED: Final[tuple[Projection, ...]] = (
    # `int8[]` cells: the signedness is published as a DataType a client can discover.
    Blob("nav_msgs/msg/OccupancyGrid", field="data"),
)
"""The curation this package ships, ahead of the generic walk and behind anything registered."""

DEFAULT_REGISTRY: Final = GENERIC_REGISTRY.extended(SHIPPED)
"""What a Machine renders through until it registers curation of its own.

A Machine's registry extends this with the application's entries, and a Component's extends the
Machine's, so two Components can render one message type differently.
"""
