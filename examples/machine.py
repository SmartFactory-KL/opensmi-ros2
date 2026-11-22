# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Machine this example serves: one Runtime, one Platform Component.

``ROS2MachineMixin`` starts the injected Runtime before opensmi builds anything and stops it
after teardown. The rest is opensmi's own machinery (see :mod:`opensmi.ros2.server.machine`).
"""

from __future__ import annotations

from asyncua import ua
from opensmi.server import BaseMachine

from examples.component import Platform
from opensmi.ros2.server import ROS2MachineMixin

MANUFACTURER = ua.LocalizedText("Example Manufacturer", "en-US")


class Robot(ROS2MachineMixin, BaseMachine):
    """A single Platform, under a single Runtime."""

    async def _write_identification(self) -> None:
        """Fill the identification block opensmi asks of every Machine."""
        await self.identification.SerialNumber.write("example-0001")
        await self.identification.ProductInstanceUri.write("urn:example:opensmi-ros2:robot")
        await self.identification.Manufacturer.write(MANUFACTURER)

    async def _init(self) -> None:
        """Add the Platform to the Components opensmi and this package already added."""
        await super()._init()
        await self.add_component(Platform(name="Platform"))
