# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Classes and functions that machine files in the `test_loader_classes` tests name by import path."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from asyncua import ua

from opensmi.ros2.server import ConfiguredComponent, ConfiguredMachine, StandardLifecycleDriver
from opensmi.ros2.server.projection import PlanVariable, join_path
from opensmi.ros2.server.required import ROS2StartupSkill
from opensmi.ros2.server.startup import ROS2LifecycleStartupSkill

if TYPE_CHECKING:
    from opensmi.ros2.server.projection import PlanEntry, ProjectionContext

BUILT: list[str] = []
"""What the tests' own classes constructed, in order, for the cases a client cannot see."""

IMPORTED: list[str] = []
"""The modules of this package that were imported because a file named them."""


class CustomMachine(ConfiguredMachine):
    """A Machine that signs its serial number, which is what a client sees of it."""

    async def _write_identification(self) -> None:
        await super()._write_identification()
        await self.identification.SerialNumber.write("custom-machine")


class CustomComponent(ConfiguredComponent):
    """A Component that signs its serial number with its name."""

    async def _write_identification(self) -> None:
        await super()._write_identification()
        await self.identification.SerialNumber.write(f"custom-{self.name}")


class CustomStartupSkill(ROS2StartupSkill):
    """A StartupSkill of the deployer's, for a Machine or a Component that waits on Bindings."""


class CustomLifecycleStartupSkill(ROS2LifecycleStartupSkill):
    """A StartupSkill of the deployer's, for a Component that drives lifecycle nodes."""


class CustomDriver(StandardLifecycleDriver):
    """The standard coordinator flavour that says it was built."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        BUILT.append("driver")


class Plain:
    """A class that is none of the things a file may ask for."""


NOT_A_CLASS = 5


def voltage_shape(_message_type: Any, context: ProjectionContext) -> tuple[PlanEntry, ...]:
    """Render a Voltage as one String variable, the way a curated Projection would."""
    return (
        PlanVariable(
            name="Text",
            display_name="text",
            source=join_path(context.path, "data"),
            initial="",
            variant_type=ua.VariantType.String,
        ),
    )


def voltage_decode(message: Any, _context: ProjectionContext) -> dict[str, object]:
    """Read a Voltage as text."""
    return {"Text": f"{message.data} V"}
