# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Serve the Platform example against a live ROS2 graph.

Run as ``python -m examples.run_server`` from the repository root, with ``opensmi-ros2`` installed
(``pip install -e .``) and ROS2 Jazzy sourced. Point it at a Nav2 stack (simulated or real) or any
ROS2 graph with similar topics, services and actions. The server starts whether or not Nav2 is
running. The ROS2 settings are in ``ros2.toml``.

Connect an OPC UA client to ``opc.tcp://localhost:4841/server`` as one of opensmi's default users
(each password is its name): ``visitor`` may browse and read, ``orchestrator`` may also start
Skills and call Methods, and ``operator`` may also write Parameters and use the introspection
mutators. The Machine has a Component named Platform (Monitoring, ParameterSet, SkillSet,
MethodSet, Identification, Notification) and one named ROS2Introspection, whose Methods list the
topics, services, actions and parameters on the graph and the Bindings declared.

``Runtime`` is imported inside :func:`main` because it is the one name in ``opensmi.ros2.runtime``
that needs ``rclpy`` to resolve. This keeps ``import examples.run_server`` (as
``tests/test_examples.py`` does) free of a ROS2 installation.
"""

from __future__ import annotations

import asyncio
import contextlib
from pathlib import Path

from opensmi.server import Server

from examples.machine import Robot
from opensmi.ros2.server import ROS2Config

ROS2_CONFIG_PATH = Path(__file__).parent / "ros2.toml"


async def main() -> None:
    """Load the ROS2 settings, build the Runtime, and serve the Robot until stopped."""
    from opensmi.ros2.runtime import Runtime

    config = ROS2Config.from_toml(ROS2_CONFIG_PATH)
    runtime = Runtime(domain_id=config.domain_id, node_name=config.node_name, namespace=config.namespace)

    async with Server() as server:
        await server.add_machine(Robot(runtime=runtime, ros2_config=config))
        await server.start(blocking=True)


if __name__ == "__main__":
    with contextlib.suppress(KeyboardInterrupt):
        asyncio.run(main())
