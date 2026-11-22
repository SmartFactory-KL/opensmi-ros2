# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Expose a ROS2 system as an OPC UA server, built on opensmi.server and the SkillSet information model.

A ROS2 robot shows up to an OPC UA client as a Machine with Components that carry Skills, Methods and
Monitoring variables, not as a list of topics, services and actions. You declare what to expose with
Bindings on a Component. A Binding ties one ROS2 noun to its place in the address space:

========================  =====================================================================
ROS2 side                 OPC UA side
========================  =====================================================================
Topic, read               Monitoring variables
Topic, published          Skill
Service                   Method
Action                    Skill, with feedback and a Result
Lifecycle node            StartupSkill
TF tree                   RSL Locations inside a SpatialObject
Diagnostics topic         One member per diagnostic id, and an event per level change
Node parameters           Parameters, writable, with ROS2's own answer as the status code
========================  =====================================================================

The distribution is OpenSMI-ROS2, imported as the namespace package ``opensmi.ros2`` next to
``opensmi.server`` and ``opensmi.core``. It has two halves:

* :mod:`opensmi.ros2.runtime` owns ``rclpy`` and knows nothing about OPC UA. It imports only the
  standard library, ``rclpy`` and ``tf2_ros`` from ROS2, ``structlog`` and ``typing-extensions``.
* :mod:`opensmi.ros2.server` is the OPC UA layer on top of ``opensmi.server``. Start there.

Requirements: a sourced ROS2 Jazzy (no other distribution), Linux and Python 3.11 or newer. ``rclpy``
comes from the ROS2 installation, not from PyPI. On WSL, Fast DDS needs ``FASTDDS_BUILTIN_TRANSPORTS=SHM``.

A whole server with one Component that reads the ``std_msgs/String`` topic ``/chatter``, as the
ROS2 demo talker publishes it:

.. code-block:: python

    import asyncio

    from asyncua import ua
    from opensmi.server import BaseComponent, BaseMachine, Server

    from opensmi.ros2.runtime import Runtime
    from opensmi.ros2.server import MonitoringBinding, ROS2ComponentMixin, ROS2MachineMixin


    class Talker(ROS2ComponentMixin, BaseComponent):
        bindings = (MonitoringBinding(name="Chatter", path="/chatter", message_type="std_msgs/msg/String"),)

        async def _write_identification(self) -> None:
            await self.identification.SerialNumber.write("talker-0001")
            await self.identification.Manufacturer.write(ua.LocalizedText("Example", "en-US"))


    class Robot(ROS2MachineMixin, BaseMachine):
        async def _write_identification(self) -> None:
            await self.identification.SerialNumber.write("robot-0001")
            await self.identification.ProductInstanceUri.write("urn:example:robot")
            await self.identification.Manufacturer.write(ua.LocalizedText("Example", "en-US"))

        async def _init(self) -> None:
            await super()._init()
            await self.add_component(Talker(name="Talker"))


    async def main() -> None:
        async with Server() as server:
            await server.add_machine(Robot(runtime=Runtime(domain_id=0)))
            await server.start(blocking=True)


    asyncio.run(main())

Start ``ros2 run demo_nodes_py talker``, run the script and connect a client. The Component Talker
then has a Monitoring variable ``Chatter/Data`` that changes with every message. Mixins go before the
opensmi base class. The application creates the Runtime and hands it to the Machine, which starts it
before anything is built and stops it afterwards.

The other kinds work the same way: :class:`~opensmi.ros2.server.MethodBinding`,
:class:`~opensmi.ros2.server.ActionBinding`, :class:`~opensmi.ros2.server.PublisherBinding`,
:class:`~opensmi.ros2.server.ROS2ParameterBinding`, :class:`~opensmi.ros2.server.LifecycleBinding`,
:class:`~opensmi.ros2.server.KeyedBinding` and :class:`~opensmi.ros2.server.TFBinding`. The example
Component in the repository (``examples/component.py``) declares one of each. A server can also come up
from one TOML file with ``opensmi-ros2 serve machine.toml``; see :mod:`opensmi.ros2.server.loader` and
:mod:`opensmi.ros2.cli`.

Not supported yet: removing Bindings or nodes at runtime, and a suspendable StartupSkill.
"""
