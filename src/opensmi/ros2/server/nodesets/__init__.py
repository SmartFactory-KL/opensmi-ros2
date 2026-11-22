# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The nodeset XML this project ships, and the NodeId class generated from it.

A package so that the XML lands in the wheel as package data and opensmi.server can load it by name
(``Server.ua_register_nodeset_xml``). :mod:`opensmi.ros2.server.events` registers and imports it.
:class:`SmartFactoryROS2NodeIds` is generated from the XML with opensmi's nodeset generator and
committed; every NodeId of ours is read off it.
"""

from opensmi.ros2.server.nodesets.smart_factory_ros2 import SmartFactoryROS2NodeIds

__all__ = ["PACKAGE", "SmartFactoryROS2NodeIds"]

PACKAGE = __name__
"""The package the nodeset XML is installed in, which is what the server loads it from."""
