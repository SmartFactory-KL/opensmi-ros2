# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Wrap the ROS1 HC10 description of ros-industrial/motoman as two ROS2 packages, at image build time.

Upstream is a catkin package (``motoman_hc10_support``, plus ``motoman_resources`` for the colours its
macro includes). Nothing of it is kept in this repository: ``install.sh upstream`` clones the
repository at a pinned commit, and this script turns the two packages into ament packages in a
colcon workspace:

* the ``urdf/`` and ``meshes/hc10/`` folders are copied as they are;
* ``package.xml`` and ``CMakeLists.txt`` are replaced by ament ones written here;
* ``urdf/hc10_macro.xacro`` gets an ``<inertial>`` on every link that moves, which the ROS1 macro does
  not have and Gazebo needs. The masses add up to the HC10's 47 kg; the inertias are rough
  estimates from the shape of each link, good enough for a position-controlled arm in simulation and
  for nothing else.

Usage: ``python3 port_hc10.py <motoman checkout> <workspace src folder>``
"""

from __future__ import annotations

import re
import shutil
import sys
from pathlib import Path

INERTIA = {
    # link: (mass in kg, centre of mass xyz in the link frame, ixx, iyy, izz)
    "base_link": (6.0, (0.0, 0.0, 0.10), 0.05, 0.05, 0.05),
    "link_1_s": (9.0, (0.0, 0.0, 0.0), 0.08, 0.08, 0.06),
    "link_2_l": (10.0, (0.0, 0.0, 0.35), 0.45, 0.45, 0.04),
    "link_3_u": (6.0, (0.0, 0.0, 0.0), 0.05, 0.05, 0.04),
    "link_4_r": (8.0, (0.0, 0.0, 0.25), 0.18, 0.18, 0.02),
    "link_5_b": (5.0, (0.0, 0.0, 0.0), 0.02, 0.02, 0.02),
    "link_6_t": (3.0, (0.0, 0.0, -0.06), 0.01, 0.01, 0.01),
}
"""The inertial of each moving link, added to the upstream macro."""

PACKAGE_XML = """<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>0.0.0</version>
  <description>{description}</description>
  <maintainer email="noreply@example.com">OpenSMI-ROS2 example</maintainer>
  <license>BSD-3-Clause</license>
  <url type="repository">https://github.com/ros-industrial/motoman</url>
  <buildtool_depend>ament_cmake</buildtool_depend>
{depends}  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""

CMAKELISTS = """cmake_minimum_required(VERSION 3.8)
project({name})
find_package(ament_cmake REQUIRED)
install(DIRECTORY {folders} DESTINATION share/${{PROJECT_NAME}})
ament_package()
"""


def inertial(link: str) -> str:
    """Return the ``<inertial>`` element of one link."""
    mass, (x, y, z), ixx, iyy, izz = INERTIA[link]
    return (
        f'\n      <inertial>\n        <origin xyz="{x} {y} {z}" rpy="0 0 0"/>\n        <mass value="{mass}"/>\n'
        f'        <inertia ixx="{ixx}" ixy="0" ixz="0" iyy="{iyy}" iyz="0" izz="{izz}"/>\n      </inertial>'
    )


def add_inertials(macro: str) -> str:
    """Return the macro with an inertial inserted at the start of every link in :data:`INERTIA`.

    :raises ValueError: When a link is not found exactly once, because upstream changed.
    """
    for link in INERTIA:
        pattern = re.compile(rf'(<link name="\$\{{prefix\}}{link}">)')
        macro, count = pattern.subn(lambda found, link=link: found.group(1) + inertial(link), macro)
        if count != 1:
            message = f"the HC10 macro has {count} links named {link!r}, expected one"
            raise ValueError(message)
    return macro


def package(destination: Path, name: str, description: str, folders: list[str], depends: list[str]) -> None:
    """Write the ament ``package.xml`` and ``CMakeLists.txt`` of one wrapped package."""
    lines = "".join(f"  <exec_depend>{each}</exec_depend>\n" for each in depends)
    (destination / "package.xml").write_text(
        PACKAGE_XML.format(name=name, description=description, depends=lines), encoding="utf-8"
    )
    (destination / "CMakeLists.txt").write_text(
        CMAKELISTS.format(name=name, folders=" ".join(folders)), encoding="utf-8"
    )


def main() -> None:
    """Copy and wrap the two packages."""
    checkout, src = Path(sys.argv[1]), Path(sys.argv[2])

    resources = src / "motoman_resources"
    shutil.copytree(checkout / "motoman_resources" / "urdf", resources / "urdf")
    package(resources, "motoman_resources", "Colours of the Motoman descriptions (ported from ROS1).", ["urdf"], [])

    support = src / "motoman_hc10_support"
    shutil.copytree(checkout / "motoman_hc10_support" / "urdf", support / "urdf")
    shutil.copytree(checkout / "motoman_hc10_support" / "meshes" / "hc10", support / "meshes" / "hc10")
    macro = support / "urdf" / "hc10_macro.xacro"
    macro.write_text(add_inertials(macro.read_text(encoding="utf-8")), encoding="utf-8")
    package(
        support,
        "motoman_hc10_support",
        "The Motoman HC10 description of ros-industrial/motoman, wrapped for ROS2.",
        ["urdf", "meshes"],
        ["motoman_resources", "xacro"],
    )


if __name__ == "__main__":
    main()
