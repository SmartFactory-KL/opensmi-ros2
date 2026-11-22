# syntax=docker/dockerfile:1
# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT
#
# One image for the whole example: Gazebo, ros2_control, MoveIt, the Machine served by
# opensmi-ros2, and the scenario client. Build it from the repository root, so opensmi-ros2 is
# installed from this checkout:
#
#   docker build -f examples/moveit-hc10-robotiq/Dockerfile -t opensmi-ex-hc10:local .
#
# compose.yaml next to this file does that for you. Each step of docker/install.sh is a layer of its
# own, so a change to the example's ROS2 packages or to opensmi-ros2 rebuilds only what follows it.

# ros:jazzy-ros-base, the image the repository's CI pins (resolved 2026-09-19).
FROM ros@sha256:c3706ef0a0aa45413c07803cf433602f543b22e45b4855f6fca955c2d8ecc4e8

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV REPO=/src
# Another Ubuntu mirror than archive.ubuntu.com, e.g. http://de.archive.ubuntu.com/ubuntu, if that one is slow.
ARG UBUNTU_MIRROR=

COPY examples/moveit-hc10-robotiq/docker/install.sh /src/examples/moveit-hc10-robotiq/docker/install.sh
RUN UBUNTU_MIRROR="$UBUNTU_MIRROR" bash /src/examples/moveit-hc10-robotiq/docker/install.sh packages

COPY examples/moveit-hc10-robotiq/docker/port_hc10.py /src/examples/moveit-hc10-robotiq/docker/port_hc10.py
RUN bash /src/examples/moveit-hc10-robotiq/docker/install.sh upstream

COPY examples/moveit-hc10-robotiq/ros2_ws /src/examples/moveit-hc10-robotiq/ros2_ws
RUN bash /src/examples/moveit-hc10-robotiq/docker/install.sh workspace

COPY pyproject.toml constraints.txt README.md /src/
COPY LICENSES /src/LICENSES
COPY src /src/src
RUN bash /src/examples/moveit-hc10-robotiq/docker/install.sh opensmi

COPY examples/moveit-hc10-robotiq /src/examples/moveit-hc10-robotiq

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PATH=/opt/venv/bin:$PATH
WORKDIR /src/examples/moveit-hc10-robotiq
ENTRYPOINT ["bash", "/src/examples/moveit-hc10-robotiq/docker/entrypoint.sh"]
