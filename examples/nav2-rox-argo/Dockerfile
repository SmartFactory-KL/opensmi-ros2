# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT
#
# One image for every service of the example (the stage `example`): the Gazebo simulation with Nav2,
# the Machine and the scenario. The stage `recorder` adds ffmpeg for record.sh. Build from the
# repository root (compose.yaml does), so opensmi-ros2 is installed from this checkout:
#
#   docker build -f examples/nav2-rox-argo/Dockerfile --target example .
#
# What goes in is install.sh's, one step per layer; the CI job (ci.sh) runs the same script. The
# Neobotix packages are cloned from GitHub at the commits install.sh pins and built here; nothing of
# them is carried in this repository. Their licences are listed in the example's README.

# ros:jazzy-ros-base, the same digest the repository's CI pins.
FROM ros:jazzy-ros-base@sha256:c3706ef0a0aa45413c07803cf433602f543b22e45b4855f6fca955c2d8ecc4e8 AS example

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV REPO=/opt/opensmi-ros2 EXAMPLE=/opt/example

COPY examples/nav2-rox-argo/install.sh /opt/install.sh
# An Ubuntu mirror to use instead of archive.ubuntu.com and security.ubuntu.com, for example
# http://ftp.fau.de/ubuntu when those are slow from where you build. Empty keeps the image's own.
ARG UBUNTU_MIRROR=
RUN UBUNTU_MIRROR="$UBUNTU_MIRROR" bash /opt/install.sh packages
RUN bash /opt/install.sh upstream

# opensmi-ros2 from this checkout, against the opensmi releases constraints.txt pins.
COPY pyproject.toml constraints.txt README.md /opt/opensmi-ros2/
COPY LICENSES /opt/opensmi-ros2/LICENSES
COPY src /opt/opensmi-ros2/src
RUN bash /opt/install.sh opensmi

# The example itself, the map drawn from its world, and its Nav2 parameters.
COPY examples/nav2-rox-argo /opt/example
RUN bash /opt/install.sh example

# The opensmi-ros2 commit the image is built from, for the scenario's report (run.sh passes it).
# Declared this late so that a new commit does not rebuild the layers above.
ARG OPENSMI_ROS_COMMIT=unknown
ENV PATH=/opt/venv/bin:$PATH \
    OPENSMI_ROS_COMMIT=$OPENSMI_ROS_COMMIT
WORKDIR /opt/example
ENTRYPOINT ["bash", "/opt/example/entrypoint.sh"]

# The recorder of record.sh: the same image, plus ffmpeg and a font for the captions.
FROM example AS recorder
RUN apt-get update && apt-get install -y --no-install-recommends ffmpeg fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*
