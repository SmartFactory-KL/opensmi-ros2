#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT
#
# Installs what the example needs into a ros:jazzy image. The Dockerfile runs it one step at a time,
# so each step is a layer of its own; the CI job runs `install.sh all` in its job container.
#
#   install.sh packages    the ROS2 packages from packages.ros.org (MoveIt 2, Gazebo, ros2_control, ...);
#                          UBUNTU_MIRROR, if set, replaces archive.ubuntu.com
#   install.sh upstream    the HC10 description, fetched at a pinned commit and wrapped for ROS2
#   install.sh workspace   our own description and MoveIt packages, built with colcon
#   install.sh opensmi     opensmi-ros2 from the repository checkout, into /opt/venv
#   install.sh all         all four, in that order
#
# Paths: the repository checkout is REPO (default: the current directory), the colcon workspace
# goes to /opt/hc10_ws.
set -euo pipefail

REPO="${REPO:-$PWD}"
EXAMPLE="$REPO/examples/moveit-hc10-robotiq"
WS=/opt/hc10_ws

# ros-industrial/motoman, branch kinetic-devel: the only HC10 description there is (ROS1, BSD).
MOTOMAN_URL=https://github.com/ros-industrial/motoman.git
MOTOMAN_COMMIT=f951b2863cb294eb4e049c8a1ff4b3da3548287a

packages() {
    # A download that stalls is retried rather than waited for.
    printf 'Acquire::http::Timeout "30";\nAcquire::Retries "5";\n' > /etc/apt/apt.conf.d/80-retries
    if [ -n "${UBUNTU_MIRROR:-}" ]; then
        # Another Ubuntu mirror than the image's, for where archive.ubuntu.com is slow.
        sed -i "s#http://archive.ubuntu.com/ubuntu#$UBUNTU_MIRROR#g" /etc/apt/sources.list.d/ubuntu.sources
    fi
    apt-get update
    # The base image is pinned, packages.ros.org is not: bring what is installed up to the same
    # sync, so everything comes from one consistent set of ROS2 packages.
    apt-get dist-upgrade -y
    # The parts of MoveIt, Gazebo and ros2_control the example uses, rather than their meta
    # packages, which bring perception, RViz and the rest of Gazebo's bridges along.
    apt-get install -y --no-install-recommends \
        git python3-pip python3-venv python3-colcon-common-extensions \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-robotiq-description \
        ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-gz-ros2-control \
        ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller \
        ros-jazzy-parallel-gripper-controller \
        ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-planners-ompl \
        ros-jazzy-pilz-industrial-motion-planner ros-jazzy-moveit-kinematics \
        ros-jazzy-moveit-simple-controller-manager ros-jazzy-moveit-configs-utils
    rm -rf /var/lib/apt/lists/*
}

upstream() {
    local checkout
    checkout="$(mktemp -d)"
    git -C "$checkout" init -q
    git -C "$checkout" fetch -q --depth 1 "$MOTOMAN_URL" "$MOTOMAN_COMMIT"
    git -C "$checkout" checkout -q FETCH_HEAD
    mkdir -p "$WS/src"
    python3 "$EXAMPLE/docker/port_hc10.py" "$checkout" "$WS/src"
    rm -rf "$checkout"
}

workspace() {
    mkdir -p "$WS/src"
    cp -r "$EXAMPLE/ros2_ws/src/." "$WS/src/"
    # ROS2's setup scripts read unset variables.
    set +u
    # shellcheck disable=SC1091
    . /opt/ros/jazzy/setup.bash
    set -u
    (cd "$WS" && colcon build --merge-install --event-handlers console_direct-)
    rm -rf "$WS/build" "$WS/log"
}

opensmi() {
    # A venv that sees the system's ROS2 packages (rclpy), as in the repository's CI.
    python3 -m venv --system-site-packages /opt/venv
    /opt/venv/bin/python -m pip install --no-cache-dir --upgrade pip
    /opt/venv/bin/python -m pip install --no-cache-dir -c "$REPO/constraints.txt" "$REPO"
}

case "${1:-}" in
    packages) packages ;;
    upstream) upstream ;;
    workspace) workspace ;;
    opensmi) opensmi ;;
    all)
        packages
        upstream
        workspace
        opensmi
        ;;
    *)
        echo "usage: install.sh packages|upstream|workspace|opensmi|all" >&2
        exit 2
        ;;
esac
