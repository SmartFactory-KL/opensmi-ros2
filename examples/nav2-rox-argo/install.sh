#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT
#
# Installs what the example needs into a ros:jazzy image. The Dockerfile runs it one step at a time,
# so each step is a layer of its own; ci.sh runs `install.sh all` in its CI job container.
#
#   install.sh packages    Gazebo, Nav2 and the rest from packages.ros.org and Ubuntu;
#                          UBUNTU_MIRROR, if set, replaces archive.ubuntu.com and security.ubuntu.com
#   install.sh upstream    the Neobotix packages, fetched at the commits below and built with colcon
#   install.sh opensmi     opensmi-ros2 from the repository checkout, into /opt/venv
#   install.sh example     the map drawn from world.sdf, and the Nav2 parameters (see below)
#   install.sh all         all four, in that order
#
# Paths: the repository checkout is REPO (default: the current directory), the example's files are
# EXAMPLE (default: its folder in REPO; the derived files are written there), the colcon workspace
# goes to /opt/rox_ws. The commits of the Neobotix sources are written to /opt/rox_ws/sources.env,
# which entrypoint.sh and ci.sh read, so the scenario's report can name them.
set -euo pipefail

REPO="${REPO:-$PWD}"
EXAMPLE="${EXAMPLE:-$REPO/examples/nav2-rox-argo}"
WS=/opt/rox_ws

# The Neobotix repositories on GitHub, each at a pinned commit (licences: see the example's README).
SOURCES=(
    "rox 0ce87400074c186bbbcd4625af0d0e2dff9e74bf ROX_COMMIT"
    "neo_nav2_bringup 71676a3644e0a820188cef5ac464d4bc60783a93 NEO_NAV2_BRINGUP_COMMIT"
    "neo_localization2 7806584bd98e630eaef3b03c27b7eb7776302787 NEO_LOCALIZATION2_COMMIT"
    "neo_local_planner2 0ad9a086f4e6811debda1b245850aae32639e85f NEO_LOCAL_PLANNER2_COMMIT"
    "neo_gz_worlds acfcd704938a0d2e67de9e346abf10317db1bd8a NEO_GZ_WORLDS_COMMIT"
)

packages() {
    # A download that stalls is retried rather than waited for.
    printf 'Acquire::http::Timeout "30";\nAcquire::Retries "5";\n' > /etc/apt/apt.conf.d/80-retries
    if [ -n "${UBUNTU_MIRROR:-}" ]; then
        sed -i -E "s#https?://(archive|security)\.ubuntu\.com/ubuntu/?#${UBUNTU_MIRROR%/}/#g" \
            /etc/apt/sources.list.d/ubuntu.sources
    fi
    apt-get update
    # Gazebo Harmonic (through ros_gz), Nav2, what rox_bringup's simulation launch starts (including
    # the keyboard teleop it opens in an xterm), the build tools, Xvfb for the scanners' rendering
    # without a display, and Cyclone DDS.
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        git build-essential python3-colcon-common-extensions python3-venv python3-pip xvfb xterm \
        ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-nav2-common ros-jazzy-topic-tools \
        ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-diagnostic-msgs \
        ros-jazzy-teleop-twist-keyboard ros-jazzy-rmw-cyclonedds-cpp
    rm -rf /var/lib/apt/lists/*
}

upstream() {
    mkdir -p "$WS/src"
    : > "$WS/sources.env"
    local entry name commit variable
    for entry in "${SOURCES[@]}"; do
        read -r name commit variable <<< "$entry"
        rm -rf "${WS:?}/src/$name"
        git init -q "$WS/src/$name"
        git -C "$WS/src/$name" fetch -q --depth 1 "https://github.com/neobotix/$name.git" "$commit"
        git -C "$WS/src/$name" checkout -q FETCH_HEAD
        rm -rf "$WS/src/$name/.git"
        echo "export $variable=$commit" >> "$WS/sources.env"
    done
    # ROS2's setup scripts read unset variables.
    set +u
    # shellcheck disable=SC1091
    . /opt/ros/jazzy/setup.bash
    set -u
    (cd "$WS" && colcon build --merge-install --event-handlers console_direct- \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF)
    rm -rf "$WS/build" "$WS/log"
}

opensmi() {
    # A venv that sees the system's ROS2 packages (rclpy), as in the repository's CI.
    python3 -m venv --system-site-packages /opt/venv
    /opt/venv/bin/python -m pip install --no-cache-dir --upgrade pip
    /opt/venv/bin/python -m pip install --no-cache-dir -c "$REPO/constraints.txt" "$REPO"
}

example() {
    python3 "$EXAMPLE/make_map.py" "$EXAMPLE/world.sdf" "$EXAMPLE/map"
    # Neobotix's Nav2 parameters for the Argo, with one change: the behavior tree waits 200 ms instead
    # of 20 ms for a Nav2 server to accept a goal. On a busy host (several simulations, the recorder,
    # a shared CI runner) 20 ms runs out and the drive aborts.
    sed 's/^\( *default_server_timeout:\) 20$/\1 200/' \
        "$WS/install/share/rox_navigation/configs/navigation_omni.yaml" > "$EXAMPLE/navigation.yaml"
    test "$(grep -c 'default_server_timeout: 200$' "$EXAMPLE/navigation.yaml")" -ge 1
}

case "${1:-}" in
    packages) packages ;;
    upstream) upstream ;;
    opensmi) opensmi ;;
    example) example ;;
    all)
        packages
        upstream
        opensmi
        example
        ;;
    *)
        echo "usage: install.sh packages|upstream|opensmi|example|all" >&2
        exit 2
        ;;
esac
