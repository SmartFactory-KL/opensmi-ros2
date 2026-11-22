#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT
#
# The HC10 example in CI, without Docker: the `example:hc10` job runs this in a plain ros:jazzy job
# container, on a runner that hands in no Docker socket. It installs what the example's image holds
# with the same docker/install.sh, starts the simulation (Gazebo, ros2_control, MoveIt) and the
# Machine as background processes, runs scenario.py against them and stops both again, also when a
# step fails. The report and both logs land in reports/, which the job keeps as an artifact.
#
# Run from the repository root, as root (it installs packages): examples/moveit-hc10-robotiq/ci.sh
# The exit status is the scenario's: 0 only if every step passed.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export REPO
REPO="$(cd "$here/../.." && pwd)"

bash "$here/docker/install.sh" all

# ROS2's setup scripts read unset variables.
set +u
# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
. /opt/hc10_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=72 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp GZ_PARTITION=hc10-ci
export PATH="/opt/venv/bin:$PATH"

cd "$here"
mkdir -p reports
started=()
cleanup() {
    # Each process was started in a session of its own, so its whole group (launch's children,
    # Gazebo's server) gets the signal.
    for pid in "${started[@]}"; do
        kill -INT -- "-$pid" 2>/dev/null || true
    done
    sleep 10
    for pid in "${started[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT

setsid ros2 launch hc10_robotiq_moveit_config bringup.launch.py > reports/sim.log 2>&1 &
started+=("$!")
setsid opensmi-ros2 serve machine.toml > reports/machine.log 2>&1 &
started+=("$!")

status=0
OPCUA_URL=opc.tcp://127.0.0.1:4840/server REPORT=reports/report.md python scenario.py || status=$?
exit "$status"
