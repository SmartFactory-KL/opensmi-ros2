#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT
#
# The ROX Argo example in CI, without Docker: the `example:rox-argo` job runs this in a plain ros:jazzy
# job container, on a runner that hands in no Docker socket. It installs what the example's image
# holds with the same install.sh, starts the simulation (Gazebo on a virtual display, Nav2) as a
# background process, waits until Nav2 reports itself active, starts the Machine, runs scenario.py
# against it and stops everything again, also when a step fails. The report and the logs land in
# reports/, which the job keeps as an artifact.
#
# Run from the repository root, as root (it installs packages): examples/nav2-rox-argo/ci.sh
# The exit status is the scenario's: 0 only if every step passed.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export REPO
REPO="$(cd "$here/../.." && pwd)"
timeout="${ROX_STARTUP_TIMEOUT:-600}"

bash "$here/install.sh" all

# ROS2's setup scripts read unset variables.
set +u
# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
. /opt/rox_ws/install/setup.bash
# shellcheck disable=SC1091
. /opt/rox_ws/sources.env
set -u
export ROS_DOMAIN_ID=71 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp GZ_PARTITION=opensmi-ex-rox-ci DISPLAY=:99
export PATH="/opt/venv/bin:$PATH"
export OPENSMI_ROS_COMMIT="${CI_COMMIT_SHA:-$(git -C "$REPO" rev-parse HEAD 2>/dev/null || echo unknown)}"

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

# The laser scanners render even without a window, so Gazebo gets a virtual display.
setsid Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp > reports/xvfb.log 2>&1 &
started+=("$!")
setsid ros2 launch "$here/simulation_launch.py" > reports/sim.log 2>&1 &
started+=("$!")

# What compose.yaml's health check asks: both Nav2 lifecycle managers report their nodes active.
active() {
    local manager
    for manager in localization navigation; do
        timeout 20 ros2 service call "/lifecycle_manager_$manager/is_active" std_srvs/srv/Trigger \
            | grep -q success=True || return 1
    done
}
deadline=$((SECONDS + timeout))
until active >/dev/null 2>&1; do
    if [ "$SECONDS" -ge "$deadline" ]; then
        echo "Nav2 was not active after ${timeout} s; see reports/sim.log" >&2
        exit 1
    fi
    sleep 10
done
echo "Nav2 is active after ${SECONDS} s."

setsid opensmi-ros2 serve machine.toml > reports/machine.log 2>&1 &
started+=("$!")

status=0
python scenario.py --endpoint opc.tcp://127.0.0.1:4840/server \
    --output "reports/rox-argo-${OPENSMI_ROS_COMMIT:0:7}.md" || status=$?
exit "$status"
