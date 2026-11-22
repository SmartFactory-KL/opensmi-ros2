#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT
#
# Sources ROS2 Jazzy and the example's workspace, then runs the command it was given.
set -eo pipefail
# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
. /opt/hc10_ws/install/setup.bash
exec "$@"
