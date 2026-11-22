#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT
#
# Every service of the example starts here: ROS2 Jazzy and the Neobotix workspace sourced, the
# Neobotix commits in the environment (for the scenario's report), then the service's command.
set -eo pipefail
# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
. /opt/rox_ws/install/setup.bash
# shellcheck disable=SC1091
. /opt/rox_ws/sources.env
exec "$@"
