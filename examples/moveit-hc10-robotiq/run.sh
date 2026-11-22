#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT
#
# The whole example in one command: builds the image, starts Gazebo, MoveIt and the Machine, runs
# the pick and place through OPC UA, and removes every container and the network again, also when
# a step fails or the run is interrupted. The image stays, so the next run starts quickly.
#
#   ./run.sh            report in reports/report.md, the stack's log in reports/stack.log
#
# Exits with the scenario's status: 0 only if every step passed.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
compose=(docker compose -p opensmi-ex-hc10 -f "$here/compose.yaml" --profile scenario)
mkdir -p "$here/reports"

cleanup() {
    "${compose[@]}" logs --no-color --timestamps > "$here/reports/stack.log" 2>&1 || true
    "${compose[@]}" down --remove-orphans --timeout 20 > /dev/null 2>&1 || true
}
trap cleanup EXIT

"${compose[@]}" build sim
"${compose[@]}" up -d sim machine
status=0
"${compose[@]}" run --rm scenario || status=$?
exit "$status"
