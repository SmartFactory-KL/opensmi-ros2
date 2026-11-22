#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT
#
# The ROX Argo example in one command: build the image, start the simulation and the Machine, run the
# scenario against the Machine, copy its report to reports/, and remove everything it started, also
# when a step fails or the run is interrupted.
#
#   examples/nav2-rox-argo/run.sh [REPORT_FILE]
#
# The report goes to reports/rox-argo-<commit>.md next to this script unless a file is named. The
# exit status is the scenario's: 0 only if every step passed. It uses the compose project
# opensmi-ex-rox and refuses to start while that project already runs, so it never stops a run that
# somebody else started.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$here"
compose=(docker compose -p opensmi-ex-rox -f compose.yaml --profile scenario)
scenario=opensmi-ex-rox-scenario

if [ -n "$("${compose[@]}" ps -q 2>/dev/null)" ]; then
    echo "The example already runs (compose project opensmi-ex-rox); stop it with 'docker compose down' first." >&2
    exit 1
fi

OPENSMI_ROS_COMMIT="${OPENSMI_ROS_COMMIT:-$(git rev-parse HEAD 2>/dev/null || echo unknown)}"
export OPENSMI_ROS_COMMIT
# The scenario reaches the Machine on the compose network; the port published on the host only has to
# be free, so unless one is named Docker picks one (4840 is often taken by another OPC UA server).
export OPCUA_PORT="${OPCUA_PORT-}"
report="${1:-$here/reports/rox-argo-${OPENSMI_ROS_COMMIT:0:7}.md}"

cleanup() {
    status=$?
    echo "Removing the example's containers and network." >&2
    "${compose[@]}" logs --no-color --tail 200 sim machine >"$here/reports/compose.log" 2>&1 || true
    docker rm --force "$scenario" >/dev/null 2>&1 || true
    "${compose[@]}" down --remove-orphans --timeout 20 >/dev/null 2>&1 || true
    exit "$status"
}
mkdir -p "$here/reports"
trap cleanup EXIT
trap 'exit 130' INT TERM

"${compose[@]}" build sim
# Waits until the simulation is healthy (Nav2 active) and the Machine has started.
"${compose[@]}" up --detach --wait --wait-timeout 900 sim machine

status=0
"${compose[@]}" run -T --name "$scenario" scenario || status=$?
mkdir -p "$(dirname "$report")"
docker cp "$scenario:/reports/report.md" "$report" && echo "report: $report"
exit "$status"
