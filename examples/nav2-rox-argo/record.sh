#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT
#
# Film one scenario run: like run.sh, plus the recorder service, which shows the Gazebo window on a
# virtual display inside its container and films it with ffmpeg. Needs no display on the host. When
# the scenario is done, recording/make_video.py cuts the film to the steps, speeds it up, captions each
# step with its result, and the video and a poster frame are copied to media/:
#
#   examples/nav2-rox-argo/record.sh
#
# The report and the step timeline go to reports/ as with run.sh. The exit status is the scenario's.
# Everything it started is removed afterwards, also on failure or Ctrl+C.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$here"
compose=(docker compose -p opensmi-ex-rox -f compose.yaml --profile scenario --profile record)
scenario=opensmi-ex-rox-scenario

if [ -n "$("${compose[@]}" ps -q 2>/dev/null)" ]; then
    echo "The example already runs (compose project opensmi-ex-rox); stop it with 'docker compose down' first." >&2
    exit 1
fi

OPENSMI_ROS_COMMIT="${OPENSMI_ROS_COMMIT:-$(git rev-parse HEAD 2>/dev/null || echo unknown)}"
export OPENSMI_ROS_COMMIT
export OPCUA_PORT="${OPCUA_PORT-}"
reports="$here/reports"

cleanup() {
    status=$?
    echo "Removing the example's containers and network." >&2
    "${compose[@]}" logs --no-color --tail 200 sim machine recorder >"$reports/compose.log" 2>&1 || true
    docker rm --force "$scenario" >/dev/null 2>&1 || true
    "${compose[@]}" down --remove-orphans --timeout 20 >/dev/null 2>&1 || true
    exit "$status"
}
mkdir -p "$reports" "$here/media"
trap cleanup EXIT
trap 'exit 130' INT TERM

"${compose[@]}" build sim recorder
"${compose[@]}" up --detach --wait --wait-timeout 900 sim machine
"${compose[@]}" up --detach recorder
echo "Waiting for the recorder to start filming."
for _ in $(seq 1 60); do
    "${compose[@]}" exec -T recorder test -f /tmp/recording/raw.start 2>/dev/null && break
    sleep 5
done
"${compose[@]}" exec -T recorder test -f /tmp/recording/raw.start

status=0
"${compose[@]}" run -T --name "$scenario" scenario python /opt/example/scenario.py \
    --endpoint opc.tcp://machine:4840/server --output /reports/report.md --timeline /reports/timeline.json \
    || status=$?
docker cp "$scenario:/reports/report.md" "$reports/rox-argo-${OPENSMI_ROS_COMMIT:0:7}.md"
docker cp "$scenario:/reports/timeline.json" "$reports/timeline.json"

# Finish the film, then cut it.
"${compose[@]}" exec -T recorder bash -c 'kill -INT "$(cat /tmp/recording/ffmpeg.pid)"'
for _ in $(seq 1 60); do
    "${compose[@]}" exec -T recorder test -f /tmp/recording/raw.done && break
    sleep 1
done
recorder="$("${compose[@]}" ps -q recorder)"
docker cp "$reports/timeline.json" "$recorder:/tmp/recording/timeline.json"
"${compose[@]}" exec -T recorder python3 /opt/example/recording/make_video.py /tmp/recording
docker cp "$recorder:/tmp/recording/rox-argo.mp4" "$here/media/rox-argo.mp4"
docker cp "$recorder:/tmp/recording/rox-argo-poster.png" "$here/media/rox-argo-poster.png"
echo "video: $here/media/rox-argo.mp4"
exit "$status"
