#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT
#
# Films one run of the scenario: run.sh with a camera. Gazebo's GUI runs on a virtual X server in a
# container of its own (no desktop needed), ffmpeg captures it, and afterwards the capture is cut
# to the pick and place, every step is captioned from the scenario's timeline, and the result is
# exported. Everything it started is removed again, also when a step fails.
#
#   ./record.sh         media/hc10-pick-and-place.mp4 and media/hc10-pick-and-place.png
#
# Exits with the scenario's status; the video is written either way, failed steps marked.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
compose=(docker compose -p opensmi-ex-hc10 -f "$here/compose.yaml" --profile scenario --profile record)
inside=/src/examples/moveit-hc10-robotiq
raw="$here/reports/recording"
mkdir -p "$here/reports" "$here/media"

cleanup() {
    "${compose[@]}" logs --no-color --timestamps > "$here/reports/stack.log" 2>&1 || true
    "${compose[@]}" down --remove-orphans --timeout 20 > /dev/null 2>&1 || true
}
trap cleanup EXIT

"${compose[@]}" build sim
"${compose[@]}" build recorder
"${compose[@]}" up -d sim machine recorder

# The scenario starts once the camera runs.
for _ in $(seq 1 120); do
    [ -s "$raw/capture-start" ] && break
    sleep 1
done
[ -s "$raw/capture-start" ] || { echo "the recorder did not start capturing" >&2; exit 1; }

status=0
"${compose[@]}" run --rm -e TIMELINE="$inside/reports/recording/timeline.json" scenario || status=$?

# A few seconds more for the video's tail, then SIGINT, which lets ffmpeg finish the file.
sleep 4
"${compose[@]}" kill -s INT recorder > /dev/null
"${compose[@]}" wait recorder > /dev/null 2>&1 || true
"${compose[@]}" run --rm --no-deps --entrypoint python3 recorder record/caption.py \
    "$inside/reports/recording" "$inside/media/hc10-pick-and-place.mp4" "$inside/media/hc10-pick-and-place.png"
exit "$status"
