#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT
#
# Runs in the recorder container (see compose.yaml): Gazebo's GUI on a virtual X server, joined to
# the headless simulation, and ffmpeg capturing that screen until it is stopped (SIGINT or SIGTERM).
# Writes OUT/raw.mkv, and OUT/capture-start with the wall-clock time the capture began, which
# caption.py lines the scenario's step timeline up with.
set -euo pipefail

out="${OUT:-/out}"
mkdir -p "$out"
rm -f "$out/raw.mkv" "$out/capture-start"

Xvfb :99 -screen 0 1280x720x24 -nolisten tcp > "$out/xvfb.log" 2>&1 &
export DISPLAY=:99
sleep 2
gz sim -g --gui-config "$(dirname "$0")/gui.config" > "$out/gui.log" 2>&1 &

# The GUI needs a while to come up and to receive the scene from the server.
sleep "${GUI_SETTLE:-30}"
date +%s.%N > "$out/capture-start"
exec ffmpeg -hide_banner -loglevel warning -f x11grab -draw_mouse 0 -framerate 15 -video_size 1280x720 -i :99 \
    -c:v libx264 -preset ultrafast -crf 16 "$out/raw.mkv"
