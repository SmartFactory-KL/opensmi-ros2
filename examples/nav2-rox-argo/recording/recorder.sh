#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT
#
# The recorder service of the example (see record.sh): a virtual display, the Gazebo window on it as a
# client of the headless simulation, and ffmpeg filming the display into /tmp/recording/raw.mkv. It
# writes the moment filming started to raw.start, so make_video.py can time the captions. Interrupt
# ffmpeg (its pid is in ffmpeg.pid) to finish the file; the container then stays up for make_video.py.
set -euo pipefail

out=/tmp/recording
mkdir -p "$out"
Xvfb :1 -screen 0 1280x768x24 -nolisten tcp &
export DISPLAY=:1
sleep 2
nice -n 10 gz sim -g --gui-config /opt/example/recording/gui.config >"$out/gui.log" 2>&1 &
# The window has to have the scene from the simulation before it is worth filming.
sleep "${RECORD_SETTLE:-30}"
date +%s.%N >"$out/raw.start"
nice -n 10 ffmpeg -nostdin -loglevel warning -f x11grab -draw_mouse 0 -framerate 10 -video_size 1280x720 -i :1+0,48 \
    -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p "$out/raw.mkv" &
echo $! >"$out/ffmpeg.pid"
wait "$(cat "$out/ffmpeg.pid")" || true
touch "$out/raw.done"
exec sleep infinity
