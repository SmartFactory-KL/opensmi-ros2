# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Cut the raw screen recording of a scenario run into the example's video, with the steps as captions.

Runs in the recorder container (record.sh calls it) with ffmpeg and the DejaVu fonts. It reads when
filming started (``raw.start``) and the scenario's ``--timeline``: when each step started and ended,
and what it did on the OPC UA side and what that triggers in ROS2, each with its time. It keeps the
stretch from just before the first step to just after the last and draws a lower third: the step in
progress (then its result) and, below it, the latest OPC UA call with its ROS2 counterpart.

The film runs in real time, except where a caption would be on screen for less than
:data:`READABLE` seconds: that stretch is slowed down until it lasts that long, and the corner says
so. It writes ``rox-argo.mp4`` (H.264, yuv420p, faststart, no sound) and ``rox-argo-poster.png``, a
frame from the drive to station A, and lowers the quality until the video fits :data:`MAX_BYTES`.
"""

from __future__ import annotations

import argparse
import itertools
import json
import subprocess
import textwrap
from dataclasses import dataclass
from pathlib import Path
from typing import Any

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
LEAD, TAIL = 3.0, 4.0
"""Seconds kept before the first step starts and after the last one ends."""
READABLE = 3.5
"""The shortest time, in seconds of video, any caption stays on screen."""
WRAP = 104
"""Characters per line of the OPC UA and ROS2 caption at its font size."""
MAX_BYTES = 8_000_000
TITLE = "OpenSMI-ROS2: a simulated Neobotix ROX Argo with Nav2, operated through OPC UA"
BOX = "fontcolor=white:box=1:boxcolor=black@0.65:boxborderw=10"


@dataclass(frozen=True)
class Caption:
    """One caption, with when it is on screen in seconds of the cut film."""

    text: str
    start: float
    end: float
    y: str
    size: int


def captions(steps: list[dict[str, Any]], length: float) -> list[Caption]:
    """Return the step titles and the call lines, timed in seconds of the cut film."""
    found: list[Caption] = []
    calls = [(float(call["at"]), str(call["text"])) for step in steps for call in step["calls"]]
    for index, (at, text) in enumerate(calls):
        until = calls[index + 1][0] if index + 1 < len(calls) else length
        found.append(Caption("\n".join(textwrap.wrap(text, WRAP)), at, until, "h-82", 20))
    for index, step in enumerate(steps):
        started, ended = float(step["started"]), float(step["ended"])
        following = float(steps[index + 1]["started"]) if index + 1 < len(steps) else length
        label = f"Step {index + 1}/{len(steps)}: {step['name']}"
        found.append(Caption(label, started, ended, "h-134", 26))
        found.append(Caption(f"{label}  [{'PASS' if step['passed'] else 'FAIL'}]", ended, following, "h-134", 26))
    return [each for each in found if each.end > each.start]


def segments(found: list[Caption], length: float) -> list[tuple[float, float, float]]:
    """Return the film cut where any caption changes, each piece with how much to slow it down."""
    cuts = sorted({0.0, length, *(each.start for each in found), *(each.end for each in found)})
    cuts = [each for each in cuts if 0.0 <= each <= length]
    pieces = [(a, b) for a, b in itertools.pairwise(cuts) if b - a > 0.05]
    # A stretch that is nearly long enough stays in real time rather than being slowed by a few percent.
    return [(a, b, READABLE / (b - a) if b - a < 0.9 * READABLE else 1.0) for a, b in pieces]


def later(pieces: list[tuple[float, float, float]], moment: float) -> float:
    """Return where a moment of the cut film lands in the slowed-down video."""
    total = 0.0
    for a, b, factor in pieces:
        if moment <= b:
            return total + (max(moment, a) - a) * factor
        total += (b - a) * factor
    return total


def graph(found: list[Caption], pieces: list[tuple[float, float, float]], begin: float, work: Path) -> str:
    """Return the filter graph: trim, captions, a speed label per piece, then each piece slowed and joined."""
    draw = [f"trim=start={begin:.2f}:end={begin + pieces[-1][1]:.2f}", "setpts=PTS-STARTPTS"]

    def text(name: str, value: str) -> str:
        path = work / name
        path.write_text(value, encoding="utf-8")
        return f"fontfile={FONT}:textfile={path}"

    draw.append(f"drawtext={text('title.txt', TITLE)}:x=20:y=20:fontsize=22:{BOX}")
    for index, (a, b, factor) in enumerate(pieces):
        label = "real time" if factor == 1.0 else f"slowed down {factor:.1f}x"
        draw.append(
            f"drawtext={text(f'speed{index}.txt', label)}:x=w-tw-20:y=20:fontsize=22:{BOX}"
            f":enable='between(t,{a:.3f},{b:.3f})'"
        )
    for index, each in enumerate(found):
        draw.append(
            f"drawtext={text(f'caption{index}.txt', each.text)}:x=20:y={each.y}:fontsize={each.size}"
            f":line_spacing=4:{BOX}:enable='between(t,{each.start:.3f},{each.end:.3f})'"
        )
    labels = "".join(f"[s{index}]" for index in range(len(pieces)))
    parts = [f"[0:v]{','.join(draw)},split={len(pieces)}{labels}"]
    parts.extend(
        f"[s{index}]trim=start={a:.3f}:end={b:.3f},setpts=(PTS-STARTPTS)*{factor:.4f}[p{index}]"
        for index, (a, b, factor) in enumerate(pieces)
    )
    joined = "".join(f"[p{index}]" for index in range(len(pieces)))
    parts.append(f"{joined}concat=n={len(pieces)}:v=1:a=0,fps=20[out]")
    return ";".join(parts)


def encode(raw: Path, filters: Path, output: Path) -> None:
    """Encode the video, raising the compression until it fits :data:`MAX_BYTES`."""
    for crf in (26, 29, 32, 35):
        subprocess.run(
            [
                *("ffmpeg", "-y", "-nostdin", "-loglevel", "warning", "-i", str(raw)),
                *("-filter_complex_script", str(filters), "-map", "[out]", "-an"),
                *("-c:v", "libx264", "-preset", "slow", "-crf", str(crf), "-pix_fmt", "yuv420p"),
                *("-movflags", "+faststart", str(output)),
            ],
            check=True,
        )
        if output.stat().st_size <= MAX_BYTES:
            return
    message = f"{output} is {output.stat().st_size} bytes even at the lowest quality tried"
    raise RuntimeError(message)


def main() -> None:
    """Read the recording and the timeline, and write the video and the poster."""
    parser = argparse.ArgumentParser(description="Cut the scenario recording into the example's video.")
    parser.add_argument("directory", type=Path, help="holds raw.mkv, raw.start and timeline.json; gets the output")
    arguments = parser.parse_args()
    directory: Path = arguments.directory
    filmed = float((directory / "raw.start").read_text().strip())
    steps = json.loads((directory / "timeline.json").read_text(encoding="utf-8"))
    if not steps:
        message = "the timeline has no steps"
        raise SystemExit(message)
    begin = max(0.0, steps[0]["started"] - filmed - LEAD)
    length = steps[-1]["ended"] - filmed + TAIL - begin

    def shifted(moment: float) -> float:
        return moment - filmed - begin

    for step in steps:
        step["started"], step["ended"] = shifted(step["started"]), shifted(step["ended"])
        step["calls"] = [{**call, "at": shifted(call["at"])} for call in step["calls"]]
    found = captions(steps, length)
    pieces = segments(found, length)
    work = directory / "captions"
    work.mkdir(exist_ok=True)
    filters = work / "filters.txt"
    filters.write_text(graph(found, pieces, begin, work), encoding="utf-8")
    video = directory / "rox-argo.mp4"
    encode(directory / "raw.mkv", filters, video)
    # The poster: halfway through the drive to station A, the fourth step.
    drive = steps[min(3, len(steps) - 1)]
    moment = later(pieces, (drive["started"] + drive["ended"]) / 2)
    subprocess.run(
        [
            *("ffmpeg", "-y", "-nostdin", "-loglevel", "warning", "-ss", f"{moment:.2f}", "-i", str(video)),
            *("-frames:v", "1", "-update", "1", str(directory / "rox-argo-poster.png")),
        ],
        check=True,
    )


if __name__ == "__main__":
    main()
