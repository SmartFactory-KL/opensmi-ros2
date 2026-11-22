# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Cut the raw screen capture to the pick and place, caption every step, and export the video and a poster.

Runs in the recorder container after the scenario (``record.sh`` calls it). The scenario's step
timeline (``TIMELINE``, seconds since the epoch) and the time the capture began line up on the
wall clock, so each step's caption shows exactly while the step runs. The video starts shortly
before the first motion and ends after the last step, as recorded: nothing is sped up, but the
simulation itself runs slower than real time while the GUI renders in software. The start-up
before the first motion is cut, and the title says so. A step shorter than ``SHORTEST`` (a read
takes no time at all) has its last frame held until its caption can be read, and the caption says
so. Gazebo's window toolbar is cut off and a
black band with the title takes its place.

Each step gets a lower third of three lines: its name; the OPC UA nodes it used and what it did
there (``Arm > SkillSet > Move: Parameters written, Reset(), Start()``), or, for a step that only
reads, the variables it read; and the ROS2 interfaces behind them, with an arrow that says which way
it went. The scenario recorded both in the timeline, the nodes as the server names them and the ROS2
name and type from the Binding in the machine file, so the caption cannot drift from either. The
poster is the frame in the middle of the carry, with the cube in the gripper.

Usage: ``python3 caption.py <folder with raw.mkv, capture-start and timeline.json> <video> <poster>``
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
MONO = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
SEPARATOR, TO, FROM = " \u203a ", "\u2192", "\u2190"
"""How a caption separates browse names, and its arrows: OPC UA drove ROS2, or read what ROS2 sent."""
BAND = 128
"""Pixels of the lower third, at the bottom of the frame, below the table top where the grasp happens."""
FIRST_MOTION = "pre-grasp"
"""The step the video starts with: the steps before it only wait for the cell and read values."""
POSTER_STEP = "move above the place position"
"""The step whose middle frame is the poster: the cube is carried in the gripper."""
TITLE = (
    "Yaskawa HC10 + Robotiq 2F-85, MoveIt 2 in Gazebo, every step started through OPC UA "
    "(opensmi-ros2)  |  as recorded, start-up cut"
)
TOOLBAR = 48
"""Pixels of Gazebo's window toolbar at the top of the capture, cut off and replaced by the title band."""
FPS = 15
"""Frames per second of the capture and of the video."""
SHORTEST = 2.5
"""Seconds a step's caption shows at least: a shorter step's last frame is held until then."""
LEAD, TAIL = 1.5, 2.5
"""Seconds of video before the first motion and after the last step."""


def drawtext(
    textfile: Path, *, size: int, y: str, x: str = "(w-text_w)/2", font: str = FONT, enable: str | None = None
) -> str:
    """Return one drawtext filter: white text from a file (so nothing needs escaping)."""
    parts = [f"fontfile={font}", f"textfile={textfile}", f"fontsize={size}", "fontcolor=white", f"x={x}", f"y={y}"]
    if enable:
        parts.append(f"enable='{enable}'")
    return "drawtext=" + ":".join(parts)


def main() -> None:
    """Write the captioned video and the poster."""
    folder, video, poster = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    captured = float((folder / "capture-start").read_text(encoding="utf-8").strip())
    steps = json.loads((folder / "timeline.json").read_text(encoding="utf-8"))
    first = next(index for index, each in enumerate(steps) if each["name"].startswith(FIRST_MOTION))
    shown = steps[first:]
    begin = shown[0]["start"] - captured - LEAD
    end = shown[-1]["end"] - captured + TAIL
    if begin < 0:
        message = f"the capture began {-begin:.1f} s after the first motion; start the scenario later"
        raise SystemExit(message)

    texts = folder / "captions"
    texts.mkdir(exist_ok=True)
    (texts / "title.txt").write_text(TITLE, encoding="utf-8")
    filters = [f"fps={FPS}", f"crop=1280:{720 - TOOLBAR}:0:{TOOLBAR}", f"pad=1280:720:0:{TOOLBAR}:black"]
    captions: list[str] = []
    at = 0.0
    held = 0.0
    """Seconds of held frames before the current step, by which its times move in the video."""
    for number, each in enumerate(shown, start=1):
        start, stop = each["start"] - captured - begin + held, each["end"] - captured - begin + held
        hold = max(0.0, SHORTEST - (stop - start))
        verdict = "" if each["passed"] else "  [FAILED]"
        note = f"  (last frame held {hold:.1f} s)" if hold else ""
        driven = each["direction"] == "to"
        lines = {
            "name": f"{number}/{len(shown)}  {each['name']}{verdict}{note}",
            "opcua": f"OPC UA   {'' if driven else 'read '}{each['opcua'].replace(' > ', SEPARATOR)}",
            "ros2": f"ROS2  {TO if driven else FROM}  {each['ros2']}",
        }
        for key, text in lines.items():
            (texts / f"step{number}-{key}.txt").write_text(text, encoding="utf-8")
        if each["name"].startswith(POSTER_STEP):
            at = (start + stop) / 2
        if hold:
            # The step's last frame stands still long enough to read its caption.
            filters.append(f"loop=loop={round(hold * FPS)}:size=1:start={round(stop * FPS)}")
            stop += hold
            held += hold
        enable = f"between(t,{start:.2f},{stop:.2f})"
        captions += [
            drawtext(texts / f"step{number}-name.txt", size=30, x="28", y=f"h-{BAND - 14}", enable=enable),
            drawtext(texts / f"step{number}-opcua.txt", size=21, x="28", y=f"h-{BAND - 60}", font=MONO, enable=enable),
            drawtext(texts / f"step{number}-ros2.txt", size=21, x="28", y=f"h-{BAND - 92}", font=MONO, enable=enable),
        ]
    filters += [
        f"setpts=N/{FPS}/TB",
        drawtext(texts / "title.txt", size=18, y="14"),
        f"drawbox=x=0:y=ih-{BAND}:w=iw:h={BAND}:color=black@0.62:t=fill",
        *captions,
    ]
    script = folder / "captions.filter"
    script.write_text(",\n".join(filters), encoding="utf-8")

    video.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            *("ffmpeg", "-hide_banner", "-loglevel", "warning", "-y"),
            *("-ss", f"{begin:.2f}", "-to", f"{end:.2f}", "-i", str(folder / "raw.mkv")),
            *("-filter_script:v", str(script)),
            *("-c:v", "libx264", "-preset", "slow", "-crf", "27", "-pix_fmt", "yuv420p", "-r", "15"),
            *("-movflags", "+faststart", "-an", str(video)),
        ],
        check=True,
    )
    subprocess.run(
        [
            *("ffmpeg", "-hide_banner", "-loglevel", "warning", "-y"),
            *("-ss", f"{at:.2f}", "-i", str(video), "-frames:v", "1", "-update", "1", str(poster)),
        ],
        check=True,
    )
    print(f"{video}: {end - begin + held:.0f} s; poster at {at:.1f} s: {poster}")  # noqa: T201


if __name__ == "__main__":
    main()
