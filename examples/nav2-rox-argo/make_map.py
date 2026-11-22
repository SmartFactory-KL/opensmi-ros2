# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Draw the occupancy map Nav2 localizes against from the boxes of the example's Gazebo world.

Run at image build time: ``python3 make_map.py world.sdf map`` writes ``map.pgm`` and ``map.yaml``.
Every static model whose collision is a box becomes an occupied rectangle; the world has no rotated
boxes, so a rectangle is exact. The map's origin is the world's origin, which is also where the robot
spawns, so the robot starts localized at the home position without an initial pose.
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

RESOLUTION = 0.05
"""Metres per map cell."""
MARGIN = 0.5
"""How far the map reaches beyond the outermost box, in metres."""
FREE, OCCUPIED = 254, 0


def boxes(world: Path) -> list[tuple[float, float, float, float]]:
    """Return every box of the world as its x and y extent: (x_min, x_max, y_min, y_max)."""
    found = []
    for model in ET.parse(world).getroot().iter("model"):
        size = model.find("link/collision/geometry/box/size")
        pose = model.find("pose")
        if size is None or size.text is None:
            continue
        x, y = (float(each) for each in (pose.text or "0 0").split()[:2]) if pose is not None else (0.0, 0.0)
        width, depth = (float(each) for each in size.text.split()[:2])
        found.append((x - width / 2, x + width / 2, y - depth / 2, y + depth / 2))
    return found


def draw(world: Path, stem: Path) -> None:
    """Write ``<stem>.pgm`` and ``<stem>.yaml`` for the boxes of a world."""
    rectangles = boxes(world)
    x0 = min(each[0] for each in rectangles) - MARGIN
    x1 = max(each[1] for each in rectangles) + MARGIN
    y0 = min(each[2] for each in rectangles) - MARGIN
    y1 = max(each[3] for each in rectangles) + MARGIN
    width, height = round((x1 - x0) / RESOLUTION), round((y1 - y0) / RESOLUTION)
    cells = bytearray([FREE]) * (width * height)
    for left, right, bottom, top in rectangles:
        for row in range(height):
            # Row 0 of an image is the top of the map, the largest y.
            y = y1 - (row + 0.5) * RESOLUTION
            if not bottom <= y <= top:
                continue
            for column in range(width):
                if left <= x0 + (column + 0.5) * RESOLUTION <= right:
                    cells[row * width + column] = OCCUPIED
    stem.with_suffix(".pgm").write_bytes(f"P5\n{width} {height}\n255\n".encode() + bytes(cells))
    stem.with_suffix(".yaml").write_text(
        f"image: {stem.with_suffix('.pgm').name}\n"
        "mode: trinary\n"
        f"resolution: {RESOLUTION}\n"
        f"origin: [{x0:.3f}, {y0:.3f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n",
        encoding="utf-8",
    )


def main() -> None:
    """Read the world and the output stem from the command line, and draw the map."""
    parser = argparse.ArgumentParser(description="Draw the map of the example world for Nav2.")
    parser.add_argument("world", type=Path)
    parser.add_argument("stem", type=Path, help="the map files to write, without suffix")
    arguments = parser.parse_args()
    draw(arguments.world, arguments.stem)


if __name__ == "__main__":
    main()
