# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The ROX Argo example's files stay usable without the simulation.

The example serves its machine file only with the simulation running in Docker, and draws its map
only while the image is built; checking either needs neither.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import TYPE_CHECKING

from opensmi.ros2.server import load_machine
from tests.fake_runtime import FakeRuntime

if TYPE_CHECKING:
    from types import ModuleType

EXAMPLE = Path(__file__).resolve().parent.parent / "examples" / "nav2-rox-argo"


def _module(name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(f"rox_argo_{name}", EXAMPLE / f"{name}.py")
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    # Dataclasses look their module up while the class is made.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _make_map() -> ModuleType:
    return _module("make_map")


def test_the_rox_argo_machine_file_is_accepted_by_the_loader() -> None:
    machine = load_machine(
        EXAMPLE / "machine.toml", runtime_factory=lambda config: FakeRuntime(domain_id=config.domain_id)
    )
    assert machine.name == "RoxArgo"


def test_the_scenario_names_the_ros2_side_of_every_binding_its_captions_mention() -> None:
    bindings = _module("scenario").Bindings.load()
    assert bindings.ros2("Platform/NavigateToPose") == "/navigate_to_pose (nav2_msgs/action/NavigateToPose)"
    assert bindings.ros2("Platform/Odom") == "/odom (nav_msgs/msg/Odometry)"
    assert bindings.ros2("Platform/Scan", typed=False) == "/scan"
    assert bindings.ros2("Platform/Diagnostics") == "/diagnostics (diagnostic_msgs/msg/DiagnosticArray)"
    assert bindings.of("Platform", "monitoring", required=True) == ["Platform/Odom", "Platform/Scan"]
    assert [bindings.ros2(each) for each in bindings.of("Platform", "lifecycle")] == [
        "/lifecycle_manager_localization (nav2_msgs/srv/ManageLifecycleNodes)",
        "/lifecycle_manager_navigation (nav2_msgs/srv/ManageLifecycleNodes)",
    ]


def test_the_map_has_the_world_s_obstacles_and_leaves_the_goals_free(tmp_path: Path) -> None:
    make_map = _make_map()
    make_map.draw(EXAMPLE / "world.sdf", tmp_path / "map")
    header, _, cells = (tmp_path / "map.pgm").read_bytes().partition(b"\n255\n")
    width, height = (int(each) for each in header.split()[1:3])
    origin = next(line for line in (tmp_path / "map.yaml").read_text().splitlines() if line.startswith("origin"))
    x0, y0 = (float(each) for each in origin.split("[")[1].split(",")[:2])

    def cell(x: float, y: float) -> int:
        column = int((x - x0) / make_map.RESOLUTION)
        row = height - 1 - int((y - y0) / make_map.RESOLUTION)
        return cells[row * width + column]

    for obstacle in ((3.0, 0.0), (7.4, 2.2), (7.4, -2.2), (-2.1, 0.0), (3.0, 3.6)):
        assert cell(*obstacle) == make_map.OCCUPIED, obstacle
    for free in ((0.0, 0.0), (5.5, 2.2), (5.5, -2.2)):
        assert cell(*free) == make_map.FREE, free
