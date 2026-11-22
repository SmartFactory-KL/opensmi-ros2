# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The machine file of the HC10 example stays a valid machine file.

The example serves it only next to its simulation, in Docker; reading it needs neither.
"""

from __future__ import annotations

from pathlib import Path

from opensmi.ros2.server import load_machine
from tests.fake_runtime import FakeRuntime

MACHINE_FILE = Path(__file__).resolve().parent.parent / "examples" / "moveit-hc10-robotiq" / "machine.toml"


def test_the_hc10_example_machine_file_is_accepted_by_the_loader() -> None:
    load_machine(MACHINE_FILE, runtime_factory=lambda config: FakeRuntime(domain_id=config.domain_id))
