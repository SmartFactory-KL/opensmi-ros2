# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""``python -m opensmi.ros2``: the same command line as the ``opensmi-ros2`` console script."""

import sys

from opensmi.ros2.cli import main

sys.exit(main())
