# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A module whose import appends to `tests.fixture_custom.IMPORTED`, to show when a machine file imports."""

from __future__ import annotations

from tests import fixture_custom

fixture_custom.IMPORTED.append(__name__)


class Anything:
    """A class that is not a Machine."""
