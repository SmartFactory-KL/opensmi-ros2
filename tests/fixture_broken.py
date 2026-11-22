# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A module that cannot be imported, because its own code fails."""

from __future__ import annotations

message = "this module fails when it is imported"
raise RuntimeError(message)
