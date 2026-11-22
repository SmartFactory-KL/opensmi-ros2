# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A worked example: a Nav2-style mobile platform on a stock opensmi server.

One Machine (:mod:`examples.machine`) carries one Component (:mod:`examples.component`) that
declares one Binding of every kind this package ships. :mod:`examples.run_server` wires them to a
Runtime and serves them.

Not part of the distribution (``pyproject.toml`` packages only ``src/``). ``tests/test_examples.py``
serves it against the fake Runtime and checks that every Binding kind arrives in the address space.
"""
