<!--
SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.

SPDX-License-Identifier: MIT
-->

# The definition catalog

The catalog holds verbatim ROS 2 interface definitions for interface packages that cannot be
installed, pinned by commit SHA in [`sources.toml`](sources.toml).

Without it, a Component author would have no static types for a package that ships no Python, and a
Projection for such a package could not be tested. The generated stubs carry the structure into the
distribution. CI builds a message dataclass from the text and runs a Projection against it with
nothing from the vendor installed.

## Membership

An interface package belongs here if the project needs it and it has no binary release for the
target distribution. Today that is four packages, listed in `sources.toml`. A package with a Jazzy
binary release is an `apt install` away and does not belong here. The installed package is the better
source, because introspecting a generated class keeps fixed-size arrays, bounded strings, float
widths and fields whose names match a constant.

`tools/catalog_check.py` asks the ROS 2 apt index whether an entry has gained a `ros-jazzy-*` binary,
and fails the pipeline when it has, so an entry cannot linger as a duplicate.

## What the catalog is not

It is read while authoring and testing, never while serving. A definition here makes a message
describable but not receivable, because `rclpy` cannot subscribe to a type without typesupport. The
catalog lives outside `src/`, so nothing importable can reach it, and it does not ship. The generated
stubs in `src/opensmi/ros2/runtime/messages/` do.

## Refreshing

A person makes a refresh as a data change in a reviewed merge request; a bot never does. A refreshed
definition can silently change what a Projection means, so the entry and the Projection written
against it land together. To move a pin, edit `sources.toml` and run the sync.

```sh
python -m tools.catalog_sync                     # re-extract every entry at its pin, then stubs
python -m tools.catalog_sync --package <name>    # one entry
python -m tools.catalog_sync --stubs             # regenerate the stubs alone, no network
python -m tools.catalog_sync --vendor            # refresh the vendored parser at its pin
python -m tools.catalog_check                    # what the CI pipeline runs
```

## Licensing

Vendored files are never edited, not even to add an SPDX header, because byte-identical re-extraction
at a pinned commit is what makes the pin checkable. The annotation lives in
[`REUSE.toml`](../REUSE.toml). `sources.toml` records whether each license identifier is the vendor's
statement (`declared`) or our reading of the evidence (`determined`), with the evidence and the
residual risk. Nothing at run time is affected: the Runtime reads installed packages and never this
directory.

## Where the pieces are

| What                           | Where                                          |
| ------------------------------ | ---------------------------------------------- |
| Pins, licenses, the apt digest | `catalog/sources.toml`                         |
| The carried text               | `catalog/packages/<package>/{msg,srv,action}/` |
| The vendored `.msg` parser     | `tools/catalog/_vendor/`                       |
| Producer, hash, grading, stubs | `tools/catalog/`                               |
| The generated stubs that ship  | `src/opensmi/ros2/runtime/messages/*.pyi`      |
