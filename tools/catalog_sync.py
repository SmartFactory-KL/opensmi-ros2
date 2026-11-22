# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Refresh the definition catalog at its pins, and regenerate the stubs from it.

A refresh is **a data change, in a reviewed merge request, run by a person.** This tool does the
fetching; it does not decide anything, and it is deliberately not wired to a bot that commits what
it finds. A refreshed definition can silently change what a Projection means, and a robot's
address space changing because a scheduled job pushed a commit is the invisible-change failure the
whole declarative API exists to remove.

The unit of work is one merge request holding both halves: the catalog entry and the Projection
authored against it. There is no separate catalog owner, because separate merge requests only
create a window in which one exists without the other.

::

    python -m tools.catalog_sync                   # re-extract every entry at its pin, then stubs
    python -m tools.catalog_sync --package neo_msgs2
    python -m tools.catalog_sync --stubs           # regenerate the stubs alone, no network
    python -m tools.catalog_sync --vendor          # refresh the vendored parser at its pin

Changing what is *pinned* is editing ``catalog/sources.toml`` and running this; the tool never
moves a pin by itself.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from tools.catalog.extraction import vendored_text, write_package
from tools.catalog.sources import REPOSITORY_ROOT, load_sources
from tools.catalog.stubs import generated_files


def refresh_package(name: str) -> None:
    """Re-extract one catalog entry at its pin."""
    source = load_sources().package(name)
    written = write_package(source)
    print(f"{name}: {len(written)} interface files at {source.commit[:7]}")


def refresh_vendored() -> None:
    """Re-fetch every vendored third-party file at its pin."""
    for source in load_sources().vendored:
        path = REPOSITORY_ROOT / source.path
        path.write_bytes(vendored_text(source))
        print(f"{source.name}: {path.relative_to(REPOSITORY_ROOT).as_posix()} at {source.commit[:7]}")


def generate_stubs() -> list[Path]:
    """Write the module and stub pair for every catalog package, and name what was written."""
    written: list[Path] = []
    for package in (source.name for source in load_sources().packages):
        for path, text in generated_files(package):
            # Written with LF whatever the platform is, so that regenerating on Windows does not
            # read back as a change to every line of every generated file.
            path.write_text(text, encoding="utf-8", newline="\n")
            written.append(path)
    for path in written:
        print(f"stub: {path.relative_to(REPOSITORY_ROOT).as_posix()}")
    return written


def main(argv: list[str] | None = None) -> int:
    """Run the refresh a developer asked for."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--package", action="append", default=[], help="refresh only this catalog package")
    parser.add_argument("--stubs", action="store_true", help="regenerate the stubs and nothing else")
    parser.add_argument("--vendor", action="store_true", help="refresh the vendored third-party files")
    arguments = parser.parse_args(argv)

    if arguments.stubs:
        generate_stubs()
        return 0
    if arguments.vendor:
        refresh_vendored()
        return 0
    for name in arguments.package or [source.name for source in load_sources().packages]:
        refresh_package(name)
    generate_stubs()
    return 0


if __name__ == "__main__":
    sys.exit(main())
