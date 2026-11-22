# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The merge-request gate: whether this repository's copy of the catalog is still true.

Three questions, and all three mean *this repository is wrong* rather than *the world moved* --
which is why this one hard-fails while the scheduled staleness job never does:

1. **Reproducible.** Every carried interface file is byte-identical to what its pinned commit
   produces. A commit hashes its tree, so the only way this fails is that somebody vendored text
   that is not at the pin -- exactly the "whatever was current the day the script ran" state the
   pin exists to make visible.
2. **Membership.** No catalog entry has gained a binary release. The day one does, the entry stops
   being the best available source of its definitions and becomes a stale duplicate of a package
   CI could install and introspect.
3. **Generated.** The shipped stubs are what the catalog says they should be, so the structure
   that ships cannot drift from the text it was generated from.

The vendored parser is checked the same way as the catalog, because it is vendored text with a pin
like any other.

::

    python -m tools.catalog_check              # everything
    python -m tools.catalog_check --offline    # only the checks that need no network

``--offline`` exists for the developer who has no route to GitHub, and it is not what CI runs: a
gate that quietly skipped its two network checks would pass on precisely the repository states it
is there to catch.
"""

from __future__ import annotations

import argparse
import sys

from tools.catalog.extraction import ExtractionError, reproduction_failures, vendored_text
from tools.catalog.membership import membership_failures
from tools.catalog.sources import REPOSITORY_ROOT, load_sources
from tools.catalog.stubs import generated_files


def reproducible_failures() -> list[str]:
    """Return every way the carried text differs from what the pins produce."""
    sources = load_sources()
    failures = [line for source in sources.packages for line in reproduction_failures(source)]
    for source in sources.vendored:
        path = REPOSITORY_ROOT / source.path
        if not path.is_file():
            failures.append(f"{source.name}: {source.path} is missing")
        elif path.read_bytes() != vendored_text(source):
            failures.append(
                f"{source.name}: {source.path} differs from {source.repository} at "
                f"{source.commit[:7]}. Vendored files are never edited -- not even to add a "
                f"licence header, which is why the annotation lives in REUSE.toml."
            )
    return failures


def stub_failures() -> list[str]:
    """Return every generated file that is not what the catalog would generate now."""
    failures: list[str] = []
    for package in (source.name for source in load_sources().packages):
        for path, text in generated_files(package):
            carried = path.read_text(encoding="utf-8") if path.is_file() else None
            if carried != text:
                failures.append(
                    f"{package}: {path.relative_to(REPOSITORY_ROOT).as_posix()} is not what the "
                    f"catalog generates. Run `python -m tools.catalog_sync --stubs` and commit "
                    f"the result."
                )
    return failures


def main(argv: list[str] | None = None) -> int:
    """Run the gate, and return the exit status a pipeline should take from it."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--offline", action="store_true", help="skip the checks that need a network")
    arguments = parser.parse_args(argv)

    failures = stub_failures()
    if not arguments.offline:
        try:
            failures += reproducible_failures()
            failures += list(membership_failures(load_sources()))
        except ExtractionError as error:
            failures.append(f"a pin could not be resolved, so nothing was checked: {error}")

    for failure in failures:
        print(f"catalog: {failure}")
    if failures:
        print(f"\n{len(failures)} problem(s). The catalog and what it generates have to agree with their pins.")
        return 1
    print("catalog: reproducible, still uninstallable, and the stubs match.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
