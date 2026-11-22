# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Extraction: a pinned commit becomes the interface text this repository carries.

Two operations and they are the same walk: **write** the text at an entry's pin into the catalog,
and **check** that what is already there is what the pin produces. The second is the load-bearing
one. A commit hashes its tree, so fetching that commit and copying files out of it is
byte-identical by construction -- which turns "somebody vendored whatever was current the day they
ran the script" from an invisible state into a pipeline failure.

There is deliberately no container anywhere here. These four packages have never been built by
anyone -- that is why they are in the catalog -- so there is nothing to build, and the ROS2 half
of the closest prior art (Foxglove's archived ``rosmsg-msgs-common``) ran against git repositories
for the same reason. Its one weakness was pinning to branches, which move; that is the part this
module does not copy.

A generated tarball was rejected for the same check's sake: GitHub's are not byte-stable across
gzip changes, so they would turn the reproducibility check into a false-alarm generator.
"""

from __future__ import annotations

import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import TYPE_CHECKING

from tools.catalog.sources import INTERFACE_DIRECTORIES, SUFFIXES, PackageSource, VendoredSource

if TYPE_CHECKING:
    from collections.abc import Iterator, Mapping

__all__ = [
    "ExtractionError",
    "branch_head",
    "checkout",
    "extracted_text",
    "reproduction_failures",
    "vendored_text",
    "write_package",
]


class ExtractionError(RuntimeError):
    """A pin could not be resolved: the network, the repository or the commit was not there."""


def _run(arguments: list[str], *, cwd: Path) -> None:
    """Run one git command, turning its failure into an error naming what was asked."""
    finished = subprocess.run(
        ["git", *arguments],
        cwd=cwd,
        capture_output=True,
        text=True,
        check=False,
    )
    if finished.returncode != 0:
        message = f"git {' '.join(arguments)} failed: {finished.stderr.strip()}"
        raise ExtractionError(message)


def checkout(repository: str, commit: str, into: Path) -> Path:
    """Fetch exactly one commit of one repository, and return the checked-out tree.

    A shallow fetch of a commit rather than a clone of a branch: the commit is the pin, and
    fetching it by name is what makes the result reproducible without trusting a ref to have
    stayed where it was.
    """
    into.mkdir(parents=True, exist_ok=True)
    # Line endings are forced off at every step. git's `autocrlf` is a per-machine setting, and
    # with it on -- the Windows default -- a checkout rewrites every LF in the tree, which would
    # make "byte-identical to the pin" mean "byte-identical to the pin, on this developer's
    # machine". The repository's own `.gitattributes` keeps the committed copies out of the same
    # rewrite.
    _run(["init", "--quiet"], cwd=into)
    _run(["config", "core.autocrlf", "false"], cwd=into)
    _run(["config", "core.eol", "lf"], cwd=into)
    _run(["remote", "add", "origin", repository], cwd=into)
    _run(["fetch", "--quiet", "--depth", "1", "origin", commit], cwd=into)
    _run(["checkout", "--quiet", "FETCH_HEAD"], cwd=into)
    return into


def branch_head(repository: str, branch: str) -> str:
    """Return the commit one branch currently points at, without cloning anything.

    What the scheduled staleness job asks first, and the only question it can answer cheaply:
    everything else it does is conditional on this differing from the pin.
    """
    finished = subprocess.run(
        ["git", "ls-remote", repository, f"refs/heads/{branch}"],
        capture_output=True,
        text=True,
        check=False,
    )
    if finished.returncode != 0 or not finished.stdout.strip():
        message = f"could not resolve {branch!r} of {repository}: {finished.stderr.strip()}"
        raise ExtractionError(message)
    return finished.stdout.split()[0]


def extracted_text(source: PackageSource) -> Mapping[str, bytes]:
    """Return every interface file one package's pin produces, keyed by ``kind/Name.ext``.

    Bytes rather than text, because byte-identity is the claim: a line ending or a trailing
    newline that changed is a change, and decoding would hide it.
    """
    with tempfile.TemporaryDirectory() as scratch:
        tree = checkout(source.repository, source.commit, Path(scratch) / source.name)
        root = tree / source.subdirectory if source.subdirectory else tree
        return {
            f"{directory}/{path.name}": path.read_bytes()
            for directory in INTERFACE_DIRECTORIES
            for path in sorted((root / directory).glob(f"*{SUFFIXES[directory]}"))
        }


def vendored_text(source: VendoredSource) -> bytes:
    """Return the bytes one vendored third-party file has at its pin."""
    with tempfile.TemporaryDirectory() as scratch:
        tree = checkout(source.repository, source.commit, Path(scratch) / "vendored")
        return (tree / source.source_path).read_bytes()


def write_package(source: PackageSource) -> tuple[str, ...]:
    """Replace one package's extracted text with what its pin produces, and name what landed.

    The directory is emptied first, so that an interface upstream *deleted* leaves rather than
    lingering as a definition nothing can be checked against.
    """
    extracted = extracted_text(source)
    if source.directory.exists():
        shutil.rmtree(source.directory)
    for name, content in extracted.items():
        path = source.directory / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
    return tuple(extracted)


def reproduction_failures(source: PackageSource) -> Iterator[str]:
    """Yield one line per way the carried text differs from what the pin produces.

    Empty means the entry is reproducible, which is the whole of what the merge-request pipeline
    asks of it.
    """
    extracted = extracted_text(source)
    carried = source.carried_text()
    for name in sorted(set(extracted) - set(carried)):
        yield f"{source.name}: {name} is at the pin but not in the catalog"
    for name in sorted(set(carried) - set(extracted)):
        yield f"{source.name}: {name} is in the catalog but not at the pin"
    for name in sorted(set(carried) & set(extracted)):
        if carried[name] != extracted[name]:
            yield f"{source.name}: {name} differs from the pin, byte for byte"
