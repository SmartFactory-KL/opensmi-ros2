# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""``catalog/sources.toml``, as values: where every definition in this project came from.

One file holds the provenance of all three tiers -- the git-only packages the catalog carries, the
parser vendored to read them, and the pinned image the installable packages arrive in -- so that
the diff of that one file is the audit trail of every refresh.

TOML is read here and nowhere near the server: the ruling that a deployment-editable
configuration file may not name a Python import path is about the server's config, and this file
names none, is not deploy-time, and is read by a tool rather than by a running Machine.
"""

from __future__ import annotations

import tomllib
from dataclasses import dataclass
from functools import cache
from pathlib import Path
from typing import Final

__all__ = [
    "CATALOG_ROOT",
    "PACKAGES_ROOT",
    "REPOSITORY_ROOT",
    "SOURCES_FILE",
    "AptTier",
    "PackageSource",
    "Sources",
    "VendoredSource",
    "load_sources",
]

REPOSITORY_ROOT: Final = Path(__file__).resolve().parents[2]
"""The repository root, found from this file rather than from the working directory.

Every path in this package is relative to it, and it is defined here so that a tool run from a
subdirectory -- or from a scheduled job that never chose one -- reaches the same files.
"""

CATALOG_ROOT: Final = REPOSITORY_ROOT / "catalog"
"""The catalog's root."""

PACKAGES_ROOT: Final = CATALOG_ROOT / "packages"
"""Where one directory per catalog package holds its verbatim interface text."""

SOURCES_FILE: Final = CATALOG_ROOT / "sources.toml"

DECLARED: Final = "declared"
"""The licence provenance of a package that states an SPDX identifier we can use as written."""

DETERMINED: Final = "determined"
"""The licence provenance of a package whose licence we read out of the available evidence.

Recorded rather than smoothed over, because two of the four entries rest on it: a later reader
has to be able to tell a vendor's statement from our reading of one.
"""

INTERFACE_DIRECTORIES: Final = ("msg", "srv", "action")
"""The three directories extraction copies, and nothing else."""

SUFFIXES: Final = {"msg": ".msg", "srv": ".srv", "action": ".action"}
"""The file suffix each interface directory holds."""


@dataclass(frozen=True, slots=True, kw_only=True)
class PackageSource:
    """One interface package the catalog carries, and the commit it was taken from.

    :ivar branch: The branch the staleness job watches. Never what extraction resolves -- a
        branch moves, which is the whole failure the pin exists against.
    :ivar commit: What extraction resolves, always, including where a clean tag exists.
    :ivar tag: A human-readable label when upstream has one, and empty when it has none. Three
        of the four packages have no tags at all.
    :ivar subdirectory: Where the interface directories sit in the repository, for the package
        that is one directory of a larger repository.
    :ivar declared_licence: What the package's own manifest says, verbatim -- which for two of
        these is not an SPDX identifier at all.
    :ivar licence: The SPDX identifier REUSE.toml annotates the files with.
    :ivar licence_provenance: ``declared`` or ``determined``; see :data:`DETERMINED`.
    """

    name: str
    repository: str
    branch: str
    commit: str
    tag: str
    subdirectory: str
    declared_licence: str
    licence: str
    licence_provenance: str
    copyright: str
    extracted: str

    @property
    def directory(self) -> Path:
        """Where this package's extracted text lives."""
        return PACKAGES_ROOT / self.name

    @property
    def determined(self) -> bool:
        """Whether this entry's licence is our own reading rather than the vendor's statement."""
        return self.licence_provenance == DETERMINED

    def interface_files(self) -> tuple[Path, ...]:
        """Return every extracted interface file of this package, in a stable order."""
        found = [
            path
            for directory in INTERFACE_DIRECTORIES
            for path in sorted((self.directory / directory).glob(f"*{SUFFIXES[directory]}"))
        ]
        return tuple(found)

    def carried_text(self) -> dict[str, bytes]:
        """Return the text this package carries, keyed as extraction keys it: ``kind/Name.ext``.

        Bytes rather than text, and one key format rather than two: byte-identity against the pin
        is the claim, and the two callers that compare against an extraction have to be asking
        about the same keys or the comparison quietly answers a different question.
        """
        return {f"{path.parent.name}/{path.name}": path.read_bytes() for path in self.interface_files()}


@dataclass(frozen=True, slots=True, kw_only=True)
class VendoredSource:
    """One third-party source file vendored into this repository.

    :ivar source_path: Where the file sits in the upstream repository.
    :ivar path: Where it sits here, relative to the repository root.
    """

    name: str
    repository: str
    branch: str
    commit: str
    tag: str
    source_path: str
    path: str
    licence: str
    licence_provenance: str
    copyright: str
    extracted: str


@dataclass(frozen=True, slots=True, kw_only=True)
class AptTier:
    """The pinned image the installable interface packages arrive in.

    Pinned by digest rather than by an exact apt version, because packages.ros.org keeps exactly
    one build of each package: an exact-version pin works until the day the next build lands.
    """

    image: str
    digest: str
    resolved: str


@dataclass(frozen=True, slots=True, kw_only=True)
class Sources:
    """Everything ``catalog/sources.toml`` records.

    :ivar distro: The distro the membership rule is asked about.
    :ivar ubuntu: The Ubuntu release that distro targets, which names the index below.
    :ivar binary_index: The apt index the membership rule is asked against, over one request, so
        that the check needs neither a ROS installation nor apt itself.
    """

    distro: str
    ubuntu: str
    binary_index: str
    packages: tuple[PackageSource, ...]
    vendored: tuple[VendoredSource, ...]
    apt: AptTier

    def package(self, name: str) -> PackageSource:
        """Return one package entry by name.

        :raises KeyError: When no entry carries that name.
        """
        for source in self.packages:
            if source.name == name:
                return source
        raise KeyError(name)


@cache
def load_sources(path: Path = SOURCES_FILE) -> Sources:
    """Read ``catalog/sources.toml``, once.

    Memoised because half a dozen call sites ask for it and the answer is a frozen value read off
    disk. Nothing in this package writes the file, so there is no staleness to worry about -- and
    a tool that changed a pin would be doing so in an editor, in a merge request.
    """
    parsed = tomllib.loads(path.read_text(encoding="utf-8"))
    return Sources(
        distro=parsed["catalog"]["distro"],
        ubuntu=parsed["catalog"]["ubuntu"],
        binary_index=parsed["catalog"]["binary_index"],
        packages=tuple(PackageSource(**entry) for entry in parsed["package"]),
        vendored=tuple(VendoredSource(**entry) for entry in parsed["vendored"]),
        apt=AptTier(**parsed["apt"]),
    )
