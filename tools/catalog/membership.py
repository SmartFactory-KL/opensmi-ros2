# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Membership is a rule, and the rule retires entries by itself.

    An interface package belongs in this catalog **iff** it is in the required scope **and** has
    no binary release for the target distro.

The second half is what this module asks, of the ROS2 apt index itself -- the place the answer
actually lives. The first half is a judgement about what this package is for, is recorded in
``catalog/sources.toml`` entry by entry, and is not something a checker can hold an opinion about.

The asymmetry matters. A hand-kept list of catalog packages is fine on the day it is written and
becomes a lie silently: Neobotix will bloom-release ``neo_msgs2`` eventually, and on that day its
entry stops being the best available source of the definition and becomes a stale duplicate of a
package apt can install -- while introspection over the installed class is strictly richer than
any text parse. So the merge-request pipeline **fails** on an entry that has gained a binary
release, and the failure says to delete the entry rather than to refresh it.

The index is queried rather than rosdistro, for two reasons that point the same way: rosdistro
records what someone *declared* while the apt index records what was actually *built* -- the two
disagree in the wild, with one vendor package at 3.9.0-1 in rosdistro and 3.8.0 in the index --
and the apt index needs no YAML parser, which keeps this tool on the standard library.

That is a deliberate narrowing of the rule as first stated ("no binary release ... in
rosdistro"), and it is the same question the CI check asks ("check that no
``ros-jazzy-<pkg>`` exists"). The difference shows in one case: a package released in rosdistro
but not yet built stays in the catalog. That is the answer this catalog wants, because what
retires an entry is CI being able to *install and introspect* the package -- not somebody having
declared an intention to release it.
"""

from __future__ import annotations

import gzip
import urllib.request
from typing import TYPE_CHECKING, Final

if TYPE_CHECKING:
    from collections.abc import Iterator

    from tools.catalog.sources import Sources

__all__ = ["debian_name", "membership_failures", "released_package_names", "released_packages"]

_TIMEOUT: Final = 120
"""Seconds to wait on the index. Generous: it is two megabytes over one request, once per job."""


def debian_name(package: str, distro: str) -> str:
    """Return the Debian package name one ROS2 interface package would be released under.

    ``neo_msgs2`` on Jazzy is ``ros-jazzy-neo-msgs2``: the prefix carries the distro and
    underscores become hyphens, which is the Debian convention the ROS build farm follows.
    """
    return f"ros-{distro}-{package.replace('_', '-')}"


def released_package_names(index: str) -> frozenset[str]:
    """Return every package name in a decompressed apt ``Packages`` index.

    Split out from :func:`released_packages` so that the parsing is a pure function of a string,
    and can therefore be tested against a couple of stanzas rather than against the network.
    """
    return frozenset(
        line.removeprefix("Package:").strip() for line in index.splitlines() if line.startswith("Package:")
    )


def released_packages(index_url: str) -> frozenset[str]:
    """Fetch the ROS2 apt index and return every binary package name in it."""
    with urllib.request.urlopen(index_url, timeout=_TIMEOUT) as response:
        payload = response.read()
    return released_package_names(gzip.decompress(payload).decode("utf-8", errors="replace"))


def membership_failures(sources: Sources, released: frozenset[str] | None = None) -> Iterator[str]:
    """Yield one line per catalog entry that no longer satisfies the membership rule.

    Empty means every entry is still the best available source of its definitions, which is the
    whole of what the merge-request pipeline asks of the catalog's *membership*.

    :param released: The binary package names to judge against, for a caller that already has
        them. Defaults to fetching the index, which is what the pipeline does; a test hands in a
        written-out one, so that the rule is checkable without a network.
    """
    released = released_packages(sources.binary_index) if released is None else released
    for source in sources.packages:
        debian = debian_name(source.name, sources.distro)
        if debian in released:
            yield (
                f"{source.name}: {debian} now has a binary release, so the catalog entry is a "
                f"stale duplicate of a package CI can install. Delete "
                f"catalog/packages/{source.name}/ and its sources.toml entry, and let the "
                f"installed package be the definition."
            )
