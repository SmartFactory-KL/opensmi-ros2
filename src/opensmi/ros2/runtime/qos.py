# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""QoS: where a profile came from, and therefore how it may be treated.

A profile is declared (written on the Binding), discovered (computed from the topic's live
publishers) or defaulted (the middleware's reliable, volatile profile, for an outward endpoint
with nothing declared). This is its provenance, and one rule follows: the QoS of an endpoint adapts
if and only if it was discovered. Declared and defaulted profiles are honoured exactly.

Discovery picks, for each policy, the least demanding value any current publisher offers (see
:func:`resolve`), which is compatible with all of them regardless of order. Adaptation only ever
relaxes. The one exception is the bootstrap: a topic with no publisher at init is subscribed with
:data:`MOST_PERMISSIVE` and re-resolved once on the first publisher it matches, since a
``VOLATILE`` request would match a ``TRANSIENT_LOCAL`` publisher and receive none of its history.

QoS is never fatal. A declared profile that clashes with a publisher is logged and reported as an
event, not refused at startup, since whether the publisher is up then is a matter of timing. Queue
depth is not part of a profile because it takes no part in matching.

Nothing here imports ``rclpy``. The :class:`Policy` values are the ``rmw`` policy-kind numbers.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import Enum, IntEnum
from typing import TYPE_CHECKING, Final, final

if TYPE_CHECKING:
    from collections.abc import Iterable, Sequence

__all__ = [
    "DEFAULT_PROFILE",
    "FOREVER",
    "LATCHED",
    "MOST_PERMISSIVE",
    "SENSOR_DATA",
    "Durability",
    "EndpointInfo",
    "Liveliness",
    "Policy",
    "Provenance",
    "QoSIncompatibility",
    "QoSNegotiation",
    "QoSProfile",
    "Reliability",
    "most_permissive",
    "resolve",
]

FOREVER: Final = math.inf
"""The duration a deadline or a liveliness lease has when there is none (``math.inf``).

ROS2 spells it ``INT64_MAX`` nanoseconds on the wire and as a zero duration in a fresh ``rclpy``
profile. Neither is usable in arithmetic.
"""


class Reliability(Enum):
    """Whether the middleware retries a sample until it is acknowledged.

    ``BEST_EFFORT`` is the more permissive request: it matches a publisher offering either,
    while a ``RELIABLE`` request matches only a ``RELIABLE`` publisher.
    """

    BEST_EFFORT = "best_effort"
    RELIABLE = "reliable"


class Durability(Enum):
    """Whether a publisher keeps its samples for subscribers that join later.

    ``VOLATILE`` is the more permissive request, but it matches a ``TRANSIENT_LOCAL`` publisher
    and receives none of its history.
    """

    VOLATILE = "volatile"
    TRANSIENT_LOCAL = "transient_local"


class Liveliness(Enum):
    """How a publisher asserts that it is still alive.

    ``AUTOMATIC`` is the more permissive request: it matches a publisher that asserts either way.
    """

    AUTOMATIC = "automatic"
    MANUAL_BY_TOPIC = "manual_by_topic"


class Policy(IntEnum):
    """One matching policy, by the number ``rmw`` gives it.

    Only policies that take part in matching are here. :data:`INVALID` is what an unrecognised
    kind becomes. A Jazzy test checks the numbers against the live enumeration.
    """

    INVALID = 1
    DURABILITY = 2
    DEADLINE = 4
    LIVELINESS = 8
    RELIABILITY = 16
    LIVELINESS_LEASE_DURATION = 256


class Provenance(Enum):
    """Where a profile came from, and therefore whether it may be adapted.

    :data:`DISCOVERED` adapts; the other two are honoured exactly. It belongs to an endpoint, not
    a Binding: a TF Binding holds two discovered subscriptions and a declared publisher.
    """

    DECLARED = "declared"
    DISCOVERED = "discovered"
    DEFAULTED = "defaulted"


@dataclass(frozen=True, slots=True, kw_only=True)
class QoSProfile:
    """The matching policies of one endpoint, and nothing else.

    The defaults are the middleware's usual values, so a profile written with one keyword is the
    middleware's own profile with that one change.

    :ivar reliability: Whether samples are retried until acknowledged.
    :ivar durability: Whether a publisher keeps its samples for late joiners.
    :ivar deadline: The longest gap, in seconds, between samples that this endpoint promises
        (offered) or tolerates (requested). :data:`FOREVER` means no deadline.
    :ivar liveliness: How a publisher asserts that it is still alive.
    :ivar lease: Seconds an assertion of liveliness stays good for. :data:`FOREVER` means no lease.
    """

    reliability: Reliability = Reliability.RELIABLE
    durability: Durability = Durability.VOLATILE
    deadline: float = FOREVER
    liveliness: Liveliness = Liveliness.AUTOMATIC
    lease: float = FOREVER

    def __post_init__(self) -> None:
        """Refuse a duration that is not a duration.

        :raises ValueError: When the deadline or the lease is not a positive number of seconds.
            Zero is refused because ``rclpy`` uses it for "middleware default"; write
            :data:`FOREVER` instead.
        """
        for name, seconds in (("deadline", self.deadline), ("lease", self.lease)):
            if seconds > 0:
                continue
            message = (
                f"a QoS profile gives {name}={seconds!r}; it is a positive number of seconds, and "
                f"the absence of one is spelled FOREVER rather than zero"
            )
            raise ValueError(message)

    def __str__(self) -> str:
        """Return the profile as one line for event texts and logs."""
        return (
            f"{self.reliability.value}/{self.durability.value}, deadline {_seconds(self.deadline)}, "
            f"liveliness {self.liveliness.value} with a lease of {_seconds(self.lease)}"
        )


DEFAULT_PROFILE: Final = QoSProfile()
"""The middleware's default profile (reliable, volatile, no deadline, automatic liveliness).

A publishing endpoint uses it when nothing was declared. Under it the only reachable mismatches
are durability, which would replay stale commands to a late joiner, and deadline, a rate this
package does not control.
"""

MOST_PERMISSIVE: Final = QoSProfile(
    reliability=Reliability.BEST_EFFORT,
    durability=Durability.VOLATILE,
    deadline=FOREVER,
    liveliness=Liveliness.AUTOMATIC,
    lease=FOREVER,
)
"""The request that matches every possible publisher and receives the least from them.

A discovered Binding with no publisher at init subscribes with it until its first match event.
Left in place it would drop a latched topic's history.
"""

SENSOR_DATA: Final = QoSProfile(reliability=Reliability.BEST_EFFORT)
"""Best effort, as high-rate sensors usually publish."""

LATCHED: Final = QoSProfile(durability=Durability.TRANSIENT_LOCAL)
"""Transient local, for topics whose value is a fact rather than a stream (such as ``/tf_static``)."""


@dataclass(frozen=True, slots=True, kw_only=True)
class EndpointInfo:
    """One endpoint on a topic, as the client library's publisher-info call reports it.

    This is the input of :func:`resolve`, kept free of ``rclpy`` types.

    :ivar node_name: The fully qualified name of the node that owns the endpoint. It takes no
        part in matching but lets logs say which publisher forced a subscription down.
    :ivar qos: What that endpoint offers.
    """

    node_name: str
    qos: QoSProfile


@dataclass(frozen=True, slots=True, kw_only=True)
class QoSIncompatibility:
    """One incompatibility the middleware reported, and what this package did about it.

    :ivar topic: The ROS2 topic it happened on.
    :ivar policy: The policy the middleware named. It reports only the last clash it found.
    :ivar request: The profile the subscription was asking for when it fired.
    :ivar adapted: What the subscription moved to, or ``None`` when it did not move (declared
        profile, or already as permissive as the graph asks for).
    :ivar provenance: Where the profile that clashed came from.
    """

    topic: str
    policy: Policy
    request: QoSProfile
    adapted: QoSProfile | None
    provenance: Provenance


def _clashes(request: QoSProfile, offer: QoSProfile) -> tuple[Policy, ...]:
    """Return every policy on which a subscriber's request cannot be served by a publisher's offer.

    Empty means the two match. A request is served when it asks for no more than the offer
    promises: for the three enumerations no stronger, for the two durations no shorter (checked
    against a live Jazzy graph). Nothing calls this at run time, since the middleware does the
    matching; it states the rule so tests can assert that :func:`resolve` is compatible.
    """
    found: list[Policy] = []
    if _rank(request.reliability) > _rank(offer.reliability):
        found.append(Policy.RELIABILITY)
    if _rank(request.durability) > _rank(offer.durability):
        found.append(Policy.DURABILITY)
    if request.deadline < offer.deadline:
        found.append(Policy.DEADLINE)
    if _rank(request.liveliness) > _rank(offer.liveliness):
        found.append(Policy.LIVELINESS)
    if request.lease < offer.lease:
        found.append(Policy.LIVELINESS_LEASE_DURATION)
    return tuple(found)


def most_permissive(profiles: Iterable[QoSProfile]) -> QoSProfile:
    """Return the request that is, policy by policy, the least demanding of those given.

    Adaptation relaxes with this too: combining the held profile with what the graph now resolves
    to is what makes it only ever move down. With nothing to combine it returns
    :data:`MOST_PERMISSIVE`.
    """
    held = list(profiles)
    if not held:
        return MOST_PERMISSIVE
    combined, rest = held[0], held[1:]
    for profile in rest:
        combined = replace(
            combined,
            reliability=min(combined.reliability, profile.reliability, key=_rank),
            durability=min(combined.durability, profile.durability, key=_rank),
            deadline=max(combined.deadline, profile.deadline),
            liveliness=min(combined.liveliness, profile.liveliness, key=_rank),
            lease=max(combined.lease, profile.lease),
        )
    return combined


def resolve(endpoints: Iterable[EndpointInfo]) -> QoSProfile:
    """Return the most permissive request compatible with every one of these publishers (discovery).

    Pure and independent of the order of the endpoints. With no publishers it returns
    :data:`MOST_PERMISSIVE`; :class:`QoSNegotiation` re-resolves once the first one appears.
    """
    return most_permissive(endpoint.qos for endpoint in endpoints)


@final
class QoSNegotiation:
    """One endpoint's provenance, its current profile, and the two events that may move it.

    The adaptation rule as a pure state machine: the Runtime re-subscribes when an event method
    returns a new profile, and gets ``None`` when nothing moved.
    """

    def __init__(self, *, declared: QoSProfile | None, endpoints: Sequence[EndpointInfo] = ()) -> None:
        """Settle the provenance and the opening profile from what was declared and what publishes.

        :param declared: The profile written on the declaration, or ``None`` to discover one.
        :param endpoints: Every publisher on the topic right now. Ignored when a profile was
            declared, even if it is provably incompatible with what is publishing.
        """
        self._declared = declared
        self._provenance = Provenance.DECLARED if declared is not None else Provenance.DISCOVERED
        self._profile = declared if declared is not None else resolve(endpoints)
        # A declaration has nothing to bootstrap; so does a discovery that already saw a publisher.
        self._bootstrapped = declared is not None or bool(endpoints)

    def __repr__(self) -> str:
        """Return a representation naming the provenance and the profile."""
        return f"<QoSNegotiation {self._provenance.value}: {self._profile}>"

    @property
    def profile(self) -> QoSProfile:
        """Return the profile this endpoint is asking for right now."""
        return self._profile

    @property
    def provenance(self) -> Provenance:
        """Return where that profile came from, which decides whether it may move."""
        return self._provenance

    def on_first_publisher(self, endpoints: Sequence[EndpointInfo]) -> QoSProfile | None:
        """Re-resolve once, against the first publishers this endpoint ever matched.

        The only tightening in the rule, at most once: a discovered Binding whose topic was empty
        at init asks for :data:`MOST_PERMISSIVE`, which would miss a latched publisher's history.

        :param endpoints: Every publisher on the topic now. An empty list does not spend the
            bootstrap.
        :return: The profile to re-subscribe with, or ``None`` when nothing moved.
        """
        if self._bootstrapped or not endpoints:
            return None
        self._bootstrapped = True
        return self._moved_to(resolve(endpoints))

    def on_incompatibility(self, endpoints: Sequence[EndpointInfo]) -> QoSProfile | None:
        """Relax towards what the graph now offers, if and only if this profile was discovered.

        The result is the most permissive of what is held and what the current publishers resolve
        to, so a departed publisher never tightens the subscription. It also spends the bootstrap,
        since a later first-match tightening would undo this.

        :param endpoints: Every publisher on the topic now. Empty relaxes the whole way.
        :return: The profile to re-subscribe with, or ``None`` when nothing moved, which includes
            every declared and defaulted profile.
        """
        self._bootstrapped = True
        if self._provenance is not Provenance.DISCOVERED:
            return None
        return self._moved_to(most_permissive((self._profile, resolve(endpoints))))

    def incompatibility(self, topic: str, policy: Policy, endpoints: Sequence[EndpointInfo]) -> QoSIncompatibility:
        """Answer one incompatibility the middleware reported, as the record of what happened.

        This is :meth:`on_incompatibility` plus the record, so the Runtime and test doubles share
        one code path.

        :return: What clashed, what was being asked for, and what this moved to, if anything.
        """
        request = self._profile
        return QoSIncompatibility(
            topic=topic,
            policy=policy,
            request=request,
            adapted=self.on_incompatibility(endpoints),
            provenance=self._provenance,
        )

    def _moved_to(self, profile: QoSProfile) -> QoSProfile | None:
        """Take a newly resolved profile, answering ``None`` when it is the one already held."""
        if profile == self._profile:
            return None
        self._profile = profile
        return profile


def _rank(value: Reliability | Durability | Liveliness) -> int:
    """Return how demanding a policy value is as a request; lower is more permissive."""
    return _RANKS[value]


_RANKS: Final[dict[Reliability | Durability | Liveliness, int]] = {
    Reliability.BEST_EFFORT: 0,
    Reliability.RELIABLE: 1,
    Durability.VOLATILE: 0,
    Durability.TRANSIENT_LOCAL: 1,
    Liveliness.AUTOMATIC: 0,
    Liveliness.MANUAL_BY_TOPIC: 1,
}
"""How demanding each enumerated policy value is, as a request.

The enumerations are string-valued (not ``IntEnum``) because TOML files name them.
"""


def _seconds(value: float) -> str:
    """Return a duration as the words an event text uses, instead of ``inf``."""
    return "none" if value == FOREVER else f"{value}s"
