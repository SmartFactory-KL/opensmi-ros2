# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The per-Binding provenance entry: what one endpoint asked for, and why it is asking that.

QoS has no status variable. A profile is a fact about how this server is attached to the robot, so
it is listed on the read-only introspection surface as a value object. The mirrors build
one entry per Binding and the introspection Component lists them. A profile is declared (a
contract), discovered (adapts) or defaulted (what an outward endpoint gets when nobody said
otherwise), which is the first thing to check when a topic is silent.

Provenance attaches to an endpoint. Monitoring, Keyed and
publishing Bindings hold one endpoint each and are listed by Binding name. The TF Binding's
subscriptions and publisher belong to the Machine, so they are not listed yet; their profiles are
:data:`~opensmi.ros2.runtime.tf.TF_QOS` and :data:`~opensmi.ros2.runtime.tf.TF_STATIC_QOS`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Final, Protocol

if TYPE_CHECKING:
    from collections.abc import Sequence

    from opensmi.ros2.runtime.qos import Provenance, QoSIncompatibility, QoSProfile

__all__ = ["QOS_INCOMPATIBLE", "BindingQoS", "QoSEndpoint", "qos_clash_text"]

QOS_INCOMPATIBLE: Final = "QOS_INCOMPATIBLE"
"""What a Binding reports when the middleware refuses a publisher over a QoS policy.

A discovered profile relaxes and the event is a warning. A declared one does not move, so the event
is an error. The text names the policy, what was asked for and what happened next. Both mirrors
raise it, because a Keyed Binding attaches to a topic as a Monitoring one does.
"""


class QoSEndpoint(Protocol):
    """A ROS2 endpoint, as a provenance entry reads it: a profile and where it came from.

    Both Runtime handles satisfy it. It is the smaller of the two surfaces: a Subscription also
    carries incompatibilities and a Publisher has none, so those are passed in separately.
    """

    @property
    def qos(self) -> QoSProfile:
        """Return the profile this endpoint is asking for, or offering, right now."""
        ...

    @property
    def qos_provenance(self) -> Provenance:
        """Return where that profile came from."""
        ...


@dataclass(frozen=True, slots=True, kw_only=True)
class BindingQoS:
    """What one Binding's ROS2 endpoint is asking for, and where that came from.

    :ivar binding: The name of the Binding, which is what a client browses it by.
    :ivar path: The ROS2 topic the endpoint is on.
    :ivar profile: The profile in effect right now. For a discovered subscription this may differ
        from the one it opened with.
    :ivar provenance: Where that profile came from, and therefore whether it can still move.
    :ivar adaptations: How many times an incompatibility has moved the profile. Non-zero means the
        Binding discovered its profile and had to give something up.
    :ivar last_incompatibility: The most recent clash reported on this topic, or ``None``. Kept
        because the event that announced it is gone if nobody was subscribed.
    """

    binding: str
    path: str
    profile: QoSProfile
    provenance: Provenance
    adaptations: int
    last_incompatibility: QoSIncompatibility | None = None

    @classmethod
    def of(
        cls,
        binding: str,
        path: str,
        endpoint: QoSEndpoint,
        incompatibilities: Sequence[QoSIncompatibility] = (),
    ) -> BindingQoS:
        """Return the entry for one live endpoint, read off the handle every time.

        The adaptation count is derived from the incompatibilities, so the two cannot disagree.
        """
        return cls(
            binding=binding,
            path=path,
            profile=endpoint.qos,
            provenance=endpoint.qos_provenance,
            adaptations=sum(1 for each in incompatibilities if each.adapted is not None),
            last_incompatibility=incompatibilities[-1] if incompatibilities else None,
        )

    def __str__(self) -> str:
        """Return the one line an operator reads: the Binding, its profile and its provenance."""
        adapted = f", adapted {self.adaptations} time(s)" if self.adaptations else ""
        return f"{self.binding} ({self.path}): {self.provenance.value} {self.profile}{adapted}"


def qos_clash_text(path: str, incompatibility: QoSIncompatibility) -> str:
    """Return the human half of a :data:`QOS_INCOMPATIBLE` event: what clashed, and what followed.

    One function, because both mirrors raise the same event and it should read the same on both.
    """
    adapted = incompatibility.adapted
    answer = (
        f"the subscription relaxed to {adapted}"
        if adapted is not None
        else f"the {incompatibility.provenance.value} profile stands and that publisher is not received"
    )
    return (
        f"a publisher on {path} offers QoS this subscription cannot accept on "
        f"{incompatibility.policy.name}; it was asking for {incompatibility.request}, and {answer}"
    )
