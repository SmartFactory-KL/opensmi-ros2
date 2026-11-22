# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""QoS provenance, the resolver and the adaptation rule as pure functions over endpoint infos.

The compatibility directions were read off a live Jazzy graph; `test_runtime_jazzy.py` re-checks them.
"""

from __future__ import annotations

import pytest

from opensmi.ros2.runtime.qos import (
    DEFAULT_PROFILE,
    FOREVER,
    LATCHED,
    MOST_PERMISSIVE,
    SENSOR_DATA,
    Durability,
    EndpointInfo,
    Liveliness,
    Policy,
    Provenance,
    QoSNegotiation,
    QoSProfile,
    Reliability,
    _clashes,
    most_permissive,
    resolve,
)


def offering(**policies: object) -> EndpointInfo:
    """Return one publisher's endpoint info, named for whatever it differs from the default in."""
    return EndpointInfo(node_name="/a_publisher", qos=QoSProfile(**policies))  # type: ignore[arg-type]


# --- the profile itself ----------------------------------------------------------------------


def test_a_profile_carries_no_depth() -> None:
    assert not hasattr(QoSProfile(), "depth")


def test_the_default_profile_is_the_middlewares_own() -> None:
    assert (
        QoSProfile(
            reliability=Reliability.RELIABLE,
            durability=Durability.VOLATILE,
            deadline=FOREVER,
            liveliness=Liveliness.AUTOMATIC,
            lease=FOREVER,
        )
        == DEFAULT_PROFILE
    )


def test_the_most_permissive_profile_asks_for_the_least_of_every_policy() -> None:
    assert (
        QoSProfile(
            reliability=Reliability.BEST_EFFORT,
            durability=Durability.VOLATILE,
            deadline=FOREVER,
            liveliness=Liveliness.AUTOMATIC,
            lease=FOREVER,
        )
        == MOST_PERMISSIVE
    )


def test_the_two_shipped_conveniences_are_what_they_are_named_for() -> None:
    assert SENSOR_DATA.reliability is Reliability.BEST_EFFORT
    assert LATCHED.durability is Durability.TRANSIENT_LOCAL


@pytest.mark.parametrize("policy", ["deadline", "lease"])
@pytest.mark.parametrize("bad", [0.0, -1.0])
def test_a_duration_that_is_not_positive_is_refused_where_it_was_typed(policy: str, bad: float) -> None:
    with pytest.raises(ValueError, match=policy):
        QoSProfile(**{policy: bad})  # type: ignore[arg-type]


# --- compatibility ---------------------------------------------------------------------------


def test_the_most_permissive_request_clashes_with_nothing() -> None:
    offers = [
        QoSProfile(),
        QoSProfile(reliability=Reliability.BEST_EFFORT),
        QoSProfile(durability=Durability.TRANSIENT_LOCAL, deadline=0.5),
        QoSProfile(liveliness=Liveliness.MANUAL_BY_TOPIC, lease=0.25),
    ]

    assert all(_clashes(MOST_PERMISSIVE, offer) == () for offer in offers)


def test_a_reliable_request_clashes_with_a_best_effort_offer_and_not_the_other_way() -> None:
    reliable = QoSProfile(reliability=Reliability.RELIABLE)
    best_effort = QoSProfile(reliability=Reliability.BEST_EFFORT)

    assert _clashes(reliable, best_effort) == (Policy.RELIABILITY,)
    assert _clashes(best_effort, reliable) == ()


def test_a_transient_local_request_clashes_with_a_volatile_offer_and_not_the_other_way() -> None:
    latched = QoSProfile(durability=Durability.TRANSIENT_LOCAL)
    volatile = QoSProfile(durability=Durability.VOLATILE)

    assert _clashes(latched, volatile) == (Policy.DURABILITY,)
    assert _clashes(volatile, latched) == ()


def test_a_deadline_request_clashes_with_a_slower_offer_and_not_a_faster_one() -> None:
    # Verified on live Jazzy: DEADLINE fires only when the request is shorter than the offer.
    assert _clashes(QoSProfile(deadline=1.0), QoSProfile(deadline=2.0)) == (Policy.DEADLINE,)
    assert _clashes(QoSProfile(deadline=2.0), QoSProfile(deadline=1.0)) == ()
    assert _clashes(QoSProfile(deadline=FOREVER), QoSProfile(deadline=2.0)) == ()


def test_a_manual_liveliness_request_clashes_with_an_automatic_offer() -> None:
    manual = QoSProfile(liveliness=Liveliness.MANUAL_BY_TOPIC)
    automatic = QoSProfile(liveliness=Liveliness.AUTOMATIC)

    assert _clashes(manual, automatic) == (Policy.LIVELINESS,)
    assert _clashes(automatic, manual) == ()


def test_a_lease_request_clashes_with_a_longer_offer() -> None:
    assert _clashes(QoSProfile(lease=1.0), QoSProfile(lease=2.0)) == (Policy.LIVELINESS_LEASE_DURATION,)
    assert _clashes(QoSProfile(lease=2.0), QoSProfile(lease=1.0)) == ()


def test_every_policy_that_clashes_is_named_not_only_the_first() -> None:
    request = QoSProfile(reliability=Reliability.RELIABLE, durability=Durability.TRANSIENT_LOCAL, deadline=1.0)
    offer = QoSProfile(reliability=Reliability.BEST_EFFORT, durability=Durability.VOLATILE, deadline=2.0)

    assert _clashes(request, offer) == (Policy.RELIABILITY, Policy.DURABILITY, Policy.DEADLINE)


# --- the resolver ----------------------------------------------------------------------------


def test_no_publisher_resolves_to_the_maximally_permissive_request() -> None:
    assert resolve(()) == MOST_PERMISSIVE


def test_one_publisher_resolves_to_exactly_what_it_offers() -> None:
    offered = QoSProfile(reliability=Reliability.RELIABLE, durability=Durability.TRANSIENT_LOCAL, deadline=2.0)

    assert resolve([EndpointInfo(node_name="/talker", qos=offered)]) == offered


def test_the_resolution_is_the_most_permissive_of_every_offer_not_the_first() -> None:
    endpoints = [
        offering(reliability=Reliability.RELIABLE, durability=Durability.TRANSIENT_LOCAL),
        offering(reliability=Reliability.BEST_EFFORT, durability=Durability.TRANSIENT_LOCAL),
    ]

    assert resolve(endpoints) == QoSProfile(
        reliability=Reliability.BEST_EFFORT,
        durability=Durability.TRANSIENT_LOCAL,
    )


def test_the_resolution_is_compatible_with_every_offer_it_was_computed_from() -> None:
    endpoints = [
        offering(reliability=Reliability.RELIABLE, durability=Durability.TRANSIENT_LOCAL, deadline=1.0),
        offering(reliability=Reliability.BEST_EFFORT, durability=Durability.VOLATILE, deadline=3.0),
        offering(liveliness=Liveliness.MANUAL_BY_TOPIC, lease=2.0),
    ]

    resolved = resolve(endpoints)

    assert all(_clashes(resolved, endpoint.qos) == () for endpoint in endpoints)


def test_the_resolution_does_not_depend_on_the_order_the_publishers_arrived_in() -> None:
    endpoints = [
        offering(reliability=Reliability.RELIABLE, deadline=1.0),
        offering(reliability=Reliability.BEST_EFFORT, deadline=3.0),
        offering(durability=Durability.TRANSIENT_LOCAL),
    ]

    assert resolve(endpoints) == resolve(list(reversed(endpoints)))


def test_a_longer_deadline_is_the_more_permissive_request() -> None:
    endpoints = [offering(deadline=1.0), offering(deadline=3.0)]

    assert resolve(endpoints).deadline == 3.0


def test_one_publisher_with_no_deadline_makes_the_whole_resolution_deadline_free() -> None:
    endpoints = [offering(deadline=1.0), offering(deadline=FOREVER)]

    assert resolve(endpoints).deadline == FOREVER


def test_transient_local_survives_only_when_every_publisher_offers_it() -> None:
    both = [offering(durability=Durability.TRANSIENT_LOCAL), offering(durability=Durability.TRANSIENT_LOCAL)]
    one = [offering(durability=Durability.TRANSIENT_LOCAL), offering(durability=Durability.VOLATILE)]

    assert resolve(both).durability is Durability.TRANSIENT_LOCAL
    assert resolve(one).durability is Durability.VOLATILE


def test_most_permissive_of_nothing_is_the_maximally_permissive_profile() -> None:
    assert most_permissive(()) == MOST_PERMISSIVE


# --- provenance ------------------------------------------------------------------------------


def test_a_declared_profile_is_honoured_exactly_whatever_is_publishing() -> None:
    declared = QoSProfile(reliability=Reliability.RELIABLE, durability=Durability.TRANSIENT_LOCAL)
    negotiation = QoSNegotiation(declared=declared, endpoints=[offering(reliability=Reliability.BEST_EFFORT)])

    assert negotiation.provenance is Provenance.DECLARED
    assert negotiation.profile == declared


def test_no_declaration_and_a_live_publisher_is_discovered() -> None:
    negotiation = QoSNegotiation(declared=None, endpoints=[offering(reliability=Reliability.BEST_EFFORT)])

    assert negotiation.provenance is Provenance.DISCOVERED
    assert negotiation.profile.reliability is Reliability.BEST_EFFORT


def test_no_declaration_and_no_publisher_subscribes_maximally_permissive() -> None:
    negotiation = QoSNegotiation(declared=None, endpoints=())

    assert negotiation.provenance is Provenance.DISCOVERED
    assert negotiation.profile == MOST_PERMISSIVE
    assert not negotiation._bootstrapped


def test_a_binding_that_discovered_a_publisher_at_init_needs_no_bootstrap() -> None:
    negotiation = QoSNegotiation(declared=None, endpoints=[offering()])

    assert negotiation._bootstrapped
    assert negotiation.on_first_publisher([offering(durability=Durability.TRANSIENT_LOCAL)]) is None


# --- the one bootstrap tightening ------------------------------------------------------------


def test_the_first_publisher_tightens_a_binding_that_started_with_none() -> None:
    # A VOLATILE request on a TRANSIENT_LOCAL publisher gets no history, hence the tightening.
    negotiation = QoSNegotiation(declared=None, endpoints=())

    tightened = negotiation.on_first_publisher([offering(durability=Durability.TRANSIENT_LOCAL)])

    assert tightened is not None
    assert tightened.durability is Durability.TRANSIENT_LOCAL
    assert negotiation.profile == tightened
    assert negotiation._bootstrapped


def test_the_bootstrap_tightening_happens_exactly_once() -> None:
    negotiation = QoSNegotiation(declared=None, endpoints=())
    negotiation.on_first_publisher([offering(durability=Durability.TRANSIENT_LOCAL)])

    assert negotiation.on_first_publisher([offering(reliability=Reliability.RELIABLE)]) is None


def test_a_match_event_carrying_no_publisher_does_not_spend_the_bootstrap() -> None:
    negotiation = QoSNegotiation(declared=None, endpoints=())

    assert negotiation.on_first_publisher(()) is None
    assert not negotiation._bootstrapped


def test_a_declared_binding_never_bootstraps_even_with_no_publisher_at_init() -> None:
    declared = QoSProfile(reliability=Reliability.BEST_EFFORT)
    negotiation = QoSNegotiation(declared=declared, endpoints=())

    assert negotiation.on_first_publisher([offering(durability=Durability.TRANSIENT_LOCAL)]) is None
    assert negotiation.profile == declared


def test_a_bootstrap_that_resolves_to_what_is_already_held_moves_nothing() -> None:
    negotiation = QoSNegotiation(declared=None, endpoints=())

    assert negotiation.on_first_publisher([offering(reliability=Reliability.BEST_EFFORT)]) is None
    assert negotiation._bootstrapped


# --- adaptation ------------------------------------------------------------------------------


def test_a_discovered_binding_adapts_down_on_an_incompatibility() -> None:
    negotiation = QoSNegotiation(declared=None, endpoints=[offering(reliability=Reliability.RELIABLE)])

    adapted = negotiation.on_incompatibility(
        [offering(reliability=Reliability.RELIABLE), offering(reliability=Reliability.BEST_EFFORT)]
    )

    assert adapted is not None
    assert adapted.reliability is Reliability.BEST_EFFORT
    assert negotiation.profile == adapted


def test_a_declared_binding_never_adapts_however_incompatible_it_is() -> None:
    declared = QoSProfile(reliability=Reliability.RELIABLE)
    negotiation = QoSNegotiation(declared=declared, endpoints=())

    assert negotiation.on_incompatibility([offering(reliability=Reliability.BEST_EFFORT)]) is None
    assert negotiation.profile == declared


def test_adaptation_is_monotone_down_so_a_publisher_leaving_never_tightens_it_back() -> None:
    negotiation = QoSNegotiation(declared=None, endpoints=[offering(reliability=Reliability.RELIABLE)])
    negotiation.on_incompatibility([offering(reliability=Reliability.BEST_EFFORT)])

    assert negotiation.on_incompatibility([offering(reliability=Reliability.RELIABLE)]) is None
    assert negotiation.profile.reliability is Reliability.BEST_EFFORT


def test_an_incompatibility_with_nothing_on_the_graph_relaxes_the_whole_way() -> None:
    negotiation = QoSNegotiation(declared=None, endpoints=[offering(durability=Durability.TRANSIENT_LOCAL)])

    adapted = negotiation.on_incompatibility(())

    assert adapted == MOST_PERMISSIVE


def test_an_incompatibility_that_asks_for_nothing_new_reports_no_change() -> None:
    negotiation = QoSNegotiation(declared=None, endpoints=[offering(reliability=Reliability.BEST_EFFORT)])

    assert negotiation.on_incompatibility([offering(reliability=Reliability.BEST_EFFORT)]) is None
    assert negotiation.profile.reliability is Reliability.BEST_EFFORT


def test_adapting_spends_the_bootstrap_too_so_it_can_never_tighten_afterwards() -> None:
    negotiation = QoSNegotiation(declared=None, endpoints=())
    negotiation.on_incompatibility([offering(reliability=Reliability.BEST_EFFORT)])

    assert negotiation._bootstrapped


def test_one_call_answers_an_incompatibility_and_records_what_it_did() -> None:
    negotiation = QoSNegotiation(declared=None, endpoints=[offering(reliability=Reliability.RELIABLE)])

    reported = negotiation.incompatibility("/scan", Policy.RELIABILITY, [offering(reliability=Reliability.BEST_EFFORT)])

    assert reported.topic == "/scan"
    assert reported.policy is Policy.RELIABILITY
    assert reported.request.reliability is Reliability.RELIABLE, "what it was asking for, before it moved"
    assert reported.adapted is not None
    assert reported.adapted.reliability is Reliability.BEST_EFFORT
    assert reported.provenance is Provenance.DISCOVERED


def test_the_same_call_on_a_declared_profile_records_that_nothing_moved() -> None:
    declared = QoSProfile(reliability=Reliability.RELIABLE)
    negotiation = QoSNegotiation(declared=declared, endpoints=())

    reported = negotiation.incompatibility("/scan", Policy.RELIABILITY, [offering(reliability=Reliability.BEST_EFFORT)])

    assert reported.adapted is None
    assert reported.request == declared
    assert negotiation.profile == declared
