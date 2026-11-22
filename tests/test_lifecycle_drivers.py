# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The lifecycle Drivers against a fake Runtime, with no OPC UA server."""

from __future__ import annotations

from typing import Any

import pytest

from opensmi.ros2.runtime.errors import MissingInterfacePackageError
from opensmi.ros2.server.bindings import LifecycleBinding
from opensmi.ros2.server.drivers import (
    NAV2_COMMANDS,
    Nav2ManagerDriver,
    StandardLifecycleDriver,
)
from opensmi.ros2.server.errors import DeclarationError, ImplementationClassError
from opensmi.ros2.server.lifecycle import LifecycleNodeState, LifecycleNodeTransition
from tests import fixture_lifecycle
from tests.fake_runtime import FakeRuntime

TALKER = "/lc_talker"
LISTENER = "/lc_listener"

UNCONFIGURED = int(LifecycleNodeState.UNCONFIGURED)
INACTIVE = int(LifecycleNodeState.INACTIVE)
ACTIVE = int(LifecycleNodeState.ACTIVE)
FINALIZED = int(LifecycleNodeState.FINALIZED)


@pytest.fixture
async def runtime() -> FakeRuntime:
    started = FakeRuntime(
        message_types=fixture_lifecycle.MESSAGE_TYPES,
        service_types=fixture_lifecycle.SERVICE_TYPES,
    )
    await started.start()
    return started


def standard(**keywords) -> LifecycleBinding:
    return LifecycleBinding(name="Talker", path=TALKER, **keywords)


def nav2(**keywords) -> LifecycleBinding:
    return LifecycleBinding(
        name="Navigation",
        path="/lifecycle_manager",
        nodes=(TALKER, LISTENER),
        driver="nav2",
        **keywords,
    )


async def opened(runtime: FakeRuntime, binding: LifecycleBinding):
    driver = binding.driver_class(binding=binding, runtime=runtime)
    await driver.open()
    return driver


def requests(runtime: FakeRuntime, call: str) -> list[tuple[str, Any]]:
    """Return every request that reached one kind of service, in order."""
    return [(service, request) for service, request in runtime.requests if service.endswith(call)]


def sent_field(request: Any, name: str) -> Any:
    """Read one field off a request, the way everything reads a message built at run time."""
    return getattr(request, name)


# -- the declaration ---------------------------------------------------------------------


def test_the_standard_flavour_is_the_default_and_its_path_is_its_node() -> None:
    binding = standard()

    assert binding.driver == "lifecycle"
    assert binding.observed == (TALKER,)
    assert binding.driver_class is StandardLifecycleDriver


def test_a_flavour_that_coordinates_a_set_must_be_given_the_set() -> None:
    with pytest.raises(DeclarationError, match="no lifecycle state of its own"):
        LifecycleBinding(name="Navigation", path="/lifecycle_manager", driver="nav2")


def test_an_unregistered_flavour_is_fatal_where_it_was_typed() -> None:
    with pytest.raises(DeclarationError, match="the flavours that ship are: lifecycle, nav2"):
        LifecycleBinding(name="Talker", path=TALKER, driver="ros1")


def test_a_driver_is_chosen_by_its_name() -> None:
    assert LifecycleBinding(name="N", path=TALKER, nodes=(LISTENER,), driver="nav2").driver_class is Nav2ManagerDriver


def test_a_driver_is_chosen_by_its_class() -> None:
    class Mine(StandardLifecycleDriver):
        flavour = "mine"

    assert LifecycleBinding(name="Talker", path=TALKER, driver=Mine).driver_class is Mine


def test_a_driver_that_is_not_a_driver_class_is_fatal_where_it_was_typed() -> None:
    with pytest.raises(ImplementationClassError, match="not a subclass of LifecycleDriver"):
        LifecycleBinding(name="Talker", path=TALKER, driver=dict)


def test_a_driver_that_is_neither_a_name_nor_a_class_is_fatal_where_it_was_typed() -> None:
    with pytest.raises(DeclarationError, match="name nor a LifecycleDriver subclass"):
        LifecycleBinding(name="Talker", path=TALKER, driver=3)  # type: ignore[arg-type]


def test_a_custom_driver_decides_for_itself_whether_it_needs_the_nodes_listed() -> None:
    class Coordinated(StandardLifecycleDriver):
        flavour = "coordinated"
        coordinates_a_set = True

    with pytest.raises(DeclarationError, match="Coordinated"):
        LifecycleBinding(name="Talker", path="/coordinator", driver=Coordinated)


def test_a_flavour_whose_interface_package_is_absent_is_fatal_at_init() -> None:
    # A missing nav2 is a wrong deployment, so a traceback while building.
    bare = FakeRuntime(
        message_types=fixture_lifecycle.MESSAGE_TYPES,
        service_types=fixture_lifecycle.STANDARD_SERVICE_TYPES,
    )

    with pytest.raises(MissingInterfacePackageError, match="nav2_msgs"):
        Nav2ManagerDriver(binding=nav2(), runtime=bare)


# -- observing ---------------------------------------------------------------------------


async def test_a_driver_subscribes_to_every_node_it_observes(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, nav2())

    assert driver.paths == (TALKER, LISTENER)
    assert runtime.deliver(f"{TALKER}/transition_event", fixture_lifecycle.transition_event(start=10, goal=2)) == 1
    assert runtime.deliver(f"{LISTENER}/transition_event", fixture_lifecycle.transition_event(start=10, goal=2)) == 1


async def test_a_transition_reaches_the_driver_as_the_state_pair(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, standard())
    stream = driver.transitions(TALKER)
    runtime.deliver(f"{TALKER}/transition_event", fixture_lifecycle.transition_event(start=14, goal=INACTIVE))

    transition = await anext(stream)

    assert (transition.path, transition.start, transition.goal) == (TALKER, 14, INACTIVE)
    assert "deactivating -> inactive" in repr(transition)


async def test_a_node_appearing_is_a_match_event_rather_than_a_graph_read(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, standard())
    counts = driver.appearances(TALKER)

    assert await anext(counts) == 0
    runtime.note_publishers(f"{TALKER}/transition_event", 1)
    assert await anext(counts) == 1
    assert runtime.graph.polls == 0


async def test_reading_a_state_is_one_call_and_a_node_that_is_off_answers_nothing(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, standard())

    assert await driver.read_state(TALKER) is None

    fixture_lifecycle.serve_nodes(runtime, {TALKER: ACTIVE})

    assert await driver.read_state(TALKER) == LifecycleNodeState.ACTIVE


# -- the gate ----------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("resting", "expected"),
    [
        (UNCONFIGURED, {LifecycleNodeTransition.CONFIGURE}),
        (INACTIVE, {LifecycleNodeTransition.CLEANUP, LifecycleNodeTransition.ACTIVATE}),
        (ACTIVE, {LifecycleNodeTransition.DEACTIVATE}),
        (FINALIZED, set()),
    ],
)
async def test_the_gate_is_what_the_node_offers_minus_what_we_never_command(
    runtime: FakeRuntime,
    resting: int,
    expected: set[LifecycleNodeTransition],
) -> None:
    driver = await opened(runtime, standard())
    fixture_lifecycle.serve_nodes(runtime, {TALKER: resting})

    assert await driver.offered_transitions(TALKER) == expected


async def test_a_node_that_did_not_answer_is_not_a_node_that_offered_nothing(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, standard())

    assert await driver.offered_transitions(TALKER) is None

    states = fixture_lifecycle.serve_nodes(runtime, {TALKER: FINALIZED})

    assert await driver.offered_transitions(TALKER) == frozenset()
    assert states == {TALKER: FINALIZED}


async def test_a_robot_that_is_off_is_a_fault_rather_than_a_trigger_with_nothing_to_do(
    runtime: FakeRuntime,
) -> None:
    # Without this every trigger succeeds with nothing sent to a powered-off robot.
    driver = await opened(runtime, LifecycleBinding(name="Pair", path=TALKER, nodes=(TALKER, LISTENER)))

    faults = await driver.apply("start")

    assert faults == (
        f"{TALKER} did not answer when asked which transitions it offers",
        f"{LISTENER} did not answer when asked which transitions it offers",
    )


async def test_a_transition_the_node_does_not_offer_is_never_sent(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, standard())
    fixture_lifecycle.serve_nodes(runtime, {TALKER: FINALIZED})

    faults = await driver.apply("reset")

    assert faults == ()
    assert [service for service, _ in runtime.requests] == [f"{TALKER}/get_available_transitions"]


# -- commanding: the standard flavour ----------------------------------------------------


async def test_the_standard_flavour_sends_one_change_state_per_node(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, LifecycleBinding(name="Pair", path=TALKER, nodes=(TALKER, LISTENER)))
    states = fixture_lifecycle.serve_nodes(runtime, {TALKER: UNCONFIGURED, LISTENER: UNCONFIGURED})

    assert await driver.apply("reset") == ()

    assert states == {TALKER: INACTIVE, LISTENER: INACTIVE}
    assert [service for service, _ in runtime.requests if service.endswith("change_state")] == [
        f"{TALKER}/change_state",
        f"{LISTENER}/change_state",
    ]


async def test_halt_deactivates_and_then_cleans_up_and_never_shuts_down(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, standard())
    states = fixture_lifecycle.serve_nodes(runtime, {TALKER: ACTIVE})
    ledger = {TALKER: {LifecycleNodeTransition.CONFIGURE, LifecycleNodeTransition.ACTIVATE}}

    assert await driver.apply("halt", ledger) == ()

    assert states == {TALKER: UNCONFIGURED}
    assert not ledger[TALKER]
    sent = [sent_field(request, "transition").id for service, request in requests(runtime, "change_state")]
    assert sent == [int(LifecycleNodeTransition.DEACTIVATE), int(LifecycleNodeTransition.CLEANUP)]


async def test_halt_leaves_a_node_the_ledger_does_not_hold(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, standard())
    states = fixture_lifecycle.serve_nodes(runtime, {TALKER: ACTIVE})

    assert await driver.apply("halt", {}) == ()

    assert states == {TALKER: ACTIVE}
    assert not runtime.requests


async def test_halt_takes_back_only_the_transitions_the_ledger_holds(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, standard())
    states = fixture_lifecycle.serve_nodes(runtime, {TALKER: ACTIVE})
    ledger = {TALKER: {LifecycleNodeTransition.ACTIVATE}}

    assert await driver.apply("halt", ledger) == ()

    assert states == {TALKER: INACTIVE}
    assert not ledger[TALKER]


async def test_reset_and_start_record_only_what_they_sent(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, LifecycleBinding(name="Pair", path=TALKER, nodes=(TALKER, LISTENER)))
    fixture_lifecycle.serve_nodes(runtime, {TALKER: ACTIVE, LISTENER: UNCONFIGURED})
    ledger: dict[str, set[LifecycleNodeTransition]] = {}

    assert await driver.apply("reset", ledger) == ()
    assert await driver.apply("start", ledger) == ()

    assert ledger == {LISTENER: {LifecycleNodeTransition.CONFIGURE, LifecycleNodeTransition.ACTIVATE}}


async def test_a_halt_that_is_refused_keeps_what_it_could_not_take_back(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, standard())
    fixture_lifecycle.serve_nodes(runtime, {TALKER: ACTIVE}, refuse={TALKER})
    ledger = {TALKER: {LifecycleNodeTransition.CONFIGURE, LifecycleNodeTransition.ACTIVATE}}

    faults = await driver.apply("halt", ledger)

    assert faults == (f"{TALKER} refused to deactivate",)
    assert ledger[TALKER] == {LifecycleNodeTransition.CONFIGURE, LifecycleNodeTransition.ACTIVATE}


async def test_halting_a_node_that_is_already_down_commands_nothing(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, standard())
    states = fixture_lifecycle.serve_nodes(runtime, {TALKER: UNCONFIGURED})
    ledger = {TALKER: {LifecycleNodeTransition.CONFIGURE, LifecycleNodeTransition.ACTIVATE}}

    assert await driver.apply("halt", ledger) == ()

    assert states == {TALKER: UNCONFIGURED}
    assert [service for service, _ in runtime.requests if service.endswith("change_state")] == []
    assert not ledger[TALKER]


async def test_a_node_that_refuses_is_named_in_a_fault(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, LifecycleBinding(name="Pair", path=TALKER, nodes=(TALKER, LISTENER)))
    fixture_lifecycle.serve_nodes(runtime, {TALKER: INACTIVE, LISTENER: INACTIVE}, refuse={LISTENER})

    faults = await driver.apply("start")

    assert faults == (f"{LISTENER} refused to activate",)


async def test_a_node_whose_services_vanish_between_the_gate_and_the_command(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, standard())
    fixture_lifecycle.serve_nodes(runtime, {TALKER: INACTIVE})
    runtime.serve_service(f"{TALKER}/change_state", None)  # takes the request, never answers

    faults = await driver.apply("start")

    assert faults == (f"{TALKER} did not answer a activate request",)


# -- commanding: the nav2 flavour --------------------------------------------------------


async def test_the_nav2_flavour_sends_one_manage_nodes_call_for_the_whole_set(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, nav2())
    fixture_lifecycle.serve_nodes(runtime, {TALKER: INACTIVE, LISTENER: INACTIVE})
    runtime.serve_service(
        "/lifecycle_manager/manage_nodes",
        lambda _request: fixture_lifecycle.ManageLifecycleNodesResponse(success=True),
    )

    assert await driver.apply("start") == ()

    managed = [sent_field(request, "command") for _, request in requests(runtime, "manage_nodes")]
    assert managed == [NAV2_COMMANDS[LifecycleNodeTransition.ACTIVATE]]


async def test_the_nav2_flavour_gates_on_the_managed_nodes_own_transitions(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, nav2())
    fixture_lifecycle.serve_nodes(runtime, {TALKER: FINALIZED, LISTENER: FINALIZED})

    assert await driver.apply("start") == ()

    assert [service for service, _ in runtime.requests if service.endswith("manage_nodes")] == []


async def test_a_manager_that_refuses_is_named_in_a_fault(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, nav2())
    fixture_lifecycle.serve_nodes(runtime, {TALKER: INACTIVE, LISTENER: INACTIVE})
    runtime.serve_service(
        "/lifecycle_manager/manage_nodes",
        lambda _request: fixture_lifecycle.ManageLifecycleNodesResponse(success=False),
    )

    faults = await driver.apply("start")

    assert faults == (f"the manager /lifecycle_manager refused to activate {TALKER}, {LISTENER}",)


async def test_both_flavours_command_the_same_four_transitions(runtime: FakeRuntime) -> None:
    assert set(NAV2_COMMANDS) == set(LifecycleNodeTransition)
    assert NAV2_COMMANDS[LifecycleNodeTransition.ACTIVATE] == 2
    assert NAV2_COMMANDS[LifecycleNodeTransition.DEACTIVATE] == 1


async def test_closing_a_driver_releases_every_subscription(runtime: FakeRuntime) -> None:
    driver = await opened(runtime, nav2())

    await driver.close()

    assert runtime.deliver(f"{TALKER}/transition_event", fixture_lifecycle.transition_event(start=10, goal=2)) == 0
    await driver.close()
