# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Hand-shaped lifecycle interfaces under the real ROS2 type names, which a Driver names in its own source."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from opensmi.ros2.runtime.messages import NestedType, PrimitiveType, SequenceType, ServiceTypes
from tests.fixture_messages import STRING, build, describe, nested

if TYPE_CHECKING:
    from tests.fake_runtime import FakeRuntime

UINT8 = (PrimitiveType(name="uint8"), "uint8")
UINT64 = (PrimitiveType(name="uint64"), "uint64")
BOOL = (PrimitiveType(name="bool"), "boolean")

State = build(describe("lifecycle_msgs/msg/State", id=UINT8, label=STRING))

Transition = build(describe("lifecycle_msgs/msg/Transition", id=UINT8, label=STRING))

TransitionDescription = build(
    describe(
        "lifecycle_msgs/msg/TransitionDescription",
        transition=nested("lifecycle_msgs/msg/Transition"),
        start_state=nested("lifecycle_msgs/msg/State"),
        goal_state=nested("lifecycle_msgs/msg/State"),
    )
)

# `timestamp` and `transition` are never read: rcl_lifecycle leaves both empty, so the Binding follows the state pair.
TransitionEvent = build(
    describe(
        "lifecycle_msgs/msg/TransitionEvent",
        timestamp=UINT64,
        transition=nested("lifecycle_msgs/msg/Transition"),
        start_state=nested("lifecycle_msgs/msg/State"),
        goal_state=nested("lifecycle_msgs/msg/State"),
    )
)

GetStateRequest = build(describe("lifecycle_msgs/srv/GetState_Request"))
GetStateResponse = build(
    describe("lifecycle_msgs/srv/GetState_Response", current_state=nested("lifecycle_msgs/msg/State"))
)
GetState = ServiceTypes(
    type_name="lifecycle_msgs/srv/GetState",
    request=GetStateRequest,
    response=GetStateResponse,
)

ChangeStateRequest = build(
    describe("lifecycle_msgs/srv/ChangeState_Request", transition=nested("lifecycle_msgs/msg/Transition"))
)
ChangeStateResponse = build(describe("lifecycle_msgs/srv/ChangeState_Response", success=BOOL))
ChangeState = ServiceTypes(
    type_name="lifecycle_msgs/srv/ChangeState",
    request=ChangeStateRequest,
    response=ChangeStateResponse,
)

GetAvailableTransitionsRequest = build(describe("lifecycle_msgs/srv/GetAvailableTransitions_Request"))
GetAvailableTransitionsResponse = build(
    describe(
        "lifecycle_msgs/srv/GetAvailableTransitions_Response",
        available_transitions=(
            SequenceType(element=NestedType(type_name="lifecycle_msgs/msg/TransitionDescription")),
            "sequence<TransitionDescription>",
        ),
    )
)
GetAvailableTransitions = ServiceTypes(
    type_name="lifecycle_msgs/srv/GetAvailableTransitions",
    request=GetAvailableTransitionsRequest,
    response=GetAvailableTransitionsResponse,
)

ManageLifecycleNodesRequest = build(describe("nav2_msgs/srv/ManageLifecycleNodes_Request", command=UINT8))
ManageLifecycleNodesResponse = build(describe("nav2_msgs/srv/ManageLifecycleNodes_Response", success=BOOL))
ManageLifecycleNodes = ServiceTypes(
    type_name="nav2_msgs/srv/ManageLifecycleNodes",
    request=ManageLifecycleNodesRequest,
    response=ManageLifecycleNodesResponse,
)

MESSAGE_TYPES = {"lifecycle_msgs/msg/TransitionEvent": TransitionEvent}
"""What a fake Runtime is given so that a Driver can subscribe to a node's transitions."""

SERVICE_TYPES = {
    "lifecycle_msgs/srv/GetState": GetState,
    "lifecycle_msgs/srv/ChangeState": ChangeState,
    "lifecycle_msgs/srv/GetAvailableTransitions": GetAvailableTransitions,
    "nav2_msgs/srv/ManageLifecycleNodes": ManageLifecycleNodes,
}
"""What a fake Runtime is given so that both shipped flavours resolve their types."""

STANDARD_SERVICE_TYPES = {name: types for name, types in SERVICE_TYPES.items() if not name.startswith("nav2_msgs")}
"""The same, without nav2 -- which is a robot where declaring the nav2 flavour is fatal at init."""

# `label` is filled as a real node fills it; nothing reads it.
_LABELS = {
    0: "unknown",
    1: "unconfigured",
    2: "inactive",
    3: "active",
    4: "finalized",
    10: "configuring",
    11: "cleaningup",
    12: "shuttingdown",
    13: "activating",
    14: "deactivating",
    15: "errorprocessing",
}

OFFERED = {1: (1, 5), 2: (2, 3, 6), 3: (4, 7), 4: ()}
"""Which transitions a node offers from each primary state, taken from a live Jazzy node.

Read off `get_available_transitions` on the `lifecycle` demo `lc_talker`: `unconfigured` offers
configure and shutdown, `inactive` offers cleanup, activate and shutdown, `active` offers
deactivate and shutdown, and `finalized` offers nothing at all, permanently.
"""

LANDS_IN = {1: 2, 2: 1, 3: 3, 4: 2, 5: 4, 6: 4, 7: 4}
"""Which primary state each transition leaves a node in, when it succeeds."""


def state(identifier: int) -> Any:
    """Build a `lifecycle_msgs/msg/State`, the way one arrives across the seam."""
    return State(id=identifier, label=_LABELS.get(identifier, ""))


def transition_event(*, start: int, goal: int) -> Any:
    """Build a transition event carrying one state pair, and the empty fields a real one has."""
    return TransitionEvent(
        timestamp=0,
        transition=Transition(id=0, label=""),
        start_state=state(start),
        goal_state=state(goal),
    )


def described(*, transition: int, start: int, goal: int) -> Any:
    """Build one entry of what `get_available_transitions` answers with."""
    return TransitionDescription(
        transition=Transition(id=transition, label=""),
        start_state=state(start),
        goal_state=state(goal),
    )


def serve_nodes(
    runtime: FakeRuntime,
    states: dict[str, int],
    *,
    refuse: set[str] | None = None,
) -> dict[str, int]:
    """Stand a set of live lifecycle nodes up on a fake Runtime, and hand back what they hold.

    The dictionary *is* the robot: node path to the primary state it rests in, moved by a
    `change_state` call exactly as a real node moves itself, and readable by a test afterwards to
    assert what was actually commanded. What this registers is the three `lifecycle_msgs`
    services over it -- with the available-transition list **derived from the state** rather than
    listed, because deriving it from a live node's own table is what makes a test of the gate a
    test of the real rule.

    :param runtime: The fake Runtime the Driver was built with.
    :param states: What each node is resting in, keyed by node path.
    :param refuse: Node paths that answer `change_state` with `success=False` rather than moving,
        which is what a C++ node does to a transition it will not take.
    :return: The same dictionary, so a test can read it as *what the robot did*.
    """
    refused = refuse or set()
    for node in states:
        runtime.serve_service(f"{node}/get_state", _get_state(states, node))
        runtime.serve_service(f"{node}/get_available_transitions", _transitions(states, node))
        runtime.serve_service(f"{node}/change_state", _change_state(states, node, node in refused))
        runtime.note_publishers(f"{node}/transition_event", 1)
    return states


def _get_state(states: dict[str, int], node: str) -> Any:
    """Return how one node answers `get_state`: whatever the test's dictionary says it is."""
    return lambda _request: GetStateResponse(current_state=state(states[node]))


def _transitions(states: dict[str, int], node: str) -> Any:
    """Return how one node answers `get_available_transitions`, derived from where it is."""
    return lambda _request: GetAvailableTransitionsResponse(
        available_transitions=tuple(
            described(transition=each, start=states[node], goal=LANDS_IN[each]) for each in OFFERED[states[node]]
        )
    )


def _change_state(states: dict[str, int], node: str, refuses: bool) -> Any:
    """Return how one node answers `change_state`: it moves, or it says no and does not."""

    def answer(request: Any) -> Any:
        if refuses:
            return ChangeStateResponse(success=False)
        states[node] = LANDS_IN[request.transition.id]
        return ChangeStateResponse(success=True)

    return answer
