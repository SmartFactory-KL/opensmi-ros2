# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A repeated message as an array of a structure, read and written as a stranger would.

Wire-level tests run the client in a subprocess that knows none of this package's types.
"""

from __future__ import annotations

import struct
from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua
from asyncua.ua.object_ids import ObjectIds

from opensmi.ros2.server.bindings import ActionBinding, MonitoringBinding
from opensmi.ros2.server.events import root_of
from tests import fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.fixture_structures import encoding_id as encoding_node_id
from tests.served import TestComponent, TestMachine, connected, elements_of, leaf, serve, until_elements, wire_value
from tests.wire_client import run_client

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from opensmi.server import Server

MESSAGE_TYPES = {
    "fixture_msgs/msg/Time": fixture_messages.Time,
    "fixture_msgs/msg/Header": fixture_messages.Header,
    "fixture_msgs/msg/Pair": fixture_messages.Pair,
    "fixture_msgs/msg/Unit": fixture_messages.Unit,
    "fixture_msgs/msg/Units": fixture_messages.Units,
    "fixture_msgs/msg/Waypoint": fixture_messages.Waypoint,
    "fixture_msgs/msg/Waypoints": fixture_messages.Waypoints,
}

ACTION_TYPES = {"fixture_msgs/action/Inventory": fixture_messages.Inventory}

TRAJECTORY = MonitoringBinding(name="Trajectory", path="/trajectory", message_type="fixture_msgs/msg/Waypoints")
INVENTORY = MonitoringBinding(name="Inventory", path="/inventory", message_type="fixture_msgs/msg/Units")
COUNT = ActionBinding(name="Count", path="/count", action_type="fixture_msgs/action/Inventory")


class Arm(TestComponent):
    bindings = (TRAJECTORY, INVENTORY, COUNT)


@pytest.fixture
def runtime() -> FakeRuntime:
    started = FakeRuntime(message_types=MESSAGE_TYPES, action_types=ACTION_TYPES)
    started.serve_action("/count")
    return started


@pytest.fixture
async def served(runtime: FakeRuntime) -> AsyncIterator[tuple[Server, Any]]:
    machine = TestMachine(Arm(name="Arm"), runtime=runtime)
    server = await serve(machine)
    try:
        yield server, machine.added[0]
    finally:
        await server.shutdown()


def variable_of(arm: Any, name: str) -> Any:
    return leaf(arm.monitoring, name)


# -- the bytes of an element, packed by hand ----------------------------------------------------


def string(text: str) -> bytes:
    raw = text.encode()
    return struct.pack("<i", len(raw)) + raw


def pair_body(key: str, value: str) -> bytes:
    return string(key) + string(value)


def unit_body(
    name: str, *pairs: tuple[str, str], level: int = 0, hardware_id: str = "robot_1", message: str = ""
) -> bytes:
    """A unit as the structure of ``fixture_msgs/msg/Unit`` encodes it, its pairs inline."""
    return (
        struct.pack("<b", level)
        + string(name)
        + string(hardware_id)
        + string(message)
        + struct.pack("<i", len(pairs))
        + b"".join(pair_body(*each) for each in pairs)
    )


def waypoint_body(seed: float, positions: int = 2) -> bytes:
    """A waypoint as the structure of ``fixture_msgs/msg/Waypoint`` encodes it."""
    raw = b"\x00\x01\xff"
    return (
        struct.pack("<i", positions)
        + b"".join(struct.pack("<d", seed + each) for each in range(positions))
        + struct.pack("<i3d", 3, seed, 0.0, -seed)  # covariance, a fixed array that still counts
        + struct.pack("<i", len(raw))
        + raw
        + struct.pack("<iI", int(seed), 500)  # time_from_start/sec, nanosec
        + struct.pack("<?", True)
    )


WAYPOINT_ONE = {
    "positions": [1.0, 2.0],
    "covariance": [1.0, 0.0, -1.0],
    "raw": b"\x00\x01\xff",
    "time_from_start.sec": 1,
    "time_from_start.nanosec": 500,
    "valid": True,
}
"""``fixture_messages.waypoint(seed=1.0)`` as the maps the projection decodes it to."""


# -- reading, as a stranger -----------------------------------------------------------------------


async def test_a_stranger_reads_the_definition_and_the_bytes_of_a_trajectory(served, runtime: FakeRuntime) -> None:
    server, arm = served
    runtime.note_publishers("/trajectory", 1)
    runtime.deliver("/trajectory", fixture_messages.waypoints(fixture_messages.waypoint(seed=1.0)))
    variable = variable_of(arm, "Trajectory.Points")
    async with connected(server, "visitor") as client:
        await until_elements(client.get_node(variable.ua_node.nodeid), variable, [WAYPOINT_ONE])

    result = await run_client(server, "visitor", variable.ua_node.nodeid, {"definition": True, "writes": []})

    namespace = arm.ros2_namespace_index
    points = encoding_node_id("fixture_msgs/msg/Waypoint", namespace).to_string()
    assert result[0] == ["initial", [[points, waypoint_body(1.0).hex()]]]
    label, datatype, rank, fields = result[1]
    assert (label, datatype, rank) == ("definition", f"ns={namespace};s=fixture_msgs/msg/Waypoint", 1)
    double = "i=11"
    assert fields == [
        ["positions", double, 1, [0]],
        ["covariance", double, 1, [0]],
        ["raw", "i=15", -1, []],
        ["time_from_start", f"ns={namespace};s=fixture_msgs/msg/Time", -1, []],
        ["valid", "i=1", -1, []],
    ]


async def test_a_stranger_reads_a_list_inside_each_element_of_a_list(served, runtime: FakeRuntime) -> None:
    server, arm = served
    runtime.note_publishers("/inventory", 1)
    runtime.deliver(
        "/inventory",
        fixture_messages.units(
            fixture_messages.unit(name="motor", level=1, values=(("temperature", "41.0"), ("firmware", "3b1c"))),
            fixture_messages.unit(name="brake"),
        ),
    )
    variable = variable_of(arm, "Inventory.Units")
    async with connected(server, "visitor") as client:
        node = client.get_node(variable.ua_node.nodeid)
        await until_elements(
            node,
            variable,
            [
                {
                    "level": 1,
                    "name": "motor",
                    "hardware_id": "robot_1",
                    "message": "",
                    "values": [{"key": "temperature", "value": "41.0"}, {"key": "firmware", "value": "3b1c"}],
                },
                {"level": 0, "name": "brake", "hardware_id": "robot_1", "message": "", "values": []},
            ],
        )

    result = await run_client(server, "visitor", variable.ua_node.nodeid, {"definition": True, "writes": []})

    namespace = arm.ros2_namespace_index
    units = encoding_node_id("fixture_msgs/msg/Unit", namespace).to_string()
    assert result[0] == [
        "initial",
        [
            [units, unit_body("motor", ("temperature", "41.0"), ("firmware", "3b1c"), level=1).hex()],
            [units, unit_body("brake").hex()],
        ],
    ]
    pair = f"ns={namespace};s=fixture_msgs/msg/Pair"
    assert result[1][3][-1] == ["values", pair, 1, [0]]
    subtypes = await server.ua_server.nodes.base_structure_type.get_children(refs=ObjectIds.HasSubtype)
    names = [(await each.read_browse_name()).Name for each in subtypes]
    # The element types, and the levels of the goal, feedback and result that nest them.
    assert {
        "fixture_msgs/msg/Pair",
        "fixture_msgs/msg/Time",
        "fixture_msgs/msg/Unit",
        "fixture_msgs/msg/Waypoint",
    } <= set(names)


# -- writing, as a stranger -----------------------------------------------------------------------


async def test_a_stranger_is_refused_a_body_that_cannot_be_read_whole_inside_a_list(served) -> None:
    server, arm = served
    skill = arm.skill_set["Count"]
    parameter = leaf(skill.parameter_set, "Units")
    namespace = arm.ros2_namespace_index
    units = encoding_node_id("fixture_msgs/msg/Unit", namespace).to_string()
    written = [
        [units, unit_body("motor", ("temperature", "41.0"), ("firmware", "3b1c"), level=1).hex()],
        [units, unit_body("brake").hex()],
    ]
    run_out = unit_body("arm", ("a", "1"))[:-3].hex()
    steps = {
        "lock": root_of(arm).lock.ua_node.nodeid.to_string(),
        "writes": [
            ["good", "elements", written],
            ["body runs out", "elements", [[units, run_out]]],
            ["empty", "elements", []],
        ],
    }

    result = await run_client(server, "operator", parameter.ua_node.nodeid, steps)

    outcomes = {label: (status, stored) for label, status, stored in result[1:]}
    assert outcomes["good"] == ("Good", written)
    # asyncua refuses a body that ends mid-field while decoding the request.
    assert outcomes["body runs out"] == ("BadInternalError", written)
    assert outcomes["empty"] == ("Good", [])
    assert (await skill.build_goal(await skill.read_parameters())) == fixture_messages.InventoryGoal(units=())


async def test_what_a_stranger_wrote_becomes_the_goal_message(served) -> None:
    server, arm = served
    skill = arm.skill_set["Count"]
    parameter = leaf(skill.parameter_set, "Units")
    units = encoding_node_id("fixture_msgs/msg/Unit", arm.ros2_namespace_index).to_string()
    steps = {
        "lock": root_of(arm).lock.ua_node.nodeid.to_string(),
        "writes": [
            [
                "good",
                "elements",
                [
                    [units, unit_body("motor", ("temperature", "41.0"), ("firmware", "3b1c"), level=1).hex()],
                    [units, unit_body("brake", hardware_id="robot_2", message="stopped").hex()],
                ],
            ]
        ],
    }

    assert (await run_client(server, "operator", parameter.ua_node.nodeid, steps))[1][1] == "Good"

    goal = await skill.build_goal(await skill.read_parameters())
    assert goal == fixture_messages.InventoryGoal(
        units=(
            fixture_messages.unit(name="motor", level=1, values=(("temperature", "41.0"), ("firmware", "3b1c"))),
            fixture_messages.unit(name="brake", hardware_id="robot_2", message="stopped"),
        )
    )


# -- writing and reading in the server's own process --------------------------------------------------


async def test_a_client_writes_waypoints_and_reads_back_what_it_wrote(served) -> None:
    server, arm = served
    skill = arm.skill_set["Count"]
    parameter = leaf(skill.parameter_set, "Units")
    elements = [
        {"level": 2, "name": "x", "hardware_id": "h", "message": "m", "values": [{"key": "k", "value": "v"}]},
        {"level": 0, "name": "y", "hardware_id": "h", "message": "", "values": []},
    ]

    async with connected(server) as client:
        await client.get_node(root_of(arm).lock.ua_node.nodeid).call_method("2:InitLock")
        node = client.get_node(parameter.ua_node.nodeid)
        await node.write_value(ua.Variant(wire_value(parameter, elements), ua.VariantType.ExtensionObject))
        stored = elements_of(parameter, await node.read_value())

    assert stored == elements
    assert (await skill.build_goal(await skill.read_parameters())).units == (
        fixture_messages.unit(name="x", level=2, hardware_id="h", message="m", values=(("k", "v"),)),
        fixture_messages.unit(name="y", hardware_id="h"),
    )
