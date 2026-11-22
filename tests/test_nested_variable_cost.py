# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""What rendering a message as a complex Variable costs per message, against one Variable per field.

Nested (how the package renders): the message is one complex opensmi Variable, written once per
message; opensmi brings every child in step in the same call. Flat (the baseline the nesting replaced): one
plain opensmi Variable per field, written one await per field. Both go through the server's access control
the way the Bindings write, with hand-built field trees on an opensmi server. Run on demand:
``pytest -m perf -s tests/test_nested_variable_cost.py``. The thresholds only guard against an order of
magnitude (the criterion: the loop is not held longer than one period of a 100 Hz stream).
"""

from __future__ import annotations

import asyncio
import os
import statistics
import threading
import time
from dataclasses import dataclass
from typing import Any

import pytest
from asyncua import ua
from asyncua.client.client import Client
from opensmi.server import UaVariable

from opensmi.ros2.server.mutation import NodeAddition
from opensmi.ros2.server.projection import PlanVariable
from opensmi.ros2.server.status import GOOD, status_value
from opensmi.ros2.server.structures import structure_class, structure_instance
from tests import fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.served import TestComponent, TestMachine, serve

pytestmark = pytest.mark.perf

VT = ua.VariantType
D, S, I32, U32 = VT.Double, VT.String, VT.Int32, VT.UInt32

# A tree maps a field name to a subtree or to a leaf (variant type, array length or None, 0 for a sequence).
Tree = dict[str, Any]
Leaf = tuple[VT, int | None]

_STAMP: Tree = {"sec": (I32, None), "nanosec": (U32, None)}
_XYZ: Tree = {"x": (D, None), "y": (D, None), "z": (D, None)}
_WITH_COVARIANCE = (D, 36)

ODOMETRY: Tree = {
    "header": {"stamp": _STAMP, "frame_id": (S, None)},
    "child_frame_id": (S, None),
    "pose": {
        "pose": {"position": _XYZ, "orientation": {**_XYZ, "w": (D, None)}},
        "covariance": _WITH_COVARIANCE,
    },
    "twist": {"twist": {"linear": _XYZ, "angular": _XYZ}, "covariance": _WITH_COVARIANCE},
}
JOINT_STATE: Tree = {
    "header": {"stamp": _STAMP, "frame_id": (S, None)},
    "name": (S, 0),
    "position": (D, 0),
    "velocity": (D, 0),
    "effort": (D, 0),
}


def _deep(depth: int) -> Tree:
    """Return a binary tree of messages, each level with one scalar leaf of its own."""
    tree: Tree = {"v": (D, None)}
    if depth:
        tree |= {"a": _deep(depth - 1), "b": _deep(depth - 1)}
    return tree


DEEP: Tree = _deep(4)
"""No deep message stub exists; this one has 31 messages and 31 leaves, five levels down."""

MESSAGES: dict[str, Tree] = {
    "nav_msgs/Odometry": ODOMETRY,
    "sensor_msgs/JointState": JOINT_STATE,
    "synthetic/Deep5": DEEP,
}
RATES = {"one topic at 100 Hz": 100, "envelope, 5000 msg/s": 5000}
WARMUP, MESSAGES_PER_ROUND, ROUNDS = 100, 1000, int(os.environ.get("OPENSMI_PERF_ROUNDS", "3"))
SEQUENCE_LENGTH = 7


def leaves(tree: Tree, prefix: str = "") -> dict[str, Leaf]:
    """Return every leaf of a tree by its dotted path."""
    found: dict[str, Leaf] = {}
    for name, child in tree.items():
        if isinstance(child, dict):
            found |= leaves(child, f"{prefix}{name}.")
        else:
            found[f"{prefix}{name}"] = child
    return found


def inner_paths(tree: Tree, prefix: str = "") -> list[str]:
    """Return the path of every inner level, the root as the empty string, parents first."""
    found = [prefix.rstrip(".")]
    for name, child in tree.items():
        if isinstance(child, dict):
            found += inner_paths(child, f"{prefix}{name}.")
    return found


def sample(path: str, leaf: Leaf, version: int) -> object:
    """Return a value for a leaf that differs between versions, so every write is a change."""
    variant, length = leaf
    count = length or SEQUENCE_LENGTH
    if variant is S:
        return [f"{path}{version}"] * count if length is not None else f"{path}{version}"
    if variant in (I32, U32):
        return version
    return [version + i / 7 for i in range(count)] if length is not None else version + 0.5


@dataclass
class Rendering:
    """The nodes of one message type on the server, in one of the two renderings.

    :ivar roots: The nodes a message is written to: the root of a complex Variable, or every field.
    :ivar everything: Every node a client may monitor.
    :ivar structure: The dataclass of the message, for the nested rendering.
    """

    roots: dict[str, UaVariable[Any]]
    everything: list[UaVariable[Any]]
    structure: type | None


def structure_of(name: str, tree: Tree, path: str = "") -> type:
    """Describe a tree as nested structures, one per level, as the package's per-type classes are."""
    fields = tuple(
        (
            field,
            structure_of(name, child, f"{path}{field}.")
            if isinstance(child, dict)
            else (list[getattr(ua, child[0].name)] if child[1] is not None else getattr(ua, child[0].name)),
            0 if isinstance(child, dict) else child[1] or 0,
        )
        for field, child in tree.items()
    )
    return structure_class(f"{name}/{path or 'root'}".rstrip("."), fields)


def descendants(variable: UaVariable[Any]) -> list[UaVariable[Any]]:
    """Return a variable and every child below it."""
    return [variable, *(below for child in variable.children.values() for below in descendants(child))]


async def build(item: Any, name: str, tree: Tree, *, nested: bool) -> Rendering:
    """Create the Variables of one message in one rendering, in the Monitoring of a Component."""
    kind = "Nested" if nested else "Flat"
    first = {path: sample(path, leaf, 0) for path, leaf in leaves(tree).items()}
    if nested:
        structure = structure_of(f"{kind}{name}", tree)
        root = UaVariable(
            structure_instance(structure, first),
            name=f"{kind}{name}",
            description="a message",
        )
        await item.monitoring.add(root)
        return Rendering({"": root}, descendants(root), structure)
    fields: dict[str, UaVariable[Any]] = {}
    for index, (path, leaf) in enumerate(leaves(tree).items()):
        variable = UaVariable(ua.Variant(first[path], leaf[0]), name=f"{kind}{name}{index}", description=path)
        await item.monitoring.add(variable)
        fields[path] = variable
    return Rendering(fields, list(fields.values()), None)


async def write_message(tree: Tree, rendering: Rendering, version: int) -> tuple[int, int]:
    """Write one message the way the Bindings do; return the ns spent encoding and the ns in total."""
    flat = {path: sample(path, leaf, version) for path, leaf in leaves(tree).items()}
    start = time.perf_counter_ns()  # the values come from ROS2 in production, so not part of the cost
    encoded = 0
    if rendering.structure is not None:
        before = time.perf_counter_ns()
        value = structure_instance(rendering.structure, flat)
        encoded = time.perf_counter_ns() - before
        await rendering.roots[""].ua_node.write_value(status_value(value, VT.ExtensionObject, GOOD))
    else:
        for path, variable in rendering.roots.items():
            await variable.ua_node.write_value(status_value(flat[path], leaf_variant(tree, path), GOOD))
    return encoded, time.perf_counter_ns() - start


def leaf_variant(tree: Tree, path: str) -> VT:
    return leaves(tree)[path][0]


@dataclass
class Stats:
    """Microseconds per message."""

    median: float
    p95: float
    p99: float

    @classmethod
    def of(cls, nanoseconds: list[int]) -> Stats:
        cuts = statistics.quantiles([n / 1000 for n in nanoseconds], n=100)
        return cls(statistics.median(nanoseconds) / 1000, cuts[94], cuts[98])


async def measure(
    tree: Tree, rendering: Rendering, count: int, *, yielding: bool = False
) -> tuple[list[int], list[int], float]:
    """Write ``count`` messages; return per-message totals, encode times and the wall time per message in ns."""
    totals: list[int] = []
    encodes: list[int] = []
    begin = time.perf_counter_ns()
    for version in range(1, count + 1):
        encoded, total = await write_message(tree, rendering, version)
        totals.append(total)
        encodes.append(encoded)
        if yielding:
            await asyncio.sleep(0)
    return totals, encodes, (time.perf_counter_ns() - begin) / count


class Subscriber(threading.Thread):
    """One client on its own loop and thread, monitoring the given NodeIds and counting notifications."""

    def __init__(self, url: str, node_ids: list[ua.NodeId]) -> None:
        super().__init__(daemon=True)
        self.url, self.node_ids = url, node_ids
        self.received = 0
        self.ready, self.stop = threading.Event(), threading.Event()

    def datachange_notification(self, node: Any, val: Any, data: Any) -> None:
        self.received += 1

    def run(self) -> None:
        asyncio.run(self._run())

    async def _run(self) -> None:
        async with Client(self.url) as client:
            subscription = await client.create_subscription(10, self)
            await subscription.subscribe_data_change([client.get_node(each) for each in self.node_ids])
            self.ready.set()
            await asyncio.get_running_loop().run_in_executor(None, self.stop.wait)


async def run_scenario(item: Any, url: str, name: str, tree: Tree) -> dict[str, Any]:
    """Measure one message type in both renderings, without and with a subscriber on every node."""
    result: dict[str, Any] = {"leaves": len(leaves(tree)), "inner": len(inner_paths(tree))}
    for nested in (False, True):
        rendering = await build(item, name, tree, nested=nested)
        await measure(tree, rendering, WARMUP)
        totals: list[int] = []
        encodes: list[int] = []
        for _ in range(ROUNDS):
            more, encoded, _ = await measure(tree, rendering, MESSAGES_PER_ROUND)
            totals += more
            encodes += encoded
        subscriber = Subscriber(url, [each.ua_node.nodeid for each in rendering.everything])
        subscriber.start()
        await asyncio.get_running_loop().run_in_executor(None, subscriber.ready.wait, 10)
        subscribed, _, wall = await measure(tree, rendering, MESSAGES_PER_ROUND * ROUNDS, yielding=True)
        await asyncio.sleep(0.2)
        subscriber.stop.set()
        await asyncio.get_running_loop().run_in_executor(None, subscriber.join, 5)
        key = "nested" if nested else "flat"
        result[key] = Stats.of(totals)
        result[f"{key}_encode"] = statistics.median(encodes) / 1000
        result[f"{key}_subscribed"] = Stats.of(subscribed)
        result[f"{key}_wall_us"] = wall / 1000
        result[f"{key}_received"] = subscriber.received
        result[f"{key}_writes"] = len(rendering.roots)
    return result


def report(results: dict[str, dict[str, Any]]) -> None:
    print(f"\n{MESSAGES_PER_ROUND * ROUNDS} messages per cell, microseconds per message")
    columns = ("median", "p95", "p99", "encode", "sub p50", "sub p99", "wall")
    print(f"{'message':24}{'render':8}{'writes':>7}" + "".join(f"{each:>9}" for each in columns))
    for name, r in results.items():
        for key in ("flat", "nested"):
            s, w = r[key], r[f"{key}_subscribed"]
            writes = r[f"{key}_writes"]
            print(
                f"{name:24}{key:8}{writes:>7}{s.median:>9.0f}{s.p95:>9.0f}{s.p99:>9.0f}{r[f'{key}_encode']:>9.0f}"
                f"{w.median:>9.0f}{w.p99:>9.0f}{r[f'{key}_wall_us']:>9.0f}"
            )
        ratio = r["nested"].median / r["flat"].median
        print(f"{'':24}ratio nested/flat: median {ratio:.2f}, p99 {r['nested'].p99 / r['flat'].p99:.2f}")
        for label, rate in RATES.items():
            share = {k: r[k].median * rate / 1e6 for k in ("flat", "nested")}
            print(f"{'':24}loop share, {label}: flat {share['flat']:.1%}, nested {share['nested']:.1%}")


async def test_the_cost_of_nested_variables_per_message() -> None:
    """Print the table and fail only if nested rendering of a real message holds the loop for a 100 Hz period."""
    machine = TestMachine(TestComponent(name="Perf"), runtime=FakeRuntime())
    server = await serve(machine)
    bserver = server.ua_server.bserver
    assert bserver is not None
    url = f"opc.tcp://visitor:visitor@127.0.0.1:{bserver.port}/server"
    results: dict[str, dict[str, Any]] = {}
    try:
        for name, tree in MESSAGES.items():
            results[name] = await run_scenario(machine.added[0], url, name.replace("/", "").replace("_", ""), tree)
    finally:
        await server.shutdown()
    report(results)
    for name in ("nav_msgs/Odometry", "sensor_msgs/JointState"):
        assert results[name]["nested"].p99 < 10_000, f"{name}: nested p99 exceeds one 100 Hz period"
        assert results[name]["flat_received"] > 0
        assert results[name]["nested_received"] > 0


FIELDS = 8


async def test_the_cost_of_a_message_with_top_level_fields_per_message() -> None:
    """Print what writing a message whose fields are plain Variables costs (a Skill goal, a Keyed member)."""
    machine = TestMachine(TestComponent(name="Perf"), runtime=FakeRuntime())
    server = await serve(machine)
    item = machine.added[0]
    try:
        names = [f"F{index}" for index in range(FIELDS)]
        addition = NodeAddition(
            binding="Top",
            path="/top",
            container=item.monitoring,
            message_type=fixture_messages.Point,
            plan=tuple(PlanVariable(name=name, source=name, initial=0.0, variant_type=D) for name in names),
        )
        [rendered] = await item.mutate_address_space([addition])
        assert rendered is not None
        totals: list[int] = []
        for version in range(WARMUP + MESSAGES_PER_ROUND * ROUNDS):
            values = dict.fromkeys(names, version + 0.5)
            start = time.perf_counter_ns()
            await rendered.write(values)
            totals.append(time.perf_counter_ns() - start)
    finally:
        await server.shutdown()
    stats = Stats.of(totals[WARMUP:])
    print(f"\n{FIELDS} top-level fields, one message: median {stats.median:.0f} us, p99 {stats.p99:.0f} us")
    assert stats.p99 < 10_000, "a flat message holds the loop for a 100 Hz period"
