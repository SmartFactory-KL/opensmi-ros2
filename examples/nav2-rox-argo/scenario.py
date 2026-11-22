# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The ROX Argo scenario: an OPC UA client operates the simulated robot and writes a report.

Every step goes through OPC UA alone: the script is a client of the Machine that
``opensmi-ros2 serve machine.toml`` serves, and touches neither ROS2 nor Gazebo. As the user
``operator`` it takes the Machine's Lock and then

1. starts the Platform's StartupSkill, which waits for odometry and scans and follows both Nav2
   lifecycle managers, and reads the Identification and the Diagnostics;
2. sends the robot towards station A with the NavigateToPose Skill and sees odometry, the laser
   scan and the Diagnostics change while it moves;
3. watches that drive to its end, with Nav2's feedback in the Skill's Monitoring, then drives to
   station B;
4. starts a drive home and halts it halfway, and sees the robot stop;
5. drives home.

Each step is reported as PASS or FAIL, a failed step does not stop the ones after it, and the report
is written as Markdown. The exit status is 0 only if every step passed. ``run.sh`` runs it in the
example's ``scenario`` container; it also runs anywhere else that reaches the endpoint.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import math
import os
import platform
import sys
import time
import tomllib
from dataclasses import dataclass, field
from datetime import UTC, datetime
from importlib import metadata
from pathlib import Path
from typing import TYPE_CHECKING, Any

from asyncua import ua
from asyncua.client.client import Client
from opensmi.core.nodesets import DiNodeIds, SmartFactoryMachineSetNodeIds

from opensmi.ros2.server.drivers import CHANGE_STATE_TYPE, MANAGE_NODES_TYPE

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from asyncua.common.node import Node

MACHINE = "RoxArgo"
MACHINE_FILE = Path(__file__).with_name("machine.toml")
"""The machine file the Machine is served from; what a step triggers in ROS2 is read from its Bindings."""
SEP = "\N{SINGLE RIGHT-POINTING ANGLE QUOTATION MARK}"
"""Between the browse names of a path in the captions."""
TYPE_KEYS = {
    "monitoring": "message_type",
    "keyed": "message_type",
    "publisher": "message_type",
    "action": "action_type",
    "method": "service_type",
}
"""Where each Binding kind of a machine file names its ROS2 interface type."""
USER = "operator"
"""The user the scenario logs in as; opensmi's default users have their name as password."""
HOME = (0.0, 0.0)
STATION_A = (5.5, 2.2)
STATION_B = (5.5, -2.2)
"""Where the robot stops in front of each station, in the map frame (world.sdf has the stations)."""
ARRIVED = 0.35
"""How close to a goal, in metres, the odometry must be for the drive to count as arrived."""
STILL = 0.02
"""The speed, in m/s, below which the robot counts as standing."""
OBSERVE = 3.0
"""Seconds the monitoring step goes on watching the robot drive."""
DRIVE_TIMEOUT = 180.0
CONNECT_TIMEOUT = 300.0
CLIENT_TIMEOUT = 30.0


def say(line: str) -> None:
    """Print one line of progress."""
    sys.stdout.write(line + "\n")
    sys.stdout.flush()


# -- the report -------------------------------------------------------------------------------------


@dataclass
class Step:
    """One step of the scenario and how it went."""

    name: str
    passed: bool
    detail: str
    started: float = 0.0
    """When the step started, in seconds since the epoch (the recording's captions are timed by it)."""
    ended: float = 0.0
    """When the step ended, in seconds since the epoch."""
    calls: list[tuple[float, str]] = field(default_factory=list)
    """What the step did on the OPC UA side and what that means in ROS2, each with when it began."""


@dataclass
class Report:
    """The steps, in order."""

    steps: list[Step] = field(default_factory=list)
    calls: list[tuple[float, str]] = field(default_factory=list)
    """The calls of the step in progress (see :meth:`call`)."""

    @property
    def passed(self) -> bool:
        """Whether anything ran and everything that ran passed."""
        return bool(self.steps) and all(each.passed for each in self.steps)

    def call(self, text: str) -> None:
        """Record what the step in progress does now, for instance which Skill it starts."""
        self.calls.append((time.time(), text))

    async def step(self, name: str, body: Callable[[], Awaitable[str]]) -> None:
        """Run one step; whatever it raises is its failure, and the scenario goes on."""
        started = time.time()
        self.calls = []
        try:
            detail = await body()
        except Exception as error:
            self.steps.append(Step(name, passed=False, detail=f"{type(error).__name__}: {error}"))
        else:
            self.steps.append(Step(name, passed=True, detail=detail))
        last = self.steps[-1]
        last.started, last.ended, last.calls = started, time.time(), self.calls
        say(f"[{'PASS' if last.passed else 'FAIL'}] {last.name}: {last.detail}")


# REUSE-IgnoreStart
LICENCE_HEADER = (
    "<!--",
    "SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.",
    "",
    "SPDX-License-Identifier: MIT",
    "-->",
)
# REUSE-IgnoreEnd


def version(distribution: str) -> str:
    """Return the installed version of a distribution, or say that it is missing."""
    try:
        return metadata.version(distribution)
    except metadata.PackageNotFoundError:
        return "(not installed)"


def render(report: Report, endpoint: str, started: datetime) -> str:
    """Return the report as Markdown."""
    upstream = ", ".join(
        f"{name} `{os.environ.get(variable, 'unknown')[:12]}`"
        for name, variable in (
            ("rox", "ROX_COMMIT"),
            ("neo_nav2_bringup", "NEO_NAV2_BRINGUP_COMMIT"),
            ("neo_localization2", "NEO_LOCALIZATION2_COMMIT"),
            ("neo_local_planner2", "NEO_LOCAL_PLANNER2_COMMIT"),
            ("neo_gz_worlds", "NEO_GZ_WORLDS_COMMIT"),
        )
    )
    verdict = "passed" if report.passed else f"NOT PASSED ({sum(not each.passed for each in report.steps)} failed)"
    lines = [
        *LICENCE_HEADER,
        "",
        f"# ROX Argo example: {verdict}",
        "",
        "Written by `examples/nav2-rox-argo/scenario.py`. See the example's README.",
        "",
        "| | |",
        "| --- | --- |",
        f"| opensmi-ros2 commit | `{os.environ.get('OPENSMI_ROS_COMMIT', 'unknown')}` |",
        f"| OpenSMI-ROS2 / OpenSMI-Server / OpenSMI-Core / asyncua | {version('OpenSMI-ROS2')} / "
        f"{version('OpenSMI-Server')} / {version('OpenSMI-Core')} / {version('asyncua')} |",
        f"| Neobotix sources | {upstream} |",
        f"| ROS distribution | {os.environ.get('ROS_DISTRO', 'unknown')}, "
        f"{os.environ.get('RMW_IMPLEMENTATION', 'default middleware')}, "
        f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')} |",
        f"| endpoint | `{endpoint}`, as `{USER}` |",
        f"| Python | {platform.python_version()} on {platform.platform()} |",
        f"| run | {started.strftime('%Y-%m-%d %H:%M:%S')} UTC |",
        "",
        "## Steps",
        "",
        "| Step | Result | What was seen |",
        "| --- | --- | --- |",
    ]
    lines.extend(
        f"| {each.name} | {'pass' if each.passed else 'FAIL'} | {each.detail.replace('|', '/')} |"
        for each in report.steps
    )
    lines.append("")
    return "\n".join(lines)


# -- a client's way around the address space --------------------------------------------------------


async def until(read: Callable[[], Awaitable[Any]], within: float = 20.0) -> Any:
    """Poll until ``read`` answers something truthy, and return it."""
    deadline = time.monotonic() + within
    seen: Any = None
    while True:
        try:
            seen = await read()
        except (ua.UaError, OSError, TimeoutError, LookupError):
            seen = None
        if seen:
            return seen
        if time.monotonic() > deadline:
            message = f"still {seen!r} after {within:.0f} s"
            raise TimeoutError(message)
        await asyncio.sleep(0.2)


async def below(node: Node, *names: str) -> Node:
    """Follow browse names one level at a time."""
    for name in names:
        for child in await node.get_children():
            if (await child.read_browse_name()).Name == name:
                node = child
                break
        else:
            have = [(await each.read_browse_name()).Name for each in await node.get_children()]
            message = f"{name!r} is not below {node}; there are {have}"
            raise KeyError(message)
    return node


async def find(node: Node, name: str, depth: int = 6) -> Node:
    """Return the first node below ``node`` with a browse name, searching breadth first."""
    level = [node]
    for _ in range(depth):
        following: list[Node] = []
        for each in level:
            for child in await each.get_children():
                if (await child.read_browse_name()).Name == name:
                    return child
                following.append(child)
        level = following
    message = f"no node named {name!r} below {node}"
    raise KeyError(message)


async def write(node: Node, value: object) -> None:
    """Write a value with the variant type the server declares for the node."""
    declared = await node.read_data_value()
    assert declared.Value is not None
    await node.write_value(ua.Variant(value, declared.Value.VariantType))


async def read(node: Node, *path: str) -> Any:
    """Read the variable at a path of browse names below a node."""
    return await (await below(node, *path)).read_value()


@dataclass(frozen=True)
class Bindings:
    """The ROS2 side of the Machine's Bindings, as the machine file declares it, by ``Component/Binding``."""

    tables: dict[str, dict[str, Any]]

    @classmethod
    def load(cls, path: Path = MACHINE_FILE) -> Bindings:
        """Read every Binding table of a machine file."""
        data = tomllib.loads(path.read_text(encoding="utf-8"))
        return cls(
            {
                f"{component['name']}/{each['name']}": {**each, "kind": kind}
                for component in data.get("component", [])
                for kind in (*TYPE_KEYS, "lifecycle")
                for each in component.get(kind, [])
            }
        )

    def interface(self, binding: str) -> tuple[str, str]:
        """Return a Binding's ROS2 name and interface type (for a lifecycle Binding, its Driver's service)."""
        table = self.tables[binding]
        if table["kind"] == "lifecycle":
            kind = MANAGE_NODES_TYPE if table.get("driver") == "nav2" else CHANGE_STATE_TYPE
        else:
            kind = table[TYPE_KEYS[table["kind"]]]
        return str(table["path"]), str(kind)

    def ros2(self, binding: str, *, typed: bool = True) -> str:
        """Return a Binding's ROS2 name, and its interface type unless told not to."""
        path, kind = self.interface(binding)
        return f"{path} ({kind})" if typed else path

    def group(self, bindings: list[str]) -> str:
        """Return several Bindings' ROS2 names, each type named once after the names that have it."""
        kinds: dict[str, list[str]] = {}
        for each in bindings:
            path, kind = self.interface(each)
            kinds.setdefault(kind, []).append(path)
        return ", ".join(f"{', '.join(paths)} ({kind})" for kind, paths in kinds.items())

    def of(self, component: str, kind: str, *, required: bool = False) -> list[str]:
        """Return the names (``Component/Binding``) of one Component's Bindings of a kind."""
        return [
            name
            for name, table in self.tables.items()
            if name.startswith(f"{component}/") and table["kind"] == kind and (table.get("required") or not required)
        ]


# -- the scenario -----------------------------------------------------------------------------------


@dataclass
class Snapshot:
    """What the Platform's Monitoring says at one moment."""

    x: float
    y: float
    speed: float
    scan_stamp: float
    drive: str
    """The message of the drive's diagnostic entry: ``moving`` or ``standing``."""


@dataclass
class Drive:
    """One NavigateToPose goal as the client saw it."""

    goal: tuple[float, float]
    start: Snapshot
    feedback: dict[str, list[float]] = field(default_factory=dict)
    """Every value of the feedback variables seen while the Skill ran."""
    state: str = ""


@dataclass
class Operator:
    """The operator's session: the Platform, its Skills, and the drive in progress."""

    client: Client
    machine: Node
    platform: Node
    report: Report
    bindings: Bindings
    current: Drive | None = None

    async def path(self, node: Node) -> str:
        """Return a node's browse path from its Machine or Component on, as a client finds it."""
        names: list[str] = []
        while node is not None:
            name = (await node.read_browse_name()).Name
            if name in ("Components", "Machines"):
                break
            names.append(name)
            parent = await node.get_parent()
            if parent is None:
                break
            node = parent
        return f" {SEP} ".join(reversed(names))

    async def index(self, nodeids: type[Any]) -> int:
        """Return the namespace index a nodeset was imported at."""
        return int(await self.client.get_namespace_index(nodeids.URI))

    async def skill(self, name: str) -> Node:
        """Return one of the Platform's Skills."""
        return await below(self.platform, "SkillSet", name)

    async def press(self, skill: Node, verb: str) -> None:
        """Press Start, Halt or Reset on a Skill's state machine."""
        state_machine = await (await find(skill, verb)).get_parent()
        assert state_machine is not None
        await state_machine.call_method(f"{await self.index(SmartFactoryMachineSetNodeIds)}:{verb}")

    @staticmethod
    async def state(skill: Node) -> str:
        """Return the name of the state a Skill is in."""
        value = await (await find(skill, "CurrentState")).read_value()
        return str(getattr(value, "Text", value))

    async def in_state(self, skill: Node, *wanted: str) -> str | None:
        """Return the Skill's state if it is one of those wanted, else nothing."""
        seen = await self.state(skill)
        return seen if seen in wanted else None

    async def start(self, skill: Node) -> None:
        """Reset a Skill that is not ready, then Start it."""
        if await self.state(skill) != "Ready":
            await self.press(skill, "Reset")
            await until(lambda: self.in_state(skill, "Ready"))
        await self.press(skill, "Start")

    async def snapshot(self) -> Snapshot:
        """Read odometry, the scan's stamp and the drive's diagnostic message."""
        odom = await below(self.platform, "Monitoring", "Odom")
        position = await below(odom, "Pose", "Pose", "Position")
        linear = await below(odom, "Twist", "Twist", "Linear")
        speed = math.hypot(float(await read(linear, "X")), float(await read(linear, "Y")))
        stamp = await below(self.platform, "Monitoring", "Scan", "Header", "Stamp")
        return Snapshot(
            x=float(await read(position, "X")),
            y=float(await read(position, "Y")),
            speed=speed,
            scan_stamp=float(await read(stamp, "Sec")) + float(await read(stamp, "Nanosec")) * 1e-9,
            drive=(await self.diagnostics()).get("rox_argo_sim: drive", ("", ""))[1],
        )

    async def diagnostics(self) -> dict[str, tuple[int, str]]:
        """Return each diagnostic member's level and message, by its raw key's name."""
        group = await below(self.platform, "Monitoring", "Diagnostics")
        members: dict[str, tuple[int, str]] = {}
        for member in await group.get_children():
            if await member.read_node_class() != ua.NodeClass.Object:
                continue
            name = str(await read(member, "Name"))
            members[name] = (int(await read(member, "Level")), str(await read(member, "Message")))
        return members

    # -- the steps --------------------------------------------------------------------------------

    async def startup(self) -> str:
        """Start the StartupSkill and see it reach Running, with every lifecycle node it follows."""
        skill = await self.skill("StartupSkill")
        before = await self.state(skill)
        required = [
            self.bindings.ros2(each, typed=False) for each in self.bindings.of("Platform", "monitoring", required=True)
        ]
        followed = self.bindings.group(self.bindings.of("Platform", "lifecycle"))
        self.report.call(
            f"OPC UA: {await self.path(skill)} — Start()  →  ROS2: waits for {', '.join(required)}; follows {followed}"
        )
        if before != "Running":
            await self.start(skill)
        reached = await until(lambda: self.in_state(skill, "Running", "Halted"), within=150.0)
        if reached != "Running":
            message = f"the StartupSkill went from {before} to {reached}"
            raise AssertionError(message)
        states: dict[str, str] = {}

        async def walk(node: Node, path: str) -> None:
            for each in await node.get_children():
                name = (await each.read_browse_name()).Name
                if name == "State":
                    states[path] = str(await each.read_value())
                elif await each.read_node_class() == ua.NodeClass.Object:
                    await walk(each, f"{path}/{name}" if path else name)

        await walk(await find(skill, "Monitoring"), "")
        return f"StartupSkill {before} -> {reached}; lifecycle nodes it follows: {states}"

    async def identification(self) -> str:
        """Read the Machine's and the Platform's Identification, and the Diagnostics members."""
        machine = await below(self.machine, "Identification")
        platform_ = await below(self.platform, "Identification")
        group = await below(self.platform, "Monitoring", "Diagnostics")
        self.report.call(
            f"OPC UA read: {await self.path(machine)}, {await self.path(platform_)}, {await self.path(group)}  ←  "
            f"ROS2: {self.bindings.ros2('Platform/Diagnostics')}"
        )
        manufacturer = await read(machine, "Manufacturer")
        serial = await read(machine, "SerialNumber")
        component_serial = await read(platform_, "SerialNumber")
        members = await until(self.diagnostics)
        worst = await read(self.platform, "Monitoring", "Diagnostics", "WorstLevel")
        return (
            f"Machine: Manufacturer {getattr(manufacturer, 'Text', manufacturer)!r}, SerialNumber {serial!r}; "
            f"Platform: SerialNumber {component_serial!r}; Diagnostics: {len(members)} members "
            f"{ {name: f'level {level}, {text!r}' for name, (level, text) in members.items()} }, WorstLevel {worst}"
        )

    async def send(self, goal: tuple[float, float]) -> Drive:
        """Write a goal into the NavigateToPose Skill's Parameters and start it."""
        skill = await self.skill("NavigateToPose")
        if await self.state(skill) == "Running":
            message = "NavigateToPose is still running from an earlier step"
            raise AssertionError(message)
        pose = await below(skill, "SkillExecution", "ParameterSet", "Pose")
        await write(await below(pose, "Header", "FrameId"), "map")
        position = await below(pose, "Pose", "Position")
        await write(await below(position, "X"), goal[0])
        await write(await below(position, "Y"), goal[1])
        orientation = await below(pose, "Pose", "Orientation")
        for axis, value in (("X", 0.0), ("Y", 0.0), ("Z", 0.0), ("W", 1.0)):
            await write(await below(orientation, axis), value)
        self.current = Drive(goal, await self.snapshot())
        self.report.call(
            f"OPC UA: {await self.path(skill)} — Parameters Pose = ({goal[0]:g}, {goal[1]:g}), Start()  →  "
            f"ROS2: goal to {self.bindings.ros2('Platform/NavigateToPose')}"
        )
        await self.start(skill)
        await until(lambda: self.in_state(skill, "Running", "Completed", "Halted"))
        return self.current

    async def follow(self, drive: Drive, *, until_moving: bool = False) -> Snapshot:
        """Watch the running drive, recording its feedback, until it ends or (if asked) until it moves.

        :raises TimeoutError: When the drive neither ends nor (if asked) moves within the timeout.
        """
        skill = await self.skill("NavigateToPose")
        monitoring = await find(skill, "Monitoring")
        distance = await below(monitoring, "DistanceRemaining")
        recoveries = await below(monitoring, "NumberOfRecoveries")
        deadline = time.monotonic() + DRIVE_TIMEOUT
        while True:
            drive.state = await self.state(skill)
            drive.feedback.setdefault("DistanceRemaining", []).append(float(await distance.read_value()))
            drive.feedback.setdefault("NumberOfRecoveries", []).append(float(await recoveries.read_value()))
            now = await self.snapshot()
            if drive.state in ("Completed", "Halted"):
                return now
            if until_moving and now.speed > STILL and math.dist((now.x, now.y), (drive.start.x, drive.start.y)) > 0.3:
                return now
            if time.monotonic() > deadline:
                message = f"NavigateToPose was still {drive.state} after {DRIVE_TIMEOUT:.0f} s"
                raise TimeoutError(message)
            await asyncio.sleep(0.25)

    async def monitoring_while_moving(self) -> str:
        """Send the robot to station A and see odometry, the scan and the Diagnostics change on the way."""
        drive = await self.send(STATION_A)
        before = drive.start
        if before.speed > STILL or before.drive != "standing":
            message = f"the robot was not standing before the drive: speed {before.speed:.3f}, drive {before.drive!r}"
            raise AssertionError(message)
        moving = await self.follow(drive, until_moving=True)
        if drive.state != "Running":
            message = f"NavigateToPose ended {drive.state} before the robot was seen moving"
            raise AssertionError(message)
        monitoring = await below(self.platform, "Monitoring")
        variables = ("Odom", "Scan", "Diagnostics")
        self.report.call(
            f"OPC UA read: {await self.path(monitoring)} {SEP} {', '.join(variables)}  ←  "
            f"ROS2: {', '.join(self.bindings.ros2(f'Platform/{each}') for each in variables)}"
        )

        async def diagnosed() -> Snapshot | None:
            now = await self.snapshot()
            return now if now.drive == "moving" and now.scan_stamp > before.scan_stamp else None

        during = await until(diagnosed, within=10.0)
        # Keep watching for a few seconds: the odometry has to go on changing, not just once.
        await asyncio.sleep(OBSERVE)
        later = await self.snapshot()
        if math.dist((later.x, later.y), (during.x, during.y)) < 0.2 or later.scan_stamp <= during.scan_stamp:
            message = f"odometry or scan stood still while the robot drove: {during} -> {later}"
            raise AssertionError(message)
        return (
            f"Odom: ({before.x:.2f}, {before.y:.2f}) at {before.speed:.2f} m/s -> ({moving.x:.2f}, {moving.y:.2f}) "
            f"at {moving.speed:.2f} m/s -> ({later.x:.2f}, {later.y:.2f}) {OBSERVE:g} s later; Scan stamp "
            f"{before.scan_stamp:.1f} s -> {later.scan_stamp:.1f} s; Diagnostics drive {before.drive!r} -> "
            f"{during.drive!r}"
        )

    async def arrive(self, drive: Drive | None, name: str) -> str:
        """Follow a drive to its end and check it completed at its goal, with feedback on the way."""
        if drive is None:
            message = "no drive was started"
            raise AssertionError(message)
        skill = await self.skill("NavigateToPose")
        self.report.call(
            f"OPC UA read: {await self.path(await find(skill, 'Monitoring'))} {SEP} DistanceRemaining  ←  "
            f"ROS2: feedback of {self.bindings.ros2('Platform/NavigateToPose')}"
        )
        end = await self.follow(drive)
        off = math.dist((end.x, end.y), drive.goal)
        distances = drive.feedback.get("DistanceRemaining", [])
        if drive.state != "Completed":
            message = f"NavigateToPose ended {drive.state} at ({end.x:.2f}, {end.y:.2f}), {off:.2f} m from {name}"
            raise AssertionError(message)
        if off > ARRIVED:
            message = f"NavigateToPose completed {off:.2f} m from {name}, at ({end.x:.2f}, {end.y:.2f})"
            raise AssertionError(message)
        if len(set(distances)) < 3 or max(distances) <= 0.0:
            message = f"the feedback DistanceRemaining hardly changed: {sorted(set(distances))[:5]}"
            raise AssertionError(message)
        nonzero = [each for each in distances if each > 0.0]
        return (
            f"Completed at ({end.x:.2f}, {end.y:.2f}), {off:.2f} m from {name} {drive.goal}; feedback "
            f"DistanceRemaining {max(nonzero):.2f} m -> {nonzero[-1]:.2f} m in {len(set(distances))} distinct "
            f"values, NumberOfRecoveries {max(drive.feedback.get('NumberOfRecoveries', [0.0])):.0f}"
        )

    async def to_station_a(self) -> str:
        """Follow the drive to station A to its end."""
        if self.current is None or self.current.goal != STATION_A:
            await self.send(STATION_A)
        return await self.arrive(self.current, "station A")

    async def to_station_b(self) -> str:
        """Drive to station B."""
        return await self.arrive(await self.send(STATION_B), "station B")

    async def halt(self) -> str:
        """Start a drive home, halt it once the robot moves, and see the robot stop short of home."""
        drive = await self.send(HOME)
        moving = await self.follow(drive, until_moving=True)
        if drive.state != "Running":
            message = f"NavigateToPose ended {drive.state} before it could be halted"
            raise AssertionError(message)
        skill = await self.skill("NavigateToPose")
        self.report.call(
            f"OPC UA: {await self.path(skill)} — Halt()  →  "
            f"ROS2: cancels the goal on {self.bindings.ros2('Platform/NavigateToPose')}"
        )
        await self.press(skill, "Halt")
        halted = await until(lambda: self.in_state(skill, "Halted", "Completed"), within=15.0)

        async def stopped() -> tuple[Snapshot, Snapshot] | None:
            first = await self.snapshot()
            await asyncio.sleep(2.0)
            second = await self.snapshot()
            still = (
                max(first.speed, second.speed) < STILL and math.dist((first.x, first.y), (second.x, second.y)) < 0.02
            )
            return (first, second) if still else None

        _, rest = await until(stopped, within=20.0)
        left = math.dist((rest.x, rest.y), HOME)
        if halted != "Halted" or left < 1.0:
            message = f"the Skill ended {halted} with the robot {left:.2f} m from home: the drive was not cut short"
            raise AssertionError(message)
        return (
            f"moving at {moving.speed:.2f} m/s at ({moving.x:.2f}, {moving.y:.2f}); Halt -> {halted}; the robot "
            f"stood still at ({rest.x:.2f}, {rest.y:.2f}), {left:.2f} m short of home; Diagnostics drive {rest.drive!r}"
        )

    async def home(self) -> str:
        """Drive home."""
        return await self.arrive(await self.send(HOME), "home")

    async def run(self, report: Report) -> None:
        """Take the Lock, run the steps in order, and give the Lock back."""
        lock = await find(self.machine, "Lock")
        await lock.call_method(f"{await self.index(DiNodeIds)}:InitLock", "rox argo scenario")
        try:
            await report.step("Startup: the StartupSkill brings the Platform up", self.startup)
            await report.step("Startup: Identification and Diagnostics are readable", self.identification)
            await report.step(
                "Monitoring: Odom, Scan and Diagnostics change while the robot moves", self.monitoring_while_moving
            )
            await report.step("NavigateToPose Skill to station A, with feedback", self.to_station_a)
            await report.step("NavigateToPose Skill to station B, with feedback", self.to_station_b)
            await report.step("Halt a running NavigateToPose: the robot stops", self.halt)
            await report.step("Return home", self.home)
        finally:
            with contextlib.suppress(ua.UaError):
                await lock.call_method(f"{await self.index(DiNodeIds)}:ExitLock")


async def connect(endpoint: str) -> Client:
    """Return a connected client once the Machine is browsable, retrying while the server comes up."""
    deadline = time.monotonic() + CONNECT_TIMEOUT
    while True:
        client = Client(url=endpoint, timeout=CLIENT_TIMEOUT)
        client.set_user(USER)
        client.set_password(USER)
        try:
            await client.connect()
            await below(client.nodes.objects, "Machines", MACHINE, "Components", "Platform")
        except Exception as not_yet:
            with contextlib.suppress(Exception):
                await client.disconnect()
            if time.monotonic() > deadline:
                message = f"the Machine was not served at {endpoint} within {CONNECT_TIMEOUT:.0f} s (last: {not_yet!r})"
                raise TimeoutError(message) from None
            await asyncio.sleep(2.0)
        else:
            return client


async def scenario(endpoint: str) -> Report:
    """Connect, run every step, and return the report."""
    report = Report()
    try:
        client = await connect(endpoint)
    except TimeoutError as error:
        report.steps.append(Step("connect to the Machine", passed=False, detail=str(error)))
        return report
    say(f"connected to {endpoint} as {USER}")
    try:
        machine = await below(client.nodes.objects, "Machines", MACHINE)
        platform_ = await below(machine, "Components", "Platform")
        await Operator(client, machine, platform_, report, Bindings.load()).run(report)
    finally:
        with contextlib.suppress(Exception):
            await client.disconnect()
    return report


def main() -> None:
    """Run the scenario, write the report, and exit non-zero unless every step passed."""
    parser = argparse.ArgumentParser(description="Run the ROX Argo scenario against the served Machine.")
    parser.add_argument("--endpoint", default="opc.tcp://localhost:4840/server", help="the Machine's OPC UA endpoint")
    parser.add_argument("--output", type=Path, default=Path("report.md"), help="the report file")
    parser.add_argument("--timeline", type=Path, help="also write each step's name, result, start and end as JSON")
    arguments = parser.parse_args()
    started = datetime.now(UTC)
    report = asyncio.run(scenario(arguments.endpoint))
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(render(report, arguments.endpoint, started), encoding="utf-8", newline="\n")
    say(f"report: {arguments.output}")
    if arguments.timeline:
        steps = [
            {
                "name": each.name,
                "passed": each.passed,
                "started": each.started,
                "ended": each.ended,
                "calls": [{"at": at, "text": text} for at, text in each.calls],
            }
            for each in report.steps
        ]
        arguments.timeline.write_text(json.dumps(steps, indent=2) + "\n", encoding="utf-8", newline="\n")
    sys.exit(0 if report.passed else 1)


if __name__ == "__main__":
    main()
