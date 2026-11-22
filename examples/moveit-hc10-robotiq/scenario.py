# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The pick and place of the HC10 example, driven through OPC UA only, with a report.

An OPC UA client of the Machine that ``opensmi-ros2 serve machine.toml`` serves next to the
simulation. It never touches ROS2: every step is a Skill started, a Method called or a variable read
on the server, as any OPC UA client would do it. Run it through ``run.sh`` (or the compose service
``scenario``) once the simulation and the Machine are up; it waits for both.

The steps, each reported as PASS or FAIL with what was seen:

1. Startup: both StartupSkills reach Running, which they do only once the joint states, MoveIt's
   ``move_action`` and the gripper's action are on the graph, and ListControllers shows the three
   controllers active; the identification of the Machine and its Components and the Diagnostics
   built from ``/diagnostics`` are readable.
2. Monitoring: the joint states of the arm and the gripper arrive and move on with simulated time.
3. Pick and place, each motion one start of the Arm's ``Move`` Skill (MoveIt's MoveGroup action with
   Pilz's PTP or LIN planner) and each gripper motion one start of the Gripper's ``GripperCommand``
   Skill: pre-grasp pose, open, approach, close on the cube, lift, carry to above the place
   position, lower, open, retreat, home.
4. The cube lies at the place position, as the Workpiece Component reads it from Gazebo.
5. Halt: a slow motion is started and halted from OPC UA; the Skill ends Halted and the arm stands
   still short of its goal. Then the arm goes home again.

Environment: ``OPCUA_URL`` (default ``opc.tcp://machine:4840/server``) is the endpoint without user,
``REPORT`` (default ``reports/report.md``) the report file, and ``TIMELINE``, if set, a JSON file
with each step's start and end, which ``record.sh`` captions its video by. The exit status is 0 only if every step
passed.
"""

from __future__ import annotations

import asyncio
import dataclasses
import json
import math
import os
import sys
import time
import tomllib
import typing
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast

from asyncua import ua
from asyncua.client.client import Client
from opensmi.core.nodesets import DiNodeIds, SmartFactoryMachineSetNodeIds, SmartFactorySkillSetNodeIds

from opensmi.ros2.server.structures import structure_class

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from asyncua.common.node import Node

MACHINE = "PickAndPlaceCell"
MACHINE_FILE = Path(__file__).with_name("machine.toml")
"""The machine file the server serves: where the ROS2 name and type behind each Binding come from."""
ARM_JOINTS = ("joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t")
KNUCKLE = "robotiq_85_left_knuckle_joint"
HOME = (0.0, 0.0, 1.5708, 0.0, -1.5708, 0.0)
"""The arm's home pose, joint by joint: the SRDF's ``home`` group state."""

# The cell, in the world frame: the table top is at z = 0.7, the cube is 5 cm.
PICK = (0.55, -0.30)
PLACE = (0.55, 0.30)
CUBE_Z = 0.725
"""Where the cube's centre is when it lies on the table."""
GRASP_Z = 0.73
"""Where the point between the fingertips (``tcp``) is when the fingers close on the cube."""
ABOVE = 0.15
"""How far above the grasp the pre-grasp and carry poses are."""
DOWN = (0.7071068, 0.7071068, 0.0, 0.0)
"""The tool's orientation as a quaternion (x, y, z, w): pointing straight down, fingers closing along y.

Along y is the way the cube is carried from the pick to the place position, so the pads push it
rather than holding it by friction alone."""

OPEN = 0.0
CLOSED = 0.8
"""The knuckle angle of the 2F-85 when open and when fully closed, in radians."""

HALT_SPEED = 0.1
"""The speed of the motion the Halt step stops: slow, so it is halted well short of its goal."""
HALT_AFTER = 0.3
"""How far, in radians, joint 1 has turned when Halt is pressed: far enough to see the arm move."""
PIPELINE = "pilz_industrial_motion_planner"
SUCCESS = 1
"""moveit_msgs/MoveItErrorCodes SUCCESS."""
PLACE_TOLERANCE = 0.03
"""How far, in metres, the cube may end from the place position."""
CLIENT_TIMEOUT = 60.0


# -- the report -------------------------------------------------------------------------------------

# REUSE-IgnoreStart
LICENCE_HEADER = (
    "<!--",
    "SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.",
    "",
    "SPDX-License-Identifier: MIT",
    "-->",
)
"""The licence header the report carries, so it passes ``reuse lint`` like any other file."""
# REUSE-IgnoreEnd


@dataclass
class Step:
    """One line of the scenario and how it went."""

    name: str
    passed: bool
    detail: str


@dataclass
class Call:
    """One OPC UA node a step used, what it did there, and the ROS2 interface behind it."""

    path: tuple[str, ...]
    """The node's browse path from its Component down, as the server names it."""
    actions: list[str]
    """What the step did there, in order: ``Parameters written``, ``Start()``, ``Halt()``, ``read``."""
    ros2: str
    """The ROS2 name and type of the Binding behind the node, from the machine file; empty if none."""

    @property
    def is_read(self) -> bool:
        """Whether the step only read the node."""
        return self.actions == ["read"]


@dataclass
class Report:
    """What ran, in order."""

    steps: list[Step] = field(default_factory=list)
    timeline: list[dict[str, Any]] = field(default_factory=list)
    """When each step started and ended, in seconds since the epoch, and the OPC UA nodes it used and
    the ROS2 interfaces behind them: what record.sh captions a video by."""
    calls: list[Call] = field(default_factory=list)
    """The nodes the running step has used so far; the Operator adds to it."""

    def interfaces(self) -> dict[str, str]:
        """Return what the step that just ran did, for a caption: its Skill or Method calls, else its reads.

        ``opcua`` names the nodes and what was done there, ``ros2`` the interfaces behind them, and
        ``direction`` is ``to`` when OPC UA drove ROS2 and ``from`` when it only read what ROS2 sent.
        """
        driven = [each for each in self.calls if not each.is_read]
        shown = driven or self.calls
        opcua = "  |  ".join(
            " > ".join(each.path) + ("" if each.is_read else f": {', '.join(each.actions)}") for each in shown
        )
        ros2 = "  |  ".join(dict.fromkeys(each.ros2 for each in shown if each.ros2))
        return {"opcua": opcua, "ros2": ros2, "direction": "to" if driven else "from"}

    @property
    def passed(self) -> bool:
        """Whether anything ran and everything that ran passed."""
        return bool(self.steps) and all(each.passed for each in self.steps)

    async def step(self, name: str, body: Callable[[], Awaitable[str]]) -> bool:
        """Run one step and record it; whatever it raises is its failure. Return whether it passed."""
        started = time.monotonic()
        wall = time.time()
        self.calls.clear()
        try:
            detail = await body()
        except Exception as error:  # a failed step is reported, and the report goes on
            self.steps.append(Step(name, passed=False, detail=f"{type(error).__name__}: {error}"))
        else:
            self.steps.append(Step(name, passed=True, detail=detail))
        last = self.steps[-1]
        took = time.monotonic() - started
        self.timeline.append(
            {"name": name, "start": wall, "end": wall + took, "passed": last.passed, **self.interfaces()}
        )
        print(f"[{'PASS' if last.passed else 'FAIL'}] {name} ({took:.1f} s): {last.detail}", flush=True)  # noqa: T201
        return last.passed

    def render(self, url: str, started: datetime) -> str:
        """Return the report as Markdown."""
        verdict = "passed" if self.passed else f"FAILED ({sum(not each.passed for each in self.steps)} steps)"
        lines = [
            *LICENCE_HEADER,
            "",
            f"# HC10 pick and place through OPC UA: {verdict}",
            "",
            f"Written by `examples/moveit-hc10-robotiq/scenario.py` against `{url}`, "
            f"{started.strftime('%Y-%m-%d %H:%M:%S')} UTC.",
            "",
            "| Step | Result | What was seen |",
            "| --- | --- | --- |",
        ]
        lines.extend(
            f"| {each.name} | {'PASS' if each.passed else 'FAIL'} | {each.detail.replace('|', '/')} |"
            for each in self.steps
        )
        lines.append("")
        return "\n".join(lines)


# -- a client's way around the address space --------------------------------------------------------


async def until(read: Callable[[], Awaitable[Any]], within: float = 30.0) -> Any:
    """Poll until ``read`` answers something truthy, and return it.

    :raises TimeoutError: When it has not within the given seconds, naming the last answer.
    """
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


def as_map(value: object) -> Any:
    """Return a decoded structure as nested maps of its fields, and lists of them as lists."""
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return {each.name: as_map(getattr(value, each.name)) for each in dataclasses.fields(value)}
    if isinstance(value, list):
        return [as_map(each) for each in cast("list[object]", value)]
    return value


def _element(annotation: Any) -> tuple[Any, bool]:
    """Return the type a field annotation names and whether the field is a list of it."""
    if typing.get_origin(annotation) is list:
        return typing.get_args(annotation)[0], True
    return annotation, False


def instance(cls: Any, values: dict[str, Any]) -> Any:
    """Return an instance of a structure class, its fields from a map by ROS2 field name.

    A field the map leaves out gets the empty value of its type (zero, an empty string or list, or a
    structure of such), so a goal names only what it means.
    """
    built: dict[str, Any] = {}
    for each in dataclasses.fields(cls):
        kind, array = _element(each.type)
        nested = dataclasses.is_dataclass(kind)
        if each.name in values:
            given = values[each.name]
            if array and nested:
                built[each.name] = [instance(kind, item) for item in given]
            elif nested:
                built[each.name] = instance(kind, given)
            else:
                built[each.name] = given
        elif array:
            built[each.name] = []
        elif nested:
            built[each.name] = instance(kind, {})
        else:
            built[each.name] = False if kind is ua.Boolean else kind()
    return cls(**built)


# -- the goals --------------------------------------------------------------------------------------

HEADER = {"frame_id": "world"}


def pose_goal(x: float, y: float, z: float, orientation: tuple[float, float, float, float] = DOWN) -> dict[str, Any]:
    """Return the moveit_msgs/Constraints of a pose of the ``tcp`` link in the world frame.

    One position constraint (a sphere of half a millimetre around the point) and one orientation
    constraint: the form Pilz's planners take a Cartesian goal in.
    """
    qx, qy, qz, qw = orientation
    return {
        "position_constraints": [
            {
                "header": HEADER,
                "link_name": "tcp",
                "constraint_region": {
                    "primitives": [{"type": 2, "dimensions": [0.0005]}],  # SolidPrimitive.SPHERE
                    "primitive_poses": [{"position": {"x": x, "y": y, "z": z}, "orientation": {"w": 1.0}}],
                },
                "weight": 1.0,
            }
        ],
        "orientation_constraints": [
            {
                "header": HEADER,
                "link_name": "tcp",
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
                "absolute_x_axis_tolerance": 0.001,
                "absolute_y_axis_tolerance": 0.001,
                "absolute_z_axis_tolerance": 0.001,
                "weight": 1.0,
            }
        ],
    }


def joint_goal(positions: tuple[float, ...]) -> dict[str, Any]:
    """Return the moveit_msgs/Constraints of joint positions of the arm."""
    return {
        "joint_constraints": [
            {"joint_name": name, "position": value, "tolerance_above": 0.001, "tolerance_below": 0.001, "weight": 1.0}
            for name, value in zip(ARM_JOINTS, positions, strict=True)
        ]
    }


# -- the operator -----------------------------------------------------------------------------------


@dataclass
class Operator:
    """An operator's session on the served Machine: the client, the Components and the steps."""

    client: Client
    machine: Node
    arm: Node
    gripper: Node
    workpiece: Node
    calls: list[Call] = field(default_factory=list)
    """Where the nodes the running step uses are noted; the Report's list, which it clears per step."""
    interfaces: dict[tuple[str, str], str] = field(default_factory=lambda: binding_interfaces(MACHINE_FILE))
    """The ROS2 name and type behind each Binding, by Component and Binding name, from the machine file."""
    paths: dict[ua.NodeId, tuple[str, ...]] = field(default_factory=dict)

    async def path(self, node: Node) -> tuple[str, ...]:
        """Return a node's browse path from its Component down, as the server names it."""
        if node.nodeid not in self.paths:
            names: list[str] = []
            here: Node | None = node
            while here is not None:
                name = (await here.read_browse_name()).Name
                if name == "Components":
                    break
                names.append(name)
                here = await here.get_parent()
            self.paths[node.nodeid] = tuple(reversed(names))
        return self.paths[node.nodeid]

    async def note(self, node: Node, action: str) -> None:
        """Note that the running step did something at a node, with the ROS2 interface of its Binding.

        The Binding is the node's own name for a Skill or Method, and the variable's for Monitoring.
        """
        path = await self.path(node)
        for each in self.calls:
            if each.path == path:
                if each.actions[-1] != action:
                    each.actions.append(action)
                return
        key = (*path, "", "", "")[:3]  # the Component, its container, the Binding
        self.calls.append(Call(path, [action], self.interfaces.get((key[0], key[2]), "")))

    async def index(self, nodeids: type[Any]) -> int:
        """Return the namespace index a nodeset was imported at."""
        return int(await self.client.get_namespace_index(nodeids.URI))

    async def structure(self, datatype: ua.NodeId) -> Any:
        """Return the class of a structure DataType, made from the definition the server publishes."""
        if datatype not in ua.extension_objects_by_datatype:
            definition = cast(
                "ua.StructureDefinition", await self.client.get_node(datatype).read_data_type_definition()
            )
            fields = []
            for each in definition.Fields:
                nested = each.DataType.NamespaceIndex != 0
                element = (
                    await self.structure(each.DataType)
                    if nested
                    else getattr(ua, ua.VariantType(each.DataType.Identifier).name)
                )
                array = each.ValueRank == ua.ValueRank.OneDimension
                length = int(each.ArrayDimensions[0]) if array and each.ArrayDimensions else 0
                fields.append((each.Name, list[element] if array else element, length))
            name = str(datatype.Identifier)
            ua.register_extension_object(
                name, definition.DefaultEncodingId, structure_class(name, tuple(fields)), datatype
            )
        return ua.extension_objects_by_datatype[datatype]

    # -- Skills and Methods -----------------------------------------------------------------------

    @staticmethod
    async def state(skill: Node) -> str:
        """Return the name of the state a Skill is in."""
        value = await (await find(skill, "CurrentState")).read_value()
        return str(getattr(value, "Text", value))

    async def press(self, skill: Node, verb: str) -> None:
        """Press Start, Halt or Reset on a Skill's state machine."""
        state_machine = await (await find(skill, verb)).get_parent()
        assert state_machine is not None
        await state_machine.call_method(f"{await self.index(SmartFactoryMachineSetNodeIds)}:{verb}")
        await self.note(skill, f"{verb}()")

    async def in_state(self, skill: Node, *wanted: str) -> str | None:
        """Return the Skill's state if it is one of those wanted, else nothing."""
        seen = await self.state(skill)
        return seen if seen in wanted else None

    async def start(self, skill: Node) -> None:
        """Reset a Skill that is not ready, then start it."""
        if await self.state(skill) != "Ready":
            await self.press(skill, "Reset")
            await until(lambda: self.in_state(skill, "Ready"))
        await self.press(skill, "Start")

    async def put(self, skill: Node, path: str, value: object) -> None:
        """Write one Parameter of a Skill by its slash-separated browse path, structures included."""
        node = await below(skill, "SkillExecution", "ParameterSet", *path.split("/"))
        declared = (await node.read_data_value()).Value
        assert declared is not None
        datatype = await node.read_data_type()
        if datatype.NamespaceIndex != 0:
            cls = await self.structure(datatype)
            value = [instance(cls, each) for each in cast("list[dict[str, Any]]", value)]
        await node.write_value(ua.Variant(value, declared.VariantType))
        await self.note(skill, "Parameters written")

    @staticmethod
    async def result(skill: Node, path: str) -> Any:
        """Read one variable of a Skill's or Method's FinalResultData, by its slash-separated path."""
        return await (await below(await find(skill, "FinalResultData"), *path.split("/"))).read_value()

    async def run_skill(self, skill: Node, within: float = 120.0) -> str:
        """Start a Skill and wait until it ends; return its end state."""
        await self.start(skill)
        return await until(lambda: self.in_state(skill, "Completed", "Halted"), within=within)

    # -- what the Machine reports ----------------------------------------------------------------

    async def joint_states(self, component: Node) -> dict[str, tuple[float, float]]:
        """Return each joint's position and velocity as a Component's JointStates hold them."""
        states = await below(component, "Monitoring", "JointStates")
        names = [str(each) for each in await (await below(states, "Name")).read_value()]
        positions = [float(each) for each in await (await below(states, "Position")).read_value()]
        velocities = [float(each) for each in await (await below(states, "Velocity")).read_value()]
        await self.note(states, "read")
        return dict(zip(names, zip(positions, velocities, strict=True), strict=True))

    async def arm_joints(self) -> list[float]:
        """Return the arm's joint positions, in the order of :data:`ARM_JOINTS`."""
        states = await self.joint_states(self.arm)
        return [states[each][0] for each in ARM_JOINTS]

    async def cube(self) -> tuple[float, float, float]:
        """Return where the cube is, from the Workpiece's Pose."""
        pose = await below(self.workpiece, "Monitoring", "Pose")
        position = await below(pose, "Pose", "Position")
        seen = [float(await (await below(position, axis)).read_value()) for axis in ("X", "Y", "Z")]
        await self.note(pose, "read")
        return cast("tuple[float, float, float]", tuple(seen))

    # -- the motions ------------------------------------------------------------------------------

    async def move(self, planner: str, goal: dict[str, Any], *, speed: float = 0.3, wait: bool = True) -> str:
        """Plan and execute one motion with the Arm's Move Skill (MoveIt's MoveGroup action).

        :param planner: Pilz's ``PTP`` (point to point in joint space) or ``LIN`` (a straight line).
        :param goal: One moveit_msgs/Constraints as a map, from :func:`pose_goal` or :func:`joint_goal`.
        :param speed: The velocity and acceleration scaling factor.
        :param wait: Whether to wait for the end; without, return once the Skill was started.
        :raises AssertionError: When the motion does not complete with MoveIt's SUCCESS.
        """
        skill = await below(self.arm, "SkillSet", "Move")
        for path, value in (
            ("Request/GroupName", "arm"),
            ("Request/PipelineId", PIPELINE),
            ("Request/PlannerId", planner),
            ("Request/NumPlanningAttempts", 1),
            ("Request/AllowedPlanningTime", 5.0),
            ("Request/MaxVelocityScalingFactor", speed),
            ("Request/MaxAccelerationScalingFactor", speed),
            ("Request/GoalConstraints", [goal]),
        ):
            await self.put(skill, path, value)
        if not wait:
            await self.start(skill)
            return "started"
        started = time.monotonic()
        ended = await self.run_skill(skill)
        code = await self.result(skill, "ErrorCode/Val")
        if ended != "Completed" or code != SUCCESS:
            message = f"Move ({planner}) ended {ended} with MoveIt error code {code}"
            raise AssertionError(message)
        return f"Move {planner}: Completed, error code {code} (SUCCESS), {time.monotonic() - started:.1f} s"

    async def grip(self, position: float) -> tuple[str, float, bool, bool]:
        """Move the gripper with its GripperCommand Skill; return how it ended and its Result.

        The Skill is control_msgs/ParallelGripperCommand: its goal is a sensor_msgs/JointState that
        names the knuckle joint and its target angle; the controller's own ``max_effort`` limits the force.
        """
        skill = await below(self.gripper, "SkillSet", "GripperCommand")
        await self.put(skill, "Command/Name", [KNUCKLE])
        await self.put(skill, "Command/Position", [position])
        ended = await self.run_skill(skill, within=60.0)
        reached = float((await self.result(skill, "State/Position"))[0])
        stalled = bool(await self.result(skill, "Stalled"))
        goal = bool(await self.result(skill, "ReachedGoal"))
        return ended, reached, stalled, goal

    # -- steps ------------------------------------------------------------------------------------

    async def startup(self) -> str:
        """Start both StartupSkills, and see the three controllers active."""
        seen = []
        for label, component in (("Arm", self.arm), ("Gripper", self.gripper)):
            skill = await below(component, "SkillSet", "StartupSkill")
            before = await self.state(skill)
            await self.start(skill)
            reached = await until(lambda s=skill: self.in_state(s, "Running", "Completed", "Halted"), within=240.0)
            if reached == "Halted":
                message = f"the {label} StartupSkill halted: a required Binding did not appear"
                raise AssertionError(message)
            seen.append(f"{label} StartupSkill {before} -> {reached}")
        active = await until(self.controllers_active, within=120.0)
        return f"{'; '.join(seen)}; ListControllers: {active}"

    async def controllers_active(self) -> str | None:
        """Call ListControllers; return the three controllers' states once all are active."""
        method = await below(self.arm, "MethodSet", "ListControllers")
        await method.call_method(f"{await self.index(SmartFactorySkillSetNodeIds)}:Call")
        await self.note(method, "Call()")
        variable = await below(await find(method, "FinalResultData"), "Controller")
        await self.structure(await variable.read_data_type())
        states = {str(each["name"]): str(each["state"]) for each in as_map(await variable.read_value())}
        wanted = ("joint_state_broadcaster", "arm_controller", "gripper_controller")
        if all(states.get(each) == "active" for each in wanted):
            return ", ".join(f"{name} {states[name]}" for name in wanted)
        return None

    async def identification(self) -> str:
        """Read the identification of the Machine and its Components, and the Diagnostics."""
        parts = []
        for label, node in (("Machine", self.machine), ("Arm", self.arm), ("Gripper", self.gripper)):
            ident = await below(node, "Identification")
            manufacturer = await (await below(ident, "Manufacturer")).read_value()
            serial = await (await below(ident, "SerialNumber")).read_value()
            parts.append(f"{label}: {getattr(manufacturer, 'Text', manufacturer)!s}, serial {serial}")
        group = await below(self.arm, "Monitoring", "Diagnostics")

        async def members() -> list[Node]:
            return [each for each in await group.get_children() if await each.read_node_class() == ua.NodeClass.Object]

        found = await until(members, within=60.0)
        levels = {}
        for member in found:
            levels[(await member.read_display_name()).Text] = await (await below(member, "Level")).read_value()
        worst = await (await below(group, "WorstLevel")).read_value()
        return f"{'; '.join(parts)}; Diagnostics: {len(found)} members, worst level {worst}: {levels}"

    async def monitoring(self) -> str:
        """Read the joint states of arm and gripper twice, and see simulated time move on."""
        stamp = await below(self.arm, "Monitoring", "JointStates", "Header", "Stamp")
        sec = await below(stamp, "Sec")
        nanosec = await below(stamp, "Nanosec")

        async def now() -> float:
            return float(await sec.read_value()) + float(await nanosec.read_value()) * 1e-9

        first = await until(now)

        async def later() -> float | None:
            seen = await now()
            return seen if seen > first else None

        second = await until(later)
        arm = await self.joint_states(self.arm)
        gripper = await self.joint_states(self.gripper)
        missing = [each for each in ARM_JOINTS if each not in arm] + ([] if KNUCKLE in gripper else [KNUCKLE])
        if missing:
            message = f"the joint states lack {missing}"
            raise AssertionError(message)
        positions = ", ".join(f"{arm[each][0]:.3f}" for each in ARM_JOINTS)
        return (
            f"stamp {first:.2f} s -> {second:.2f} s; arm joints [{positions}]; "
            f"gripper knuckle {gripper[KNUCKLE][0]:.3f} rad ({len(arm)} joints in /joint_states)"
        )

    async def open_gripper(self) -> str:
        """Open the gripper and see it reach its goal."""
        ended, position, _, reached = await self.grip(OPEN)
        if ended != "Completed" or not reached:
            message = f"GripperCommand(open) ended {ended}, position {position:.3f}, reached_goal {reached}"
            raise AssertionError(message)
        return f"GripperCommand(position={OPEN}): {ended}, position {position:.3f}, reached_goal {reached}"

    async def close_on_cube(self) -> str:
        """Close the gripper on the cube: it stalls short of closed, because the cube is between the fingers."""
        ended, position, stalled, reached = await self.grip(CLOSED)
        if ended != "Completed" or position > CLOSED - 0.1:
            message = (
                f"GripperCommand(close) ended {ended} at {position:.3f} rad (stalled {stalled}); "
                "closed that far, there is no cube between the fingers"
            )
            raise AssertionError(message)
        return (
            f"GripperCommand(position={CLOSED}): {ended}, stopped at {position:.3f} rad on the cube "
            f"(stalled {stalled}, reached_goal {reached})"
        )

    async def lift(self) -> str:
        """Lift the cube straight up, and see it come along."""
        before = await self.cube()
        done = await self.move("LIN", pose_goal(*PICK, GRASP_Z + ABOVE))
        after = await self.cube()
        if after[2] - before[2] < ABOVE / 2:
            message = f"{done}, but the cube stayed at z = {after[2]:.3f} (was {before[2]:.3f}): it was not grasped"
            raise AssertionError(message)
        return f"{done}; the cube rose from z = {before[2]:.3f} to {after[2]:.3f}"

    async def placed(self) -> str:
        """See the cube lie at the place position."""
        x, y, z = await self.cube()
        off = math.hypot(x - PLACE[0], y - PLACE[1])
        if off > PLACE_TOLERANCE or abs(z - CUBE_Z) > 0.01:
            message = (
                f"the cube is at ({x:.3f}, {y:.3f}, {z:.3f}), {off * 1000:.0f} mm from the place position "
                f"({PLACE[0]}, {PLACE[1]}, {CUBE_Z})"
            )
            raise AssertionError(message)
        return (
            f"cube at ({x:.3f}, {y:.3f}, {z:.3f}): {off * 1000:.0f} mm from the place position "
            f"({PLACE[0]}, {PLACE[1]}), on the table"
        )

    async def halt(self) -> str:
        """Start a slow motion, halt it from OPC UA, and see the arm stand still short of its goal."""
        skill = await below(self.arm, "SkillSet", "Move")
        start = await self.arm_joints()
        goal = (1.2, *HOME[1:])
        await self.move("PTP", joint_goal(goal), speed=HALT_SPEED, wait=False)
        await until(lambda: self.in_state(skill, "Running"), within=30.0)

        async def moving() -> float | None:
            moved = abs((await self.arm_joints())[0] - start[0])
            return moved if moved > HALT_AFTER else None

        await until(moving, within=30.0)
        await self.press(skill, "Halt")
        ended = await until(lambda: self.in_state(skill, "Halted", "Completed"), within=15.0)
        if ended != "Halted":
            message = f"the Move Skill ended {ended} instead of Halted"
            raise AssertionError(message)
        await asyncio.sleep(1.0)
        first = await self.arm_joints()
        await asyncio.sleep(1.0)
        second = await self.arm_joints()
        drift = max(abs(a - b) for a, b in zip(first, second, strict=True))
        if drift > 0.002 or abs(second[0] - goal[0]) < 0.1:
            message = f"after Halt joint 1 is at {second[0]:.3f} (goal {goal[0]}), still moving {drift:.4f} rad/s"
            raise AssertionError(message)
        return (
            f"PTP to joint 1 = {goal[0]} at {HALT_SPEED:.0%} speed started, Halt pressed while Running: Skill Halted; "
            f"joint 1 stopped at {second[0]:.3f} rad (from {start[0]:.3f}), largest joint motion over the "
            f"next second {drift:.4f} rad"
        )

    async def run(self, report: Report) -> None:
        """Take the Lock, run every step in order, and give the Lock back."""
        lock = await find(self.machine, "Lock")
        await lock.call_method(f"{await self.index(DiNodeIds)}:InitLock", "hc10 example scenario")
        try:
            await self.steps(report)
        finally:
            await lock.call_method(f"{await self.index(DiNodeIds)}:ExitLock")

    async def steps(self, report: Report) -> None:
        """Run the scenario's steps; a failed motion stops the pick and place, since the next builds on it."""
        if not await report.step("startup: controllers active, MoveIt ready", self.startup):
            return
        await report.step("identification and diagnostics readable", self.identification)
        await report.step("monitoring: joint states of arm and gripper", self.monitoring)
        pick: list[tuple[str, Callable[[], Awaitable[str]]]] = [
            (
                "pre-grasp pose above the cube (MoveIt, PTP)",
                lambda: self.move("PTP", pose_goal(*PICK, GRASP_Z + ABOVE)),
            ),
            ("open the gripper", self.open_gripper),
            ("approach the cube (MoveIt, LIN)", lambda: self.move("LIN", pose_goal(*PICK, GRASP_Z), speed=0.1)),
            ("close the gripper on the cube", self.close_on_cube),
            ("lift the cube (MoveIt, LIN)", self.lift),
            (
                "move above the place position (MoveIt, PTP)",
                lambda: self.move("PTP", pose_goal(*PLACE, GRASP_Z + ABOVE), speed=0.2),
            ),
            ("lower the cube (MoveIt, LIN)", lambda: self.move("LIN", pose_goal(*PLACE, GRASP_Z), speed=0.1)),
            ("open the gripper to release the cube", self.open_gripper),
            ("retreat (MoveIt, LIN)", lambda: self.move("LIN", pose_goal(*PLACE, GRASP_Z + ABOVE))),
            ("return home (MoveIt, PTP, joint goal)", lambda: self.move("PTP", joint_goal(HOME))),
        ]
        for name, body in pick:
            if not await report.step(name, body):
                break
        await report.step("the cube lies at the place position", self.placed)
        await report.step("halt a running motion from OPC UA", self.halt)
        await report.step("return home after the halt", lambda: self.move("PTP", joint_goal(HOME)))


BINDING_TABLES = {
    "monitoring": "message_type",
    "keyed": "message_type",
    "action": "action_type",
    "method": "service_type",
}
"""The kinds of Binding the machine file declares here, and the key that holds each one's ROS2 type."""


def binding_interfaces(machine_file: Path) -> dict[tuple[str, str], str]:
    """Return each Binding's ROS2 name and type, ``/path (type)``, by Component and Binding name."""
    data = tomllib.loads(machine_file.read_text(encoding="utf-8"))
    found: dict[tuple[str, str], str] = {}
    for component in data["component"]:
        for kind, type_key in BINDING_TABLES.items():
            for binding in component.get(kind, []):
                found[component["name"], binding["name"]] = f"{binding['path']} ({binding[type_key]})"
    return found


def endpoint(url: str, user: str) -> str:
    """Return the URL a user logs in with; opensmi's default users have their name as password."""
    scheme, rest = url.split("://", 1)
    return f"{scheme}://{user}:{user}@{rest}"


async def serving(url: str, within: float = 600.0) -> None:
    """Wait until a client can see the Machine's Components."""
    deadline = time.monotonic() + within
    while True:
        try:
            async with Client(url=url, timeout=5.0) as client:
                await below(client.nodes.objects, "Machines", MACHINE, "Components")
        except Exception as not_yet:
            if time.monotonic() > deadline:
                message = f"the Machine was not served within {within:.0f} s (last: {not_yet!r})"
                raise TimeoutError(message) from None
            await asyncio.sleep(2.0)
        else:
            return


async def scenario(url: str, report: Report) -> None:
    """Wait for the Machine, connect as the operator and run the steps."""

    async def served() -> str:
        await serving(endpoint(url, "visitor"))
        return f"the Machine {MACHINE} is served at {url}"

    if not await report.step("the Machine is served", served):
        return
    async with Client(url=endpoint(url, "operator"), timeout=CLIENT_TIMEOUT) as client:
        machine = await below(client.nodes.objects, "Machines", MACHINE)
        components = await find(machine, "Components")
        operator = Operator(
            client,
            machine,
            await below(components, "Arm"),
            await below(components, "Gripper"),
            await below(components, "Workpiece"),
            calls=report.calls,
        )
        await operator.run(report)


def main() -> None:
    """Run the scenario, write the report, and exit non-zero if any step failed."""
    url = os.environ.get("OPCUA_URL", "opc.tcp://machine:4840/server")
    output = Path(os.environ.get("REPORT", "reports/report.md"))
    started = datetime.now(UTC)
    report = Report()
    try:
        asyncio.run(scenario(url, report))
    except Exception as error:  # the report says what broke the session
        report.steps.append(Step("the OPC UA session", passed=False, detail=f"{type(error).__name__}: {error}"))
        print(f"[FAIL] the OPC UA session: {type(error).__name__}: {error}", flush=True)  # noqa: T201
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(report.render(url, started), encoding="utf-8", newline="\n")
    if timeline := os.environ.get("TIMELINE"):
        Path(timeline).write_text(json.dumps(report.timeline, indent=1), encoding="utf-8", newline="\n")
    print(f"report: {output} ({'passed' if report.passed else 'FAILED'})", flush=True)  # noqa: T201
    sys.exit(0 if report.passed else 1)


if __name__ == "__main__":
    main()
