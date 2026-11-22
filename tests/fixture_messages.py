# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Message dataclasses shaped like real ROS2 types, in the package `fixture_msgs`, for tests without ROS2."""

from __future__ import annotations

from typing import Any

from opensmi.ros2.runtime.messages import (
    ActionTypes,
    ArrayType,
    FieldDescription,
    FieldType,
    Message,
    MessageDescription,
    NestedType,
    PrimitiveType,
    SequenceType,
    ServiceTypes,
    StringType,
    build_message_dataclass,
)

_BUILT: dict[str, type[Message]] = {}


def describe(type_name: str, **fields: tuple[FieldType, str]) -> MessageDescription:
    """Build a MessageDescription from `name=(field_type, ros2_text)` pairs."""
    return MessageDescription(
        type_name=type_name,
        fields=tuple(
            FieldDescription(name=name, field_type=field_type, ros2_text=text)
            for name, (field_type, text) in fields.items()
        ),
    )


def build(description: MessageDescription) -> Any:
    """Build a dataclass whose nested types resolve out of this module rather than out of ROS2.

    Returns `Any` for the same reason `test_message_dataclasses.py` does: a dataclass built at
    run time has no static field set, the generated stubs cover the definition catalog's packages
    rather than these fixtures, and annotating one field at a time would be a lie rather than a
    check.
    """
    built = build_message_dataclass(description, resolve=_BUILT.__getitem__)
    _BUILT[description.type_name] = built
    return built


def nested(type_name: str) -> tuple[FieldType, str]:
    """Describe a field whose type is another fixture message."""
    return NestedType(type_name=type_name), type_name.replace("/msg/", "/")


FLOAT64 = (PrimitiveType(name="float64"), "double")
INT32 = (PrimitiveType(name="int32"), "int32")
UINT32 = (PrimitiveType(name="uint32"), "uint32")
STRING = (StringType(), "string")

Time = build(describe("fixture_msgs/msg/Time", sec=INT32, nanosec=UINT32))

Header = build(describe("fixture_msgs/msg/Header", stamp=nested("fixture_msgs/msg/Time"), frame_id=STRING))

Point = build(describe("fixture_msgs/msg/Point", x=FLOAT64, y=FLOAT64, z=FLOAT64))

Quaternion = build(describe("fixture_msgs/msg/Quaternion", x=FLOAT64, y=FLOAT64, z=FLOAT64, w=FLOAT64))

Pose = build(
    describe(
        "fixture_msgs/msg/Pose",
        position=nested("fixture_msgs/msg/Point"),
        orientation=nested("fixture_msgs/msg/Quaternion"),
    )
)

PoseStamped = build(
    describe(
        "fixture_msgs/msg/PoseStamped",
        header=nested("fixture_msgs/msg/Header"),
        pose=nested("fixture_msgs/msg/Pose"),
    )
)

PoseWithCovariance = build(
    describe(
        "fixture_msgs/msg/PoseWithCovariance",
        pose=nested("fixture_msgs/msg/Pose"),
        covariance=(ArrayType(element=PrimitiveType(name="float64"), length=4), "double[4]"),
    )
)

# Container shapes: a bounded list of strings, a list of numbers, a byte buffer.
Sensor = build(
    describe(
        "fixture_msgs/msg/Sensor",
        ok=(PrimitiveType(name="bool"), "boolean"),
        labels=(SequenceType(element=StringType()), "sequence<string>"),
        readings=(SequenceType(element=PrimitiveType(name="float64")), "sequence<double>"),
        payload=(SequenceType(element=PrimitiveType(name="uint8")), "sequence<uint8>"),
    )
)

# A repeated message field, rendered as an array of a structure.
Path = build(
    describe(
        "fixture_msgs/msg/Path",
        header=nested("fixture_msgs/msg/Header"),
        poses=(SequenceType(element=NestedType(type_name="fixture_msgs/msg/PoseStamped")), "sequence<PoseStamped>"),
    )
)

FLOAT32 = (PrimitiveType(name="float32"), "float")
INT8 = (PrimitiveType(name="int8"), "int8")
UINT8 = (PrimitiveType(name="uint8"), "uint8")

Voltage = build(describe("fixture_msgs/msg/Voltage", data=FLOAT64))

# A message carrying nothing (std_msgs/Empty, std_srvs/srv/Empty).
Nothing = build(describe("fixture_msgs/msg/Nothing"))

# A no-argument service request: one field whose type is field-less (nav2 ClearEntireCostmap).
Knock = build(describe("fixture_msgs/srv/Knock_Request", request=nested("fixture_msgs/msg/Nothing")))

# `data` is int8[] like an occupancy grid's, so it arrives as a tuple of signed numbers.
Grid = build(
    describe(
        "fixture_msgs/msg/Grid",
        width=UINT32,
        data=(SequenceType(element=PrimitiveType(name="int8")), "sequence<int8>"),
    )
)

Scan = build(
    describe(
        "fixture_msgs/msg/Scan",
        frame_id=STRING,
        ranges=(SequenceType(element=PrimitiveType(name="float32")), "sequence<float>"),
    )
)

Mode = build(describe("fixture_msgs/msg/Mode", mode=INT8, detail=STRING))

# A field called `rate`, which a continuous publishing Skill reserves.
Ramp = build(describe("fixture_msgs/msg/Ramp", rate=FLOAT64, limit=FLOAT64))

# A `stamp` that is a Time, a Time under another name, and a `stamp` that is not a Time.
Timed = build(
    describe(
        "fixture_msgs/msg/Timed",
        stamp=nested("fixture_msgs/msg/Time"),
        duration=nested("fixture_msgs/msg/Time"),
        label=STRING,
    )
)

Mislabelled = build(describe("fixture_msgs/msg/Mislabelled", stamp=nested("fixture_msgs/msg/Point")))


def pose_stamped(*, frame_id: str = "map", x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Any:
    """Build a PoseStamped payload, the way one arrives across the seam."""
    return PoseStamped(
        header=Header(stamp=Time(sec=1, nanosec=2), frame_id=frame_id),
        pose=Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
    )


# ROS2 spells generated action members `Navigate_Goal`, not `NavigateGoal`.
NavigateGoal = build(
    describe(
        "fixture_msgs/action/Navigate_Goal",
        target=nested("fixture_msgs/msg/Point"),
        label=STRING,
    )
)

NavigateFeedback = build(describe("fixture_msgs/action/Navigate_Feedback", distance_remaining=FLOAT64))

NavigateResult = build(
    describe(
        "fixture_msgs/action/Navigate_Result",
        total_distance=FLOAT64,
        waypoints=INT32,
    )
)

Navigate = ActionTypes(
    type_name="fixture_msgs/action/Navigate",
    goal=NavigateGoal,
    feedback=NavigateFeedback,
    result=NavigateResult,
)

DockGoal = build(describe("fixture_msgs/action/Dock_Goal"))
DockFeedback = build(describe("fixture_msgs/action/Dock_Feedback"))
DockResult = build(describe("fixture_msgs/action/Dock_Result", docked=(PrimitiveType(name="bool"), "boolean")))

Dock = ActionTypes(
    type_name="fixture_msgs/action/Dock",
    goal=DockGoal,
    feedback=DockFeedback,
    result=DockResult,
)


# nav2 shape: the goal is a list the client sizes; the result carries a computed plan.
PlanGoal = build(
    describe(
        "fixture_msgs/action/Plan_Goal",
        poses=(SequenceType(element=NestedType(type_name="fixture_msgs/msg/PoseStamped")), "sequence<PoseStamped>"),
    )
)
PlanFeedback = build(describe("fixture_msgs/action/Plan_Feedback", remaining=INT32))
PlanResult = build(
    describe(
        "fixture_msgs/action/Plan_Result",
        path=nested("fixture_msgs/msg/Path"),
        planning_time=FLOAT64,
    )
)

Plan = ActionTypes(
    type_name="fixture_msgs/action/Plan",
    goal=PlanGoal,
    feedback=PlanFeedback,
    result=PlanResult,
)


def path(*poses: Any, frame_id: str = "map") -> Any:
    """Build a Path payload carrying the poses it was given, however many that is."""
    return Path(header=Header(stamp=Time(sec=1, nanosec=2), frame_id=frame_id), poses=poses)


ChargeGoal = build(describe("fixture_msgs/action/Charge_Goal", limit=nested("fixture_msgs/msg/Voltage")))
ChargeFeedback = build(describe("fixture_msgs/action/Charge_Feedback", charged=FLOAT64))
ChargeResult = build(describe("fixture_msgs/action/Charge_Result", stored=FLOAT64))

Charge = ActionTypes(
    type_name="fixture_msgs/action/Charge",
    goal=ChargeGoal,
    feedback=ChargeFeedback,
    result=ChargeResult,
)

# A field named like a Machinery Monitoring sub-structure, which only the top level of a machinery
# item's Monitoring reserves: below a Binding's root and in a Skill's Monitoring it is an ordinary name.
Job = build(describe("fixture_msgs/msg/Job", status=INT32, progress=FLOAT64))
RunGoal = build(describe("fixture_msgs/action/Run_Goal"))
RunFeedback = build(describe("fixture_msgs/action/Run_Feedback", status=INT32))
RunResult = build(describe("fixture_msgs/action/Run_Result", progress=FLOAT64))

Run = ActionTypes(
    type_name="fixture_msgs/action/Run",
    goal=RunGoal,
    feedback=RunFeedback,
    result=RunResult,
)

BOOL = (PrimitiveType(name="bool"), "boolean")

# ROS2 spells generated service members `SetMode_Request`.
# `success` is never auto-detected; a Method treats it as the verdict only when declared.
SetModeRequest = build(
    describe(
        "fixture_msgs/srv/SetMode_Request",
        mode=nested("fixture_msgs/msg/Mode"),
        label=STRING,
    )
)

SetModeResponse = build(
    describe(
        "fixture_msgs/srv/SetMode_Response",
        success=BOOL,
        message=STRING,
        applied=INT32,
    )
)

SetMode = ServiceTypes(
    type_name="fixture_msgs/srv/SetMode",
    request=SetModeRequest,
    response=SetModeResponse,
)

PingRequest = build(describe("fixture_msgs/srv/Ping_Request"))
PingResponse = build(describe("fixture_msgs/srv/Ping_Response", replies=INT32))

Ping = ServiceTypes(
    type_name="fixture_msgs/srv/Ping",
    request=PingRequest,
    response=PingResponse,
)

# ClearEntireCostmap exactly: an Empty request and an Empty response.
ClearResponse = build(describe("fixture_msgs/srv/Clear_Response", response=nested("fixture_msgs/msg/Nothing")))

Clear = ServiceTypes(
    type_name="fixture_msgs/srv/Clear",
    request=Knock,
    response=ClearResponse,
)


# Shaped like a DiagnosticArray: multi-field key, state, key-value pairs.
Pair = build(describe("fixture_msgs/msg/Pair", key=STRING, value=STRING))

Unit = build(
    describe(
        "fixture_msgs/msg/Unit",
        level=INT8,
        name=STRING,
        hardware_id=STRING,
        message=STRING,
        values=(SequenceType(element=NestedType(type_name="fixture_msgs/msg/Pair")), "sequence<Pair>"),
    )
)

Units = build(
    describe(
        "fixture_msgs/msg/Units",
        header=nested("fixture_msgs/msg/Header"),
        units=(SequenceType(element=NestedType(type_name="fixture_msgs/msg/Unit")), "sequence<Unit>"),
    )
)


def unit(
    *,
    name: str = "motor",
    hardware_id: str = "robot_1",
    level: int = 0,
    message: str = "",
    values: tuple[tuple[str, str], ...] = (),
) -> Any:
    """Build one element of a keyed array, the way one arrives inside a message."""
    return Unit(
        level=level,
        name=name,
        hardware_id=hardware_id,
        message=message,
        values=tuple(Pair(key=key, value=value) for key, value in values),
    )


def units(*elements: Any, frame_id: str = "robot") -> Any:
    """Build the message a Keyed Binding reads, carrying the elements it was given."""
    return Units(header=Header(stamp=Time(sec=1, nanosec=2), frame_id=frame_id), units=elements)


# Every element repeats a message of its own, like a robot's named joint targets.
InventoryGoal = build(
    describe(
        "fixture_msgs/action/Inventory_Goal",
        units=(SequenceType(element=NestedType(type_name="fixture_msgs/msg/Unit")), "sequence<Unit>"),
    )
)
InventoryFeedback = build(describe("fixture_msgs/action/Inventory_Feedback", remaining=INT32))
InventoryResult = build(
    describe(
        "fixture_msgs/action/Inventory_Result",
        units=(SequenceType(element=NestedType(type_name="fixture_msgs/msg/Unit")), "sequence<Unit>"),
    )
)

Inventory = ActionTypes(
    type_name="fixture_msgs/action/Inventory",
    goal=InventoryGoal,
    feedback=InventoryFeedback,
    result=InventoryResult,
)


# Shaped like tf2_msgs/msg/TFMessage: the topology read is structural.
Vector3 = build(describe("fixture_msgs/msg/Vector3", x=FLOAT64, y=FLOAT64, z=FLOAT64))

# Shaped like geometry_msgs/msg/Twist: the velocity command a halt message stops.
Twist = build(
    describe(
        "fixture_msgs/msg/Twist", linear=nested("fixture_msgs/msg/Vector3"), angular=nested("fixture_msgs/msg/Vector3")
    )
)

Transform = build(
    describe(
        "fixture_msgs/msg/Transform",
        translation=nested("fixture_msgs/msg/Vector3"),
        rotation=nested("fixture_msgs/msg/Quaternion"),
    )
)

TransformStamped = build(
    describe(
        "fixture_msgs/msg/TransformStamped",
        header=nested("fixture_msgs/msg/Header"),
        child_frame_id=STRING,
        transform=nested("fixture_msgs/msg/Transform"),
    )
)

TFMessage = build(
    describe(
        "fixture_msgs/msg/TFMessage",
        transforms=(
            SequenceType(element=NestedType(type_name="fixture_msgs/msg/TransformStamped")),
            "sequence<TransformStamped>",
        ),
    )
)


def transform_stamped(
    parent: str,
    child: str,
    *,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> Any:
    """Build one element of a TF message: a parent, a child and where the child sits."""
    return TransformStamped(
        header=Header(stamp=Time(sec=1, nanosec=2), frame_id=parent),
        child_frame_id=child,
        transform=Transform(
            translation=Vector3(x=position[0], y=position[1], z=position[2]),
            rotation=Quaternion(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3]),
        ),
    )


def tf_message(*elements: Any) -> Any:
    """Build the message both transform topics carry, holding the elements it was given."""
    return TFMessage(transforms=elements)


# Like a joint trajectory: waypoints with lists, a fixed array, bytes and a time.
BOOL = (PrimitiveType(name="bool"), "boolean")
Waypoint = build(
    describe(
        "fixture_msgs/msg/Waypoint",
        positions=(SequenceType(element=PrimitiveType(name="float64")), "sequence<double>"),
        covariance=(ArrayType(element=PrimitiveType(name="float64"), length=3), "double[3]"),
        raw=(SequenceType(element=PrimitiveType(name="uint8")), "sequence<uint8>"),
        time_from_start=nested("fixture_msgs/msg/Time"),
        valid=BOOL,
    )
)

Waypoints = build(
    describe(
        "fixture_msgs/msg/Waypoints",
        joint_names=(SequenceType(element=StringType()), "sequence<string>"),
        points=(SequenceType(element=NestedType(type_name="fixture_msgs/msg/Waypoint")), "sequence<Waypoint>"),
    )
)


def waypoint(*, seed: float = 1.0, positions: int = 2, raw: bytes = b"\x00\x01\xff") -> Any:
    """Build one waypoint whose values tell it from another seeded differently."""
    return Waypoint(
        positions=tuple(seed + each for each in range(positions)),
        covariance=(seed, 0.0, -seed),
        raw=raw,
        time_from_start=Time(sec=int(seed), nanosec=500),
        valid=True,
    )


def waypoints(*points: Any, joint_names: tuple[str, ...] = ("joint_1", "joint_2")) -> Any:
    """Build a trajectory holding the waypoints it was given."""
    return Waypoints(joint_names=joint_names, points=points)
