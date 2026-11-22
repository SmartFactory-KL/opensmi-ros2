# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The provenance report's own logic: row contents, which Bindings get one, and when one counts as waiting."""

from __future__ import annotations

import datetime as dt
from dataclasses import dataclass
from typing import Any

from opensmi.ros2.runtime.graph import GraphSnapshot
from opensmi.ros2.runtime.qos import DEFAULT_PROFILE, Provenance
from opensmi.ros2.server.bindings import ActionBinding, KeyedBinding, MethodBinding, MonitoringBinding
from opensmi.ros2.server.config import ROS2Config, TransformConfig
from opensmi.ros2.server.introspection import (
    BindingReport,
    WaitingClock,
    machinery_items,
    qos_entries_of,
    reports_of,
)
from opensmi.ros2.server.provenance import BindingQoS

POSE = MonitoringBinding(name="CurrentPose", path="/current_pose", message_type="x/msg/Pose", required=True)
SET_MODE_SERVICE = MethodBinding(name="SetModeService", path="/set_mode", service_type="x/srv/SetMode")
NAVIGATE_ACTION = ActionBinding(name="Navigate", path="/navigate", action_type="x/action/Navigate")
UNITS = KeyedBinding(name="Units", path="/units", message_type="x/msg/Units", repeated_field="units", key=("name",))


@dataclass(frozen=True)
class Publishing:
    """A kind with a path and no presence to read: we are the publisher."""

    name: str
    path: str


@dataclass(frozen=True)
class Frames:
    """A kind with no ROS2 path of its own: where it reads from is the Machine's."""

    name: str


CMD_VEL = Publishing(name="cmd_vel", path="/cmd_vel")
SET_MODE = Publishing(name="SetMode", path="/set_mode")
FRAMES = Frames(name="frames")


class Item:
    """The smallest thing `reports_of` can answer for: some Bindings and a presence map."""

    def __init__(self, *bindings: Any, present: dict[str, bool] | None = None, **held: Any) -> None:
        self.path = "Robot/Platform"
        self.ros2_bindings = bindings
        self.ros2_monitoring_presence = present or {}
        self.ros2_config = ROS2Config()
        self.skill_set: list[Any] = []
        self.__dict__.update(held)


class Machine:
    """A Machine with Components under it, as `machinery_items` walks one."""

    def __init__(self, name: str, *components: Any) -> None:
        self.path = name
        self.ros2_bindings = ()
        self.components = components


def _at(minute: int) -> dt.datetime:
    """Return one fixed moment, for a test that is about which of two moments was kept."""
    return dt.datetime(2026, 9, 20, 9, minute, tzinfo=dt.UTC)


def row(item: Any, name: str, clock: WaitingClock | None = None) -> BindingReport:
    return next(each for each in reports_of(item, clock or WaitingClock()) if each.binding == name)


# -- what a row says ---------------------------------------------------------------------------


def test_a_row_names_the_item_the_binding_and_the_kind() -> None:
    reported = row(Item(POSE, present={"CurrentPose": True}), "CurrentPose")

    assert (reported.item, reported.binding, reported.kind) == ("Robot/Platform", "CurrentPose", "MonitoringBinding")
    assert reported.path == "/current_pose"


def test_a_required_binding_with_a_publisher_is_satisfied_and_is_not_waiting() -> None:
    reported = row(Item(POSE, present={"CurrentPose": True}), "CurrentPose")

    assert reported.required
    assert reported.satisfied
    assert reported.waiting_since is None


def test_a_binding_with_no_publisher_is_absent_and_has_been_waiting_since_the_first_sample() -> None:
    reported = row(Item(POSE, present={"CurrentPose": False}), "CurrentPose")

    assert reported.satisfied is False
    assert reported.waiting_since is not None


def test_a_publishing_binding_has_no_presence_to_report() -> None:
    reported = row(Item(CMD_VEL), "cmd_vel")

    assert reported.satisfied is None
    assert reported.waiting_since is None


def test_a_tf_bindings_ros2_path_is_the_machines_two_transform_topics() -> None:
    reported = row(Item(FRAMES), "frames")

    assert reported.path == "/tf + /tf_static"


def test_a_remapped_transform_topic_reaches_the_report() -> None:
    item = Item(FRAMES, ros2_config=ROS2Config(transforms=TransformConfig(path="/robot/tf")))

    assert row(item, "frames").path == "/robot/tf + /tf_static"


def test_a_row_reads_as_one_line_naming_the_state_and_the_provenance() -> None:
    reported = BindingReport(
        item="Robot/Platform",
        binding="CurrentPose",
        kind="MonitoringBinding",
        path="/current_pose",
        required=True,
        satisfied=False,
        waiting_since=dt.datetime(2026, 9, 20, 9, 14, 2, tzinfo=dt.UTC),
        qos=BindingQoS(
            binding="CurrentPose",
            path="/current_pose",
            profile=DEFAULT_PROFILE,
            provenance=Provenance.DISCOVERED,
            adaptations=2,
        ),
    )

    assert str(reported).startswith(
        "Robot/Platform/CurrentPose [MonitoringBinding] /current_pose -- "
        "required, absent since 2026-09-20T09:14:02+00:00; QoS: discovered "
    )
    assert str(reported).endswith(", adapted 2 time(s)")


def test_a_kind_with_no_endpoint_of_its_own_says_so_rather_than_guessing() -> None:
    assert "QoS: no endpoint" in str(row(Item(SET_MODE), "SetMode"))


class Graph:
    """The one thing a Method's presence probe reads off the Runtime: its last graph snapshot."""

    def __init__(self, snapshot: GraphSnapshot) -> None:
        self.snapshot = snapshot


class Runtime:
    def __init__(self, snapshot: GraphSnapshot) -> None:
        self.graph = Graph(snapshot)


def test_a_service_methods_presence_is_its_service_on_the_last_graph_snapshot() -> None:
    there = Item(SET_MODE_SERVICE, runtime=Runtime(GraphSnapshot(services={"/set_mode": ("x/srv/SetMode",)})))
    gone = Item(SET_MODE_SERVICE, runtime=Runtime(GraphSnapshot()))

    assert row(there, "SetModeService").satisfied is True
    assert row(gone, "SetModeService").satisfied is False


def test_an_action_skills_presence_is_its_action_on_the_last_graph_snapshot() -> None:
    there = Item(NAVIGATE_ACTION, runtime=Runtime(GraphSnapshot(actions={"/navigate": ("x/action/Navigate",)})))
    gone = Item(NAVIGATE_ACTION, runtime=Runtime(GraphSnapshot(services={"/navigate": ("x/srv/Navigate",)})))

    assert row(there, "Navigate").satisfied is True
    assert row(gone, "Navigate").satisfied is False


def test_a_keyed_bindings_presence_is_its_publisher_and_never_a_member() -> None:
    assert row(Item(UNITS, ros2_keyed_presence={"Units": True}), "Units").satisfied is True
    assert row(Item(UNITS, ros2_keyed_presence={}), "Units").satisfied is False


# -- which Bindings a report is built for --------------------------------------------------------


def test_the_report_walks_the_machine_and_every_component_under_it() -> None:
    inner = Machine("Robot/Platform/Arm")
    machine = Machine("Robot", Machine("Robot/Platform", inner))

    assert [each.path for each in machinery_items(machine)] == [
        "Robot",
        "Robot/Platform",
        "Robot/Platform/Arm",
    ]


def test_an_item_that_is_not_one_of_ours_contributes_no_provenance_rather_than_raising() -> None:
    class Plain:
        pass

    assert qos_entries_of(Plain()) == {}


def test_a_publishing_bindings_provenance_is_read_off_the_skill_it_became() -> None:
    entry = BindingQoS(
        binding="cmd_vel",
        path="/cmd_vel",
        profile=DEFAULT_PROFILE,
        provenance=Provenance.DEFAULTED,
        adaptations=0,
    )

    class Skill:
        ros2_qos_provenance = entry

    assert qos_entries_of(Item(CMD_VEL, skill_set=[Skill()])) == {"cmd_vel": entry}


# -- the clock ------------------------------------------------------------------------------------


def test_a_binding_that_comes_back_forgets_what_it_was_waiting_since() -> None:
    clock = WaitingClock()
    clock.note(("Robot", "pose"), satisfied=False)
    first = clock.waiting_since(("Robot", "pose"))

    clock.note(("Robot", "pose"), satisfied=True)

    assert first is not None
    assert clock.waiting_since(("Robot", "pose")) is None


def test_a_binding_that_stays_away_keeps_the_moment_it_went() -> None:
    clock = WaitingClock()
    clock.note(("Robot", "pose"), satisfied=False)
    first = clock.waiting_since(("Robot", "pose"))

    clock.note(("Robot", "pose"), satisfied=False)

    assert clock.waiting_since(("Robot", "pose")) == first


def test_a_binding_that_flaps_is_reported_waiting_since_the_second_departure() -> None:
    # A clock of the test's own, because the two departures are milliseconds apart.
    ticks = iter([_at(1), _at(2)])
    clock = WaitingClock(now=lambda: next(ticks))
    clock.note(("Robot", "pose"), satisfied=False)
    clock.note(("Robot", "pose"), satisfied=True)

    clock.note(("Robot", "pose"), satisfied=False)

    assert clock.waiting_since(("Robot", "pose")) == _at(2)


def test_a_kind_with_no_presence_to_read_is_never_waiting() -> None:
    clock = WaitingClock()

    clock.note(("Robot", "cmd_vel"), satisfied=None)

    assert clock.waiting_since(("Robot", "cmd_vel")) is None
