# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Declaring Bindings from the class attribute and the TOML file: union, collision, and what a file
cannot reach."""

from __future__ import annotations

import math
from dataclasses import fields
from pathlib import Path
from typing import Any, get_type_hints

import pytest

from opensmi.ros2.runtime.parameters import ParameterType
from opensmi.ros2.runtime.qos import Durability, QoSProfile, Reliability
from opensmi.ros2.runtime.selection import FieldSelection
from opensmi.ros2.server.bindings import (
    TOML_KINDS,
    ActionBinding,
    KeyedBinding,
    LifecycleBinding,
    MethodBinding,
    MonitoringBinding,
    PublishedFrame,
    PublisherBinding,
    ROS2ParameterBinding,
    TFBinding,
    is_required,
    load_bindings,
    resolve_bindings,
)
from opensmi.ros2.server.config import _expressible
from opensmi.ros2.server.errors import (
    BindingNameError,
    DeclarationError,
    DuplicateBindingNameError,
    FrameDirectionError,
    ImplementationClassError,
    ParameterDeclarationError,
    RequiredDeclarationError,
)
from opensmi.ros2.server.publisher import MAXIMUM_RATE, ROS2ContinuousPublisherSkill, ROS2FinitePublisherSkill

POSE = MonitoringBinding(
    name="CurrentPose",
    path="/current_pose",
    message_type="fixture_msgs/msg/PoseStamped",
    exclude=("header",),
)
ODOM = MonitoringBinding(name="Odom", path="/odom", message_type="fixture_msgs/msg/PoseWithCovariance")

TOML = """
[[monitoring]]
name = "Battery"
path = "/battery_state"
message_type = "fixture_msgs/msg/Time"
include = ["sec"]
depth = 1
"""


QOS_TOML = """
[[monitoring]]
name = "Scan"
path = "/scan"
message_type = "fixture_msgs/msg/Time"
depth = 5

[monitoring.qos]
reliability = "best_effort"
durability = "transient_local"
deadline = 0.5
"""

ACTION_TOML = """
[[action]]
name = "Navigate"
path = "/navigate_to_pose"
action_type = "fixture_msgs/action/Navigate"
include = ["target/x"]
"""

PUBLISHER_TOML = """
[[publisher]]
name = "Drive"
path = "/cmd_vel"
message_type = "fixture_msgs/msg/Point"
continuous = true
rate = 20.0
stale_after = 0.5
halt_message = { x = 0.0, y = 0 }
tolerated_failures = 3
exclude = ["z"]
"""

METHOD_TOML = """
[[method]]
name = "SetMode"
path = "/set_mode"
service_type = "fixture_msgs/srv/SetMode"
success_field = "success"
response_exclude = ["message"]
call_timeout = 5.0
"""


def write(tmp_path: Path, text: str) -> Path:
    source = tmp_path / "bindings.toml"
    source.write_text(text, encoding="utf-8")
    return source


def test_a_binding_carries_its_selection_as_the_one_mechanism() -> None:
    assert POSE.selection == FieldSelection(include=(), exclude=("header",))


def test_declarations_alone_resolve_to_themselves() -> None:
    assert resolve_bindings([POSE, ODOM]) == (POSE, ODOM)


def test_a_toml_source_unions_with_the_class_attribute(tmp_path: Path) -> None:
    resolved = resolve_bindings([POSE], write(tmp_path, TOML))

    assert [binding.name for binding in resolved] == ["CurrentPose", "Battery"]


def test_toml_lists_arrive_as_the_tuples_a_declaration_holds(tmp_path: Path) -> None:
    (battery,) = load_bindings(write(tmp_path, TOML))

    assert battery == MonitoringBinding(
        name="Battery",
        path="/battery_state",
        message_type="fixture_msgs/msg/Time",
        include=("sec",),
        depth=1,
    )


def test_a_deployment_declares_a_qos_profile_as_a_toml_table(tmp_path: Path) -> None:
    (scan,) = load_bindings(write(tmp_path, QOS_TOML))
    assert isinstance(scan, MonitoringBinding)

    assert scan.qos == QoSProfile(
        reliability=Reliability.BEST_EFFORT,
        durability=Durability.TRANSIENT_LOCAL,
        deadline=0.5,
    )
    assert scan.depth == 5


def test_a_qos_policy_a_file_misspells_says_which_ones_exist(tmp_path: Path) -> None:
    wrong = QOS_TOML.replace('reliability = "best_effort"', 'reliability = "besteffort"')

    with pytest.raises(ValueError, match="best_effort"):
        load_bindings(write(tmp_path, wrong))


def test_a_qos_profile_left_out_is_the_discovery_that_needs_no_spelling(tmp_path: Path) -> None:
    (battery,) = load_bindings(write(tmp_path, TOML))
    assert isinstance(battery, MonitoringBinding)

    assert battery.qos is None


def test_a_name_in_both_sources_is_fatal(tmp_path: Path) -> None:
    clash = TOML.replace('name = "Battery"', 'name = "CurrentPose"')

    with pytest.raises(DuplicateBindingNameError) as raised:
        resolve_bindings([POSE], write(tmp_path, clash))

    assert "CurrentPose" in str(raised.value)


def test_a_name_declared_twice_in_one_source_is_fatal() -> None:
    with pytest.raises(DuplicateBindingNameError):
        resolve_bindings([POSE, POSE])


def test_toml_cannot_name_a_python_class(tmp_path: Path) -> None:
    named = TOML + 'implementation = "tests.fixture_messages.Path"\n'

    with pytest.raises(ValueError, match="implementation"):
        load_bindings(write(tmp_path, named))


@pytest.mark.parametrize("key", ["skill_type", "goal_from", "result_to"])
def test_an_actions_python_only_keywords_are_refused_a_value_from_toml(tmp_path: Path, key: str) -> None:
    named = f'{ACTION_TOML}{key} = "tests.fixture_messages.Path"'

    with pytest.raises(ValueError, match=key):
        load_bindings(write(tmp_path, named))


@pytest.mark.parametrize("key", ["method_type", "request_from", "response_to"])
def test_a_methods_python_only_keywords_are_refused_a_value_from_toml(tmp_path: Path, key: str) -> None:
    named = f'{METHOD_TOML}{key} = "tests.fixture_messages.Path"'

    with pytest.raises(ValueError, match=key):
        load_bindings(write(tmp_path, named))


def test_a_service_declares_itself_in_toml_like_any_other_kind(tmp_path: Path) -> None:
    (set_mode,) = load_bindings(write(tmp_path, METHOD_TOML))

    assert set_mode == MethodBinding(
        name="SetMode",
        path="/set_mode",
        service_type="fixture_msgs/srv/SetMode",
        success_field="success",
        response_exclude=("message",),
        call_timeout=5.0,
    )


def test_a_service_declaration_never_needs_the_graph() -> None:
    assert not MethodBinding(name="SetMode", path="/set_mode", service_type="fixture_msgs/srv/SetMode").needs_the_graph


def test_an_action_declares_itself_in_toml_like_any_other_kind(tmp_path: Path) -> None:
    (navigate,) = load_bindings(write(tmp_path, ACTION_TOML))

    assert navigate == ActionBinding(
        name="Navigate",
        path="/navigate_to_pose",
        action_type="fixture_msgs/action/Navigate",
        include=("target/x",),
    )


def test_every_field_of_every_toml_kind_is_either_expressible_or_refused() -> None:
    for kind, binding in TOML_KINDS.items():
        hints = get_type_hints(binding)
        for field in fields(binding):
            expressible = _expressible(hints[field.name])
            assert isinstance(expressible, bool), f"{kind}.{field.name}"


def test_a_table_naming_a_kind_that_does_not_exist_says_which_ones_do(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="monitoring"):
        load_bindings(write(tmp_path, TOML.replace("[[monitoring]]", "[[monitor]]")))


def test_a_binding_that_declares_its_type_never_needs_the_graph() -> None:
    assert not POSE.needs_the_graph
    assert MonitoringBinding(name="Scan", path="/scan").needs_the_graph


def test_a_binding_is_optional_unless_it_says_otherwise() -> None:
    assert not is_required(POSE)
    assert is_required(MonitoringBinding(name="Scan", path="/scan", required=True, startup_timeout=5.0))


@pytest.mark.parametrize(
    ("required", "timeout"),
    [(False, 5.0), (True, 0.0), (True, -1.0)],
)
def test_a_startup_deadline_that_guards_nothing_or_has_passed_is_fatal(required: bool, timeout: float) -> None:
    with pytest.raises(RequiredDeclarationError, match="startup_timeout"):
        MonitoringBinding(name="Scan", path="/scan", required=required, startup_timeout=timeout)


# -- the publishing declaration ---------------------------------------------------------------

STREAM = PublisherBinding(
    name="Drive",
    path="/cmd_vel",
    message_type="fixture_msgs/msg/Point",
    continuous=True,
    rate=20.0,
)
PUSH = PublisherBinding(
    name="PushMap",
    path="/map",
    message_type="fixture_msgs/msg/Grid",
    continuous=False,
)


def test_a_publishing_binding_never_reads_the_graph() -> None:
    assert not STREAM.needs_the_graph
    assert STREAM.selection == FieldSelection(include=(), exclude=())


def test_a_publisher_declares_itself_in_toml_like_any_other_kind(tmp_path: Path) -> None:
    (drive,) = load_bindings(write(tmp_path, PUBLISHER_TOML))

    assert drive == PublisherBinding(
        name="Drive",
        path="/cmd_vel",
        message_type="fixture_msgs/msg/Point",
        continuous=True,
        rate=20.0,
        stale_after=0.5,
        halt_message={"x": 0.0, "y": 0},
        tolerated_failures=3,
        exclude=("z",),
    )


@pytest.mark.parametrize("key", ["skill_type", "message_from"])
def test_a_publishers_python_only_keywords_are_refused_a_value_from_toml(tmp_path: Path, key: str) -> None:
    named = f'{PUBLISHER_TOML}{key} = "tests.fixture_messages.Path"'

    with pytest.raises(ValueError, match=key):
        load_bindings(write(tmp_path, named))


def test_a_publisher_is_never_required() -> None:
    assert not is_required(STREAM)
    assert "required" not in {each.name for each in fields(PublisherBinding)}
    assert "startup_timeout" not in {each.name for each in fields(PublisherBinding)}


def test_a_stream_with_no_rate_is_fatal() -> None:
    with pytest.raises(DeclarationError, match="no rate"):
        PublisherBinding(name="Drive", path="/cmd_vel", message_type="fixture_msgs/msg/Point", continuous=True)


@pytest.mark.parametrize("rate", [0.0, -1.0, MAXIMUM_RATE * 2])
def test_a_rate_outside_its_published_bounds_is_fatal(rate: float) -> None:
    with pytest.raises(DeclarationError, match="rate="):
        PublisherBinding(
            name="Drive",
            path="/cmd_vel",
            message_type="fixture_msgs/msg/Point",
            continuous=True,
            rate=rate,
        )


@pytest.mark.parametrize(
    ("keyword", "value"),
    [("rate", 20.0), ("stale_after", 0.5), ("halt_message", {"x": 0.0}), ("tolerated_failures", 3)],
)
def test_a_stream_only_keyword_on_a_finite_push_is_fatal(keyword: str, value: Any) -> None:
    declared: dict[str, Any] = {keyword: value}

    with pytest.raises(DeclarationError, match=keyword):
        PublisherBinding(
            name="PushMap",
            path="/map",
            message_type="fixture_msgs/msg/Grid",
            continuous=False,
            **declared,
        )


@pytest.mark.parametrize("seconds", [0.0, -0.5, math.inf, math.nan])
def test_a_stale_after_that_is_not_a_positive_number_of_seconds_is_fatal(seconds: float) -> None:
    with pytest.raises(DeclarationError, match="stale_after"):
        PublisherBinding(
            name="Drive",
            path="/cmd_vel",
            message_type="fixture_msgs/msg/Point",
            continuous=True,
            rate=20.0,
            stale_after=seconds,
        )


def test_a_negative_tolerance_is_fatal() -> None:
    with pytest.raises(DeclarationError, match="tolerated_failures"):
        PublisherBinding(
            name="Drive",
            path="/cmd_vel",
            message_type="fixture_msgs/msg/Point",
            continuous=True,
            rate=20.0,
            tolerated_failures=-1,
        )


def test_the_continuity_decides_which_stock_class_a_custom_one_must_subclass() -> None:
    class MyStream(ROS2ContinuousPublisherSkill): ...

    class MyPush(ROS2FinitePublisherSkill): ...

    assert (
        PublisherBinding(
            name="Drive",
            path="/cmd_vel",
            message_type="fixture_msgs/msg/Point",
            continuous=True,
            rate=1.0,
            skill_type=MyStream,
        ).skill_type
        is MyStream
    )
    assert (
        PublisherBinding(
            name="PushMap",
            path="/map",
            message_type="fixture_msgs/msg/Grid",
            continuous=False,
            skill_type=MyPush,
        ).skill_type
        is MyPush
    )

    with pytest.raises(ImplementationClassError, match="ROS2FinitePublisherSkill"):
        PublisherBinding(
            name="PushMap",
            path="/map",
            message_type="fixture_msgs/msg/Grid",
            continuous=False,
            skill_type=MyStream,
        )
    with pytest.raises(ImplementationClassError, match="ROS2ContinuousPublisherSkill"):
        PublisherBinding(
            name="Drive",
            path="/cmd_vel",
            message_type="fixture_msgs/msg/Point",
            continuous=True,
            rate=1.0,
            skill_type=MyPush,
        )


def test_a_publisher_unions_and_collides_like_every_other_kind(tmp_path: Path) -> None:
    resolved = resolve_bindings([POSE, PUSH], write(tmp_path, PUBLISHER_TOML))

    assert [binding.name for binding in resolved] == ["CurrentPose", "PushMap", "Drive"]
    with pytest.raises(DuplicateBindingNameError):
        resolve_bindings([STREAM, PUSH], write(tmp_path, PUBLISHER_TOML))


# -- the ROS2-parameter declaration ------------------------------------------------------------

TUNING = ROS2ParameterBinding(
    name="Controller",
    path="/controller_server",
    parameters={"FollowPath.max_vel_x": ParameterType.DOUBLE, "use_sim_time": ParameterType.BOOL},
    read_only=("use_sim_time",),
)

PARAMETER_TOML = """
[[ros2_parameter]]
name = "Planner"
path = "/planner_server"
parameters = { "GridBased.tolerance" = "double", "use_sim_time" = "bool" }
read_only = ["use_sim_time"]
retry_interval = 2.5
write_timeout = 1.5
"""


def test_a_ros2_parameter_binding_declares_its_types_and_never_reads_the_graph() -> None:
    assert TUNING.parameters["FollowPath.max_vel_x"] is ParameterType.DOUBLE
    assert TUNING.needs_the_graph is False


def test_a_parameter_name_is_a_path_of_groups_and_a_variable() -> None:
    assert TUNING.node_path("FollowPath.max_vel_x") == ("FollowPath", "MaxVelX")
    assert TUNING.node_path("use_sim_time") == ("UseSimTime",)


def test_a_parameter_segment_that_does_not_convert_is_refused_with_the_suggestion() -> None:
    with pytest.raises(BindingNameError, match="try 'FollowPath'"):
        ROS2ParameterBinding(
            name="Controller", path="/controller_server", parameters={"follow-path.max_vel_x": ParameterType.DOUBLE}
        )


def test_read_only_is_a_property_of_one_declared_name() -> None:
    assert TUNING.writable("FollowPath.max_vel_x") is True
    assert TUNING.writable("use_sim_time") is False


def test_a_write_waits_less_than_a_client_by_default() -> None:
    # asyncua's client gives up after four seconds and then gets no status instead of BadTimeout.
    assert 0 < TUNING.write_timeout < 4.0


def test_a_declaration_that_exposes_nothing_is_fatal() -> None:
    with pytest.raises(ParameterDeclarationError, match="declares no parameters"):
        ROS2ParameterBinding(name="Controller", path="/controller_server", parameters={})


def test_a_read_only_mark_on_a_name_nobody_declared_is_fatal() -> None:
    with pytest.raises(ParameterDeclarationError, match="use_sim_time"):
        ROS2ParameterBinding(
            name="Controller",
            path="/controller_server",
            parameters={"FollowPath.max_vel_x": ParameterType.DOUBLE},
            read_only=("use_sim_time",),
        )


@pytest.mark.parametrize("seconds", [0.0, -1.0])
def test_a_retry_interval_that_is_not_a_period_is_fatal(seconds: float) -> None:
    with pytest.raises(ParameterDeclarationError, match="retry_interval"):
        ROS2ParameterBinding(
            name="Controller",
            path="/controller_server",
            parameters={"a": ParameterType.DOUBLE},
            retry_interval=seconds,
        )


@pytest.mark.parametrize("seconds", [0.0, -1.0])
def test_a_write_timeout_that_is_not_a_period_is_fatal(seconds: float) -> None:
    with pytest.raises(ParameterDeclarationError, match="write_timeout"):
        ROS2ParameterBinding(
            name="Controller",
            path="/controller_server",
            parameters={"a": ParameterType.DOUBLE},
            write_timeout=seconds,
        )


def test_a_ros2_parameter_binding_declares_itself_in_toml_like_any_other_kind(tmp_path: Path) -> None:
    (planner,) = load_bindings(write(tmp_path, PARAMETER_TOML))

    assert planner == ROS2ParameterBinding(
        name="Planner",
        path="/planner_server",
        parameters={"GridBased.tolerance": ParameterType.DOUBLE, "use_sim_time": ParameterType.BOOL},
        read_only=("use_sim_time",),
        retry_interval=2.5,
        write_timeout=1.5,
    )


def test_a_toml_file_naming_a_parameter_type_that_does_not_exist_says_which_ones_do(tmp_path: Path) -> None:
    named = PARAMETER_TOML.replace('"double"', '"float64"')

    with pytest.raises(ValueError, match="ParameterType"):
        load_bindings(write(tmp_path, named))


def test_a_ros2_parameter_binding_unions_and_collides_like_every_other_kind(tmp_path: Path) -> None:
    resolved = resolve_bindings([POSE, TUNING], write(tmp_path, PARAMETER_TOML))

    assert [binding.name for binding in resolved] == ["CurrentPose", "Controller", "Planner"]
    with pytest.raises(DuplicateBindingNameError):
        resolve_bindings([TUNING, TUNING])


# -- the TF declaration -------------------------------------------------------------------------

TF_TOML = """
[[tf]]
name = "ArmFrames"
head = "robot_base"
alternatives = [["map", "tool0"]]
exclude = ["*_cable"]
"""

#: A Machine-wide fact written on one object's Binding, which this kind refuses.
MACHINE_WIDE_TOML = """
[[tf]]
name = "ArmFrames"
head = "robot_base"
interval = 0.5
"""


def test_a_tf_binding_declares_itself_in_toml_like_any_other_kind(tmp_path: Path) -> None:
    (frames,) = load_bindings(write(tmp_path, TF_TOML))

    assert frames == TFBinding(
        name="ArmFrames",
        head="robot_base",
        alternatives=(("map", "tool0"),),
        exclude=("*_cable",),
    )


def test_a_tf_binding_that_names_no_head_is_refused() -> None:
    with pytest.raises(DeclarationError, match="names no head frame"):
        TFBinding(name="Frames", head="")


def test_a_tf_binding_that_excludes_its_own_head_is_refused() -> None:
    with pytest.raises(DeclarationError, match="excludes its own head frame"):
        TFBinding(name="Frames", head="base_link", exclude=("base_*",))


@pytest.mark.parametrize(
    "machine_wide",
    [
        {"path": "/robot/tf"},
        {"static_path": "/robot/tf_static"},
        {"depth": 40},
        {"interval": 0.5},
        {"quiet_after": 2.0},
    ],
)
def test_a_tf_binding_that_declares_a_fact_about_the_whole_machine_is_refused(machine_wide) -> None:
    with pytest.raises(DeclarationError, match=r"\[ros2\.transforms\]"):
        TFBinding(name="Frames", head="base_link", **machine_wide)


def test_a_tf_binding_that_declares_a_machine_wide_fact_in_toml_is_refused_too(tmp_path: Path) -> None:
    with pytest.raises(DeclarationError, match="interval"):
        load_bindings(write(tmp_path, MACHINE_WIDE_TOML))


@pytest.mark.parametrize("alternative", [("", "tool0"), ("map", ""), ("map", "map")])
def test_an_alternative_that_names_no_composition_is_refused(alternative) -> None:
    with pytest.raises(DeclarationError, match="are different frames"):
        TFBinding(name="Frames", head="base_link", alternatives=(alternative,))


def test_a_tf_binding_never_needs_the_graph() -> None:
    assert not TFBinding(name="Frames", head="base_link").needs_the_graph


# -- the outward half of the declaration ---------------------------------------------------------

PUBLISHES_TOML = """
[[tf]]
name = "PlatformFrames"
head = "odom"

[[tf.publishes]]
base = "map"
frame = "odom_estimate"

[[tf.publishes]]
base = "base_link"
frame = "payload_mount"
static = true
position = [0.0, 0.0, 0.54]
orientation = [0.0, 0.0, 90.0]
"""


def test_a_published_frame_is_declared_in_toml_like_everything_else_about_the_object(tmp_path: Path) -> None:
    (frames,) = load_bindings(write(tmp_path, PUBLISHES_TOML))

    assert frames == TFBinding(
        name="PlatformFrames",
        head="odom",
        publishes=(
            PublishedFrame(base="map", frame="odom_estimate"),
            PublishedFrame(
                base="base_link",
                frame="payload_mount",
                static=True,
                position=(0.0, 0.0, 0.54),
                orientation=(0.0, 0.0, 90.0),
            ),
        ),
    )


def test_a_published_frame_publishes_nothing_and_sits_at_the_origin_unless_it_says_otherwise() -> None:
    published = PublishedFrame(base="map", frame="odom")

    assert (published.static, published.position, published.orientation) == (False, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


@pytest.mark.parametrize("named", [{"base": "", "frame": "odom"}, {"base": "map", "frame": ""}])
def test_a_published_frame_that_does_not_name_two_frames_is_refused(named) -> None:
    with pytest.raises(DeclarationError, match="names no"):
        TFBinding(name="Frames", head="base_link", publishes=(PublishedFrame(**named),))


def test_a_published_frame_that_is_its_own_parent_is_refused() -> None:
    with pytest.raises(DeclarationError, match="its own parent"):
        TFBinding(name="Frames", head="base_link", publishes=(PublishedFrame(base="map", frame="map"),))


def test_publishing_the_same_frame_twice_from_one_declaration_is_refused() -> None:
    with pytest.raises(DeclarationError, match="twice"):
        TFBinding(
            name="Frames",
            head="base_link",
            publishes=(PublishedFrame(base="map", frame="odom"), PublishedFrame(base="world", frame="odom")),
        )


def test_publishing_this_object_own_head_frame_is_refused() -> None:
    with pytest.raises(FrameDirectionError, match="inward"):
        TFBinding(name="Frames", head="odom", publishes=(PublishedFrame(base="map", frame="odom"),))


def test_publishing_a_frame_this_declaration_also_excludes_is_refused() -> None:
    with pytest.raises(DeclarationError, match="excludes"):
        TFBinding(
            name="Frames",
            head="base_link",
            exclude=("*_mount",),
            publishes=(PublishedFrame(base="base_link", frame="payload_mount"),),
        )


def test_a_declaration_that_publishes_nothing_says_so() -> None:
    assert TFBinding(name="Frames", head="base_link").broadcasts is False
    assert (
        TFBinding(
            name="Frames",
            head="base_link",
            publishes=(PublishedFrame(base="map", frame="odom"),),
        ).broadcasts
        is True
    )


def test_a_lifecycle_node_is_declared_in_a_binding_file(tmp_path: Path) -> None:
    (loaded,) = load_bindings(
        write(
            tmp_path,
            '[[lifecycle]]\nname = "Navigation"\npath = "/lifecycle_manager"\n'
            'nodes = ["/planner", "/controller"]\ndriver = "nav2"\nrequired = true\n',
        )
    )

    assert isinstance(loaded, LifecycleBinding)
    assert loaded.observed == ("/planner", "/controller")
    assert loaded.required


def test_a_binding_file_cannot_name_a_driver_class(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="driver_type"):
        load_bindings(write(tmp_path, '[[lifecycle]]\nname = "a"\npath = "/a"\ndriver_type = "x.Y"\n'))


# --- a name that becomes a browse name carries no slash ----------------------------------------

NAMED_KINDS: list[Any] = [
    lambda name: MonitoringBinding(name=name, path="/a"),
    lambda name: ActionBinding(name=name, path="/a", action_type="fixture_msgs/action/Navigate"),
    lambda name: MethodBinding(name=name, path="/a", service_type="fixture_msgs/srv/SetMode"),
    lambda name: PublisherBinding(name=name, path="/a", message_type="fixture_msgs/msg/Point", continuous=False),
    lambda name: ROS2ParameterBinding(name=name, path="/a", parameters={"p": ParameterType.INTEGER}),
    lambda name: LifecycleBinding(name=name, path="/a"),
]


@pytest.mark.parametrize("make", NAMED_KINDS)
def test_a_binding_name_with_a_slash_is_refused_where_it_is_declared(make: Any) -> None:
    with pytest.raises(BindingNameError, match=r"Name 'arm/left' breaks the naming rule; try 'ArmLeft'"):
        make("arm/left")


@pytest.mark.parametrize("make", NAMED_KINDS)
def test_a_binding_name_that_breaks_the_rule_is_refused_with_the_converted_name(make: Any) -> None:
    with pytest.raises(BindingNameError) as raised:
        make("joint_states")
    assert str(raised.value) == "Name 'joint_states' breaks the naming rule; try 'JointStates'"


@pytest.mark.parametrize("make", NAMED_KINDS)
@pytest.mark.parametrize("name", ["odom", "True", "1st"])
def test_a_lowercase_reserved_or_digit_first_binding_name_is_refused(make: Any, name: str) -> None:
    with pytest.raises(BindingNameError, match="breaks the naming rule"):
        make(name)


@pytest.mark.parametrize("make", NAMED_KINDS)
def test_a_binding_name_that_follows_the_rule_is_accepted(make: Any) -> None:
    make("PlatformFrames2")


@pytest.mark.parametrize("make", NAMED_KINDS)
def test_an_empty_binding_name_is_refused_where_it_is_declared(make: Any) -> None:
    with pytest.raises(BindingNameError, match="empty name"):
        make("")


@pytest.mark.parametrize("make", NAMED_KINDS)
def test_a_name_with_a_dash_or_a_space_is_refused(make: Any) -> None:
    with pytest.raises(BindingNameError, match=r"'arm-left 2' breaks the naming rule; try 'ArmLeft2'"):
        make("arm-left 2")


def test_a_slash_in_a_field_path_is_still_a_field_path() -> None:
    binding = MonitoringBinding(name="Pose", path="/a", include=("position/x",), exclude=("header/*",))
    assert binding.include == ("position/x",)


def test_a_binding_file_with_a_slash_in_a_name_names_the_binding(tmp_path: Path) -> None:
    text = '[[monitoring]]\nname = "arm/left"\npath = "/a"\n'
    with pytest.raises(DeclarationError, match=r"Name 'arm/left' breaks the naming rule; try 'ArmLeft'"):
        load_bindings(write(tmp_path, text))


RENAME_TOML = """
[[monitoring]]
name = "Pose"
path = "/pose"
message_type = "fixture_msgs/msg/Pose"
rename = {"position/x" = "East"}

[[keyed]]
name = "Poses"
path = "/poses"
message_type = "fixture_msgs/msg/Path"
repeated_field = "poses"
key = ["header/frame_id"]
rename = {"header" = "Head"}

[[publisher]]
name = "Drive"
path = "/cmd_vel"
message_type = "fixture_msgs/msg/Point"
continuous = false
rename = {"x" = "Forward"}

[[action]]
name = "Navigate"
path = "/navigate_to_pose"
action_type = "fixture_msgs/action/Navigate"
rename = {"target/x" = "GoalX"}
feedback_rename = {"progress" = "Done"}
result_rename = {"ok" = "Success"}

[[method]]
name = "SetMode"
path = "/set_mode"
service_type = "fixture_msgs/srv/SetMode"
success_field = "success"
rename = {"mode" = "Kind"}
response_rename = {"message" = "Reason"}
"""


def test_rename_is_declared_in_toml_on_every_kind_that_renders_message_fields(tmp_path: Path) -> None:
    pose, poses, drive, navigate, set_mode = load_bindings(write(tmp_path, RENAME_TOML))
    assert isinstance(navigate, ActionBinding)
    assert isinstance(set_mode, MethodBinding)
    assert isinstance(pose, MonitoringBinding)
    assert isinstance(poses, KeyedBinding)
    assert isinstance(drive, PublisherBinding)

    assert pose.selection.rename == {"position/x": "East"}
    assert poses.selection.rename == {"header": "Head"}
    assert drive.selection.rename == {"x": "Forward"}
    assert navigate.selection.rename == {"target/x": "GoalX"}
    assert navigate.feedback_selection.rename == {"progress": "Done"}
    assert navigate.result_selection.rename == {"ok": "Success"}
    assert set_mode.selection.rename == {"mode": "Kind"}
    assert set_mode.response_selection.rename == {"message": "Reason"}


def test_rename_is_the_same_keyword_in_python() -> None:
    binding = MonitoringBinding(name="Pose", path="/pose", rename={"position/x": "East"})

    assert binding.selection == FieldSelection(rename={"position/x": "East"})


def test_a_binding_without_rename_renames_nothing() -> None:
    assert POSE.selection.rename == {}


@pytest.mark.parametrize("new_name", ["east", "East_1", "", "None", "A.B"])
def test_a_new_name_that_breaks_the_naming_rule_is_refused_at_init(new_name: str) -> None:
    with pytest.raises(BindingNameError, match=r"'position/x'.*in rename"):
        MonitoringBinding(name="Pose", path="/pose", rename={"position/x": new_name})


def test_every_kind_refuses_a_new_name_that_breaks_the_rule() -> None:
    bad = {"a": "bad_name"}
    with pytest.raises(BindingNameError):
        ActionBinding(name="Go", path="/go", action_type="x/action/Y", feedback_rename=bad)
    with pytest.raises(BindingNameError):
        ActionBinding(name="Go", path="/go", action_type="x/action/Y", result_rename=bad)
    with pytest.raises(BindingNameError):
        MethodBinding(name="Go", path="/go", service_type="x/srv/Y", response_rename=bad)
    with pytest.raises(BindingNameError):
        PublisherBinding(name="Go", path="/go", message_type="x/msg/Y", continuous=False, rename=bad)


def test_a_toml_rename_with_a_bad_name_is_reported_with_the_binding(tmp_path: Path) -> None:
    source = write(
        tmp_path,
        '[[monitoring]]\nname = "Pose"\npath = "/pose"\nrename = {"position/x" = "east"}\n',
    )

    with pytest.raises(Exception, match="east"):
        load_bindings(source)


# --- the names Machinery keeps for a machinery item's Monitoring --------------------------------

#: The kinds whose name is a top-level node of a Machine's or Component's Monitoring.
MONITORING_KINDS: list[Any] = [
    lambda name: MonitoringBinding(name=name, path="/a"),
    lambda name: KeyedBinding(
        name=name,
        path="/a",
        message_type="fixture_msgs/msg/Path",
        repeated_field="poses",
        key=("header/frame_id",),
    ),
]

MACHINERY_MONITORING = ["Status", "Health", "Process", "Consumption"]


@pytest.mark.parametrize("make", MONITORING_KINDS)
@pytest.mark.parametrize("name", MACHINERY_MONITORING)
def test_a_name_the_machinery_monitoring_reserves_is_refused_where_it_is_declared(make: Any, name: str) -> None:
    with pytest.raises(DeclarationError) as raised:
        make(name)

    assert f"Binding '{name}' takes a name reserved in a Machine's or Component's Monitoring" in str(raised.value)
    assert "for the Machinery Monitoring sub-structures" in str(raised.value)


@pytest.mark.parametrize("make", MONITORING_KINDS)
@pytest.mark.parametrize("name", ["StatusCode", "HealthState", "Processes", "PowerConsumption"])
def test_a_name_that_only_starts_or_ends_like_a_reserved_one_is_accepted(make: Any, name: str) -> None:
    assert make(name).name == name


@pytest.mark.parametrize("make", NAMED_KINDS[1:])
def test_a_kind_that_never_lands_at_the_top_of_a_machinery_monitoring_may_be_called_status(make: Any) -> None:
    assert make("Status").name == "Status"


def test_a_binding_file_with_a_reserved_name_names_the_binding(tmp_path: Path) -> None:
    text = (
        '[[keyed]]\nname = "Health"\npath = "/diagnostics"\nmessage_type = "fixture_msgs/msg/Path"\n'
        'repeated_field = "poses"\nkey = ["header/frame_id"]\n'
    )
    with pytest.raises(DeclarationError, match=r"Keyed Binding 'Health' takes a name reserved"):
        load_bindings(write(tmp_path, text))


# --- the Machinery Monitoring sub-structure a Binding is filed in -------------------------------


#: The kinds that may be filed in a group, by the group they name.
GROUPED_KINDS: list[Any] = [
    lambda group: MonitoringBinding(name="Battery", path="/a", group=group),
    lambda group: KeyedBinding(
        name="Battery",
        path="/a",
        message_type="fixture_msgs/msg/Path",
        repeated_field="poses",
        key=("header/frame_id",),
        group=group,
    ),
]


@pytest.mark.parametrize("make", GROUPED_KINDS)
@pytest.mark.parametrize("group", [*MACHINERY_MONITORING, None])
def test_a_monitoring_kind_may_name_a_machinery_group(make: Any, group: str | None) -> None:
    assert make(group).group == group


@pytest.mark.parametrize("make", GROUPED_KINDS)
@pytest.mark.parametrize("group", ["Energy", "status", "", "Process/Drive"])
def test_a_group_that_is_not_a_machinery_sub_structure_is_refused_listing_the_four(make: Any, group: str) -> None:
    with pytest.raises(DeclarationError) as raised:
        make(group)

    assert f"names the group {group!r}" in str(raised.value)
    assert "Status, Health, Process, Consumption" in str(raised.value)


@pytest.mark.parametrize("table", ["monitoring", "keyed"])
@pytest.mark.parametrize("group", MACHINERY_MONITORING)
def test_a_binding_file_files_a_monitoring_kind_in_a_machinery_group(tmp_path: Path, table: str, group: str) -> None:
    keyed = 'message_type = "fixture_msgs/msg/Path"\nrepeated_field = "poses"\nkey = ["header/frame_id"]\n'
    text = f'[[{table}]]\nname = "Battery"\npath = "/battery"\ngroup = "{group}"\n' + (
        keyed if table == "keyed" else ""
    )

    [binding] = load_bindings(write(tmp_path, text))

    assert isinstance(binding, (MonitoringBinding, KeyedBinding))
    assert binding.group == group


def test_a_binding_file_with_another_group_is_refused_listing_the_four(tmp_path: Path) -> None:
    text = '[[monitoring]]\nname = "Battery"\npath = "/battery"\ngroup = "Energy"\n'

    with pytest.raises(DeclarationError, match=r"names the group 'Energy'.*Status, Health, Process, Consumption"):
        load_bindings(write(tmp_path, text))


@pytest.mark.parametrize("kind", ["action", "method", "publisher", "ros2_parameter", "lifecycle", "tf"])
def test_a_kind_outside_the_monitoring_takes_no_group(kind: str) -> None:
    assert "group" not in {each.name for each in fields(TOML_KINDS[kind])}


def test_a_binding_file_with_a_group_on_a_skill_is_refused(tmp_path: Path) -> None:
    text = (
        '[[action]]\nname = "Navigate"\npath = "/navigate"\naction_type = "fixture_msgs/action/Navigate"\n'
        'group = "Process"\n'
    )

    with pytest.raises(ValueError, match=r"unknown configuration option\(s\) for ActionBinding: group"):
        load_bindings(write(tmp_path, text))
