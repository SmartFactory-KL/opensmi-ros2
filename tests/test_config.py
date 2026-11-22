# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The generic configuration loader and the ROS2 configuration it loads; an unknown key is fatal."""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from inspect import signature
from pathlib import Path

import pytest

from opensmi.ros2.runtime.tf import TF_DEPTH, TF_STATIC_TOPIC, TF_TOPIC
from opensmi.ros2.server.config import FieldError, ROS2Config, TransformConfig, from_dict
from opensmi.ros2.server.errors import DeclarationError
from tests.fake_runtime import FakeRuntime

ROS2_TOML = """
domain_id = 42
node_name = "platform"
namespace = "/robot"
"""


@dataclass(frozen=True, kw_only=True)
class Nested:
    depth: int = 1


@dataclass(frozen=True, kw_only=True)
class Example:
    name: str = ""
    patterns: tuple[str, ...] = ()
    nested: Nested = Nested()
    optional: str | None = None


def test_ros2_configuration_loads_from_a_file_of_its_own(tmp_path: Path) -> None:
    source = tmp_path / "ros2.toml"
    source.write_text(ROS2_TOML, encoding="utf-8")

    config = ROS2Config.from_toml(source)

    assert config == ROS2Config(domain_id=42, node_name="platform", namespace="/robot")


def test_an_absent_key_takes_the_dataclass_default() -> None:
    assert from_dict(ROS2Config, {"domain_id": 3}).stop_timeout == ROS2Config().stop_timeout


def test_an_unknown_key_is_fatal_rather_than_ignored() -> None:
    with pytest.raises(ValueError, match="doman_id"):
        from_dict(ROS2Config, {"doman_id": 3})


def test_a_list_becomes_the_tuple_a_frozen_declaration_holds() -> None:
    assert from_dict(Example, {"patterns": ["a", "b"]}).patterns == ("a", "b")


def test_a_table_becomes_the_nested_dataclass() -> None:
    assert from_dict(Example, {"nested": {"depth": 4}}).nested == Nested(depth=4)


def test_an_unknown_key_in_a_nested_table_is_fatal_too() -> None:
    with_typo = {"nested": {"deth": 4}}

    with pytest.raises(ValueError, match="deth"):
        from_dict(Example, with_typo)


def test_an_optional_value_survives_being_absent() -> None:
    assert from_dict(Example, {"optional": "here"}).optional == "here"
    assert from_dict(Example, {}).optional is None


def test_the_ros2_configuration_names_the_runtimes_own_keywords() -> None:
    # The Runtime is built from the config field by field, so the names must agree.
    # The listed settings belong to the server half; the Runtime has no opinion on them.
    not_the_runtimes = {
        "debug_goal_outcome_booleans",
        "introspection_access_level",
        "listing_access_level",
        "skill_access_level",
        "startup_timeout",
        "transforms",
    }
    accepted = set(signature(FakeRuntime.__init__).parameters)

    assert {field.name for field in fields(ROS2Config)} - not_the_runtimes <= accepted


def test_something_that_is_not_a_dataclass_is_refused() -> None:
    with pytest.raises(TypeError):
        from_dict(dict, {})  # type: ignore[type-var]


# --------------------------------------------------------------------------- the transform settings


TRANSFORMS_TOML = """
domain_id = 7

[transforms]
path = "/robot/tf"
static_path = "/robot/tf_static"
depth = 40
interval = 0.5
quiet_after = 2.0
"""


def test_the_transform_settings_are_the_machines_and_default_to_both_ros2_topics() -> None:
    settings = ROS2Config().transforms

    assert settings.topics == (TF_TOPIC, TF_STATIC_TOPIC)
    assert settings.depth == TF_DEPTH
    assert settings.interval > 0
    assert settings.quiet_after > settings.interval


def test_the_transform_settings_load_from_the_ros2_file_like_every_other_deployment_fact(tmp_path: Path) -> None:
    source = tmp_path / "ros2.toml"
    source.write_text(TRANSFORMS_TOML, encoding="utf-8")

    config = ROS2Config.from_toml(source)

    assert config.transforms == TransformConfig(
        path="/robot/tf",
        static_path="/robot/tf_static",
        depth=40,
        interval=0.5,
        quiet_after=2.0,
    )


def test_a_transform_key_the_settings_do_not_declare_is_fatal_rather_than_ignored() -> None:
    with pytest.raises(ValueError, match="intervall"):
        from_dict(ROS2Config, {"transforms": {"intervall": 0.5}})


@pytest.mark.parametrize("interval", [0.0, -1.0])
def test_polling_the_frame_network_at_a_period_that_is_not_one_is_refused(interval) -> None:
    with pytest.raises(DeclarationError, match="positive number"):
        TransformConfig(interval=interval)


@pytest.mark.parametrize("quiet_after", [0.0, -1.0])
def test_a_staleness_window_that_is_not_a_period_is_refused(quiet_after) -> None:
    with pytest.raises(DeclarationError, match="positive number"):
        TransformConfig(quiet_after=quiet_after)


@pytest.mark.parametrize("depth", [0, -1])
def test_a_transform_subscription_that_buffers_nothing_is_refused(depth) -> None:
    with pytest.raises(DeclarationError, match="at least one message"):
        TransformConfig(depth=depth)


@pytest.mark.parametrize("topics", [{"path": ""}, {"static_path": ""}])
def test_a_transform_topic_that_is_not_named_is_refused(topics) -> None:
    with pytest.raises(DeclarationError, match="names no"):
        TransformConfig(**topics)


def test_reading_both_kinds_of_transform_from_one_topic_is_refused() -> None:
    with pytest.raises(DeclarationError, match="two topics"):
        TransformConfig(path="/tf", static_path="/tf")


def test_the_default_node_name_is_one_ros2_accepts() -> None:
    assert ROS2Config().node_name == "opensmi_ros2"


def test_the_mutator_level_defaults_to_the_operator_and_may_not_drop_below_a_skill() -> None:
    from opensmi.ros2.server.access import MUTATOR_ACCESS_LEVEL, SKILL_ACCESS_LEVEL

    assert ROS2Config().introspection_access_level == MUTATOR_ACCESS_LEVEL == 20
    assert ROS2Config(introspection_access_level=SKILL_ACCESS_LEVEL).introspection_access_level == 10
    with pytest.raises(DeclarationError, match="introspection_access_level"):
        ROS2Config(introspection_access_level=SKILL_ACCESS_LEVEL - 1)


def test_the_levels_default_to_opensmis_scale() -> None:
    from opensmi.ros2.server.access import LISTING_ACCESS_LEVEL, SKILL_ACCESS_LEVEL

    config = ROS2Config()

    assert (config.listing_access_level, config.skill_access_level) == (LISTING_ACCESS_LEVEL, SKILL_ACCESS_LEVEL)
    assert (config.listing_access_level, config.skill_access_level) == (0, 10)


def test_every_level_is_overridable_in_the_ros2_file(tmp_path: Path) -> None:
    source = tmp_path / "ros2.toml"
    source.write_text(
        "listing_access_level = 1\nskill_access_level = 2\nintrospection_access_level = 3\n",
        encoding="utf-8",
    )

    config = ROS2Config.from_toml(source)

    assert (config.listing_access_level, config.skill_access_level, config.introspection_access_level) == (1, 2, 3)


def test_the_mutator_level_may_not_drop_below_a_configured_skill_level() -> None:
    with pytest.raises(DeclarationError, match="skill_access_level=30"):
        ROS2Config(skill_access_level=30)


@dataclass(frozen=True)
class _Typed:
    count: int = 0
    ratio: float = 0.0
    label: str = ""
    names: dict[int, str] = field(default_factory=dict)
    inner: ROS2Config = field(default_factory=ROS2Config)


def test_a_value_of_the_wrong_toml_type_names_its_field() -> None:
    with pytest.raises(FieldError, match=r"^label: expected str, got int 5$") as raised:
        from_dict(_Typed, {"label": 5})

    assert raised.value.field == "label"
    assert isinstance(raised.value, ValueError)
    assert isinstance(raised.value, TypeError)


def test_a_boolean_is_not_a_number_and_an_integer_is_a_float() -> None:
    with pytest.raises(FieldError, match="count"):
        from_dict(_Typed, {"count": True})

    assert from_dict(_Typed, {"ratio": 2}).ratio == 2


def test_a_field_inside_a_nested_table_is_named_by_its_path() -> None:
    with pytest.raises(FieldError) as raised:
        from_dict(_Typed, {"inner": {"domain_id": "seven"}})

    assert raised.value.field == "inner.domain_id"


def test_a_table_key_is_converted_to_the_integer_a_field_asks_for() -> None:
    assert from_dict(_Typed, {"names": {"0": "OK", "3": "STALE"}}).names == {0: "OK", 3: "STALE"}


def test_a_table_key_that_is_not_an_integer_is_refused() -> None:
    with pytest.raises(FieldError, match="not an integer"):
        from_dict(_Typed, {"names": {"zero": "OK"}})
