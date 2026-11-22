# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Building a message dataclass and decoding one from plain objects shaped like rclpy's, without ROS2."""

from __future__ import annotations

import dataclasses
from array import array
from types import SimpleNamespace
from typing import Any

import pytest

from opensmi.ros2.runtime._decode import decode
from opensmi.ros2.runtime.errors import InvalidMessageTypeNameError, MissingInterfacePackageError
from opensmi.ros2.runtime.messages import (
    ACTION_MEMBERS,
    SERVICE_MEMBERS,
    ArrayType,
    FieldDescription,
    Message,
    MessageDescription,
    NestedType,
    PrimitiveType,
    SequenceType,
    StringType,
    build_message_dataclass,
    canonical_type_name,
    message_dataclass,
    service_types,
)


def describe(type_name, **fields):
    """Build a MessageDescription from `name=(field_type, ros2_text)` pairs."""
    return MessageDescription(
        type_name=type_name,
        fields=tuple(
            FieldDescription(name=name, field_type=field_type, ros2_text=text)
            for name, (field_type, text) in fields.items()
        ),
    )


KEY_VALUE = describe(
    "fixture_msgs/msg/KeyValue",
    key=(StringType(), "string"),
    value=(StringType(), "string"),
)

STATUS = describe(
    "fixture_msgs/msg/Status",
    level=(PrimitiveType(name="byte"), "octet"),
    name=(StringType(), "string"),
    covariance=(ArrayType(element=PrimitiveType(name="float64"), length=3), "double[3]"),
    reading=(PrimitiveType(name="float32"), "float"),
    payload=(SequenceType(element=PrimitiveType(name="uint8")), "sequence<uint8>"),
    blob=(SequenceType(element=PrimitiveType(name="byte")), "sequence<octet>"),
    labels=(SequenceType(element=StringType()), "sequence<string>"),
)

PAIRS = describe(
    "fixture_msgs/msg/Pairs",
    first=(NestedType(type_name="fixture_msgs/msg/KeyValue"), "fixture_msgs/KeyValue"),
    rest=(
        SequenceType(element=NestedType(type_name="fixture_msgs/msg/KeyValue")),
        "sequence<fixture_msgs/KeyValue>",
    ),
)


# Runtime-built dataclasses have no static field set, so the checker is told so here.
@pytest.fixture
def key_value() -> Any:
    return build_message_dataclass(KEY_VALUE)


@pytest.fixture
def pairs(key_value) -> Any:
    return build_message_dataclass(PAIRS, resolve=lambda _name: key_value)


def test_a_message_dataclass_carries_its_ros2_facts(key_value) -> None:
    assert key_value.ros2 is KEY_VALUE
    assert key_value.__name__ == "KeyValue"
    assert issubclass(key_value, Message)


def test_a_message_dataclass_is_a_frozen_dataclass(key_value) -> None:
    instance: Any = key_value(key="a", value="b")

    with pytest.raises(dataclasses.FrozenInstanceError):
        instance.key = "c"

    assert dataclasses.is_dataclass(instance)
    assert instance == key_value(key="a", value="b")


def test_the_python_annotations_follow_the_ros2_types() -> None:
    built: Any = build_message_dataclass(STATUS)

    annotations = {field.name: field.type for field in dataclasses.fields(built)}
    assert annotations["level"] is int
    assert annotations["name"] is str
    assert annotations["reading"] is float
    assert annotations["payload"] is bytes
    assert annotations["blob"] is bytes
    assert annotations["covariance"] == tuple[float, ...]
    assert annotations["labels"] == tuple[str, ...]


def test_decoding_turns_every_rclpy_container_into_a_plain_python_value() -> None:
    built: Any = build_message_dataclass(STATUS)
    raw = SimpleNamespace(
        level=b"\x02",
        name="battery",
        covariance=[1.0, 2.0, 3.0],
        reading=0.5,
        payload=array("B", [7, 8, 9]),
        blob=[b"\x01", b"\xff"],
        labels=["a", "b"],
    )

    decoded: Any = decode(raw, built)

    assert decoded.level == 2
    assert decoded.covariance == (1.0, 2.0, 3.0)
    assert decoded.payload == b"\x07\x08\x09"
    assert decoded.blob == b"\x01\xff"
    assert decoded.labels == ("a", "b")


def test_decoding_recurses_into_nested_messages(pairs, key_value) -> None:
    raw = SimpleNamespace(
        first=SimpleNamespace(key="a", value="1"),
        rest=[SimpleNamespace(key="b", value="2"), SimpleNamespace(key="c", value="3")],
    )

    decoded: Any = decode(raw, pairs)

    assert decoded.first == key_value(key="a", value="1")
    assert decoded.rest == (key_value(key="b", value="2"), key_value(key="c", value="3"))
    assert isinstance(decoded.first, Message)
    assert all(isinstance(item, Message) for item in decoded.rest)


def test_an_empty_message_still_builds() -> None:
    built: Any = build_message_dataclass(describe("fixture_msgs/msg/Empty"))

    assert built() == built()


def test_a_declared_type_whose_interface_package_is_missing_is_fatal() -> None:
    # A declaration error is fatal at init: rclpy cannot subscribe without typesupport.
    with pytest.raises(MissingInterfacePackageError):
        message_dataclass("no_such_interface_package/msg/Nothing")


@pytest.mark.parametrize("name", ["Nothing", "a/b/c/d", "std_msgs/nope/String", ""])
def test_a_type_name_of_the_wrong_shape_is_fatal(name) -> None:
    with pytest.raises(InvalidMessageTypeNameError):
        message_dataclass(name)


@pytest.mark.parametrize(
    ("written", "canonical"),
    [
        ("std_msgs/String", "std_msgs/msg/String"),
        ("std_msgs/msg/String", "std_msgs/msg/String"),
        ("example_interfaces/action/Fibonacci", "example_interfaces/action/Fibonacci"),
    ],
)
def test_the_two_ways_of_writing_a_type_name_mean_one_type(written, canonical) -> None:
    assert canonical_type_name(written) == canonical


@pytest.mark.parametrize("name", ["std_srvs/SetBool", "std_srvs/msg/SetBool", "std_srvs/action/SetBool"])
def test_a_service_name_that_is_not_a_service_name_is_fatal(name) -> None:
    with pytest.raises(InvalidMessageTypeNameError, match="package/srv/Name"):
        service_types(name)


def test_a_service_whose_interface_package_is_missing_is_fatal() -> None:
    with pytest.raises(MissingInterfacePackageError):
        service_types("no_such_interface_package/srv/Nothing")


def test_an_actions_three_members_and_a_services_two_are_the_generated_spellings() -> None:
    assert ACTION_MEMBERS == ("Goal", "Feedback", "Result")
    assert SERVICE_MEMBERS == ("Request", "Response")
