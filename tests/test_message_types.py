# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The ROS2 type model as a pure function of the strings an interface class reports.

The field-named-like-a-constant case needs a real class and is in `test_runtime_jazzy.py`.
"""

from __future__ import annotations

import pytest

from opensmi.ros2.runtime.errors import MessageTypeError
from opensmi.ros2.runtime.messages import (
    ArrayType,
    NestedType,
    PrimitiveType,
    SequenceType,
    StringType,
    parse_field_type,
)


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("double[9]", ArrayType(element=PrimitiveType(name="float64"), length=9)),
        ("uint8[16]", ArrayType(element=PrimitiveType(name="uint8"), length=16)),
        ("apriltag_msgs/Point[4]", ArrayType(element=NestedType(type_name="apriltag_msgs/msg/Point"), length=4)),
        ("string<256>", StringType(wide=False, maximum_length=256)),
        ("string", StringType(wide=False, maximum_length=None)),
        ("wstring", StringType(wide=True, maximum_length=None)),
        ("wstring<8>", StringType(wide=True, maximum_length=8)),
        ("float", PrimitiveType(name="float32")),
        ("double", PrimitiveType(name="float64")),
        ("sequence<double, 3>", SequenceType(element=PrimitiveType(name="float64"), maximum_length=3)),
        ("sequence<uint8>", SequenceType(element=PrimitiveType(name="uint8"), maximum_length=None)),
        ("sequence<octet>", SequenceType(element=PrimitiveType(name="byte"), maximum_length=None)),
        ("sequence<string>", SequenceType(element=StringType(), maximum_length=None)),
        (
            "sequence<diagnostic_msgs/KeyValue>",
            SequenceType(element=NestedType(type_name="diagnostic_msgs/msg/KeyValue")),
        ),
        (
            "sequence<action_msgs/CancelGoal_Request, 1>",
            SequenceType(element=NestedType(type_name="action_msgs/msg/CancelGoal_Request"), maximum_length=1),
        ),
        ("boolean", PrimitiveType(name="bool")),
        ("octet", PrimitiveType(name="byte")),
        ("char", PrimitiveType(name="char")),
        ("int8", PrimitiveType(name="int8")),
        ("uint64", PrimitiveType(name="uint64")),
        ("std_msgs/Header", NestedType(type_name="std_msgs/msg/Header")),
    ],
)
def test_the_type_model_keeps_what_the_interface_class_said(text, expected) -> None:
    assert parse_field_type(text) == expected


def test_a_bounded_string_inside_a_bounded_sequence_keeps_both_bounds() -> None:
    assert parse_field_type("sequence<string<10>, 5>") == SequenceType(
        element=StringType(wide=False, maximum_length=10),
        maximum_length=5,
    )


@pytest.mark.parametrize(
    ("idl", "ros2"),
    [("short", "int16"), ("unsigned short", "uint16"), ("long", "int32"), ("long long", "int64")],
)
def test_the_non_explicit_idl_integer_spellings_reach_the_same_model(idl, ros2) -> None:
    assert parse_field_type(idl) == PrimitiveType(name=ros2)


def test_a_nested_type_takes_the_namespace_its_caller_resolved() -> None:
    parsed = parse_field_type(
        "example_interfaces/Fibonacci_Goal",
        nested_type_name="example_interfaces/action/Fibonacci_Goal",
    )

    assert parsed == NestedType(type_name="example_interfaces/action/Fibonacci_Goal")


@pytest.mark.parametrize("text", ["long double", "map<string, int32>", "", "sequence<>", "double[]"])
def test_a_field_type_this_runtime_does_not_model_is_fatal(text) -> None:
    with pytest.raises(MessageTypeError):
        parse_field_type(text)
