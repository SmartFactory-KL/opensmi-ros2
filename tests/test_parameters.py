# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The parameter facet and its `rcl_interfaces` marshalling, without ROS2 or a server.

The dataclasses here carry the real `rcl_interfaces` type names; the Jazzy job checks them against
the installation.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, Any

import pytest

from opensmi.ros2.runtime.errors import ServiceUnavailableError, SubscriptionClosedError
from opensmi.ros2.runtime.messages import (
    FieldDescription,
    MessageDescription,
    NestedType,
    PrimitiveName,
    PrimitiveType,
    SequenceType,
    StringType,
    build_message_dataclass,
)

if TYPE_CHECKING:
    from collections.abc import Mapping
from opensmi.ros2.runtime.parameters import (
    ParameterDescription,
    ParameterFacet,
    ParameterRange,
    ParameterReading,
    ParameterType,
    ParameterUpdate,
    ParameterValue,
    SetOutcome,
    absolute,
    describe_request,
    description_of,
    get_request,
    list_request,
    names_of,
    outcomes_of,
    reading_of,
    set_request,
    update_of,
)

BUILT: dict[str, Any] = {}


def build(type_name: str, **fields: str) -> Any:
    """Build one `rcl_interfaces` dataclass from `name=ros2_text` pairs."""
    described = MessageDescription(
        type_name=type_name,
        fields=tuple(
            FieldDescription(name=name, field_type=field_type(text), ros2_text=text) for name, text in fields.items()
        ),
    )
    BUILT[type_name] = build_message_dataclass(described, resolve=BUILT.__getitem__)
    return BUILT[type_name]


def field_type(text: str):
    """Model the few ROS2 type strings these messages are made of."""
    if text.startswith("sequence<") and "/" in text:
        return SequenceType(element=NestedType(type_name=_nested(text[len("sequence<") : -1])))
    if text.startswith("sequence<"):
        return SequenceType(element=_scalar(text[len("sequence<") : -1]))
    if "/" in text:
        return NestedType(type_name=_nested(text))
    return _scalar(text)


def _scalar(text: str):
    return StringType() if text == "string" else PrimitiveType(name=_PRIMITIVES[text])


def _nested(text: str) -> str:
    package, _, name = text.partition("/")
    return f"{package}/msg/{name}"


_PRIMITIVES: dict[str, PrimitiveName] = {
    "boolean": "bool",
    "double": "float64",
    "int64": "int64",
    "octet": "byte",
    "uint8": "uint8",
    "uint64": "uint64",
}

# ParameterValue as Jazzy declares it.
ParameterValueMessage = build(
    "rcl_interfaces/msg/ParameterValue",
    type="uint8",
    bool_value="boolean",
    integer_value="int64",
    double_value="double",
    string_value="string",
    byte_array_value="sequence<uint8>",
    bool_array_value="sequence<boolean>",
    integer_array_value="sequence<int64>",
    double_array_value="sequence<double>",
    string_array_value="sequence<string>",
)

Parameter = build("rcl_interfaces/msg/Parameter", name="string", value="rcl_interfaces/ParameterValue")

SetParametersResult = build("rcl_interfaces/msg/SetParametersResult", successful="boolean", reason="string")

GetParametersResponse = build(
    "rcl_interfaces/srv/GetParameters_Response",
    values="sequence<rcl_interfaces/ParameterValue>",
)

SetParametersRequest = build(
    "rcl_interfaces/srv/SetParameters_Request",
    parameters="sequence<rcl_interfaces/Parameter>",
)

SetParametersResponse = build(
    "rcl_interfaces/srv/SetParameters_Response",
    results="sequence<rcl_interfaces/SetParametersResult>",
)

GetParametersRequest = build("rcl_interfaces/srv/GetParameters_Request", names="sequence<string>")

ListParametersResult = build(
    "rcl_interfaces/msg/ListParametersResult",
    names="sequence<string>",
    prefixes="sequence<string>",
)

ListParametersRequest = build(
    "rcl_interfaces/srv/ListParameters_Request",
    prefixes="sequence<string>",
    depth="int64",
)

ListParametersResponse = build(
    "rcl_interfaces/srv/ListParameters_Response",
    result="rcl_interfaces/ListParametersResult",
)

FloatingPointRange = build(
    "rcl_interfaces/msg/FloatingPointRange",
    from_value="double",
    to_value="double",
    step="double",
)

IntegerRange = build(
    "rcl_interfaces/msg/IntegerRange",
    from_value="int64",
    to_value="int64",
    step="uint64",
)

ParameterDescriptor = build(
    "rcl_interfaces/msg/ParameterDescriptor",
    name="string",
    type="uint8",
    description="string",
    additional_constraints="string",
    read_only="boolean",
    dynamic_typing="boolean",
    floating_point_range="sequence<rcl_interfaces/FloatingPointRange>",
    integer_range="sequence<rcl_interfaces/IntegerRange>",
)

DescribeParametersRequest = build("rcl_interfaces/srv/DescribeParameters_Request", names="sequence<string>")

DescribeParametersResponse = build(
    "rcl_interfaces/srv/DescribeParameters_Response",
    descriptors="sequence<rcl_interfaces/ParameterDescriptor>",
)

ParameterEvent = build(
    "rcl_interfaces/msg/ParameterEvent",
    node="string",
    new_parameters="sequence<rcl_interfaces/Parameter>",
    changed_parameters="sequence<rcl_interfaces/Parameter>",
    deleted_parameters="sequence<rcl_interfaces/Parameter>",
)

EMPTY_VALUE = {
    "type": 0,
    "bool_value": False,
    "integer_value": 0,
    "double_value": 0.0,
    "string_value": "",
    "byte_array_value": b"",
    "bool_array_value": (),
    "integer_array_value": (),
    "double_array_value": (),
    "string_array_value": (),
}


def raw(parameter_type: ParameterType, value: object) -> Any:
    """Build one `ParameterValue` message holding a value of one type."""
    return ParameterValueMessage(**{**EMPTY_VALUE, "type": parameter_type.code, parameter_type.value_field: value})


def parameter(name: str, parameter_type: ParameterType, value: object) -> Any:
    """Build one `Parameter` message, as an event carries them."""
    return Parameter(name=name, value=raw(parameter_type, value))


# -- the type table ------------------------------------------------------------------------


def test_every_declarable_type_carries_the_ros2_constant_it_is_named_for() -> None:
    assert [each.code for each in ParameterType] == [1, 2, 3, 4, 5, 6, 7, 8, 9]


def test_every_declarable_type_names_a_field_the_union_actually_has() -> None:
    fields = {each.name for each in ParameterValueMessage.ros2.fields}

    assert {each.value_field for each in ParameterType} <= fields


def test_a_node_path_is_spelled_the_way_parameter_events_spells_it() -> None:
    assert absolute("controller_server") == "/controller_server"
    assert absolute("/controller_server") == "/controller_server"
    assert absolute("/nav/controller_server") == "/nav/controller_server"


# -- reading -------------------------------------------------------------------------------


def test_a_read_asks_for_exactly_one_name() -> None:
    request: Any = get_request(GetParametersRequest, "FollowPath.max_vel_x")

    assert request.names == ("FollowPath.max_vel_x",)


def test_an_empty_answer_means_the_node_does_not_have_that_name() -> None:
    # An undeclared name comes back as an empty list with no error.
    reading = reading_of("typo", GetParametersResponse(values=()))

    assert reading == ParameterReading(name="typo", present=False)


def test_a_value_comes_back_typed() -> None:
    reading = reading_of("max_vel_x", GetParametersResponse(values=(raw(ParameterType.DOUBLE, 0.26),)))

    assert reading.present
    assert reading.value == ParameterValue(type=ParameterType.DOUBLE, value=0.26)


def test_a_declared_parameter_holding_nothing_is_present_without_a_value() -> None:
    unset = ParameterValueMessage(**EMPTY_VALUE)

    reading = reading_of("pending", GetParametersResponse(values=(unset,)))

    assert reading == ParameterReading(name="pending", present=True, value=None)


@pytest.mark.parametrize(
    ("parameter_type", "value"),
    [
        (ParameterType.BOOL, True),
        (ParameterType.INTEGER, 7),
        (ParameterType.DOUBLE, 0.5),
        (ParameterType.STRING, "left"),
        (ParameterType.BYTE_ARRAY, b"\x01\x02"),
        (ParameterType.BOOL_ARRAY, (True, False)),
        (ParameterType.INTEGER_ARRAY, (1, 2)),
        (ParameterType.DOUBLE_ARRAY, (0.5, 1.5)),
        (ParameterType.STRING_ARRAY, ("a", "b")),
    ],
)
def test_each_type_survives_the_union_it_travels_in(parameter_type: ParameterType, value: object) -> None:
    reading = reading_of("p", GetParametersResponse(values=(raw(parameter_type, value),)))

    assert reading.value == ParameterValue(type=parameter_type, value=value)


# -- writing -------------------------------------------------------------------------------


def test_a_write_carries_every_name_in_the_order_it_was_given() -> None:
    request: Any = set_request(
        SetParametersRequest,
        {
            "a": ParameterValue(type=ParameterType.DOUBLE, value=1.5),
            "b": ParameterValue(type=ParameterType.STRING, value="left"),
        },
    )

    assert [each.name for each in request.parameters] == ["a", "b"]
    assert request.parameters[0].value.double_value == 1.5
    assert request.parameters[0].value.type == ParameterType.DOUBLE.code
    assert request.parameters[1].value.string_value == "left"


def test_a_write_leaves_every_other_field_of_the_union_at_its_zero() -> None:
    request: Any = set_request(SetParametersRequest, {"a": ParameterValue(type=ParameterType.INTEGER, value=3)})

    value = request.parameters[0].value

    assert value.integer_value == 3
    assert (value.double_value, value.string_value, value.bool_value) == (0.0, "", False)


def test_each_parameter_of_a_write_gets_its_own_answer() -> None:
    response = SetParametersResponse(
        results=(
            SetParametersResult(successful=True, reason=""),
            SetParametersResult(successful=False, reason="out of range. Min: 0, Max: 1, value: 5000"),
        )
    )

    outcomes = outcomes_of(("a", "b"), response)

    assert outcomes["a"] == SetOutcome(successful=True, reason="")
    assert outcomes["b"].successful is False
    assert outcomes["b"].reason == "out of range. Min: 0, Max: 1, value: 5000"


def test_a_name_the_node_answered_nothing_about_is_not_treated_as_landed() -> None:
    one = SetParametersResponse(results=(SetParametersResult(successful=True, reason=""),))

    outcomes = outcomes_of(("a", "b"), one)

    assert outcomes["a"].successful is True
    assert outcomes["b"].successful is False
    assert outcomes["b"].answered is False
    assert outcomes["b"].reason == ""
    assert outcomes["a"].answered is True


# -- listing and describing, which is what makes a parameter declarable ----------------------


def test_a_listing_asks_for_every_name_the_node_has() -> None:
    request: Any = list_request(ListParametersRequest)

    assert request.prefixes == ()
    assert request.depth == 0


def test_a_listing_answers_in_the_order_the_node_declared_them() -> None:
    response = ListParametersResponse(
        result=ListParametersResult(names=("use_sim_time", "FollowPath.max_vel_x"), prefixes=("FollowPath",))
    )

    assert names_of(response) == ("use_sim_time", "FollowPath.max_vel_x")


def test_a_node_with_no_parameters_lists_nothing() -> None:
    assert names_of(ListParametersResponse(result=ListParametersResult(names=(), prefixes=()))) == ()


def test_a_describe_asks_for_exactly_one_name() -> None:
    request: Any = describe_request(DescribeParametersRequest, "FollowPath.max_vel_x")

    assert request.names == ("FollowPath.max_vel_x",)


def test_a_description_carries_the_nodes_own_prose_and_its_constraints() -> None:
    response = DescribeParametersResponse(
        descriptors=(
            ParameterDescriptor(
                name="FollowPath.max_vel_x",
                type=ParameterType.DOUBLE.code,
                description="maximum forward velocity",
                additional_constraints="only positive",
                read_only=False,
                dynamic_typing=False,
                floating_point_range=(),
                integer_range=(),
            ),
        )
    )

    assert description_of("FollowPath.max_vel_x", response) == ParameterDescription(
        name="FollowPath.max_vel_x",
        present=True,
        type=ParameterType.DOUBLE,
        description="maximum forward velocity",
        additional_constraints="only positive",
        read_only=False,
        dynamic_typing=False,
    )


def test_a_description_carries_the_range_the_node_declares() -> None:
    response = DescribeParametersResponse(
        descriptors=(
            ParameterDescriptor(
                name="speed_limit",
                type=ParameterType.DOUBLE.code,
                description="",
                additional_constraints="",
                read_only=False,
                dynamic_typing=False,
                floating_point_range=(FloatingPointRange(from_value=0.0, to_value=100.0, step=0.0),),
                integer_range=(),
            ),
        )
    )

    described = description_of("speed_limit", response)

    assert described.range == ParameterRange(low=0.0, high=100.0, step=0.0)
    assert described.range is not None
    assert described.range.admits(100.0)
    assert not described.range.admits(100.5)


def test_an_integer_range_is_the_same_three_numbers() -> None:
    response = DescribeParametersResponse(
        descriptors=(
            ParameterDescriptor(
                name="retries",
                type=ParameterType.INTEGER.code,
                description="",
                additional_constraints="",
                read_only=True,
                dynamic_typing=False,
                floating_point_range=(),
                integer_range=(IntegerRange(from_value=1, to_value=5, step=1),),
            ),
        )
    )

    described = description_of("retries", response)

    assert described.range == ParameterRange(low=1, high=5, step=1)
    assert described.read_only is True


def test_a_description_without_a_range_has_none() -> None:
    response = DescribeParametersResponse(
        descriptors=(
            ParameterDescriptor(
                name="robot_name",
                type=ParameterType.STRING.code,
                description="",
                additional_constraints="",
                read_only=False,
                dynamic_typing=False,
                floating_point_range=(),
                integer_range=(),
            ),
        )
    )

    assert description_of("robot_name", response).range is None


def test_an_empty_description_means_the_node_does_not_have_that_name() -> None:
    assert description_of("typo", DescribeParametersResponse(descriptors=())) == ParameterDescription(
        name="typo",
        present=False,
    )


def test_a_description_of_an_unset_parameter_has_no_declarable_type() -> None:
    response = DescribeParametersResponse(
        descriptors=(
            ParameterDescriptor(
                name="unset",
                type=0,
                description="",
                additional_constraints="",
                read_only=False,
                dynamic_typing=False,
                floating_point_range=(),
                integer_range=(),
            ),
        )
    )

    assert description_of("unset", response).type is None


# -- events --------------------------------------------------------------------------------


def test_a_new_and_a_changed_parameter_are_the_same_news() -> None:
    update = update_of(
        ParameterEvent(
            node="/controller_server",
            new_parameters=(parameter("a", ParameterType.DOUBLE, 1.0),),
            changed_parameters=(parameter("b", ParameterType.STRING, "left"),),
            deleted_parameters=(),
        )
    )

    assert update.node == "/controller_server"
    assert [(each.name, each.present) for each in update.readings] == [("a", True), ("b", True)]
    assert update.readings[0].value == ParameterValue(type=ParameterType.DOUBLE, value=1.0)


def test_a_deleted_parameter_arrives_in_the_shape_a_missing_one_reads_as() -> None:
    update = update_of(
        ParameterEvent(
            node="/controller_server",
            new_parameters=(),
            changed_parameters=(),
            deleted_parameters=(parameter("gone", ParameterType.DOUBLE, 0.0),),
        )
    )

    assert update.readings == (ParameterReading(name="gone", present=False),)


# -- the facet -----------------------------------------------------------------------------


class Node:
    """A ROS2 node's parameters, as the plumbing behind a facet sees them."""

    def __init__(self, **held: ParameterValue | None) -> None:
        self.held: dict[str, ParameterValue | None] = dict(held)
        self.reads: list[tuple[str, str]] = []
        self.writes: list[tuple[str, dict[str, ParameterValue]]] = []
        self.answering = True
        self.updates: asyncio.Queue[ParameterUpdate] = asyncio.Queue()
        self.streams = 0
        self.closed = 0

    async def read(self, node: str, name: str) -> ParameterReading:
        self.reads.append((node, name))
        self._require_answering(node)
        if name not in self.held:
            return ParameterReading(name=name, present=False)
        return ParameterReading(name=name, present=True, value=self.held[name])

    async def write(self, node: str, values: Mapping[str, ParameterValue]) -> Mapping[str, SetOutcome]:
        self.writes.append((node, dict(values)))
        self._require_answering(node)
        for name, value in values.items():
            self.held[name] = value
        return {name: SetOutcome(successful=True) for name in values}

    async def list(self, node: str) -> tuple[str, ...]:
        self._require_answering(node)
        return tuple(self.held)

    async def describe(self, node: str, name: str) -> ParameterDescription:
        self.reads.append((node, name))
        self._require_answering(node)
        if name not in self.held:
            return ParameterDescription(name=name, present=False)
        value = self.held[name]
        return ParameterDescription(name=name, present=True, type=None if value is None else value.type)

    async def events(self):
        self.streams += 1
        try:
            while True:
                yield await self.updates.get()
        finally:
            self.closed += 1

    def _require_answering(self, node: str) -> None:
        if not self.answering:
            message = f"no service server answered on {node}"
            raise ServiceUnavailableError(message)


def facet_over(node: Node) -> ParameterFacet:
    return ParameterFacet(
        read=node.read,
        write=node.write,
        events=node.events,
        list_names=node.list,
        describe=node.describe,
    )


async def settled() -> None:
    """Let the facet's demux task run."""
    for _ in range(4):
        await asyncio.sleep(0)


async def test_a_read_goes_out_as_one_request_per_name() -> None:
    node = Node(a=ParameterValue(type=ParameterType.DOUBLE, value=1.0), b=None)
    facet = facet_over(node)

    await facet.get(node="controller_server", name="a")
    await facet.get(node="controller_server", name="b")

    assert node.reads == [("/controller_server", "a"), ("/controller_server", "b")]


async def test_listing_answers_every_name_the_node_declares() -> None:
    facet = facet_over(Node(a=ParameterValue(type=ParameterType.DOUBLE, value=1.0), b=None))

    assert await facet.list(node="controller_server") == ("a", "b")


async def test_a_describe_goes_out_as_one_request_per_name() -> None:
    node = Node(a=ParameterValue(type=ParameterType.DOUBLE, value=1.0))
    facet = facet_over(node)

    described = await facet.describe(node="controller_server", name="a")

    assert node.reads == [("/controller_server", "a")]
    assert described.present
    assert described.type == ParameterType.DOUBLE


async def test_a_describe_of_a_name_the_node_does_not_have_is_not_an_error() -> None:
    described = await facet_over(Node(a=None)).describe(node="/controller_server", name="typo")

    assert described == ParameterDescription(name="typo", present=False)


async def test_a_node_that_does_not_answer_raises_on_a_listing() -> None:
    node = Node()
    node.answering = False

    with pytest.raises(ServiceUnavailableError):
        await facet_over(node).list(node="controller_server")


async def test_a_read_of_a_name_the_node_does_not_have_is_not_an_error() -> None:
    facet = facet_over(Node(a=None))

    reading = await facet.get(node="/controller_server", name="typo")

    assert reading.present is False


async def test_a_node_that_does_not_answer_raises_rather_than_reading_blank() -> None:
    node = Node(a=None)
    node.answering = False

    with pytest.raises(ServiceUnavailableError):
        await facet_over(node).get(node="/controller_server", name="a")


async def test_a_write_of_several_parameters_is_one_call_answered_per_parameter() -> None:
    node = Node(a=None, b=None)
    facet = facet_over(node)

    outcomes = await facet.set(
        node="controller_server",
        values={
            "a": ParameterValue(type=ParameterType.DOUBLE, value=1.0),
            "b": ParameterValue(type=ParameterType.DOUBLE, value=2.0),
        },
    )

    assert len(node.writes) == 1
    assert sorted(outcomes) == ["a", "b"]


async def test_the_facet_offers_no_atomic_write() -> None:
    assert not [name for name in dir(ParameterFacet) if "atomic" in name.lower()]


async def test_an_empty_write_is_not_a_call() -> None:
    node = Node()

    assert await facet_over(node).set(node="/controller_server", values={}) == {}
    assert node.writes == []


async def test_nothing_is_subscribed_until_something_watches() -> None:
    node = Node()
    facet = facet_over(node)

    assert facet._watching is False

    watch = facet.watch("/controller_server")
    await settled()

    assert facet._watching is True
    assert node.streams == 1

    await watch.close()

    assert facet._watching is False
    assert node.closed == 1


async def test_one_subscription_serves_every_watch_and_closes_with_the_last() -> None:
    node = Node()
    facet = facet_over(node)
    first, second = facet.watch("/a"), facet.watch("/b")
    await settled()

    await first.close()

    assert facet._watching is True

    await second.close()

    assert facet._watching is False
    assert node.streams == 1


async def test_an_update_reaches_only_the_watch_on_its_own_node() -> None:
    node = Node()
    facet = facet_over(node)
    mine, theirs = facet.watch("controller_server"), facet.watch("/planner_server")
    try:
        node.updates.put_nowait(
            ParameterUpdate(
                node="/controller_server",
                readings=(ParameterReading(name="a", present=True),),
            )
        )
        update = await asyncio.wait_for(mine.receive(), timeout=1)

        assert update.node == "/controller_server"
        with pytest.raises(asyncio.TimeoutError):
            await asyncio.wait_for(theirs.receive(), timeout=0.05)
    finally:
        await mine.close()
        await theirs.close()


async def test_a_closed_watch_refuses_to_be_read_and_ends_its_iteration() -> None:
    facet = facet_over(Node())
    watch = facet.watch("/controller_server")
    await watch.close()

    with pytest.raises(SubscriptionClosedError):
        await watch.receive()
    assert [each async for each in watch] == []


async def test_closing_the_facet_ends_every_watch() -> None:
    node = Node()
    facet = facet_over(node)
    watch = facet.watch("/controller_server")
    await settled()

    await facet.close()

    assert watch.closed is True
    assert facet._watching is False
    assert node.closed == 1


async def test_closing_a_watch_twice_is_not_an_error() -> None:
    facet = facet_over(Node())
    watch = facet.watch("/controller_server")

    await watch.close()
    await watch.close()

    assert watch.closed is True


async def test_a_watch_opened_while_the_last_one_closes_keeps_its_own_subscription() -> None:
    # Take both handles before awaiting: closing the last watch races a new watch's second task.
    node = Node()
    facet = facet_over(node)
    first = facet.watch("/controller_server")
    await settled()

    closing = asyncio.create_task(first.close())
    await asyncio.sleep(0)
    second = facet.watch("/controller_server")
    await closing
    await settled()

    try:
        node.updates.put_nowait(
            ParameterUpdate(
                node="/controller_server",
                readings=(ParameterReading(name="a", present=True),),
            )
        )
        update = await asyncio.wait_for(second.receive(), timeout=1)

        assert update.node == "/controller_server"
        assert node.streams == 2
    finally:
        await second.close()
