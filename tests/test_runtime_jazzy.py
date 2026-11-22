# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Ring 3: the claims only a live ROS2 Jazzy graph can settle.

Covers messages arriving as dataclasses, introspection against `SLOT_TYPES` over every installed
interface, the type-mismatch abort (the one test that kills a process on purpose), Domain ID
isolation, loop affinity, an action round trip, and the TF matched-publisher count that
`TFFacet.satisfied` discounts.
"""

from __future__ import annotations

import asyncio
import contextlib
import importlib
import subprocess
import sys
import threading
import time
import warnings
from typing import Any, cast

import pytest

from opensmi.ros2.runtime import _parser
from opensmi.ros2.runtime.errors import (
    ActionServerUnavailableError,
    FieldTypeMismatchError,
    LoopAffinityError,
    ServiceCallTimeoutError,
    ServiceUnavailableError,
)
from opensmi.ros2.runtime.guard import require_sendable
from opensmi.ros2.runtime.messages import (
    ArrayType,
    ElementType,
    Message,
    NestedType,
    PrimitiveType,
    SequenceType,
    StringType,
    message_dataclass,
)
from opensmi.ros2.runtime.outcomes import GoalStatus
from opensmi.ros2.runtime.subscription import TypeHashMismatchWarning

pytestmark = pytest.mark.jazzy

# Domain IDs used only by this file, so a live robot on the default domain cannot feed it.
DOMAIN = 71
OTHER_DOMAIN = 72
# Generous: discovery on a loaded CI runner is slow, and a tight bound would flake.
ARRIVAL_TIMEOUT = 15.0
# Long enough to conclude a message is not coming, short enough not to dominate the run.
SILENCE_TIMEOUT = 3.0


@pytest.fixture
async def runtime():
    """Yield a started Runtime on a domain of its own, stopped however the test ends."""
    from opensmi.ros2.runtime import Runtime

    started = Runtime(domain_id=DOMAIN, node_name="runtime_under_test")
    await started.start()
    try:
        yield started
    finally:
        if started.started:
            await started.stop()


@pytest.fixture
def talker():
    """Yield a plain rclpy node on the Runtime's domain, for the test to publish from."""
    import rclpy.context
    import rclpy.executors
    import rclpy.node

    context = rclpy.context.Context()
    context.init(args=None, domain_id=DOMAIN)
    node = rclpy.node.Node("talker", context=context)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield node
    finally:
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        node.destroy_node()
        context.try_shutdown()


@contextlib.asynccontextmanager
async def publishing(talker, topic, interface_class, message, *, period=0.1):
    """Publish repeatedly while the block runs, because discovery is not instantaneous."""
    publisher = talker.create_publisher(interface_class, topic, 10)

    async def loop():
        while True:
            publisher.publish(message)
            await asyncio.sleep(period)

    task = asyncio.ensure_future(loop())
    try:
        yield
    finally:
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task
        talker.destroy_publisher(publisher)


async def test_a_message_published_on_a_live_topic_arrives_as_a_message_dataclass(runtime, talker) -> None:
    from std_msgs.msg import String

    handle = await runtime.subscribe(topic="/chatter", message_type="std_msgs/msg/String")

    async with publishing(talker, "/chatter", String, String(data="hello")):
        received = await asyncio.wait_for(handle.receive(), timeout=ARRIVAL_TIMEOUT)

    assert received.data == "hello"
    assert isinstance(received, Message)
    assert type(received).__module__.startswith("opensmi.ros2.")
    assert type(received) is handle.message_type
    assert handle.message_type.ros2.type_name == "std_msgs/msg/String"


async def test_a_nested_field_arrives_as_its_own_message_dataclass(runtime, talker) -> None:
    from geometry_msgs.msg import Point, Pose

    handle = await runtime.subscribe(topic="/pose", message_type="geometry_msgs/Pose")
    pose = Pose()
    pose.position = Point(x=1.0, y=2.0, z=3.0)

    async with publishing(talker, "/pose", Pose, pose):
        received: Any = await asyncio.wait_for(handle.receive(), timeout=ARRIVAL_TIMEOUT)

    assert isinstance(received.position, Message)
    position: Any = received.position
    assert (position.x, position.y, position.z) == (1.0, 2.0, 3.0)


async def test_a_fixed_size_numeric_array_arrives_with_its_elements(runtime, talker) -> None:
    from sensor_msgs.msg import Imu

    handle = await runtime.subscribe(topic="/imu", message_type="sensor_msgs/msg/Imu")
    imu = Imu()
    imu.orientation_covariance = [float(index) for index in range(9)]

    async with publishing(talker, "/imu", Imu, imu):
        received = await asyncio.wait_for(handle.receive(), timeout=ARRIVAL_TIMEOUT)

    assert received.orientation_covariance == tuple(float(index) for index in range(9))
    assert isinstance(received.header.frame_id, str)


def test_a_fixed_size_array_stays_an_array() -> None:
    covariance = _field(message_dataclass("sensor_msgs/msg/Imu"), "orientation_covariance")

    assert covariance.field_type == ArrayType(element=PrimitiveType(name="float64"), length=9)


def test_a_field_named_like_a_constant_survives() -> None:
    from visualization_msgs.msg import Marker

    built = message_dataclass("visualization_msgs/msg/Marker")
    names = {field.name for field in built.ros2.fields}

    assert {"points", "mesh_resource"} <= names
    assert {"POINTS", "MESH_RESOURCE"}.isdisjoint(names)
    assert (Marker.POINTS, Marker.MESH_RESOURCE) == (8, 10)
    assert _field(built, "points").field_type == SequenceType(element=NestedType(type_name="geometry_msgs/msg/Point"))


def test_a_bounded_string_keeps_its_bound() -> None:
    pytest.importorskip("rmw_dds_common.msg")

    namespace = _field(message_dataclass("rmw_dds_common/msg/NodeEntitiesInfo"), "node_namespace")

    assert namespace.field_type == StringType(wide=False, maximum_length=256)


def test_the_float_widths_do_not_collapse() -> None:
    narrow = _field(message_dataclass("std_msgs/msg/Float32"), "data")
    wide = _field(message_dataclass("std_msgs/msg/Float64"), "data")

    assert narrow.field_type == PrimitiveType(name="float32")
    assert wide.field_type == PrimitiveType(name="float64")
    assert narrow.field_type != wide.field_type


def test_the_two_spellings_of_a_type_name_yield_the_same_dataclass() -> None:
    assert message_dataclass("std_msgs/String") is message_dataclass("std_msgs/msg/String")


async def test_a_default_message_is_what_the_generated_class_constructs(runtime) -> None:
    twist = cast(Any, runtime.default_message(message_dataclass("geometry_msgs/msg/Twist")))

    require_sendable(twist)
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)


def test_a_nested_type_from_a_service_keeps_its_namespace() -> None:
    pytest.importorskip("composition_interfaces.srv")

    request = _field(message_dataclass("composition_interfaces/srv/LoadNode_Event"), "request")

    # The field-type string drops the srv segment; only SLOT_TYPES has it.
    assert isinstance(request.field_type, SequenceType)
    assert request.field_type.element == NestedType(type_name="composition_interfaces/srv/LoadNode_Request")


def test_the_type_strings_agree_with_the_structured_slot_types() -> None:
    """Check the introspection path against SLOT_TYPES for every installed interface."""
    import rosidl_parser.definition as definition

    checked = 0
    for type_name, interface_class in _installed_interfaces():
        described = _parser.describe(type_name)
        for field, slot in zip(described.fields, interface_class.SLOT_TYPES, strict=True):
            assert field.field_type == _model_of(slot, definition), f"{type_name}: {field.ros2_text!r}"
            checked += 1
    assert checked > 200, f"only {checked} fields were reachable; the sweep found no interfaces"


async def test_two_runtimes_on_two_domains_do_not_see_each_other(runtime, talker) -> None:
    from std_msgs.msg import String

    from opensmi.ros2.runtime import Runtime

    other = Runtime(domain_id=OTHER_DOMAIN, node_name="runtime_next_door")
    await other.start()
    try:
        near = await runtime.subscribe(topic="/isolated", message_type="std_msgs/msg/String")
        far = await other.subscribe(topic="/isolated", message_type="std_msgs/msg/String")

        async with publishing(talker, "/isolated", String, String(data="near")):
            assert (await asyncio.wait_for(near.receive(), timeout=ARRIVAL_TIMEOUT)).data == "near"
            with pytest.raises(TimeoutError):
                await asyncio.wait_for(far.receive(), timeout=SILENCE_TIMEOUT)
    finally:
        outcome = await other.stop()

    assert outcome.ok
    assert outcome.executor_thread_joined


async def test_subscribing_on_another_event_loop_is_refused(runtime) -> None:
    # A second loop would receive none of this Runtime's payloads while appearing to work.
    def on_another_loop():
        return asyncio.run(runtime.subscribe(topic="/elsewhere", message_type="std_msgs/msg/String"))

    with pytest.raises(LoopAffinityError):
        await asyncio.to_thread(on_another_loop)


async def test_a_publisher_already_on_the_topic_whose_type_hash_differs_warns(runtime, talker) -> None:
    from std_msgs.msg import Int32

    talker.create_publisher(Int32, "/drifted", 10)
    await asyncio.sleep(2.0)

    with pytest.warns(TypeHashMismatchWarning, match="built against"):
        handle = await runtime.subscribe(topic="/drifted", message_type="std_msgs/msg/String")

    (mismatch,) = handle.type_hash_mismatches
    assert mismatch.topic == "/drifted"
    assert mismatch.topic_type == "std_msgs/msg/Int32"
    assert mismatch.published_hash != mismatch.expected_hash


async def test_a_topic_whose_publishers_agree_warns_about_nothing(runtime, talker) -> None:
    from std_msgs.msg import String

    talker.create_publisher(String, "/agreed", 10)
    await asyncio.sleep(2.0)

    with warnings.catch_warnings():
        warnings.simplefilter("error", TypeHashMismatchWarning)
        handle = await runtime.subscribe(topic="/agreed", message_type="std_msgs/msg/String")

    assert handle.type_hash_mismatches == ()


async def test_stopping_from_another_event_loop_is_refused(runtime) -> None:
    # Handle buffers are bound to the opening loop, so a foreign-loop stop would set futures nobody awaits.
    def on_another_loop():
        return asyncio.run(runtime.stop())

    with pytest.raises(LoopAffinityError):
        await asyncio.to_thread(on_another_loop)


async def test_stopping_reports_a_clean_shutdown(runtime) -> None:
    outcome = await runtime.stop()

    assert outcome.ok
    assert outcome.faults == ()
    assert outcome.executor_thread_joined
    assert not runtime.started


def _field(built, name):
    (found,) = [field for field in built.ros2.fields if field.name == name]
    return found


def _installed_interfaces():
    """Yield the canonical type name and class of every installed message, service and action."""
    from rosidl_runtime_py import (
        get_action_interfaces,
        get_message_interfaces,
        get_service_interfaces,
    )

    packages: set[str] = set()
    for listing in (get_message_interfaces(), get_service_interfaces(), get_action_interfaces()):
        packages.update(listing)
    for package in sorted(packages):
        for kind in ("msg", "srv", "action"):
            try:
                submodule = importlib.import_module(f"{package}.{kind}")
            except ImportError:
                continue
            for name in dir(submodule):
                candidate = getattr(submodule, name)
                if not name.startswith("_") and hasattr(candidate, "SLOT_TYPES"):
                    yield f"{package}/{kind}/{name}", candidate


def _model_of(slot, definition):
    """Model a `rosidl_parser` slot type directly, as the reference the parse is checked against."""
    if isinstance(slot, definition.Array):
        return ArrayType(element=_element_model_of(slot.value_type, definition), length=slot.size)
    if isinstance(slot, definition.BoundedSequence):
        return SequenceType(element=_element_model_of(slot.value_type, definition), maximum_length=slot.maximum_size)
    if isinstance(slot, definition.UnboundedSequence):
        return SequenceType(element=_element_model_of(slot.value_type, definition), maximum_length=None)
    if isinstance(slot, definition.NamespacedType):
        return NestedType(type_name="/".join([*slot.namespaces, slot.name]))
    if isinstance(slot, definition.AbstractGenericString):
        return StringType(
            wide=isinstance(slot, definition.AbstractWString),
            maximum_length=slot.maximum_size if slot.has_maximum_size() else None,
        )
    from opensmi.ros2.runtime.messages import _IDL_PRIMITIVES

    return PrimitiveType(name=_IDL_PRIMITIVES[slot.typename])


def _element_model_of(slot, definition) -> ElementType:
    """Model a slot known to be an element type, which an array or a sequence always wraps."""
    return cast("ElementType", _model_of(slot, definition))


async def test_a_match_event_reports_a_publisher_appearing_and_going_away(runtime, talker) -> None:
    from std_msgs.msg import String

    handle = await runtime.subscribe(topic="/matched", message_type="std_msgs/msg/String")
    assert handle.publisher_count == 0

    counts = handle.publisher_counts()
    assert await asyncio.wait_for(counts.__anext__(), timeout=ARRIVAL_TIMEOUT) == 0

    publisher = talker.create_publisher(String, "/matched", 10)
    try:
        assert await asyncio.wait_for(counts.__anext__(), timeout=ARRIVAL_TIMEOUT) == 1
    finally:
        talker.destroy_publisher(publisher)

    assert await asyncio.wait_for(counts.__anext__(), timeout=ARRIVAL_TIMEOUT) == 0
    assert runtime.graph.polls == 0
    await counts.aclose()


async def test_a_match_event_reports_a_publisher_that_was_already_there(runtime, talker) -> None:
    from std_msgs.msg import String

    publisher = talker.create_publisher(String, "/already", 10)
    try:
        handle = await runtime.subscribe(topic="/already", message_type="std_msgs/msg/String")
        deadline = asyncio.get_running_loop().time() + ARRIVAL_TIMEOUT
        while handle.publisher_count == 0:
            assert asyncio.get_running_loop().time() < deadline, "the match event never arrived"
            await asyncio.sleep(0.05)
    finally:
        talker.destroy_publisher(publisher)

    assert handle.publisher_count == 1


async def test_the_graph_facet_reads_a_live_topic_with_its_type(runtime, talker) -> None:
    from std_msgs.msg import String

    publisher = talker.create_publisher(String, "/on_the_graph", 10)
    try:
        deadline = asyncio.get_running_loop().time() + ARRIVAL_TIMEOUT
        while True:
            snapshot = await runtime.graph.refresh()
            if "/on_the_graph" in snapshot.topics:
                break
            assert asyncio.get_running_loop().time() < deadline, "the topic never reached the graph"
            await asyncio.sleep(0.2)
    finally:
        talker.destroy_publisher(publisher)

    assert snapshot.topics["/on_the_graph"] == ("std_msgs/msg/String",)
    assert "/talker/get_parameters" in snapshot.services
    assert runtime.graph.polls > 0


async def test_the_graph_watch_reports_a_topic_appearing_on_a_live_graph(runtime, talker) -> None:
    from std_msgs.msg import String

    watch = runtime.graph.watch()
    try:
        assert runtime.graph.polling
        await asyncio.wait_for(watch.receive(), timeout=ARRIVAL_TIMEOUT)  # the graph as it stands

        publisher = talker.create_publisher(String, "/appears_later", 10)
        try:
            appeared = {}
            deadline = asyncio.get_running_loop().time() + ARRIVAL_TIMEOUT
            while "/appears_later" not in appeared:
                assert asyncio.get_running_loop().time() < deadline, "the topic never appeared"
                delta = await asyncio.wait_for(watch.receive(), timeout=ARRIVAL_TIMEOUT)
                appeared = delta.topics.appeared
        finally:
            talker.destroy_publisher(publisher)
    finally:
        await watch.close()

    assert appeared["/appears_later"] == ("std_msgs/msg/String",)
    assert not runtime.graph.polling  # the last watch closed, so the poll stopped


# -- actions, against a real server -------------------------------------------------------------

FIBONACCI = "example_interfaces/action/Fibonacci"
REJECT, ABORT, LINGER = -1, 99, 50


@pytest.fixture
def server(talker):
    """Yield a real Fibonacci action server on the Runtime's domain, reaching all four goal endings."""
    import rclpy.executors
    from example_interfaces.action import Fibonacci
    from rclpy.action import ActionServer, CancelResponse, GoalResponse

    def on_goal(request):
        return GoalResponse.REJECT if request.order == REJECT else GoalResponse.ACCEPT

    def on_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def on_execute(goal_handle):
        order = goal_handle.request.order
        sequence = [0, 1]
        for _ in range(min(order, LINGER)):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result(sequence=sequence)
            sequence.append(sequence[-1] + sequence[-2])
            goal_handle.publish_feedback(Fibonacci.Feedback(sequence=sequence))
            time.sleep(0.05)
        if order == ABORT:
            goal_handle.abort()
            return Fibonacci.Result(sequence=[])
        goal_handle.succeed()
        return Fibonacci.Result(sequence=sequence)

    # Own multi-threaded executor: a cancel must be answered while execute is still running.
    executor = rclpy.executors.MultiThreadedExecutor(context=talker.context)
    action_server = ActionServer(
        talker,
        Fibonacci,
        "/fibonacci",
        execute_callback=on_execute,
        goal_callback=on_goal,
        cancel_callback=on_cancel,
    )
    executor.add_node(talker)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield action_server
    finally:
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        action_server.destroy()


async def sent(runtime, order: int, depth: int = 10):
    """Send one Fibonacci goal through the Runtime, and return the handle."""
    types = runtime.describe_action(FIBONACCI)
    return await runtime.send_goal(
        action="/fibonacci",
        action_type=types,
        goal=types.goal(order=order),
        depth=depth,
        server_timeout=ARRIVAL_TIMEOUT,
    )


async def test_an_actions_three_types_resolve_out_of_the_installed_package(runtime) -> None:
    types = runtime.describe_action(FIBONACCI)

    assert types.type_name == FIBONACCI
    assert types.goal.ros2.type_name == "example_interfaces/action/Fibonacci_Goal"
    assert [field.name for field in types.goal.ros2.fields] == ["order"]
    assert [field.name for field in types.feedback.ros2.fields] == ["sequence"]
    assert [field.name for field in types.result.ros2.fields] == ["sequence"]


async def test_a_goal_runs_to_a_result_with_feedback_on_the_way(runtime, server) -> None:
    goal = await sent(runtime, order=4)
    try:
        assert goal.accepted

        seen = []

        async def follow():
            seen.extend([feedback.sequence async for feedback in goal])

        following = asyncio.ensure_future(follow())
        outcome = await asyncio.wait_for(goal.result(), timeout=ARRIVAL_TIMEOUT)
        await asyncio.wait_for(following, timeout=ARRIVAL_TIMEOUT)
    finally:
        await goal.close()

    assert outcome.succeeded
    assert outcome.status is GoalStatus.SUCCEEDED
    assert outcome.result.sequence == (0, 1, 1, 2, 3, 5)
    assert seen
    assert all(isinstance(each, tuple) for each in seen)


async def test_a_rejected_goal_comes_back_as_an_outcome_rather_than_an_exception(runtime, server) -> None:
    goal = await sent(runtime, order=REJECT)
    try:
        outcome = await asyncio.wait_for(goal.result(), timeout=ARRIVAL_TIMEOUT)
    finally:
        await goal.close()

    assert not goal.accepted
    assert not outcome.accepted
    assert not outcome.ok


async def test_a_server_that_gives_up_comes_back_aborted(runtime, server) -> None:
    goal = await sent(runtime, order=ABORT)
    try:
        outcome = await asyncio.wait_for(goal.result(), timeout=ARRIVAL_TIMEOUT)
    finally:
        await goal.close()

    assert outcome.aborted
    assert outcome.status is GoalStatus.ABORTED


async def test_cancelling_a_running_goal_ends_it_cancelled(runtime, server) -> None:
    goal = await sent(runtime, order=LINGER)
    try:
        await asyncio.wait_for(goal.receive_feedback(), timeout=ARRIVAL_TIMEOUT)

        await asyncio.wait_for(goal.cancel(), timeout=ARRIVAL_TIMEOUT)
        outcome = await asyncio.wait_for(goal.result(), timeout=ARRIVAL_TIMEOUT)
    finally:
        await goal.close()

    assert outcome.canceled
    assert outcome.status is GoalStatus.CANCELED


async def test_a_path_with_no_action_server_raises_rather_than_reporting(runtime) -> None:
    types = runtime.describe_action(FIBONACCI)

    with pytest.raises(ActionServerUnavailableError):
        await runtime.send_goal(
            action="/nobody_serves_this",
            action_type=types,
            goal=types.goal(order=1),
            server_timeout=SILENCE_TIMEOUT,
        )


def test_the_goal_status_numbers_are_the_ones_ros2_publishes() -> None:
    # A copy of the action_msgs numbers (not imported in the runtime half), so CI notices when the original moves.
    from action_msgs.msg import GoalStatus as ROS2GoalStatus

    published = {
        name.removeprefix("STATUS_"): getattr(ROS2GoalStatus, name)
        for name in dir(ROS2GoalStatus)
        if name.startswith("STATUS_")
    }

    assert {member.name: int(member) for member in GoalStatus} == published


def test_an_unguarded_type_mismatch_aborts_the_process() -> None:
    """Pin why the guard exists: in a subprocess, because an unguarded mismatch kills the process."""
    aborted = subprocess.run(
        [sys.executable, "-c", _ABORTS],
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )

    assert aborted.returncode != 0, "rclpy no longer aborts on a type-mismatched field value"
    assert aborted.returncode != 1, f"it raised rather than aborting:\n{aborted.stderr}"
    assert "Assertion" in aborted.stderr or "Abort" in aborted.stderr, aborted.stderr


def test_the_guard_refuses_the_value_that_would_have_aborted() -> None:
    from opensmi.ros2.runtime.messages import message_dataclass

    float64 = message_dataclass("std_msgs/msg/Float64")

    with pytest.raises(FieldTypeMismatchError, match="float64"):
        require_sendable(cast("Any", float64)(data=1))


_ABORTS = """
import rclpy.context, rclpy.node
from std_msgs.msg import Float64

context = rclpy.context.Context()
context.init(args=None, domain_id=79)
node = rclpy.node.Node("aborts", context=context)
publisher = node.create_publisher(Float64, "/aborts", 1)
message = Float64(data=1)          # accepted: rclpy checks nothing here
print("assigned", flush=True)
publisher.publish(message)         # SIGABRT: PyFloat_Check(field) fails in the C conversion
print("survived", flush=True)
"""


def test_a_byte_scalar_survives_the_round_trip_that_would_otherwise_abort(talker) -> None:
    """The Encoder's inverse of the Decoder's byte conversion: an int handed back as octet aborts at publish."""
    from std_msgs.msg import Byte

    from opensmi.ros2.runtime import _encode
    from opensmi.ros2.runtime.messages import message_dataclass

    payload = cast("Any", message_dataclass("std_msgs/msg/Byte"))(data=7)
    encoded = _encode.encode(payload)

    assert encoded.data == b"\x07"

    publisher = talker.create_publisher(Byte, "/byte_round_trip", 1)
    try:
        publisher.publish(encoded)
    finally:
        talker.destroy_publisher(publisher)


# -- a real service, asked and answered -----------------------------------------------------

SET_BOOL = "std_srvs/srv/SetBool"
REFUSE = False


@pytest.fixture
def service(talker):
    """Yield a real std_srvs/SetBool server on the Runtime's domain that answers success=data."""

    def on_request(request, response):
        response.success = request.data
        response.message = "as asked" if request.data else "refused on purpose"
        return response

    from std_srvs.srv import SetBool

    created = talker.create_service(SetBool, "/set_bool", on_request)
    try:
        yield created
    finally:
        talker.destroy_service(created)


def test_a_services_two_types_resolve_out_of_the_installed_package(runtime) -> None:
    types = runtime.describe_service(SET_BOOL)

    assert types.type_name == SET_BOOL
    assert types.request.ros2.type_name == "std_srvs/srv/SetBool_Request"
    assert [field.name for field in types.request.ros2.fields] == ["data"]
    assert [field.name for field in types.response.ros2.fields] == ["success", "message"]


async def test_a_request_goes_out_and_the_response_comes_back_as_a_message_dataclass(runtime, service) -> None:
    types = runtime.describe_service(SET_BOOL)

    response = await runtime.call_service(
        service="/set_bool",
        service_type=types,
        request=types.request(data=True),
        server_timeout=ARRIVAL_TIMEOUT,
    )

    assert isinstance(response, types.response)
    assert response.success is True
    assert response.message == "as asked"


async def test_a_service_that_said_no_answers_rather_than_raising(runtime, service) -> None:
    types = runtime.describe_service(SET_BOOL)

    response = await runtime.call_service(
        service="/set_bool",
        service_type=types,
        request=types.request(data=REFUSE),
        server_timeout=ARRIVAL_TIMEOUT,
    )

    assert response.success is False
    assert response.message == "refused on purpose"


async def test_a_path_with_no_service_server_raises_rather_than_reporting(runtime) -> None:
    types = runtime.describe_service(SET_BOOL)

    with pytest.raises(ServiceUnavailableError):
        await runtime.call_service(
            service="/nobody_serves_this",
            service_type=types,
            request=types.request(data=True),
            server_timeout=SILENCE_TIMEOUT,
        )


async def test_a_server_that_takes_the_request_and_never_answers_times_out(runtime, talker) -> None:
    # Waiting for a server succeeds here but waiting for the answer fails, hence two timeouts.
    from std_srvs.srv import SetBool

    def never(_request, _response):
        time.sleep(SILENCE_TIMEOUT * 4)
        return _response

    created = talker.create_service(SetBool, "/never_answers", never)
    types = runtime.describe_service(SET_BOOL)
    try:
        with pytest.raises(ServiceCallTimeoutError):
            await runtime.call_service(
                service="/never_answers",
                service_type=types,
                request=types.request(data=True),
                server_timeout=ARRIVAL_TIMEOUT,
                call_timeout=SILENCE_TIMEOUT,
            )
    finally:
        talker.destroy_service(created)


async def test_a_type_mismatched_request_field_is_refused_before_it_reaches_rclpy(runtime, service) -> None:
    types = runtime.describe_service(SET_BOOL)

    with pytest.raises(FieldTypeMismatchError, match="bool"):
        await runtime.call_service(
            service="/set_bool",
            service_type=types,
            request=cast("Any", types.request)(data=1),
            server_timeout=ARRIVAL_TIMEOUT,
        )


async def test_calling_a_service_on_another_event_loop_is_refused(runtime) -> None:
    types = runtime.describe_service(SET_BOOL)
    request = types.request(data=True)

    def elsewhere():
        async def call():
            await runtime.call_service(service="/set_bool", service_type=types, request=request)

        asyncio.run(call())

    with pytest.raises(LoopAffinityError):
        await asyncio.to_thread(elsewhere)


# -- publishing, onto a real topic a real subscriber reads ------------------------------------


@contextlib.asynccontextmanager
async def listening(talker, topic, interface_class, *, depth=10):
    """Yield the list a plain rclpy subscription on the talker's node fills."""
    arrived: list[Any] = []
    subscription = talker.create_subscription(interface_class, topic, arrived.append, depth)
    try:
        yield arrived
    finally:
        talker.destroy_subscription(subscription)


async def _arrives(arrived: list[Any], within: float = ARRIVAL_TIMEOUT) -> Any:
    """Wait for a message to reach a plain rclpy subscriber, or say it never did."""
    deadline = time.monotonic() + within
    while not arrived:
        assert time.monotonic() < deadline, f"nothing reached the subscriber within {within} seconds"
        await asyncio.sleep(0.05)
    return arrived[0]


async def test_a_published_message_reaches_a_real_subscriber_as_a_real_ros2_message(runtime, talker) -> None:
    from geometry_msgs.msg import Twist

    publisher = await runtime.advertise(topic="/cmd_vel", message_type="geometry_msgs/msg/Twist")
    async with listening(talker, "/cmd_vel", Twist) as arrived:
        types = message_dataclass("geometry_msgs/msg/Twist")
        linear = cast("Any", types.ros2_nested["linear"])
        angular = cast("Any", types.ros2_nested["angular"])
        command = cast("Any", types)(
            linear=linear(x=0.25, y=0.0, z=0.0),
            angular=angular(x=0.0, y=0.0, z=-1.5),
        )

        # Repeated because discovery is not instantaneous, as in publishing above.
        async def loop() -> None:
            while True:
                await publisher.publish(command)
                await asyncio.sleep(0.1)

        task = asyncio.ensure_future(loop())
        try:
            message = await _arrives(arrived)
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

    assert isinstance(message, Twist)
    assert (message.linear.x, message.angular.z) == (0.25, -1.5)
    assert publisher.published >= 1
    await publisher.close()


async def test_the_header_stamp_the_adapter_fills_is_the_nodes_own_ros2_clock(runtime, talker) -> None:
    from geometry_msgs.msg import PointStamped

    publisher = await runtime.advertise(topic="/probe_point", message_type="geometry_msgs/msg/PointStamped")
    before = time.time()
    async with listening(talker, "/probe_point", PointStamped) as arrived:
        types = message_dataclass("geometry_msgs/msg/PointStamped")
        header = cast("Any", types.ros2_nested["header"])
        stamp = cast("Any", header.ros2_nested["stamp"])
        point = cast("Any", types.ros2_nested["point"])
        unstamped = cast("Any", types)(
            header=header(stamp=stamp(sec=0, nanosec=0), frame_id="map"),
            point=point(x=1.0, y=2.0, z=3.0),
        )

        async def loop() -> None:
            while True:
                await publisher.publish(unstamped)
                await asyncio.sleep(0.1)

        task = asyncio.ensure_future(loop())
        try:
            message = await _arrives(arrived)
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

    assert message.header.frame_id == "map"
    assert message.header.stamp.sec >= int(before)
    await publisher.close()


async def test_a_type_mismatched_field_is_refused_before_it_reaches_the_publisher(runtime) -> None:
    publisher = await runtime.advertise(topic="/cmd_vel", message_type="geometry_msgs/msg/Twist")
    types = message_dataclass("geometry_msgs/msg/Twist")
    linear = cast("Any", types.ros2_nested["linear"])
    angular = cast("Any", types.ros2_nested["angular"])
    torn = cast("Any", types)(
        linear=linear(x=1, y=0.0, z=0.0),  # an int into a float64: the value that aborts
        angular=angular(x=0.0, y=0.0, z=0.0),
    )

    with pytest.raises(FieldTypeMismatchError, match="linear/x"):
        await publisher.publish(torn)

    assert publisher.published == 0
    await publisher.close()


async def test_stopping_a_runtime_closes_the_publishers_it_opened(runtime) -> None:
    publisher = await runtime.advertise(topic="/cmd_vel", message_type="geometry_msgs/msg/Twist")

    outcome = await runtime.stop()

    assert outcome.ok
    assert publisher.closed


async def test_advertising_on_another_event_loop_is_refused(runtime) -> None:
    def elsewhere():
        async def advertise():
            await runtime.advertise(topic="/cmd_vel", message_type="geometry_msgs/msg/Twist")

        asyncio.run(advertise())

    with pytest.raises(LoopAffinityError):
        await asyncio.to_thread(elsewhere)


# -- ROS2 parameters, on a node that really refuses -------------------------------------------


TUNED_NODE = "/talker"
LIMIT = "speed_limit"
MAXIMUM_SPEED = 100.0


@pytest.fixture
def tuned(talker):
    """Declare parameters on the talker, with a callback that really refuses an out-of-range one."""
    from rcl_interfaces.msg import SetParametersResult

    talker.declare_parameter(LIMIT, 10.0)
    talker.declare_parameter("robot_name", "robot_1")

    def on_set(parameters):
        for each in parameters:
            if each.name == LIMIT and each.value > MAXIMUM_SPEED:
                return SetParametersResult(
                    successful=False,
                    reason=f"{LIMIT} out of range. Min: 0, Max: {MAXIMUM_SPEED:.0f}, value: {each.value:.0f}",
                )
        return SetParametersResult(successful=True)

    talker.add_on_set_parameters_callback(on_set)
    return talker


async def test_a_declared_parameter_reads_back_with_the_type_it_was_declared_as(runtime, tuned) -> None:
    from opensmi.ros2.runtime.parameters import ParameterType

    reading = await runtime.parameters.get(node=TUNED_NODE, name=LIMIT)

    assert reading.present
    assert reading.value is not None
    assert reading.value.type is ParameterType.DOUBLE
    assert reading.value.value == 10.0


async def test_a_name_the_node_does_not_have_answers_rather_than_erroring(runtime, tuned) -> None:
    reading = await runtime.parameters.get(node=TUNED_NODE, name="max_vel_x")

    assert reading.present is False


async def test_one_undeclared_name_blanks_a_whole_batched_read(runtime, tuned) -> None:
    # One name per request: a batched read with one typo answers an empty list and no error.
    from opensmi.ros2.runtime.parameters import GET_PARAMETERS_TYPE

    types = runtime.describe_service(GET_PARAMETERS_TYPE)

    both = await runtime.call_service(
        service=f"{TUNED_NODE}/get_parameters",
        service_type=types,
        request=types.request(names=(LIMIT, "max_vel_x")),
        server_timeout=ARRIVAL_TIMEOUT,
    )
    alone = await runtime.call_service(
        service=f"{TUNED_NODE}/get_parameters",
        service_type=types,
        request=types.request(names=(LIMIT,)),
        server_timeout=ARRIVAL_TIMEOUT,
    )

    assert len(alone.values) == 1
    assert both.values == ()


async def test_a_write_the_node_refuses_comes_back_as_an_answer_with_its_reason(runtime, tuned) -> None:
    from opensmi.ros2.runtime.parameters import ParameterType, ParameterValue

    outcomes = await runtime.parameters.set(
        node=TUNED_NODE,
        values={LIMIT: ParameterValue(type=ParameterType.DOUBLE, value=5000.0)},
    )

    assert outcomes[LIMIT].successful is False
    assert outcomes[LIMIT].reason == f"{LIMIT} out of range. Min: 0, Max: 100, value: 5000"
    reading = await runtime.parameters.get(node=TUNED_NODE, name=LIMIT)
    assert reading.value is not None
    assert reading.value.value == 10.0


async def test_a_valid_entry_in_a_refused_batch_still_lands(runtime, tuned) -> None:
    # Writes are never atomic: this call reports per parameter and keeps what it could take.
    from opensmi.ros2.runtime.parameters import ParameterType, ParameterValue

    outcomes = await runtime.parameters.set(
        node=TUNED_NODE,
        values={
            LIMIT: ParameterValue(type=ParameterType.DOUBLE, value=5000.0),
            "robot_name": ParameterValue(type=ParameterType.STRING, value="robot_2"),
        },
    )

    assert outcomes[LIMIT].successful is False
    assert outcomes["robot_name"].successful is True
    reading = await runtime.parameters.get(node=TUNED_NODE, name="robot_name")
    assert reading.value is not None
    assert reading.value.value == "robot_2"


async def test_a_change_the_node_made_arrives_on_the_parameter_events_topic(runtime, tuned) -> None:
    from rclpy.parameter import Parameter

    from opensmi.ros2.runtime.parameters import ParameterType, ParameterValue

    watch = runtime.parameters.watch(TUNED_NODE)
    try:
        # A change the node makes to itself fires the topic anyway, so there is no echo suppression.
        await asyncio.sleep(1.0)
        tuned.set_parameters([Parameter(LIMIT, Parameter.Type.DOUBLE, 42.0)])

        update = await asyncio.wait_for(watch.receive(), timeout=ARRIVAL_TIMEOUT)
    finally:
        await watch.close()

    assert update.node == TUNED_NODE
    changed = {each.name: each.value for each in update.readings}
    assert changed[LIMIT] == ParameterValue(type=ParameterType.DOUBLE, value=42.0)


async def test_a_call_to_a_node_that_is_not_running_ends_on_our_own_deadline(runtime) -> None:
    # The request future never completes, so the Runtime's own deadline is the only thing that ends it.
    started = time.monotonic()
    with pytest.raises(ServiceUnavailableError):
        await runtime.parameters.get(node="/not_running", name="anything")

    assert time.monotonic() - started >= 1.0


# -- TF: the buffer, the topology, and the lookup that cannot be awaited ------------------------


async def test_the_async_transform_lookup_fails_under_asyncio_unless_it_is_already_cached() -> None:
    """Pin why Runtime._lookup_transform is synchronous: the async lookup fails under asyncio unless cached."""
    import rclpy.time
    from builtin_interfaces.msg import Time as TimeMessage
    from geometry_msgs.msg import TransformStamped
    from rclpy.duration import Duration
    from tf2_ros.buffer import Buffer

    buffer = Buffer(cache_time=Duration(seconds=10))
    stamped = TransformStamped()
    stamped.header.frame_id = "map"
    stamped.header.stamp = TimeMessage(sec=0, nanosec=0)
    stamped.child_frame_id = "base_link"
    stamped.transform.rotation.w = 1.0
    buffer.set_transform_static(stamped, "test")

    cached = await buffer.lookup_transform_async("map", "base_link", rclpy.time.Time())
    assert cached.child_frame_id == "base_link"

    with pytest.raises(RuntimeError, match="bad yield"):
        await buffer.lookup_transform_async("map", "nowhere", rclpy.time.Time())


async def test_the_synchronous_transform_lookup_does_not_block_on_a_chain_it_cannot_complete() -> None:
    # lookup_transform with tf2's zero timeout raises at once, which is safe in a poll on the event loop.
    import rclpy.time
    from rclpy.duration import Duration
    from tf2_ros import LookupException
    from tf2_ros.buffer import Buffer

    buffer = Buffer(cache_time=Duration(seconds=10))

    started = time.monotonic()
    with pytest.raises(LookupException):
        buffer.lookup_transform("map", "base_link", rclpy.time.Time())

    assert time.monotonic() - started < 0.5


async def test_the_tf_facet_reads_both_live_topics_and_answers_a_composed_lookup(runtime, talker) -> None:
    """Our own subscriptions and buffer, no listener: the facet records topology and composes map to tool0."""
    from builtin_interfaces.msg import Time as TimeMessage
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage

    from opensmi.ros2.runtime.tf import TF_STATIC_TOPIC, TF_TOPIC

    def stamped(parent: str, child: str, x: float) -> TransformStamped:
        held = TransformStamped()
        held.header.frame_id = parent
        held.header.stamp = TimeMessage(sec=0, nanosec=0)
        held.child_frame_id = child
        held.transform.translation.x = x
        held.transform.rotation.w = 1.0
        return held

    watch = await runtime.tf.watch()
    try:
        moving = TFMessage(transforms=[stamped("map", "base_link", 2.0)])
        fixed = TFMessage(transforms=[stamped("base_link", "tool0", 0.5)])
        async with (
            publishing(talker, TF_TOPIC, TFMessage, moving),
            publishing(talker, TF_STATIC_TOPIC, TFMessage, fixed),
        ):
            deadline = time.monotonic() + ARRIVAL_TIMEOUT
            while runtime.tf.lookup(target="map", source="tool0") is None:
                assert time.monotonic() < deadline, f"the facet holds {sorted(runtime.tf.frames)}"
                await asyncio.sleep(0.05)
            composed = runtime.tf.lookup(target="map", source="tool0")

        assert runtime.tf.frames["base_link"].parent == "map"
        assert runtime.tf.frames["tool0"].parent == "base_link"
        assert runtime.tf.roots == ("map",)
        assert composed is not None
        assert composed.translation[0] == pytest.approx(2.5)
        # Our own subscriptions close with the last watch; a TransformListener would keep reading.
        assert runtime.tf.reading is True
    finally:
        await watch.close()

    assert runtime.tf.reading is False


async def test_a_subscription_matches_a_publisher_created_on_its_own_node(talker) -> None:
    """Pin the rmw fact that a node's subscription matches its own publisher (see TFFacet.satisfied)."""
    from rclpy.event_handler import SubscriptionEventCallbacks
    from rclpy.qos import QoSProfile
    from tf2_msgs.msg import TFMessage

    counts: list[int] = []
    subscription = talker.create_subscription(
        TFMessage,
        "/own_publisher_probe",
        lambda message: None,
        QoSProfile(depth=10),
        event_callbacks=SubscriptionEventCallbacks(matched=lambda info: counts.append(info.current_count)),
    )
    publisher = None
    try:
        await asyncio.sleep(0.5)
        assert counts == [], "nothing else was publishing on the probe topic"

        publisher = talker.create_publisher(TFMessage, "/own_publisher_probe", QoSProfile(depth=10))
        deadline = time.monotonic() + ARRIVAL_TIMEOUT
        while not counts:
            assert time.monotonic() < deadline, "no match event arrived for our own publisher"
            await asyncio.sleep(0.05)

        assert counts[-1] == 1, "a publisher on the subscribing node itself is counted"
    finally:
        if publisher is not None:
            talker.destroy_publisher(publisher)
        talker.destroy_subscription(subscription)


async def test_a_broadcast_reaches_a_real_subscriber_on_the_transform_topics(runtime, talker) -> None:
    """A FrameLink becomes a real TFMessage on a real topic, read off the installed tf2_msgs."""
    from tf2_msgs.msg import TFMessage

    from opensmi.ros2.runtime.tf import TF_STATIC_TOPIC, TF_TOPIC, FrameLink, Transform

    received: dict[str, list[Any]] = {TF_TOPIC: [], TF_STATIC_TOPIC: []}
    subscriptions = [
        talker.create_subscription(
            TFMessage,
            topic,
            (lambda where: lambda message: received[where].append(message))(topic),
            10,
        )
        for topic in (TF_TOPIC, TF_STATIC_TOPIC)
    ]
    watch = await runtime.tf.watch(broadcasting=True)
    try:
        assert runtime.tf.broadcasting is True
        links = [
            FrameLink(
                child="odom_estimate",
                parent="map",
                transform=Transform(translation=(1.0, 2.0, 3.0), rotation=(0.0, 0.0, 0.0, 1.0)),
                static=False,
            ),
            FrameLink(
                child="payload_mount",
                parent="base_link",
                transform=Transform(translation=(0.0, 0.0, 0.54), rotation=(0.0, 0.0, 0.0, 1.0)),
                static=True,
            ),
        ]
        # Repeated because discovery is not instantaneous.
        deadline = time.monotonic() + ARRIVAL_TIMEOUT
        while not (received[TF_TOPIC] and received[TF_STATIC_TOPIC]):
            assert time.monotonic() < deadline, f"only {received} arrived"
            await runtime.tf.broadcast(links)
            await asyncio.sleep(0.1)

        (moving,) = received[TF_TOPIC][0].transforms
        assert (moving.header.frame_id, moving.child_frame_id) == ("map", "odom_estimate")
        assert (moving.transform.translation.x, moving.transform.translation.z) == (1.0, 3.0)
        assert moving.transform.rotation.w == 1.0
        assert (moving.header.stamp.sec, moving.header.stamp.nanosec) != (0, 0)

        (fixed,) = received[TF_STATIC_TOPIC][0].transforms
        assert fixed.child_frame_id == "payload_mount"
        assert fixed.transform.translation.z == pytest.approx(0.54)
    finally:
        for subscription in subscriptions:
            talker.destroy_subscription(subscription)
        await watch.close()

    assert runtime.tf.broadcasting is False, "the publishers close with the last watch"


# --- QoS --------------------------------------------------------------------------------------
#
# What the pure tests in `test_qos.py` cannot show. Every direction in this package's compatibility
# rule is a claim about DDS, and every one of them is checked here against the middleware rather
# than against a specification -- including the one the bootstrap exists for, which is invisible
# anywhere a publisher cannot actually latch a message.


def test_the_policy_numbers_are_the_middlewares_own() -> None:
    """Policy carries rmw's own values, so the translation is a cast and the numbers must match."""
    from rclpy.qos import QoSPolicyKind

    from opensmi.ros2.runtime.qos import Policy

    for policy in Policy:
        assert int(policy) == int(QoSPolicyKind[policy.name]), f"{policy.name} is not rmw's number"


async def test_publishers_on_reports_what_each_publisher_offers(runtime, talker) -> None:
    """Discovery's input comes from the client library's own call, not from a subprocess."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    from opensmi.ros2.runtime.qos import Durability, Reliability

    publishers = [
        talker.create_publisher(String, "/offers", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
        talker.create_publisher(
            String,
            "/offers",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        ),
    ]
    try:
        deadline = time.monotonic() + ARRIVAL_TIMEOUT
        while len(runtime.publishers_on("/offers")) < len(publishers):
            assert time.monotonic() < deadline, "the publishers never appeared on the graph"
            await asyncio.sleep(0.1)

        offered = {(each.qos.reliability, each.qos.durability) for each in runtime.publishers_on("/offers")}
        assert offered == {
            (Reliability.BEST_EFFORT, Durability.VOLATILE),
            (Reliability.RELIABLE, Durability.TRANSIENT_LOCAL),
        }
        # An absent deadline reads back as INT64_MAX ns, so both spellings count as FOREVER.
        assert all(each.qos.deadline == float("inf") for each in runtime.publishers_on("/offers"))
    finally:
        for publisher in publishers:
            talker.destroy_publisher(publisher)


async def test_a_subscription_discovers_the_most_permissive_of_two_publishers(runtime, talker) -> None:
    """The most permissive profile is the one that receives from both publishers."""
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    from opensmi.ros2.runtime.qos import Reliability

    strict = talker.create_publisher(String, "/mixed", QoSProfile(depth=1))
    loose = talker.create_publisher(String, "/mixed", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        deadline = time.monotonic() + ARRIVAL_TIMEOUT
        while len(runtime.publishers_on("/mixed")) < 2:
            assert time.monotonic() < deadline, "the publishers never appeared on the graph"
            await asyncio.sleep(0.1)

        handle = await runtime.subscribe(topic="/mixed", message_type="std_msgs/msg/String")

        assert handle.qos.reliability is Reliability.BEST_EFFORT

        async def keep_publishing():
            while True:
                loose.publish(String(data="from the best-effort one"))
                await asyncio.sleep(0.1)

        task = asyncio.ensure_future(keep_publishing())
        try:
            received = await asyncio.wait_for(handle.receive(), timeout=ARRIVAL_TIMEOUT)
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task
        assert received.data == "from the best-effort one"
    finally:
        talker.destroy_publisher(strict)
        talker.destroy_publisher(loose)


async def test_a_latched_publisher_already_up_hands_over_what_it_published_before_we_arrived(runtime, talker) -> None:
    """A volatile request would lose a latched publisher's history, so discovery must not default to it."""
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
    from std_msgs.msg import String

    from opensmi.ros2.runtime.qos import Durability, Provenance

    latched = talker.create_publisher(
        String,
        "/latched",
        QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    try:
        latched.publish(String(data="said once, before you arrived"))
        deadline = time.monotonic() + ARRIVAL_TIMEOUT
        while not runtime.publishers_on("/latched"):
            assert time.monotonic() < deadline, "the latched publisher never appeared on the graph"
            await asyncio.sleep(0.1)

        handle = await runtime.subscribe(topic="/latched", message_type="std_msgs/msg/String")

        assert handle.qos.durability is Durability.TRANSIENT_LOCAL, "discovery took the publisher's word"
        assert handle.qos_provenance is Provenance.DISCOVERED
        received = await asyncio.wait_for(handle.receive(), timeout=ARRIVAL_TIMEOUT)
        assert received.data == "said once, before you arrived"
    finally:
        talker.destroy_publisher(latched)


async def test_a_topic_that_was_empty_at_init_re_resolves_on_its_first_publisher(runtime, talker) -> None:
    """With nothing to resolve against the subscription opens permissive, then re-resolves on the first publisher."""
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
    from std_msgs.msg import String

    from opensmi.ros2.runtime.qos import MOST_PERMISSIVE, Durability

    handle = await runtime.subscribe(topic="/empty_then_latched", message_type="std_msgs/msg/String")
    assert handle.qos == MOST_PERMISSIVE, "nothing was publishing, so there was nothing to resolve against"

    latched = talker.create_publisher(
        String,
        "/empty_then_latched",
        QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    try:
        deadline = time.monotonic() + ARRIVAL_TIMEOUT
        while handle.qos == MOST_PERMISSIVE:
            assert time.monotonic() < deadline, "the first publisher never re-resolved the profile"
            await asyncio.sleep(0.1)

        assert handle.qos.durability is Durability.TRANSIENT_LOCAL
        latched.publish(String(data="after the tightening"))
        received = await asyncio.wait_for(handle.receive(), timeout=ARRIVAL_TIMEOUT)
        assert received.data == "after the tightening"
    finally:
        talker.destroy_publisher(latched)


async def test_a_discovered_subscription_relaxes_to_reach_a_stricter_publisher(runtime, talker) -> None:
    """Adaptation end to end: the caller's handle stays the same object across the relaxation."""
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    from opensmi.ros2.runtime.qos import Policy, Reliability

    strict = talker.create_publisher(String, "/relaxes", QoSProfile(depth=1))
    loose = None
    try:
        deadline = time.monotonic() + ARRIVAL_TIMEOUT
        while not runtime.publishers_on("/relaxes"):
            assert time.monotonic() < deadline, "the reliable publisher never appeared"
            await asyncio.sleep(0.1)

        handle = await runtime.subscribe(topic="/relaxes", message_type="std_msgs/msg/String")
        assert handle.qos.reliability is Reliability.RELIABLE

        loose = talker.create_publisher(
            String,
            "/relaxes",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        async def keep_publishing():
            while True:
                loose.publish(String(data="from the strict one's neighbour"))
                await asyncio.sleep(0.1)

        task = asyncio.ensure_future(keep_publishing())
        try:
            received = await asyncio.wait_for(handle.receive(), timeout=ARRIVAL_TIMEOUT)
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

        assert received.data == "from the strict one's neighbour"
        assert handle.qos.reliability is Reliability.BEST_EFFORT
        assert [each.policy for each in handle.qos_incompatibilities] == [Policy.RELIABILITY]
        assert handle.qos_incompatibilities[0].adapted is not None
    finally:
        if loose is not None:
            talker.destroy_publisher(loose)
        talker.destroy_publisher(strict)


async def test_a_declared_profile_is_honoured_exactly_and_reported_when_it_clashes(runtime, talker) -> None:
    """A declared profile stays a contract; a clash is reported, not fatal, so a bench and a robot behave alike."""
    from rclpy.qos import QoSProfile as RclpyQoSProfile
    from rclpy.qos import ReliabilityPolicy
    from std_msgs.msg import String

    from opensmi.ros2.runtime.qos import Policy, Provenance, QoSProfile, Reliability

    declared = QoSProfile(reliability=Reliability.RELIABLE)
    loose = talker.create_publisher(
        String,
        "/declared",
        RclpyQoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    try:
        handle = await runtime.subscribe(topic="/declared", message_type="std_msgs/msg/String", qos=declared)

        deadline = time.monotonic() + ARRIVAL_TIMEOUT
        while not handle.qos_incompatibilities:
            assert time.monotonic() < deadline, "the middleware never reported the clash"
            await asyncio.sleep(0.1)

        assert handle.qos == declared, "a declared profile is never silently changed"
        assert handle.qos_provenance is Provenance.DECLARED
        (reported,) = handle.qos_incompatibilities
        assert reported.policy is Policy.RELIABILITY
        assert reported.adapted is None, "a declared profile does not adapt"
    finally:
        talker.destroy_publisher(loose)


async def test_the_static_transform_publisher_is_transient_local(runtime, talker) -> None:
    """A late subscriber still gets a static frame, so the publisher must be transient local."""
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
    from tf2_msgs.msg import TFMessage

    from opensmi.ros2.runtime.qos import Durability, Provenance
    from opensmi.ros2.runtime.tf import TF_STATIC_TOPIC, FrameLink, Transform

    watch = await runtime.tf.watch(broadcasting=True)
    subscription = None
    try:
        await runtime.tf.broadcast(
            [
                FrameLink(
                    child="payload_mount",
                    parent="base_link",
                    transform=Transform(translation=(0.0, 0.0, 0.54), rotation=(0.0, 0.0, 0.0, 1.0)),
                    static=True,
                )
            ]
        )
        # After the broadcast on purpose: a volatile publisher would have nothing for this one.
        received: list[Any] = []
        subscription = talker.create_subscription(
            TFMessage,
            TF_STATIC_TOPIC,
            received.append,
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        deadline = time.monotonic() + ARRIVAL_TIMEOUT
        while not received:
            assert time.monotonic() < deadline, "a late subscriber received no static frame"
            await asyncio.sleep(0.1)

        (frame,) = received[0].transforms
        assert frame.child_frame_id == "payload_mount"

        (ours,) = [
            each for each in runtime.publishers_on(TF_STATIC_TOPIC) if each.node_name.endswith(runtime.node_name)
        ]
        assert ours.qos.durability is Durability.TRANSIENT_LOCAL
    finally:
        if subscription is not None:
            talker.destroy_subscription(subscription)
        await watch.close()

    assert Provenance.DECLARED.value == "declared"


async def test_the_moving_transform_publisher_stays_volatile(runtime) -> None:
    """A pose from an earlier cycle must not be replayed, so the moving publisher stays volatile."""
    from opensmi.ros2.runtime.qos import Durability
    from opensmi.ros2.runtime.tf import TF_TOPIC

    watch = await runtime.tf.watch(broadcasting=True)
    try:
        deadline = time.monotonic() + ARRIVAL_TIMEOUT
        ours: list[Any] = []
        while not ours:
            assert time.monotonic() < deadline, "our own /tf publisher never appeared on the graph"
            ours = [each for each in runtime.publishers_on(TF_TOPIC) if each.node_name.endswith(runtime.node_name)]
            await asyncio.sleep(0.1)

        assert ours[0].qos.durability is Durability.VOLATILE
    finally:
        await watch.close()
