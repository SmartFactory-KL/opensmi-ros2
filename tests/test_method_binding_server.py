# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A declared service is a Method a UA client calls and reads two answers from.

Status codes are read through a real client, so "service not there" and "service said no" can be told apart.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, Any, cast

import pytest
from asyncua import ua
from asyncua.ua.object_ids import ObjectIds
from asyncua.ua.status_codes import StatusCodes
from opensmi.core.nodesets import DiNodeIds, SmartFactorySkillSetNodeIds
from opensmi.server.base_startup_skill import BaseStartupSkill

from opensmi.ros2.runtime.errors import FieldTypeMismatchError, MissingInterfacePackageError, ServiceUnavailableError
from opensmi.ros2.server.access import SKILL_ACCESS_LEVEL
from opensmi.ros2.server.bindings import MethodBinding
from opensmi.ros2.server.config import ROS2Config
from opensmi.ros2.server.errors import (
    ImplementationClassError,
    NotInvertibleError,
    SuccessFieldError,
    UnprojectableFieldError,
)
from opensmi.ros2.server.method import ROS2ServiceMethod
from opensmi.ros2.server.projection import PlanObject, PlanVariable, generic_encode, join_path
from opensmi.ros2.server.recipes import Functions
from tests import fixture_messages
from tests.fake_runtime import FakeRuntime
from tests.served import TestComponent, TestMachine, alone, connected, leaf, leaves, serve

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Callable

    from asyncua.client.client import Client
    from opensmi.server import Server

    from opensmi.ros2.runtime.messages import Message

SERVICE_TYPES = {
    "fixture_msgs/srv/SetMode": fixture_messages.SetMode,
    "fixture_msgs/srv/Ping": fixture_messages.Ping,
}

SET_MODE = MethodBinding(
    name="SetMode",
    path="/set_mode",
    service_type="fixture_msgs/srv/SetMode",
)

GOOD = ua.StatusCode(ua.UInt32(StatusCodes.Good))


def status(code: int) -> ua.StatusCode:
    return ua.StatusCode(ua.UInt32(code))


def answered(*, success: bool = True, message: str = "", applied: int = 0) -> Callable[[Message], Message]:
    """Return a service that answers the same response whatever it is asked."""
    return lambda _request: fixture_messages.SetModeResponse(success=success, message=message, applied=applied)


class Platform(TestComponent):
    bindings = (SET_MODE,)


@pytest.fixture
def runtime() -> FakeRuntime:
    started = FakeRuntime(domain_id=7, service_types=SERVICE_TYPES)
    started.serve_service("/set_mode", answered(applied=1))
    return started


async def start(runtime: FakeRuntime, platform: TestComponent | None = None) -> tuple[Server, Any]:
    """Bring up a server carrying one Machine with one Component, and hand back both."""
    machine = TestMachine(platform or Platform(name="Platform"), runtime=runtime)
    return await serve(machine), machine


@pytest.fixture
async def served(runtime: FakeRuntime) -> AsyncIterator[tuple[Server, Any]]:
    server, machine = await start(runtime)
    try:
        yield server, machine
    finally:
        await server.shutdown()


async def running(runtime: FakeRuntime, platform: TestComponent) -> AsyncIterator[tuple[Server, Any]]:
    """Serve one Component for the length of a test that declares its own."""
    server, machine = await start(runtime, platform)
    try:
        yield server, machine
    finally:
        await server.shutdown()


def method_of(machine: Any, name: str = "SetMode") -> Any:
    return machine.added[0].method_set[name]


def names(container: Any) -> list[str]:
    return sorted(leaves(container))


async def occupy(client: Client, server: Server, machine: Any) -> None:
    """Take the Machine's Lock, which a call needs as well as an access level."""
    await alone(server, client)
    lock = client.get_node(machine.lock.ua_node.nodeid)
    await lock.call_method(f"{server.ua_get_namespace_index(DiNodeIds)}:InitLock", "the test suite")


async def release(client: Client, server: Server, machine: Any) -> None:
    """Give the Lock back, which closing the session does not do: opensmi keeps it until a timeout."""
    lock = client.get_node(machine.lock.ua_node.nodeid)
    await lock.call_method(f"{server.ua_get_namespace_index(DiNodeIds)}:ExitLock")


async def call(client: Client, server: Server, method: Any) -> ua.StatusCode:
    """Press one Method's Call through a client session and return the status code it answered."""
    browse_name = f"{server.ua_get_namespace_index(SmartFactorySkillSetNodeIds)}:Call"
    try:
        await client.get_node(method.ua_node.nodeid).call_method(browse_name)
    except ua.UaStatusCodeError as answer:
        return status(answer.code)
    return GOOD


async def called(
    server: Server,
    machine: Any,
    name: str = "SetMode",
    *,
    locked: bool = True,
    user: str = "operator",
) -> ua.StatusCode:
    """Call a Method from a session of its own, holding the Lock or not."""
    async with connected(server, user) as client:
        if not locked:
            return await call(client, server, method_of(machine, name))
        await occupy(client, server, machine)
        try:
            return await call(client, server, method_of(machine, name))
        finally:
            await release(client, server, machine)


# -- what a client sees before anything runs -------------------------------------------------


async def test_a_service_is_a_method_on_the_declaring_component(served) -> None:
    _, machine = served

    assert isinstance(method_of(machine), ROS2ServiceMethod)


async def test_a_client_browses_the_method_under_the_components_method_set(served) -> None:
    server, machine = served
    method_set = machine.added[0].method_set.ua_node

    async with connected(server, "visitor") as client:
        children = await client.get_node(method_set.nodeid).get_children()
        browsed = {(await child.read_browse_name()).Name for child in children}

    assert "SetMode" in browsed


async def test_the_request_becomes_the_methods_parameters(served) -> None:
    _, machine = served

    assert names(method_of(machine).parameter_set) == ["Label", "Mode.Detail", "Mode.Mode"]


async def test_the_response_becomes_the_methods_final_result_data(served) -> None:
    _, machine = served

    assert names(method_of(machine).final_result_data) == ["Applied", "Message", "Success"]


async def test_a_variable_is_named_for_the_field_path_and_carries_no_binding_prefix(served) -> None:
    server, machine = served
    node = leaf(method_of(machine).parameter_set, "Mode.Mode").ua_node

    async with connected(server, "visitor") as client:
        browse_name = await client.get_node(node.nodeid).read_browse_name()

    assert browse_name.Name == "Mode"


async def test_a_parameter_carries_the_datatype_and_value_rank_of_its_field(served) -> None:
    server, machine = served
    node = leaf(method_of(machine).parameter_set, "Mode.Mode").ua_node

    async with connected(server, "visitor") as client:
        seen = client.get_node(node.nodeid)
        datatype = await seen.read_data_type()
        value_rank = await seen.read_value_rank()

    assert datatype == ua.NodeId(ua.Int32(ObjectIds.SByte))
    assert value_rank == ua.ValueRank.Scalar


async def test_a_parameter_is_writable_and_a_result_is_not(served) -> None:
    server, machine = served
    method = method_of(machine)

    async with connected(server, "visitor") as client:
        parameter = await client.get_node(leaf(method.parameter_set, "Label").ua_node.nodeid).get_access_level()
        result = await client.get_node(leaf(method.final_result_data, "Applied").ua_node.nodeid).get_access_level()

    assert ua.AccessLevel.CurrentWrite in parameter
    assert ua.AccessLevel.CurrentWrite not in result


async def test_a_client_holding_the_lock_writes_a_parameter_and_the_call_sends_it(served, runtime) -> None:
    server, machine = served
    label = leaf(method_of(machine).parameter_set, "Label").ua_node

    async with connected(server) as client:
        await occupy(client, server, machine)
        await client.get_node(label.nodeid).write_value(ua.Variant("from the client", ua.VariantType.String))
        assert await call(client, server, method_of(machine)) == GOOD

    assert runtime.requests[0][1].label == "from the client"


# -- the gates, which are opensmi's ---------------------------------------------------------------


async def test_a_call_without_the_lock_answers_bad_requires_lock(served, runtime) -> None:
    server, machine = served

    assert await called(server, machine, locked=False) == status(StatusCodes.BadRequiresLock)
    assert runtime.requests == ()


async def test_a_user_below_the_skill_level_is_refused_even_holding_the_lock(served, runtime) -> None:
    server, machine = served

    assert method_of(machine).minimum_access_level == SKILL_ACCESS_LEVEL
    assert await called(server, machine, user="planner") == status(StatusCodes.BadUserAccessDenied)
    assert runtime.requests == ()


async def test_a_method_asks_for_the_skill_level_the_ros2_settings_configure(runtime) -> None:
    machine = TestMachine(
        Platform(name="Platform"),
        runtime=runtime,
        ros2_config=ROS2Config(skill_access_level=20, introspection_access_level=20),
    )
    server = await serve(machine)
    try:
        method = method_of(machine)
        async with connected(server, "visitor") as client:
            level = await client.get_node(method.ua_node.nodeid).get_child(
                f"{server.ua_get_namespace_index(SmartFactorySkillSetNodeIds)}:MinAccessLevel"
            )
            advertised = await level.read_value()

        assert method.minimum_access_level == advertised == 20
        assert await called(server, machine, user="orchestrator") == status(StatusCodes.BadUserAccessDenied)
        assert await called(server, machine, user="operator") == GOOD
    finally:
        await server.shutdown()


async def test_a_user_at_the_skill_level_holding_the_lock_calls(served, runtime) -> None:
    server, machine = served

    assert await called(server, machine, user="orchestrator") == GOOD
    assert len(runtime.requests) == 1


class NeverStarted(BaseStartupSkill):
    """A StartupSkill nobody starts, so the Component it is on never completes its startup."""

    async def _handle_running(self) -> None:
        pass


class Starting(Platform):
    """A Component whose StartupSkill exists and has not been started."""

    async def _init(self) -> None:
        await super()._init()
        await self.add_skill(NeverStarted(handle_status_change=False))


async def test_a_call_before_startup_completed_is_refused_by_opensmi(runtime) -> None:
    # an incomplete startup refuses Methods; opensmi's Call asks the StartupSkill.
    async for server, machine in running(runtime, Starting(name="Platform")):
        assert await called(server, machine) == status(StatusCodes.BadInvalidState)
        assert runtime.requests == ()


# -- the ordinary call -------------------------------------------------------------------------


async def test_calling_sends_the_request_a_client_wrote(served, runtime) -> None:
    server, machine = served
    await leaf(method_of(machine).parameter_set, "Mode.Mode").write(2)
    await leaf(method_of(machine).parameter_set, "Label").write("wide")

    assert await called(server, machine) == GOOD
    assert runtime.requests == (
        (
            "/set_mode",
            fixture_messages.SetModeRequest(
                mode=fixture_messages.Mode(mode=2, detail=""),
                label="wide",
            ),
        ),
    )


async def test_the_response_reaches_the_final_result_data(served) -> None:
    server, machine = served
    method = method_of(machine)

    await called(server, machine)

    assert await method.read_results() == {"Success": True, "Message": "", "Applied": 1}


async def test_a_client_reads_the_response_off_the_final_result_data(served) -> None:
    server, machine = served
    applied = leaf(method_of(machine).final_result_data, "Applied").ua_node

    async with connected(server) as client:
        await occupy(client, server, machine)
        await call(client, server, method_of(machine))
        assert await client.get_node(applied.nodeid).read_value() == 1


# -- the two transport failures, which are the call not happening -------------------------------


async def test_a_service_nobody_serves_answers_a_real_status_code(served, runtime) -> None:
    # Not BadInternalError: a missing service server is not an internal fault.
    server, machine = served
    runtime._services.clear()

    assert await called(server, machine) == status(StatusCodes.BadNotConnected)


async def test_a_service_that_takes_the_request_and_goes_quiet_answers_a_different_one(served, runtime) -> None:
    server, machine = served
    runtime.serve_service("/set_mode", None)

    assert await called(server, machine) == status(StatusCodes.BadTimeout)


async def test_a_transport_failure_leaves_the_result_data_alone(served, runtime) -> None:
    server, machine = served
    await called(server, machine)
    runtime._services.clear()

    assert await called(server, machine) == status(StatusCodes.BadNotConnected)
    assert await method_of(machine).read_results() == {"Success": True, "Message": "", "Applied": 1}


# -- the application failure, which is the service having answered ------------------------------


async def test_a_service_that_said_no_answers_good_plus_the_data_by_default(served, runtime) -> None:
    server, machine = served
    runtime.serve_service("/set_mode", answered(success=False, message="field 3 is out of range"))

    assert await called(server, machine) == GOOD
    assert await method_of(machine).read_results() == {
        "Success": False,
        "Message": "field 3 is out of range",
        "Applied": 0,
    }


class Verdict(TestComponent):
    bindings = (
        MethodBinding(
            name="SetMode",
            path="/set_mode",
            service_type="fixture_msgs/srv/SetMode",
            success_field="success",
        ),
    )


async def test_the_named_response_field_carries_the_verdict(runtime) -> None:
    runtime.serve_service("/set_mode", answered(success=False, message="field 3 is out of range"))
    async for server, machine in running(runtime, Verdict(name="Platform")):
        assert await called(server, machine) == status(StatusCodes.BadRequestNotAllowed)
        assert await method_of(machine).read_results() == {
            "Success": False,
            "Message": "field 3 is out of range",
            "Applied": 0,
        }


async def test_the_named_response_field_saying_yes_is_good(runtime) -> None:
    async for server, machine in running(runtime, Verdict(name="Platform")):
        assert await called(server, machine) == GOOD


async def test_a_field_called_success_is_never_auto_detected(served, runtime) -> None:
    # `success` is never auto-detected; the declaration has to name it.
    server, machine = served
    runtime.serve_service("/set_mode", answered(success=False))

    assert method_of(machine).binding.success_field is None
    assert await called(server, machine) == GOOD


async def test_a_success_field_the_response_does_not_have_is_fatal_at_init(runtime) -> None:
    class WrongVerdict(TestComponent):
        bindings = (
            MethodBinding(
                name="SetMode",
                path="/set_mode",
                service_type="fixture_msgs/srv/SetMode",
                success_field="succeeded",
            ),
        )

    with pytest.raises(SuccessFieldError, match="succeeded"):
        await start(runtime, WrongVerdict(name="Platform"))


async def test_a_success_field_that_is_not_a_bool_is_fatal_at_init(runtime) -> None:
    class NotABoolVerdict(TestComponent):
        bindings = (
            MethodBinding(
                name="SetMode",
                path="/set_mode",
                service_type="fixture_msgs/srv/SetMode",
                success_field="message",
            ),
        )

    with pytest.raises(SuccessFieldError, match="not a bool"):
        await start(runtime, NotABoolVerdict(name="Platform"))


async def test_the_verdict_is_read_off_the_response_rather_than_off_the_address_space(runtime) -> None:
    class Hidden(TestComponent):
        bindings = (
            MethodBinding(
                name="SetMode",
                path="/set_mode",
                service_type="fixture_msgs/srv/SetMode",
                response_exclude=("success",),
                success_field="success",
            ),
        )

    runtime.serve_service("/set_mode", answered(success=False))
    async for server, machine in running(runtime, Hidden(name="Platform")):
        assert names(method_of(machine).final_result_data) == ["Applied", "Message"]
        assert await called(server, machine) == status(StatusCodes.BadRequestNotAllowed)


async def test_a_success_field_named_down_a_path_is_refused_by_name(runtime) -> None:
    class NestedVerdict(TestComponent):
        bindings = (
            MethodBinding(
                name="SetMode",
                path="/set_mode",
                service_type="fixture_msgs/srv/SetMode",
                success_field="mode/ok",
            ),
        )

    with pytest.raises(SuccessFieldError, match="rather than down a path"):
        await start(runtime, NestedVerdict(name="Platform"))


# -- the two absences: no post-call hook, no automatic reset ------------------------------------


async def test_the_parameters_are_never_reset_after_a_call(served, runtime) -> None:
    server, machine = served
    await leaf(method_of(machine).parameter_set, "Mode.Mode").write(2)
    await leaf(method_of(machine).parameter_set, "Label").write("wide")

    await called(server, machine)
    await called(server, machine)

    assert await method_of(machine).read_parameters() == {"Mode.Mode": 2, "Mode.Detail": "", "Label": "wide"}
    assert [request.label for _service, request in runtime.requests] == ["wide", "wide"]


class Logging(ROS2ServiceMethod):
    """A custom class whose post-call work is ``super()`` and then its own lines."""

    after: list[object]

    async def execute_method(self) -> None:
        await super().execute_method()
        self.after = [*getattr(self, "after", []), await leaf(self.final_result_data, "Applied").read()]


class LoggingPlatform(TestComponent):
    bindings = (
        MethodBinding(
            name="SetMode",
            path="/set_mode",
            service_type="fixture_msgs/srv/SetMode",
            method_type=Logging,
        ),
    )


async def test_post_call_work_is_super_plus_your_own_lines(runtime) -> None:
    async for server, machine in running(runtime, LoggingPlatform(name="Platform")):
        await called(server, machine)

        assert method_of(machine).after == [1]


async def test_a_declaration_may_name_a_subclass_of_the_stock_method(runtime) -> None:
    async for _server, machine in running(runtime, LoggingPlatform(name="Platform")):
        assert isinstance(method_of(machine), Logging)


def test_a_class_that_is_not_a_subclass_is_fatal_where_it_is_declared() -> None:
    with pytest.raises(ImplementationClassError, match="ROS2ServiceMethod"):
        MethodBinding(
            name="SetMode",
            path="/set_mode",
            service_type="fixture_msgs/srv/SetMode",
            method_type=dict,
        )


def test_a_string_where_a_class_belongs_is_fatal_too() -> None:
    with pytest.raises(ImplementationClassError):
        MethodBinding(
            name="SetMode",
            path="/set_mode",
            service_type="fixture_msgs/srv/SetMode",
            method_type=cast("Any", "tests.test_method_binding_server.Logging"),
        )


# -- the overridable pure functions --------------------------------------------------------------


def _label_it(request_type: Any, values: Any, context: Any) -> Any:
    """Fill ``label`` through ``overrides``, which is what ``request_from`` is for."""
    return generic_encode(request_type, values, context, overrides={"label": "fixed"})


def _count_the_words(response: Any, _context: Any) -> dict[str, object]:
    return {"Applied": len(response.message.split())}


async def test_the_declaration_may_name_its_own_pure_pair(runtime) -> None:
    class Curated(TestComponent):
        bindings = (
            MethodBinding(
                name="SetMode",
                path="/set_mode",
                service_type="fixture_msgs/srv/SetMode",
                exclude=("label",),
                response_include=("applied",),
                request_from=_label_it,
                response_to=_count_the_words,
            ),
        )

    runtime.serve_service("/set_mode", answered(message="two words here"))
    async for server, machine in running(runtime, Curated(name="Platform")):
        method = method_of(machine)
        assert names(method.parameter_set) == ["Mode.Detail", "Mode.Mode"]

        await called(server, machine)

        assert runtime.requests[0][1].label == "fixed"
        assert await method.read_results() == {"Applied": 3}


# -- the declaration faults that are fatal at init ------------------------------------------------


async def test_a_service_type_whose_package_is_absent_is_fatal_at_init(runtime) -> None:
    class MissingPackage(TestComponent):
        bindings = (MethodBinding(name="SetMode", path="/set_mode", service_type="absent_msgs/srv/Nope"),)

    with pytest.raises(MissingInterfacePackageError):
        await start(runtime, MissingPackage(name="Platform"))

    assert not runtime.started


async def test_a_service_with_an_empty_request_is_declarable(runtime) -> None:
    class Trigger(TestComponent):
        bindings = (MethodBinding(name="Ping", path="/ping", service_type="fixture_msgs/srv/Ping"),)

    runtime.serve_service("/ping", lambda _request: fixture_messages.PingResponse(replies=7))
    async for server, machine in running(runtime, Trigger(name="Platform")):
        method = method_of(machine, "Ping")
        assert names(method.parameter_set) == []
        assert names(method.final_result_data) == ["Replies"]

        await called(server, machine, "Ping")

        assert await method.read_results() == {"Replies": 7}


async def test_a_service_whose_request_and_response_each_wrap_an_empty_message_is_callable() -> None:
    class Clearing(TestComponent):
        bindings = (MethodBinding(name="Clear", path="/clear", service_type="fixture_msgs/srv/Clear"),)

    runtime = FakeRuntime(domain_id=7, service_types={"fixture_msgs/srv/Clear": fixture_messages.Clear})
    runtime.serve_service(
        "/clear", lambda _request: fixture_messages.ClearResponse(response=fixture_messages.Nothing())
    )
    async for server, machine in running(runtime, Clearing(name="Platform")):
        method = method_of(machine, "Clear")
        assert names(method.parameter_set) == []
        assert names(method.final_result_data) == []

        assert await called(server, machine, "Clear") == GOOD
        assert runtime.requests == (("/clear", fixture_messages.Knock(request=fixture_messages.Nothing())),)


async def test_a_selection_that_leaves_the_request_unbuildable_is_fatal_at_init(runtime) -> None:
    # no invented request fields; use request_from.
    class PartialRequest(TestComponent):
        bindings = (
            MethodBinding(
                name="SetMode",
                path="/set_mode",
                service_type="fixture_msgs/srv/SetMode",
                exclude=("label",),
            ),
        )

    with pytest.raises(NotInvertibleError, match="label"):
        await start(runtime, PartialRequest(name="Platform"))


def _as_text(_message_type: Any, context: Any) -> tuple[PlanVariable, ...]:
    """Render Mode's ``mode`` as a String variable -- a curation that cannot be sent back."""
    return (
        PlanVariable(
            name="Mode",
            display_name="mode",
            source=join_path(context.path, "mode"),
            initial="",
            variant_type=ua.VariantType.String,
        ),
    )


def _read_as_text(message: Any, _context: Any) -> dict[str, object]:
    return {"Mode": str(message.mode)}


def _build_from_text(message_type: Any, values: Any, _context: Any) -> Any:
    """Hand the String value straight back, which is where the declaration turns out to be wrong."""
    return message_type(mode=values["Mode"], detail="")


async def test_a_request_type_reaching_a_projection_that_cannot_invert_is_fatal_at_init(runtime) -> None:
    class NoWayBack(Platform):
        projections = (Functions("fixture_msgs/msg/Mode", shaped_by=_as_text, decoded_by=_read_as_text),)

    with pytest.raises(NotInvertibleError, match="Functions"):
        await start(runtime, NoWayBack(name="Platform"))


async def test_a_declaration_whose_own_defaults_could_not_be_sent_is_fatal_at_init(runtime) -> None:
    class TextMode(Platform):
        projections = (
            Functions(
                "fixture_msgs/msg/Mode",
                shaped_by=_as_text,
                decoded_by=_read_as_text,
                encode=_build_from_text,
            ),
        )

    with pytest.raises(FieldTypeMismatchError, match="int8"):
        await start(runtime, TextMode(name="Platform"))

    assert not runtime.started


async def test_a_curated_projection_that_nests_is_refused_at_init(runtime) -> None:
    def nested(_type: Any, _context: Any) -> tuple[PlanObject]:
        child = PlanVariable(name="Mode", source="mode/mode", initial=0, variant_type=ua.VariantType.SByte)
        return (PlanObject(name="mode", children=(child,)),)

    class NestedPlatform(Platform):
        projections = (
            Functions(
                "fixture_msgs/msg/Mode",
                shaped_by=nested,
                decoded_by=lambda _message, _context: {},
                encode=_build_from_text,
            ),
        )

    with pytest.raises(UnprojectableFieldError, match="ParameterSet of the Method 'SetMode'"):
        await start(runtime, NestedPlatform(name="Platform"))


# -- how the containers are built and written ------------------------------------------------------


def _history_shape(_message_type: Any, context: Any) -> tuple[PlanVariable, ...]:
    """Render Ping's response as one list-valued variable, which a generic walk of it is not."""
    return (
        PlanVariable(
            name="history",
            source=join_path(context.path, "replies"),
            initial=[],
            variant_type=ua.VariantType.Int32,
        ),
    )


def _history_decode(message: Any, _context: Any) -> dict[str, object]:
    return {"history": [message.replies, message.replies], "elsewhere": 1}


class History(TestComponent):
    bindings = (MethodBinding(name="Ping", path="/ping", service_type="fixture_msgs/srv/Ping"),)
    projections = (Functions("fixture_msgs/srv/Ping_Response", shaped_by=_history_shape, decoded_by=_history_decode),)


async def test_a_list_valued_result_is_published_one_dimensional_and_written_whole(runtime) -> None:
    runtime.serve_service("/ping", lambda _request: fixture_messages.PingResponse(replies=7))
    async for server, machine in running(runtime, History(name="Platform")):
        history = leaf(method_of(machine, "Ping").final_result_data, "history").ua_node

        async with connected(server) as client:
            await occupy(client, server, machine)
            assert await call(client, server, method_of(machine, "Ping")) == GOOD
            seen = client.get_node(history.nodeid)
            assert await seen.read_value_rank() == ua.ValueRank.OneDimension
            assert await seen.read_array_dimensions() == [0]
            assert await seen.read_data_type() == ua.NodeId(ua.Int32(ObjectIds.Int32))
            assert await seen.read_value() == [7, 7]


async def test_a_decoded_name_with_no_variable_is_skipped_rather_than_failing_the_call(runtime) -> None:
    runtime.serve_service("/ping", lambda _request: fixture_messages.PingResponse(replies=7))
    async for server, machine in running(runtime, History(name="Platform")):
        method = method_of(machine, "Ping")

        assert await called(server, machine, "Ping") == GOOD
        assert names(method.final_result_data) == ["history"]
        assert await method.read_results() == {"history": [7, 7]}


# -- the seam, from the other side -----------------------------------------------------------------


async def test_a_request_for_a_path_with_no_server_raises_rather_than_reporting(runtime) -> None:
    await runtime.start()
    try:
        with pytest.raises(ServiceUnavailableError):
            await runtime.call_service(
                service="/nowhere",
                service_type=fixture_messages.SetMode,
                request=fixture_messages.SetModeRequest(
                    mode=fixture_messages.Mode(mode=0, detail=""),
                    label="",
                ),
            )
    finally:
        await runtime.stop()


async def test_a_type_mismatched_parameter_value_is_refused_on_send(served, runtime) -> None:
    # The send-time guard: an int in a string field would abort the process.
    _, machine = served
    method = method_of(machine)

    values = dict(await method.read_parameters(), Label=7)
    with pytest.raises(FieldTypeMismatchError, match="label"):
        await runtime.call_service(
            service="/set_mode",
            service_type=fixture_messages.SetMode,
            request=await method.build_request(values),
        )


async def test_two_methods_on_one_component_are_two_methods(runtime) -> None:
    class Both(TestComponent):
        bindings = (
            SET_MODE,
            MethodBinding(name="Ping", path="/ping", service_type="fixture_msgs/srv/Ping"),
        )

    runtime.serve_service("/ping", lambda _request: fixture_messages.PingResponse(replies=0))
    async for _server, machine in running(runtime, Both(name="Platform")):
        assert sorted(method.name for method in machine.added[0].method_set) == ["Ping", "SetMode"]


async def test_a_component_with_no_service_declared_gains_no_method(runtime) -> None:
    async for _server, machine in running(runtime, TestComponent(name="Platform")):
        assert len(machine.added[0].method_set) == 0


async def test_two_calls_in_flight_at_once_each_get_their_own_answer(served, runtime) -> None:
    # Two concurrent calls must not lose answers: the body keeps nothing on self.
    server, machine = served
    method = method_of(machine)

    async with connected(server) as client:
        await occupy(client, server, machine)
        answers = await asyncio.gather(call(client, server, method), call(client, server, method))

    assert answers == [GOOD] * 2
    assert len(runtime.requests) == 2
