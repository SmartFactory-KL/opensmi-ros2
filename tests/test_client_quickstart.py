# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The client quickstart runs against the machine file it is written for, served in process.

The server is ``examples/client/machine.toml`` on the fake Runtime and a free port, so no ROS 2 and no
fixed port is needed. The fake stands in for the two demo nodes the walkthrough starts: a Fibonacci
action server and an AddTwoInts service server. The quickstart's functions run against it through a
real asyncua client.
"""

from __future__ import annotations

import asyncio
import socket
from pathlib import Path
from typing import TYPE_CHECKING, Any

import pytest
from asyncua import ua
from asyncua.client.client import Client
from asyncua.ua.status_codes import StatusCodes

from examples.client import quickstart
from opensmi.ros2.runtime.messages import ActionTypes, PrimitiveType, SequenceType, ServiceTypes
from opensmi.ros2.runtime.outcomes import GoalStatus
from opensmi.ros2.server import load_machine
from tests.fake_runtime import FakeRuntime
from tests.fixture_messages import INT32, build, describe
from tests.served import alone, serve

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Callable

    from opensmi.server import Server

MACHINE_TOML = Path(quickstart.__file__).with_name("machine.toml")

INT64 = (PrimitiveType(name="int64"), "int64")
INT32_SEQUENCE = (SequenceType(element=PrimitiveType(name="int32")), "sequence<int32>")

FibonacciGoal = build(describe("fixture_msgs/action/Fibonacci_Goal", order=INT32))
FibonacciFeedback = build(describe("fixture_msgs/action/Fibonacci_Feedback", partial_sequence=INT32_SEQUENCE))
FibonacciResult = build(describe("fixture_msgs/action/Fibonacci_Result", sequence=INT32_SEQUENCE))
Fibonacci = ActionTypes(
    type_name="fixture_msgs/action/Fibonacci",
    goal=FibonacciGoal,
    feedback=FibonacciFeedback,
    result=FibonacciResult,
)

AddTwoIntsRequest = build(describe("fixture_msgs/srv/AddTwoInts_Request", a=INT64, b=INT64))
AddTwoIntsResponse = build(describe("fixture_msgs/srv/AddTwoInts_Response", sum=INT64))
AddTwoInts = ServiceTypes(
    type_name="fixture_msgs/srv/AddTwoInts",
    request=AddTwoIntsRequest,
    response=AddTwoIntsResponse,
)

FIBONACCI = "/fibonacci"
SEQUENCE = (0, 1, 1, 2, 3, 5, 8)
"""What the fake action server answers for ``Order = 6``."""


def adding(request: Any) -> Any:
    """Answer an AddTwoInts request the way the demo node does."""
    return AddTwoIntsResponse(sum=request.a + request.b)


def fake_runtime(add: Callable[[Any], Any] = adding) -> FakeRuntime:
    """Return a fake Runtime with the two demo nodes answering."""
    fake = FakeRuntime(
        domain_id=42,
        action_types={"action_tutorials_interfaces/action/Fibonacci": Fibonacci},
        service_types={"example_interfaces/srv/AddTwoInts": AddTwoInts},
    )
    fake.serve_action(FIBONACCI)
    fake.serve_service("/add_two_ints", add)
    return fake


async def serve_quickstart(add: Callable[[Any], Any] = adding) -> tuple[Server, FakeRuntime, str]:
    """Serve the quickstart machine file on a free port, and return the server, its fake and its endpoint."""
    runtime = fake_runtime(add)
    server = await serve(load_machine(MACHINE_TOML, runtime_factory=lambda _config: runtime))
    bserver = server.ua_server.bserver
    assert bserver is not None
    return server, runtime, f"opc.tcp://127.0.0.1:{bserver.port}/server"


@pytest.fixture
async def served() -> AsyncIterator[tuple[Server, FakeRuntime, str]]:
    server, runtime, endpoint = await serve_quickstart()
    try:
        yield server, runtime, endpoint
    finally:
        await server.shutdown()


async def answer_fibonacci(runtime: FakeRuntime, *, sequence: tuple[int, ...] = SEQUENCE) -> None:
    """Play the action server: wait for the goal, send two feedbacks the client can tell apart, then the result.

    The pauses are longer than the poll interval the tests give the client, so it reads each feedback.
    """
    async with asyncio.timeout(10):
        while True:
            if runtime.sent_goals:
                break
            await asyncio.sleep(0.01)
    for length in (3, 4):
        runtime.deliver_feedback(FIBONACCI, FibonacciFeedback(partial_sequence=sequence[:length]))
        await asyncio.sleep(0.5)
    runtime.settle_goal(FIBONACCI, status=GoalStatus.SUCCEEDED, result=FibonacciResult(sequence=sequence))


# -- connecting and logging in ---------------------------------------------------------------


async def test_the_client_connects_and_logs_in_as_the_orchestrator(served) -> None:
    _, _, endpoint = served

    async with quickstart.session(endpoint, "orchestrator") as client:
        machine = await quickstart.machine_of(client)

        assert (await machine.read_browse_name()).Name == "Demo"


async def test_a_wrong_password_is_refused_at_login(served) -> None:
    _, _, endpoint = served
    client = Client(endpoint)
    client.set_user("orchestrator")
    client.set_password("not-the-password")

    with pytest.raises(ua.UaStatusCodeError) as refused:
        await client.connect()

    assert refused.value.code == StatusCodes.BadUserAccessDenied


# -- browsing by path ------------------------------------------------------------------------


async def test_the_browse_path_leads_to_the_workcell_and_its_skills_and_methods(served) -> None:
    _, _, endpoint = served

    async with quickstart.session(endpoint, "visitor") as client:
        component = await quickstart.component_of(client)
        skills, methods = await quickstart.survey(component)

        assert (await component.read_browse_name()).Name == "Workcell"
        assert skills == ["Fibonacci", "UnservedFibonacci"]
        assert methods == ["AddTwoInts", "Unserved"]


async def test_a_name_that_is_not_there_fails_the_walkthrough(served) -> None:
    _, _, endpoint = served

    async with quickstart.session(endpoint, "visitor") as client:
        with pytest.raises(quickstart.QuickstartError, match="'Nowhere' is not below"):
            await quickstart.below(await quickstart.component_of(client), "SkillSet", "Nowhere")


# -- the Lock and the Skill ------------------------------------------------------------------


async def test_a_reset_and_a_call_without_the_lock_answer_bad_requires_lock(served) -> None:
    _, runtime, endpoint = served

    async with quickstart.session(endpoint, "orchestrator") as client:
        refusals = await quickstart.without_the_lock(await quickstart.component_of(client))

    assert refusals == ["BadRequiresLock", "BadRequiresLock"]
    assert runtime.requests == ()


async def test_the_lock_is_taken_and_given_back(served) -> None:
    server, _, endpoint = served

    async with quickstart.session(endpoint, "orchestrator") as client:
        await alone(server, client)
        machine = await quickstart.machine_of(client)
        locked = await quickstart.below(machine, "Lock", "Locked")

        await quickstart.take_lock(machine, "orchestrator")
        assert await locked.read_value() is True

        await quickstart.give_lock(machine)
        assert await locked.read_value() is False


async def test_a_skill_runs_with_feedback_and_a_result(served) -> None:
    server, runtime, endpoint = served

    async with quickstart.session(endpoint, "orchestrator") as client:
        await alone(server, client)
        machine = await quickstart.machine_of(client)
        component = await quickstart.component_of(client)
        await quickstart.take_lock(machine, "orchestrator")
        try:
            driver = asyncio.create_task(answer_fibonacci(runtime))
            seen = await quickstart.run_fibonacci(component, quickstart.ORDER, poll=0.05)
            await driver
        finally:
            await quickstart.give_lock(machine)

    [(action, goal)] = runtime.sent_goals
    assert (action, goal.order) == (FIBONACCI, quickstart.ORDER)
    assert seen.feedback == [[0, 1, 1], [0, 1, 1, 2]]
    assert seen.result == list(SEQUENCE)
    assert "Running" in seen.states
    assert seen.states[-1] == "Completed"


async def test_a_method_call_returns_the_response(served) -> None:
    server, runtime, endpoint = served

    async with quickstart.session(endpoint, "orchestrator") as client:
        await alone(server, client)
        machine = await quickstart.machine_of(client)
        await quickstart.take_lock(machine, "orchestrator")
        try:
            total = await quickstart.add_two_ints(await quickstart.component_of(client), 2, 40)
        finally:
            await quickstart.give_lock(machine)

    assert total == 42
    [(service, request)] = runtime.requests
    assert (service, request.a, request.b) == ("/add_two_ints", 2, 40)


# -- what is refused or missing --------------------------------------------------------------


async def test_a_service_nobody_serves_answers_bad_not_connected(served) -> None:
    server, runtime, endpoint = served

    async with quickstart.session(endpoint, "orchestrator") as client:
        await alone(server, client)
        machine = await quickstart.machine_of(client)
        await quickstart.take_lock(machine, "orchestrator")
        try:
            refusal = await quickstart.unserved_method(await quickstart.component_of(client))
        finally:
            await quickstart.give_lock(machine)

    assert refusal == "BadNotConnected"
    assert runtime.requests == ()


async def test_an_action_nobody_serves_is_accepted_and_the_skill_halts(served) -> None:
    server, runtime, endpoint = served

    async with quickstart.session(endpoint, "orchestrator") as client:
        await alone(server, client)
        machine = await quickstart.machine_of(client)
        await quickstart.take_lock(machine, "orchestrator")
        try:
            state = await quickstart.unserved_skill(await quickstart.component_of(client))
        finally:
            await quickstart.give_lock(machine)

    assert state == "Halted"
    assert runtime.sent_goals == ()


async def test_a_user_below_the_skill_level_holding_the_lock_is_refused(served) -> None:
    server, runtime, endpoint = served

    async with quickstart.session(endpoint, "planner") as client:
        await alone(server, client)

        assert await quickstart.below_the_skill_level(client, "planner") == "BadUserAccessDenied"

    assert runtime.sent_goals == ()


# -- the whole walkthrough -------------------------------------------------------------------


async def test_the_walkthrough_runs_start_to_finish(served, capsys) -> None:
    _, runtime, endpoint = served

    driver = asyncio.create_task(answer_fibonacci(runtime))
    await quickstart.run(endpoint)
    await driver

    said = capsys.readouterr().out
    assert "connected to" in said
    assert "feedback PartialSequence" in said
    assert "every step went as described" in said
    assert [goal.order for _, goal in runtime.sent_goals] == [quickstart.ORDER]


async def test_the_walkthrough_fails_when_a_step_does_not_go_as_described() -> None:
    def wrong(request: Any) -> Any:
        return AddTwoIntsResponse(sum=request.a + request.b + 1)

    server, runtime, endpoint = await serve_quickstart(wrong)
    try:
        driver = asyncio.create_task(answer_fibonacci(runtime))
        with pytest.raises(quickstart.QuickstartError, match="AddTwoInts did not add"):
            await quickstart.run(endpoint)
        await driver
    finally:
        await server.shutdown()


def test_the_exit_status_is_1_when_nothing_answers(capsys) -> None:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        free = probe.getsockname()[1]

    status = quickstart.main(["--endpoint", f"opc.tcp://127.0.0.1:{free}/server"])

    assert status == 1
    assert "FAILED" in capsys.readouterr().err
