# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""An OPC UA client that operates the Machine of ``examples/client/machine.toml``, step by step.

It is a plain asyncua client and imports nothing from OpenSMI-ROS2. Against a server started with
``opensmi-ros2 serve examples/client/machine.toml`` it

1. connects and logs in as ``orchestrator``, one of opensmi's default users (the password is the name);
2. browses by name to ``Machines/Demo/Components/Workcell`` and lists its Skills and Methods;
3. tries two calls without the Lock and sees ``BadRequiresLock``;
4. takes the Machine's Lock, resets and starts the ``Fibonacci`` Skill with ``Order = 6``, reads its
   feedback while it runs and its result when it has completed;
5. calls the ``AddTwoInts`` Method and reads the sum;
6. calls the ``Unserved`` Method and starts the ``UnservedFibonacci`` Skill, whose ROS 2 service and
   action nobody serves, and sees ``BadNotConnected`` and a Skill that halts;
7. gives the Lock back, logs in as ``planner`` (access level 5), takes the Lock and is refused a
   Skill with ``BadUserAccessDenied``, because Skills need level 10.

The two ROS 2 nodes the machine file names have to be running, as its header says. The exit status is
0 when every step went as described and 1 when one did not. The tests import the functions below and
run them against a server of their own.

    python examples/client/quickstart.py [--endpoint opc.tcp://localhost:4841/server]
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import logging
import sys
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING

from asyncua import ua
from asyncua.client.client import Client

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Awaitable, Callable

    from asyncua.common.node import Node

__all__ = ["QuickstartError", "main", "run"]

ENDPOINT = "opc.tcp://localhost:4841/server"
"""Where ``opensmi-ros2 serve`` listens with opensmi's default configuration."""
MACHINE = "Demo"
COMPONENT = "Workcell"
ORDER = 6
"""How many Fibonacci numbers the Skill asks the ROS 2 action server for."""
SETTLE = 30.0
"""Seconds to wait for a state, so a hung server fails the script instead of stopping it."""


class QuickstartError(Exception):
    """A step did not go as the walkthrough says it does."""


def say(line: str) -> None:
    """Print one line."""
    sys.stdout.write(line + "\n")
    sys.stdout.flush()


# -- a client's way around the address space ----------------------------------------------------


async def below(node: Node, *names: str) -> Node:
    """Return the node reached by following browse names one level at a time.

    A client finds nodes by browse path and never stores NodeIds, which may change.
    """
    for name in names:
        for child in await node.get_children():
            if (await child.read_browse_name()).Name == name:
                node = child
                break
        else:
            have = [(await each.read_browse_name()).Name for each in await node.get_children()]
            message = f"{name!r} is not below {node}; there are {have}"
            raise QuickstartError(message)
    return node


async def names_below(node: Node) -> list[str]:
    """Return the browse names of a node's children."""
    return [(await each.read_browse_name()).Name for each in await node.get_children()]


async def write(node: Node, value: object) -> None:
    """Write a value with the variant type the server declares for the node."""
    declared = (await node.read_data_value()).Value
    assert declared is not None
    await node.write_value(ua.DataValue(ua.Variant(value, declared.VariantType)))


async def state_of(skill: Node) -> str:
    """Return the name of the state a Skill is in."""
    value = await (await below(skill, "StateMachine", "CurrentState")).read_value()
    return str(getattr(value, "Text", value))


async def until(read: Callable[[], Awaitable[object]], what: str, *, poll: float = 0.1) -> None:
    """Poll until ``read`` answers something truthy, or fail after ``SETTLE`` seconds."""
    deadline = time.monotonic() + SETTLE
    while not await read():
        if time.monotonic() > deadline:
            message = f"still waiting for {what} after {SETTLE:.0f} s"
            raise QuickstartError(message)
        await asyncio.sleep(poll)


async def invoke(parent: Node, method: str, *arguments: object) -> None:
    """Call the Method with a browse name that is a child of ``parent``."""
    await parent.call_method((await below(parent, method)).nodeid, *arguments)


async def press(skill: Node, verb: str) -> None:
    """Call Start, Halt or Reset on the state machine of a Skill's SkillExecution."""
    await invoke(await below(skill, "StateMachine"), verb)


async def refused(call: Awaitable[object], what: str) -> str:
    """Await a call that has to be refused, and return the name of the status code it was refused with."""
    try:
        await call
    except ua.UaStatusCodeError as error:
        name = ua.StatusCode(error.code).name
        say(f"  {what}: {name}")
        return name
    message = f"{what} was not refused"
    raise QuickstartError(message)


# -- 1. connect and log in ---------------------------------------------------------------------


@contextlib.asynccontextmanager
async def session(endpoint: str, user: str) -> AsyncIterator[Client]:
    """Connect to the server and log in. opensmi's default users have their name as password.

    The default endpoint is unencrypted and sends the password as it is, which is fine on a development
    machine; ``docs/deployment.md`` says what to set for anything else.
    """
    client = Client(endpoint)
    client.set_user(user)
    client.set_password(user)
    async with client:
        say(f"connected to {endpoint} as {user}")
        yield client


# -- 2. browse to the Component ---------------------------------------------------------------


async def machine_of(client: Client) -> Node:
    """Return the Machine: ``Objects/Machines/Demo``."""
    return await below(client.nodes.objects, "Machines", MACHINE)


async def component_of(client: Client) -> Node:
    """Return the Component the Bindings are on: ``Machines/Demo/Components/Workcell``."""
    return await below(await machine_of(client), "Components", COMPONENT)


async def survey(component: Node) -> tuple[list[str], list[str]]:
    """Return the names of the Component's Skills and Methods, as the machine file declared them."""
    skills = await names_below(await below(component, "SkillSet"))
    methods = await names_below(await below(component, "MethodSet"))
    say(f"browsed Machines/{MACHINE}/Components/{COMPONENT}: Skills {skills}, Methods {methods}")
    return skills, methods


# -- 3. the Lock --------------------------------------------------------------------------------


async def take_lock(machine: Node, user: str) -> None:
    """Take the Machine's Lock, which every Skill and Method needs on top of an access level.

    opensmi releases it after ten minutes without a call to ``InitLock``, ``BreakLock`` or ``RenewLock``.
    """
    lock = await below(machine, "Lock")
    await invoke(lock, "InitLock", "client quickstart")
    holder = await (await below(lock, "LockingUser")).read_value()
    if holder != user:
        message = f"the Lock is held by {holder!r}, not by {user!r}"
        raise QuickstartError(message)
    say(f"took the Lock as {holder}")


async def give_lock(machine: Node) -> None:
    """Give the Lock back. Closing the session does not do it: opensmi keeps the Lock until it times out."""
    lock = await below(machine, "Lock")
    await invoke(lock, "ExitLock")
    say("gave the Lock back")


# -- 4. a Skill ---------------------------------------------------------------------------------


@dataclass
class Run:
    """One run of the Fibonacci Skill as the client saw it."""

    feedback: list[list[int]]
    """Each different value of the feedback variable while the Skill ran."""
    result: list[int]
    """The result, read from FinalResultData once the Skill had completed."""
    states: list[str]
    """The states the client saw, in order and without repeats."""


async def run_fibonacci(component: Node, order: int, *, poll: float = 0.2) -> Run:
    """Reset and start the Fibonacci Skill and follow it to the end.

    The goal of the ROS 2 action is the Skill's ParameterSet, its feedback the Monitoring and its
    result the FinalResultData. The Skill starts ``Halted``, and ``Reset`` makes it ``Ready``.

    :raises QuickstartError: When the Skill does not complete, or the result is not what was asked for.
    """
    skill = await below(component, "SkillSet", "Fibonacci", "SkillExecution")
    await press(skill, "Reset")
    await until(lambda: _is(skill, "Ready"), "the Skill to be Ready")
    await write(await below(skill, "ParameterSet", "Order"), order)
    await press(skill, "Start")
    say(f"started Fibonacci with Order = {order}")
    seen = Run(feedback=[], result=[], states=[])
    monitored = await below(skill, "Monitoring", "PartialSequence")
    while True:
        state = await state_of(skill)
        if not seen.states or seen.states[-1] != state:
            seen.states.append(state)
        partial = [int(each) for each in await monitored.read_value()]
        if partial and (not seen.feedback or seen.feedback[-1] != partial):
            seen.feedback.append(partial)
            say(f"  feedback PartialSequence = {partial}")
        if state in ("Completed", "Halted"):
            break
        await asyncio.sleep(poll)
    if state != "Completed":
        message = f"Fibonacci ended {state}, not Completed"
        raise QuickstartError(message)
    seen.result = [int(each) for each in await (await below(skill, "FinalResultData", "Sequence")).read_value()]
    say(f"  result Sequence = {seen.result}")
    return seen


async def _is(skill: Node, state: str) -> bool:
    return await state_of(skill) == state


# -- 5. a Method --------------------------------------------------------------------------------


async def add_two_ints(component: Node, a: int, b: int) -> int:
    """Call the AddTwoInts Method: write the request into its ParameterSet, call, read FinalResultData."""
    method = await below(component, "MethodSet", "AddTwoInts")
    await write(await below(method, "ParameterSet", "A"), a)
    await write(await below(method, "ParameterSet", "B"), b)
    await invoke(method, "Call")
    total = int(await (await below(method, "FinalResultData", "Sum")).read_value())
    say(f"called AddTwoInts: {a} + {b} = {total}")
    return total


# -- 6. what a client sees when something is refused or missing ----------------------------------


async def without_the_lock(component: Node) -> list[str]:
    """Reset a Skill and call a Method without holding the Lock. Both answer ``BadRequiresLock``."""
    say("without the Lock:")
    skill = await below(component, "SkillSet", "Fibonacci", "SkillExecution")
    method = await below(component, "MethodSet", "AddTwoInts")
    return [
        await refused(press(skill, "Reset"), "Reset on Fibonacci"),
        await refused(invoke(method, "Call"), "Call on AddTwoInts"),
    ]


async def unserved_method(component: Node) -> str:
    """Call a Method whose ROS 2 service nobody serves. The call is refused with ``BadNotConnected``."""
    say("a service nobody serves:")
    method = await below(component, "MethodSet", "Unserved")
    return await refused(invoke(method, "Call"), "Call on Unserved")


async def unserved_skill(component: Node) -> str:
    """Start a Skill whose ROS 2 action nobody serves. ``Start`` is accepted and the Skill then halts.

    The reason is in the server's Notification and its log, not in the status code.
    """
    say("an action nobody serves:")
    skill = await below(component, "SkillSet", "UnservedFibonacci", "SkillExecution")
    await press(skill, "Reset")
    await until(lambda: _is(skill, "Ready"), "the Skill to be Ready")
    await press(skill, "Start")
    await until(lambda: _is(skill, "Halted"), "the Skill to halt")
    say("  Start was accepted; the Skill went to Halted after the server found no action server")
    return await state_of(skill)


async def below_the_skill_level(client: Client, user: str) -> str:
    """Take the Lock as a user below access level 10 and press Reset on a Skill: ``BadUserAccessDenied``.

    The Lock and the access level are separate gates, so holding the Lock is not enough.
    """
    say(f"{user} holds the Lock but is below the Skill access level:")
    machine = await machine_of(client)
    await take_lock(machine, user)
    try:
        skill = await below(await component_of(client), "SkillSet", "Fibonacci", "SkillExecution")
        return await refused(press(skill, "Reset"), "Reset on Fibonacci")
    finally:
        await give_lock(machine)


# -- the whole walkthrough ----------------------------------------------------------------------


def expect(condition: object, message: str) -> None:
    """Fail the walkthrough when something did not go as it describes."""
    if not condition:
        raise QuickstartError(message)


async def run(endpoint: str = ENDPOINT) -> None:
    """Run every step against a server. Raises ``QuickstartError`` as soon as one does not go as described."""
    async with session(endpoint, "orchestrator") as client:
        machine = await machine_of(client)
        component = await component_of(client)
        skills, methods = await survey(component)
        expect({"Fibonacci", "UnservedFibonacci"} <= set(skills), f"unexpected Skills {skills}")
        expect({"AddTwoInts", "Unserved"} <= set(methods), f"unexpected Methods {methods}")

        expect(await without_the_lock(component) == ["BadRequiresLock"] * 2, "a call without the Lock got through")

        await take_lock(machine, "orchestrator")
        try:
            fibonacci = await run_fibonacci(component, ORDER)
            expect(fibonacci.states[-1] == "Completed", f"states {fibonacci.states}")
            expect(fibonacci.result == [0, 1, 1, 2, 3, 5, 8][: ORDER + 1], f"result {fibonacci.result}")
            expect(await add_two_ints(component, 2, 40) == 42, "AddTwoInts did not add")
            expect(await unserved_method(component) == "BadNotConnected", "the unserved Method was not refused")
            expect(await unserved_skill(component) == "Halted", "the unserved Skill did not halt")
        finally:
            await give_lock(machine)

    async with session(endpoint, "planner") as client:
        expect(await below_the_skill_level(client, "planner") == "BadUserAccessDenied", "planner started a Skill")
    say("every step went as described")


def main(argv: list[str] | None = None) -> int:
    """Run the walkthrough and return the exit status: 0 when it went as described, 1 when it did not."""
    parser = argparse.ArgumentParser(description="Operate the quickstart Machine through OPC UA.")
    parser.add_argument("--endpoint", default=ENDPOINT, help="the server's OPC UA endpoint (default: %(default)s)")
    arguments = parser.parse_args(argv)
    # asyncua warns about plain-text passwords and a shorter session timeout; both are expected here.
    logging.getLogger("asyncua").setLevel(logging.ERROR)
    try:
        asyncio.run(run(arguments.endpoint))
    except (QuickstartError, ua.UaError, OSError, TimeoutError) as error:
        sys.stderr.write(f"FAILED: {type(error).__name__}: {error}\n")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
