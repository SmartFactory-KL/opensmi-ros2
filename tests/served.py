# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A real opensmi server in process plus an asyncua client, shared by the server-half tests.

opensmi's default users are named by level (`visitor` 0, `orchestrator` 10, `operator` 20), each
with its name as its password; only `visitor` may log in twice at once.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, Any

from asyncua import ua
from asyncua.client.client import Client
from opensmi.server import BaseComponent, BaseMachine, Server
from opensmi.server.config import ServerConfiguration

from opensmi.ros2.server.component import ROS2ComponentMixin
from opensmi.ros2.server.machine import ROS2MachineMixin
from opensmi.ros2.server.structures import structure_elements, structure_instance

if TYPE_CHECKING:
    from asyncua.common.node import Node

MANUFACTURER = ua.LocalizedText("Technologie-Initiative SmartFactory KL e.V.", "en-US")


class TestComponent(ROS2ComponentMixin, BaseComponent):
    """A ROS2 Component with the identification opensmi asks of every Component, and nothing else."""

    __test__ = False

    async def _write_identification(self) -> None:
        await self.identification.SerialNumber.write(f"test-{self.name}")
        await self.identification.Manufacturer.write(MANUFACTURER)


class TestMachine(ROS2MachineMixin, BaseMachine):
    """A ROS2 Machine that adds the Components it was given, in order.

    :ivar added: The Components, as added; ``added[0]`` is the first.
    """

    __test__ = False

    def __init__(self, *components: BaseComponent, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._to_add = components
        self.added: list[Any] = []

    async def _write_identification(self) -> None:
        await self.identification.SerialNumber.write("test-machine")
        await self.identification.ProductInstanceUri.write("urn:opensmi-ros2:test:machine")
        await self.identification.Manufacturer.write(MANUFACTURER)

    async def _init(self) -> None:
        await super()._init()
        for component in self._to_add:
            self.added.append(await self.add_component(component))


async def serve(machine: BaseMachine) -> Server:
    """Bring up an in-process server on a free port carrying one Machine.

    A Machine that refuses to build leaves no server behind: it is shut down before the refusal
    reaches the test.
    """
    config = ServerConfiguration()
    config.ua_server.endpoint_address = "opc.tcp://127.0.0.1:0/server"
    server = Server(config=config)
    await server.init()
    try:
        await server.add_machine(machine)
        await server.start(blocking=False)
    except BaseException:
        await server.shutdown()
        raise
    return server


# asyncua's idle probe times out behind a long Call and drops the transport; this is longer than any Call a test makes.
WATCHDOG = 60.0


def connected(server: Server, user: str = "operator") -> Client:
    """Return a client for the server, logged in as one of opensmi's default users."""
    bserver = server.ua_server.bserver
    assert bserver is not None
    return Client(url=f"opc.tcp://{user}:{user}@127.0.0.1:{bserver.port}/server", watchdog_intervall=WATCHDOG)


async def alone(server: Server, client: Client, *, within: float = 5.0) -> None:
    """Wait until the client's session is the only one its user has on the server.

    opensmi does not notice a closed session until its next sweep, once a second, and that sweep
    gives up the Lock of every session of the same user, not of the one that closed. A client that
    took the Lock before the sweep removed the previous client's session lost it to that sweep.
    A user who may log in only once, as every user who can take a Lock here, has no other session
    left after this but the client's own. Call it before taking the Lock.
    """
    name = client._username  # pyright: ignore[reportPrivateUsage]
    assert name is not None
    user = server.access_control.users[name]
    loop = asyncio.get_running_loop()
    deadline = loop.time() + within
    while len(user.sessions) != 1:
        assert loop.time() < deadline, f"{name} still has {len(user.sessions)} sessions after {within} s"
        await asyncio.sleep(0.01)


async def names_of(node: Node) -> list[str]:
    """Return the browse names of a node's children."""
    return [(await each.read_browse_name()).Name for each in await node.get_children()]


async def browse(node: Node, *names: str) -> Node:
    """Return the node below another by the browse names between them, as a client browses."""
    for name in names:
        for each in await node.get_children():
            if (await each.read_browse_name()).Name == name:
                node = each
                break
        else:
            message = f"{name!r} is not below {node}"
            raise LookupError(message)
    return node


def leaves(container: Any, prefix: str = "") -> dict[str, Any]:
    """Return every variable below a container that is not a complex one, by the dotted path of its browse names.

    Groups and the children of complex Variables are walked into; ``Odom.Pose.Pose.Position.X`` is a
    field of the message of the Binding ``Odom``. A complex Variable is not listed, only its fields.
    """
    found: dict[str, Any] = {}
    for variable in container:
        found |= _below(variable, f"{prefix}{variable.name}")
    for name, group in container.groups.items():
        found |= leaves(group, f"{prefix}{name}.")
    return found


def _below(variable: Any, path: str) -> dict[str, Any]:
    if not variable.children:
        return {path: variable}
    found: dict[str, Any] = {}
    for name, child in variable.children.items():
        found |= _below(child, f"{path}.{name}")
    return found


def leaf(container: Any, path: str) -> Any:
    """Return the variable at a dotted path below a container."""
    return leaves(container)[path]


def monitoring_nodes(item: Any) -> dict[str, Node]:
    """Return the server-side node of every plain variable below a machinery item's Monitoring, by dotted path."""
    return {name: variable.ua_node for name, variable in leaves(item.monitoring).items()}


async def value_of(node: Node) -> Any:
    """Read a node without raising, which a bad status otherwise makes it do."""
    data_value = await node.read_data_value(raise_on_bad_status=False)
    assert data_value.Value is not None
    return data_value.Value.Value


async def until_value(node: Node, expected: object, *, within: float = 2.0) -> None:
    """Read a node until it carries the value the mirror was given."""
    loop = asyncio.get_running_loop()
    deadline = loop.time() + within
    while True:
        value = await value_of(node)
        if value == expected:
            return
        assert loop.time() < deadline, f"the node holds {value!r}, not {expected!r}"
        await asyncio.sleep(0.005)


def wire_value(variable: Any, value: object) -> object:
    """Return a plan value as the variable stores it: an array of structures as instances of the structure."""
    structure = variable._variable_type
    if structure is None:
        return value
    return [structure_instance(structure, each) for each in value]  # type: ignore[attr-defined]


def elements_of(variable: Any, value: object) -> list[dict[str, object]]:
    """Return an array of structures a variable holds as the maps of their fields."""
    return structure_elements(variable._variable_type, value)


async def until_elements(node: Node, variable: Any, expected: list[dict[str, object]], *, within: float = 2.0) -> None:
    """Read an array of structures until its elements, as the maps of their fields, are the expected ones.

    :param variable: The server-side variable behind the node, which knows the structure it holds.
    """
    loop = asyncio.get_running_loop()
    deadline = loop.time() + within
    while True:
        elements = elements_of(variable, await value_of(node))
        if elements == expected:
            return
        assert loop.time() < deadline, f"the node holds {elements!r}, not {expected!r}"
        await asyncio.sleep(0.005)


async def until_status(node: Node, expected: int, *, within: float = 2.0) -> None:
    """Read a node until it carries a status code."""
    loop = asyncio.get_running_loop()
    deadline = loop.time() + within
    while True:
        status = (await node.read_data_value(raise_on_bad_status=False)).StatusCode
        assert status is not None
        if status.value == expected:
            return
        assert loop.time() < deadline, f"the node reads {status!r}, not {expected:#x}"
        await asyncio.sleep(0.005)
