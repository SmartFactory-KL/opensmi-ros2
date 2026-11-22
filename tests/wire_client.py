# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A plain asyncua client in a subprocess, for what only the wire can show.

asyncua keeps a process-global class table, so an in-process client cannot tell a served structure
from a shared class.
"""

from __future__ import annotations

import asyncio
import json
import sys
import textwrap
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from asyncua import ua
    from opensmi.server import Server

__all__ = ["CLIENT", "run_client"]

CLIENT = textwrap.dedent("""
    import asyncio, json, logging, sys
    from asyncua import ua
    from asyncua.client.client import Client

    logging.disable(logging.CRITICAL)

    def element(encoding, body):
        # What a client without the server's types builds: the encoding byte has to be said.
        carried = ua.ExtensionObject(TypeId=ua.NodeId.from_string(encoding), Body=ua.ByteString(bytes.fromhex(body)))
        object.__setattr__(carried, "Encoding", 1)
        return carried

    def stored(value):
        return [[each.TypeId.to_string(), bytes(each.Body).hex()] for each in value]

    async def attempt(node, value, variant_type=ua.VariantType.ExtensionObject):
        try:
            await node.write_value(ua.DataValue(ua.Variant(value, variant_type)))
        except ua.UaStatusCodeError as error:
            return type(error).__name__
        return "Good"

    async def main(url, nodeid, steps):
        out = []
        async with Client(url) as client:
            if steps.get("lock"):
                await client.get_node(steps["lock"]).call_method("2:InitLock")
            node = client.get_node(nodeid)
            out.append(["initial", stored(await node.read_value())])
            if steps.get("definition"):
                datatype = await node.read_data_type()
                definition = await client.get_node(datatype).read_data_type_definition()
                out.append([
                    "definition",
                    datatype.to_string(),
                    await node.read_value_rank(),
                    [
                        [each.Name, each.DataType.to_string(), each.ValueRank, list(each.ArrayDimensions)]
                        for each in definition.Fields
                    ],
                ])
            for label, kind, items in steps["writes"]:
                if kind == "elements":
                    result = await attempt(node, [element(*each) for each in items])
                elif kind == "scalar":
                    result = await attempt(node, element(*items))
                else:
                    result = await attempt(node, items, ua.VariantType.Double)
                value = await node.read_value()
                out.append([label, result, stored(value)])
        print(json.dumps(out))

    asyncio.run(main(sys.argv[1], sys.argv[2], json.loads(sys.argv[3])))
""")


async def run_client(server: Server, user: str, node_id: ua.NodeId, steps: dict[str, Any]) -> list[Any]:
    """Run the client against a server and return what it printed.

    steps: ``lock``, ``definition`` and ``writes`` as ``[label, kind, items]`` triples.
    """
    bserver = server.ua_server.bserver
    assert bserver is not None
    process = await asyncio.create_subprocess_exec(
        sys.executable,
        "-c",
        CLIENT,
        f"opc.tcp://{user}:{user}@127.0.0.1:{bserver.port}/server",
        node_id.to_string(),
        json.dumps(steps),
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    out, err = await asyncio.wait_for(process.communicate(), timeout=60)
    assert process.returncode == 0, err.decode()
    return json.loads(out)
