# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The command line: ``opensmi-ros2 serve machine.toml`` and ``python -m opensmi.ros2``.

Both call :func:`main`, which reads the file with :func:`~opensmi.ros2.server.loader.load_machine`,
builds opensmi's ``Server``, adds the Machine and serves until Ctrl+C or SIGTERM.

.. code-block:: console

    opensmi-ros2 serve machine.toml
    opensmi-ros2 serve machine.toml --server-config other-server.toml

``rclpy`` is imported only once the whole file has been validated, so ``--help`` and a file with
mistakes work on a machine without ROS2.

``[server] config = "server.toml"`` in the machine file and ``--server-config PATH`` point to an
opensmi server configuration (endpoint, users, logging). The command line wins. A path in the file is
relative to the file, a path on the command line to the working directory.

Exit status: 0 after a clean stop, 1 when the file or a configuration cannot be used (each mistake is
printed to stderr, one per line, and nothing has been started), 2 for a command line argparse refuses.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import signal
import sys
from pathlib import Path
from typing import TYPE_CHECKING

from opensmi.server import Server

from opensmi.ros2.server.errors import FileDeclarationError
from opensmi.ros2.server.loader import load_machine

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

    from opensmi.ros2.runtime.interface import RuntimeInterface
    from opensmi.ros2.server.config import ROS2Config
    from opensmi.ros2.server.loader import ConfiguredMachine

__all__ = ["main", "serve"]

PROG = "opensmi-ros2"


class _UnusableError(Exception):
    """Something the user gave cannot be used; the message is what to tell them."""


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog=PROG, description="Expose a ROS2 system as an OPC UA server.")
    commands = parser.add_subparsers(dest="command", required=True, metavar="command")
    serve_parser = commands.add_parser(
        "serve",
        help="start a server from a machine file and run until interrupted",
        description="Start a server from a machine file and run until interrupted (Ctrl+C or SIGTERM).",
    )
    serve_parser.add_argument("machine_file", metavar="machine.toml", type=Path, help="the machine file (TOML)")
    serve_parser.add_argument(
        "--server-config",
        metavar="PATH",
        type=Path,
        help="opensmi server configuration (endpoint, users, logging); overrides [server] config in the machine file",
    )
    return parser


async def serve(machine: ConfiguredMachine, *, server_config: Path | None, stop: asyncio.Event) -> None:
    """Serve a Machine in an opensmi Server until ``stop`` is set, then shut the server down.

    :param machine: The Machine to serve.
    :param server_config: An opensmi server configuration file, or ``None`` for opensmi's defaults.
    :param stop: Set to ask for a clean stop. Cancelling the task stops it as well.
    :raises _UnusableError: When the server configuration cannot be read.
    """
    try:
        server = Server(config=server_config)
    except Exception as error:
        msg = f"cannot use the server configuration {str(server_config)!r}: {error}"
        raise _UnusableError(msg) from error
    async with server:
        await server.add_machine(machine)
        await server.start(blocking=False)
        await stop.wait()


async def _run(machine: ConfiguredMachine, server_config: Path | None) -> None:
    """Serve until SIGINT or SIGTERM, where the loop can listen for them."""
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for number in (signal.SIGINT, signal.SIGTERM):
        # Not available on Windows' loop; asyncio.run turns Ctrl+C into a cancellation there.
        with contextlib.suppress(NotImplementedError, ValueError, RuntimeError):
            loop.add_signal_handler(number, stop.set)
    await serve(machine, server_config=server_config, stop=stop)


def _require_file(given: Path | None) -> None:
    """Refuse a ``--server-config`` that is not a file, before anything is built."""
    if given is not None and not given.is_file():
        msg = f"--server-config {str(given)!r} is not a file"
        raise _UnusableError(msg)


def _refuse(message: str) -> int:
    """Say on stderr why nothing was started, and return the exit status for it."""
    sys.stderr.write(f"{PROG}: {message}\n")
    return 1


def main(
    argv: Sequence[str] | None = None,
    *,
    runtime_factory: Callable[[ROS2Config], RuntimeInterface] | None = None,
) -> int:
    """Run the command line and return its exit status.

    :param argv: The arguments after the program name; ``None`` reads ``sys.argv``.
    :param runtime_factory: Passed to ``load_machine``; ``None`` builds the real Runtime, which
        needs ``rclpy``. A test passes one that returns its fake.
    """
    arguments = _parser().parse_args(argv)
    path: Path = arguments.machine_file
    try:
        _require_file(arguments.server_config)
        machine = load_machine(path, runtime_factory=runtime_factory)
    except (FileDeclarationError, _UnusableError) as error:
        return _refuse(str(error))
    except OSError as error:
        return _refuse(f"cannot read {str(path)!r}: {error.strerror or error}")
    try:
        with contextlib.suppress(KeyboardInterrupt):
            asyncio.run(_run(machine, arguments.server_config or machine.server_config))
    except _UnusableError as error:
        return _refuse(str(error))
    return 0
