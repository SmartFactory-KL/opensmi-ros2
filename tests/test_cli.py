# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The command line starts a server from a file and refuses a bad file before starting anything.

Driven in process with a fake Runtime; `--help` and a bad file also run in a fresh interpreter to
show rclpy is absent.
"""

from __future__ import annotations

import asyncio
import contextlib
import socket
import subprocess
import sys
from importlib import metadata
from typing import TYPE_CHECKING

import pytest
from asyncua.client.client import Client

from opensmi.ros2 import cli
from opensmi.ros2.server import load_machine
from tests.test_loader import MACHINE, Factory, below

if TYPE_CHECKING:
    from collections.abc import AsyncIterator
    from pathlib import Path

    from opensmi.ros2.server.loader import ConfiguredMachine

SERVER_TOML = """
[ua_server]
name = "{name}"
endpoint_address = "opc.tcp://127.0.0.1:{port}/server"
"""


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def write(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


def server_toml(path: Path, name: str) -> tuple[Path, int]:
    """Write an opensmi server configuration with a name and a free port; return it and the port."""
    port = free_port()
    return write(path, SERVER_TOML.format(name=name, port=port)), port


def with_server_table(config: str) -> str:
    return MACHINE.replace("[ros2]", f'[server]\nconfig = "{config}"\n\n[ros2]', 1)


async def reach(port: int) -> Client:
    """Connect to a server that is still coming up, and return the connected client."""
    client = Client(url=f"opc.tcp://operator:operator@127.0.0.1:{port}/server")
    for _ in range(100):
        try:
            await client.connect()
        except (OSError, TimeoutError):
            await asyncio.sleep(0.05)
        else:
            return client
    pytest.fail(f"nothing answered on port {port}")


async def application_name(client: Client) -> str | None:
    """The name the server announces for itself, which is opensmi's ``ua_server.name``."""
    endpoints = await client.get_endpoints()
    return endpoints[0].Server.ApplicationName.Text


@contextlib.asynccontextmanager
async def serving(machine: ConfiguredMachine, config: Path | None, port: int) -> AsyncIterator[Client]:
    """Run ``cli.serve`` as a task, yield a client of it, and stop it the way a user would."""
    stop = asyncio.Event()
    task = asyncio.create_task(cli.serve(machine, server_config=config, stop=stop))
    client = await reach(port)
    try:
        yield client
    finally:
        await client.disconnect()
        stop.set()
        await asyncio.wait_for(task, 10)


# -- the server configuration reaches opensmi's server --------------------------------------------


async def test_the_files_server_config_reaches_the_server_a_client_connects_to(tmp_path: Path) -> None:
    config, port = server_toml(tmp_path / "server.toml", "Configured Cell")
    path = write(tmp_path / "machine.toml", with_server_table("server.toml"))
    machine = load_machine(path, runtime_factory=Factory())

    async with serving(machine, machine.server_config, port) as client:
        assert machine.server_config == config.resolve()
        assert await application_name(client) == "Configured Cell"
        assert await below(client.nodes.objects, "Machines", "Cell")


async def test_a_server_without_a_configuration_runs_under_opensmis_defaults(tmp_path: Path) -> None:
    machine = load_machine(write(tmp_path / "machine.toml", MACHINE), runtime_factory=Factory())

    assert machine.server_config is None


async def test_a_clean_stop_leaves_nothing_listening(tmp_path: Path) -> None:
    _, port = server_toml(tmp_path / "server.toml", "Stopped Cell")
    machine = load_machine(
        write(tmp_path / "machine.toml", with_server_table("server.toml")), runtime_factory=Factory()
    )

    async with serving(machine, machine.server_config, port):
        pass

    with pytest.raises((OSError, TimeoutError)):
        await asyncio.wait_for(asyncio.open_connection("127.0.0.1", port), 2)


async def test_cancelling_the_task_stops_the_server_as_well(tmp_path: Path) -> None:
    _, port = server_toml(tmp_path / "server.toml", "Cancelled Cell")
    machine = load_machine(
        write(tmp_path / "machine.toml", with_server_table("server.toml")), runtime_factory=Factory()
    )
    task = asyncio.create_task(cli.serve(machine, server_config=machine.server_config, stop=asyncio.Event()))
    client = await reach(port)
    await client.disconnect()

    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await asyncio.wait_for(task, 10)

    with pytest.raises((OSError, TimeoutError)):
        await asyncio.wait_for(asyncio.open_connection("127.0.0.1", port), 2)


# -- relative paths -------------------------------------------------------------------------------


def test_a_relative_server_config_resolves_against_the_files_directory_not_the_working_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, _ = server_toml(tmp_path / "deploy" / "conf" / "server.toml", "Relative")
    path = write(tmp_path / "deploy" / "machine.toml", with_server_table("conf/server.toml"))
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)

    machine = load_machine(path, runtime_factory=Factory())

    assert machine.server_config == config.resolve()


# -- the command line wires the file, the flag and the factory ---------------------------------------


@pytest.fixture
def served_with(monkeypatch: pytest.MonkeyPatch) -> list[tuple[ConfiguredMachine, Path | None]]:
    """Replace the serving loop with one that records what it was given and returns at once."""
    calls: list[tuple[ConfiguredMachine, Path | None]] = []

    async def record(machine: ConfiguredMachine, server_config: Path | None) -> None:
        calls.append((machine, server_config))

    monkeypatch.setattr(cli, "_run", record)
    return calls


def test_serve_builds_the_runtime_from_the_files_settings_and_serves_the_machine(
    tmp_path: Path, served_with: list[tuple[ConfiguredMachine, Path | None]]
) -> None:
    factory = Factory()
    path = write(tmp_path / "machine.toml", MACHINE)

    assert cli.main(["serve", str(path)], runtime_factory=factory) == 0

    assert [each.domain_id for each in factory.configs] == [7]
    [(machine, server_config)] = served_with
    assert machine.name == "Cell"
    assert server_config is None


def test_the_files_server_config_is_used_when_the_flag_is_absent(
    tmp_path: Path, served_with: list[tuple[ConfiguredMachine, Path | None]]
) -> None:
    config, _ = server_toml(tmp_path / "server.toml", "From the file")
    path = write(tmp_path / "machine.toml", with_server_table("server.toml"))

    assert cli.main(["serve", str(path)], runtime_factory=Factory()) == 0

    assert served_with[0][1] == config.resolve()


def test_the_flag_wins_over_the_file(tmp_path: Path, served_with: list[tuple[ConfiguredMachine, Path | None]]) -> None:
    server_toml(tmp_path / "server.toml", "From the file")
    flagged, _ = server_toml(tmp_path / "other.toml", "From the flag")
    path = write(tmp_path / "machine.toml", with_server_table("server.toml"))

    assert cli.main(["serve", str(path), "--server-config", str(flagged)], runtime_factory=Factory()) == 0

    assert served_with[0][1] == flagged


def test_the_flag_reaches_the_server_a_client_connects_to(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """The whole path through ``main``: the flag's endpoint is the one that answers."""
    server_toml(tmp_path / "server.toml", "From the file")
    flagged, port = server_toml(tmp_path / "other.toml", "From the flag")
    path = write(tmp_path / "machine.toml", with_server_table("server.toml"))
    seen: list[str | None] = []

    async def run_then_stop(machine: ConfiguredMachine, server_config: Path | None) -> None:
        async with serving(machine, server_config, port) as client:
            seen.append(await application_name(client))

    monkeypatch.setattr(cli, "_run", run_then_stop)

    assert cli.main(["serve", str(path), "--server-config", str(flagged)], runtime_factory=Factory()) == 0
    assert seen == ["From the flag"]


# -- a bad file starts nothing --------------------------------------------------------------------


@pytest.fixture
def never_served(monkeypatch: pytest.MonkeyPatch) -> None:
    async def refuse(machine: ConfiguredMachine, server_config: Path | None) -> None:
        pytest.fail("a server was started")

    monkeypatch.setattr(cli, "_run", refuse)


@pytest.mark.usefixtures("never_served")
def test_a_bad_file_exits_non_zero_with_every_error_and_its_path_on_stderr(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    factory = Factory()
    text = MACHINE.replace('parent = "Base"', 'parent = "Nobody"').replace("domain_id = 7", "domain_id = true")
    path = write(tmp_path / "machine.toml", text)

    status = cli.main(["serve", str(path)], runtime_factory=factory)

    captured = capsys.readouterr()
    assert status == 1
    assert "component[1].parent" in captured.err
    assert "ros2.domain_id" in captured.err
    assert captured.out == ""
    assert factory.configs == []


@pytest.mark.usefixtures("never_served")
def test_a_file_that_is_not_there_exits_non_zero_and_says_so(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    assert cli.main(["serve", str(tmp_path / "missing.toml")], runtime_factory=Factory()) == 1

    assert "missing.toml" in capsys.readouterr().err


@pytest.mark.usefixtures("never_served")
@pytest.mark.parametrize(
    ("table", "problem"),
    [
        ('config = "absent.toml"', "server.config"),
        ("config = 3", "server.config"),
        ('config = "server.toml"\nport = 1', "server.port"),
    ],
)
def test_a_bad_server_table_is_an_error_at_its_key(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], table: str, problem: str
) -> None:
    server_toml(tmp_path / "server.toml", "x")
    path = write(tmp_path / "machine.toml", MACHINE.replace("[ros2]", f"[server]\n{table}\n\n[ros2]", 1))
    factory = Factory()

    assert cli.main(["serve", str(path)], runtime_factory=factory) == 1

    assert problem in capsys.readouterr().err
    assert factory.configs == []


@pytest.mark.usefixtures("never_served")
def test_a_flag_that_is_not_a_file_exits_non_zero_before_anything_is_built(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    factory = Factory()
    path = write(tmp_path / "machine.toml", MACHINE)

    assert cli.main(["serve", str(path), "--server-config", str(tmp_path / "no.toml")], runtime_factory=factory) == 1

    assert "--server-config" in capsys.readouterr().err
    assert factory.configs == []


def test_a_server_configuration_opensmi_refuses_exits_non_zero(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    bad = write(tmp_path / "server.toml", "[ua_server]\nnonsense = 1\n")
    path = write(tmp_path / "machine.toml", MACHINE)

    assert cli.main(["serve", str(path), "--server-config", str(bad)], runtime_factory=Factory()) == 1

    assert "nonsense" in capsys.readouterr().err


# -- the command line itself --------------------------------------------------------------------------


def test_the_console_script_and_the_module_run_the_same_function() -> None:
    [script] = [each for each in metadata.entry_points(group="console_scripts") if each.name == "opensmi-ros2"]

    assert script.load() is cli.main


def test_help_names_the_command_and_both_ways_to_point_at_the_server_configuration(
    capsys: pytest.CaptureFixture[str],
) -> None:
    with pytest.raises(SystemExit) as raised:
        cli.main(["serve", "--help"])

    assert raised.value.code == 0
    assert "--server-config" in capsys.readouterr().out


def test_a_command_line_argparse_refuses_exits_with_status_two() -> None:
    with pytest.raises(SystemExit) as raised:
        cli.main([])

    assert raised.value.code == 2


def run_module(*arguments: str) -> subprocess.CompletedProcess[str]:
    """Run ``python -m opensmi.ros2`` in a fresh interpreter, isolated from the working directory."""
    return subprocess.run(
        [sys.executable, "-m", "opensmi.ros2", *arguments],
        capture_output=True,
        text=True,
        check=False,
        timeout=60,
    )


def test_help_works_without_ros2_through_python_dash_m() -> None:
    done = run_module("--help")

    assert done.returncode == 0
    assert "serve" in done.stdout


def test_a_bad_file_through_python_dash_m_fails_without_ros2_and_names_the_mistake(tmp_path: Path) -> None:
    path = write(tmp_path / "machine.toml", MACHINE.replace('parent = "Base"', 'parent = "Nobody"'))

    done = run_module("serve", str(path))

    assert done.returncode == 1
    assert "component[1].parent" in done.stderr
    assert "rclpy" not in done.stderr
