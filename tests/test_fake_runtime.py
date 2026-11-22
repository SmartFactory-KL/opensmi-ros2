# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The fake Runtime behaves like the real one where server-half tests rely on it."""

from __future__ import annotations

import asyncio
from typing import Any

import pytest

from opensmi.ros2.runtime.errors import (
    LoopAffinityError,
    MissingInterfacePackageError,
    RuntimeAlreadyStartedError,
    RuntimeNotStartedError,
    SubscriptionClosedError,
)
from opensmi.ros2.runtime.messages import (
    FieldDescription,
    MessageDescription,
    StringType,
    build_message_dataclass,
)
from tests.fake_runtime import FakeRuntime

# A run-time dataclass has no static field set, so the type checker is told so here.
CHATTER: Any = build_message_dataclass(
    MessageDescription(
        type_name="std_msgs/msg/String",
        fields=(FieldDescription(name="data", field_type=StringType(), ros2_text="string"),),
    )
)


@pytest.fixture
def runtime():
    return FakeRuntime(domain_id=7, message_types={"std_msgs/msg/String": CHATTER})


async def test_construction_starts_nothing(runtime) -> None:
    assert runtime.domain_id == 7
    assert not runtime.started


async def test_starting_twice_is_refused(runtime) -> None:
    await runtime.start()

    with pytest.raises(RuntimeAlreadyStartedError):
        await runtime.start()


async def test_subscribing_before_start_is_refused(runtime) -> None:
    with pytest.raises(RuntimeNotStartedError):
        await runtime.subscribe(topic="/chatter", message_type="std_msgs/msg/String")


async def test_stopping_something_never_started_is_refused(runtime) -> None:
    with pytest.raises(RuntimeNotStartedError):
        await runtime.stop()


async def test_a_declared_type_with_no_interface_package_is_fatal(runtime) -> None:
    await runtime.start()

    with pytest.raises(MissingInterfacePackageError):
        await runtime.subscribe(topic="/chatter", message_type="nowhere_msgs/msg/Nothing")


async def test_a_message_reaches_the_handle(runtime) -> None:
    await runtime.start()
    handle = await runtime.subscribe(topic="/chatter", message_type="std_msgs/msg/String")

    assert runtime.deliver("/chatter", CHATTER(data="hello")) == 1

    received: Any = await handle.receive()
    assert received.data == "hello"
    assert handle.message_type is CHATTER


async def test_stopping_closes_every_open_subscription(runtime) -> None:
    # The Machine mixin stops the Runtime at shutdown; that must be the whole teardown.
    await runtime.start()
    handle = await runtime.subscribe(topic="/chatter", message_type="std_msgs/msg/String")

    outcome = await runtime.stop()

    assert outcome.ok
    with pytest.raises(SubscriptionClosedError):
        await handle.receive()


async def test_a_closed_handle_is_forgotten_by_the_runtime(runtime) -> None:
    await runtime.start()
    handle = await runtime.subscribe(topic="/chatter", message_type="std_msgs/msg/String")

    await handle.close()

    assert runtime.deliver("/chatter", CHATTER(data="ignored")) == 0


async def test_the_double_refuses_a_foreign_event_loop_as_the_real_runtime_does(runtime) -> None:
    await runtime.start()

    def on_another_loop():
        return asyncio.run(runtime.subscribe(topic="/chatter", message_type="std_msgs/msg/String"))

    with pytest.raises(LoopAffinityError):
        await asyncio.to_thread(on_another_loop)
