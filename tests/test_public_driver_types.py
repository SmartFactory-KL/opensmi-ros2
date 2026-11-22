# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A custom lifecycle Driver written only against public names; its imports must come from
the public packages' `__all__`."""

from __future__ import annotations

import ast
import importlib
from pathlib import Path
from typing import TYPE_CHECKING, Any, ClassVar

import pytest

import opensmi.ros2.runtime
import opensmi.ros2.server
from opensmi.ros2.runtime import RuntimeInterface, ServiceTypes, Subscription
from opensmi.ros2.server import LifecycleBinding, LifecycleDriver, LifecycleNodeState, LifecycleNodeTransition
from tests import fixture_lifecycle
from tests.fake_runtime import FakeRuntime

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Sequence

NODE = "/lc_talker"
GUARDED = f"{NODE}/guarded/change_state"
CHANGE_STATE = "lifecycle_msgs/srv/ChangeState"


class GuardedDriver(LifecycleDriver):
    """Commands a node through ``<node>/guarded/change_state`` and keeps a second watch on it."""

    flavour: ClassVar[str] = "guarded"

    def __init__(self, *, binding: LifecycleBinding, runtime: RuntimeInterface) -> None:
        super().__init__(binding=binding, runtime=runtime)
        self._guarded_runtime = runtime
        self._guarded_binding = binding
        self._change: ServiceTypes = runtime.describe_service(CHANGE_STATE)
        self._watches: dict[str, Subscription] = {}

    async def open(self) -> None:
        await super().open()
        for path in self.paths:
            self._watches[path] = await self._guarded_runtime.subscribe(
                topic=f"{path}/transition_event",
                message_type=self._guarded_runtime.describe_message("lifecycle_msgs/msg/TransitionEvent"),
                depth=1,
            )

    async def close(self) -> None:
        watches, self._watches = self._watches, {}
        for watch in watches.values():
            await watch.close()
        await super().close()

    def watch(self, path: str) -> Subscription:
        return self._watches[path]

    async def send(self, transition: LifecycleNodeTransition, paths: tuple[str, ...]) -> Sequence[str]:
        nested: Any = self._change.request.ros2_nested["transition"]
        request_type: Any = self._change.request
        request = request_type(transition=nested(id=int(transition), label=""))
        faults: list[str] = []
        for path in paths:
            answer: Any = await self._guarded_runtime.call_service(
                service=f"{path}/guarded/change_state",
                service_type=self._change,
                request=request,
                server_timeout=self._guarded_binding.server_timeout,
                call_timeout=self._guarded_binding.call_timeout,
            )
            if not answer.success:
                faults.append(f"{path} refused to {transition.name.lower()}")
        return faults


@pytest.fixture
async def runtime() -> FakeRuntime:
    started = FakeRuntime(
        message_types=fixture_lifecycle.MESSAGE_TYPES,
        service_types=fixture_lifecycle.SERVICE_TYPES,
    )
    await started.start()
    return started


def test_both_types_are_public_and_are_the_ones_the_runtime_hands_out() -> None:
    assert {"ServiceTypes", "Subscription"} <= set(opensmi.ros2.runtime.__all__)
    assert ServiceTypes is importlib.import_module("opensmi.ros2.runtime.messages").ServiceTypes
    assert Subscription is importlib.import_module("opensmi.ros2.runtime.subscription").Subscription


def test_this_driver_names_the_package_only_through_the_public_packages() -> None:
    tree = ast.parse(Path(__file__).read_text(encoding="utf-8"))
    public = {
        "opensmi.ros2.runtime": set(opensmi.ros2.runtime.__all__),
        "opensmi.ros2.server": set(opensmi.ros2.server.__all__),
    }
    imported = {
        node.module: {alias.name for alias in node.names}
        for node in tree.body
        if isinstance(node, ast.ImportFrom) and node.module is not None and node.module.startswith("opensmi.")
    }

    assert imported.keys() == public.keys()
    for module, names in imported.items():
        assert names <= public[module], module


async def test_a_custom_driver_commands_a_node_through_its_own_service(runtime: FakeRuntime) -> None:
    binding = LifecycleBinding(name="Talker", path=NODE, driver=GuardedDriver)
    driver = GuardedDriver(binding=binding, runtime=runtime)
    await driver.open()
    states = fixture_lifecycle.serve_nodes(runtime, {NODE: int(LifecycleNodeState.UNCONFIGURED)})
    runtime.serve_service(GUARDED, lambda request: fixture_lifecycle.ChangeStateResponse(success=True))

    faults = await driver.apply("reset")

    assert faults == ()
    assert [service for service, _ in runtime.requests if service.endswith("change_state")] == [GUARDED]
    assert states == {NODE: int(LifecycleNodeState.UNCONFIGURED)}
    await driver.close()


async def test_a_custom_driver_reports_a_refusal_in_its_own_words(runtime: FakeRuntime) -> None:
    binding = LifecycleBinding(name="Talker", path=NODE, driver=GuardedDriver)
    driver = GuardedDriver(binding=binding, runtime=runtime)
    await driver.open()
    fixture_lifecycle.serve_nodes(runtime, {NODE: int(LifecycleNodeState.UNCONFIGURED)})
    runtime.serve_service(GUARDED, lambda request: fixture_lifecycle.ChangeStateResponse(success=False))

    assert await driver.apply("reset") == (f"{NODE} refused to configure",)
    await driver.close()


async def test_a_custom_driver_can_hold_and_read_a_subscription(runtime: FakeRuntime) -> None:
    binding = LifecycleBinding(name="Talker", path=NODE, driver=GuardedDriver)
    driver = GuardedDriver(binding=binding, runtime=runtime)
    await driver.open()
    watch = driver.watch(NODE)
    event = fixture_lifecycle.transition_event(start=1, goal=2)

    assert isinstance(watch, Subscription)
    assert runtime.deliver(f"{NODE}/transition_event", event) == 2
    received: AsyncIterator[Any] = watch.__aiter__()
    assert (await anext(received)).goal_state.id == 2
    await driver.close()
    assert watch._sink.closed
