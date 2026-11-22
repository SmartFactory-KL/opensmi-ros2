# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Marking a Binding required, and the shape of the wait that follows. Wiring is in
`test_required_binding_server.py`."""

from __future__ import annotations

import asyncio
from dataclasses import fields

import pytest

from opensmi.ros2.runtime.parameters import ParameterType
from opensmi.ros2.server.bindings import (
    ActionBinding,
    LifecycleBinding,
    MethodBinding,
    MonitoringBinding,
    PublisherBinding,
    ROS2ParameterBinding,
)
from opensmi.ros2.server.config import ROS2Config, from_dict
from opensmi.ros2.server.errors import RequiredDeclarationError
from opensmi.ros2.server.required import Prerequisite, wait_for

REQUIRABLE = (
    MonitoringBinding(name="Pose", path="/pose", message_type="geometry_msgs/msg/PoseStamped"),
    ActionBinding(name="Navigate", path="/navigate", action_type="nav2_msgs/action/NavigateToPose"),
    MethodBinding(name="Stop", path="/stop", service_type="std_srvs/srv/Trigger"),
    ROS2ParameterBinding(name="Tuning", path="/controller", parameters={"speed": ParameterType.DOUBLE}),
    LifecycleBinding(name="Talker", path="/lc_talker"),
)
"""One declaration of every kind that can be required, built the way a Component writes them."""


def replaced(binding, **changes):
    """Return one declaration with keywords changed, rebuilt from its own fields (not dataclasses.replace)."""
    values = {field.name: getattr(binding, field.name) for field in fields(binding)}
    return type(binding)(**{**values, **changes})


# -- required is one word on a declaration ----------------------------------------------------


@pytest.mark.parametrize("binding", REQUIRABLE, ids=lambda each: type(each).__name__)
def test_every_kind_that_can_be_required_is_optional_by_default(binding) -> None:
    assert binding.required is False
    assert binding.startup_timeout is None
    assert not hasattr(binding, "optional")


@pytest.mark.parametrize("binding", REQUIRABLE, ids=lambda each: type(each).__name__)
def test_every_kind_that_can_be_required_takes_the_mark_and_its_own_deadline(binding) -> None:
    marked = replaced(binding, required=True, startup_timeout=2.5)

    assert marked.required
    assert marked.startup_timeout == 2.5


def test_a_publishing_binding_can_never_be_required() -> None:
    # The field is absent, not refused: Satisfied is presence on the graph.
    declared = PublisherBinding(name="Drive", path="/cmd_vel", message_type="geometry_msgs/msg/Twist", continuous=False)
    assert not hasattr(declared, "required")
    assert not hasattr(declared, "startup_timeout")

    with pytest.raises(TypeError):
        PublisherBinding(
            name="Drive",
            path="/cmd_vel",
            message_type="geometry_msgs/msg/Twist",
            continuous=False,
            required=True,  # type: ignore[call-arg]
        )


def test_a_binding_file_cannot_mark_a_published_topic_required() -> None:
    with pytest.raises(ValueError, match="unknown configuration option"):
        from_dict(
            PublisherBinding,
            {
                "name": "drive",
                "path": "/cmd_vel",
                "message_type": "geometry_msgs/msg/Twist",
                "continuous": False,
                "required": True,
            },
        )


def test_a_binding_file_can_mark_a_topic_required() -> None:
    loaded = from_dict(
        MonitoringBinding,
        {"name": "Pose", "path": "/pose", "required": True, "startup_timeout": 5.0},
    )

    assert loaded.required
    assert loaded.startup_timeout == 5.0


@pytest.mark.parametrize("binding", REQUIRABLE, ids=lambda each: type(each).__name__)
def test_a_deadline_on_a_binding_nothing_waits_for_is_refused(binding) -> None:
    with pytest.raises(RequiredDeclarationError, match="is not required"):
        replaced(binding, startup_timeout=5.0)


@pytest.mark.parametrize("timeout", [0.0, -1.0])
def test_a_deadline_that_has_already_passed_is_refused(timeout) -> None:
    with pytest.raises(RequiredDeclarationError, match="positive number of seconds"):
        MonitoringBinding(name="Pose", path="/pose", required=True, startup_timeout=timeout)


def test_a_deployment_declares_the_default_deadline() -> None:
    assert ROS2Config().startup_timeout == 30.0
    assert from_dict(ROS2Config, {"startup_timeout": 90.0}).startup_timeout == 90.0


# -- the wait -----------------------------------------------------------------------------------


def prerequisite(name: str, timeout: float, satisfied) -> Prerequisite:
    """Return one prerequisite over a callable a test moves by hand."""
    return Prerequisite(binding=name, path=f"/{name}", timeout=timeout, satisfied=satisfied)


class Reporter:
    """Collects what the wait announced, in the order it announced it."""

    def __init__(self) -> None:
        self.reported: list[str] = []

    async def __call__(self, prerequisite: Prerequisite) -> None:
        self.reported.append(prerequisite.binding)


async def test_a_wait_with_nothing_to_wait_for_returns_at_once() -> None:
    reporter = Reporter()

    assert await wait_for((), report=reporter) == ()
    assert reporter.reported == []


async def test_a_prerequisite_that_is_already_there_is_never_reported() -> None:
    reporter = Reporter()

    missed = await wait_for([prerequisite("lidar", 10.0, lambda: True)], report=reporter)

    assert missed == ()
    assert reporter.reported == []


async def test_a_prerequisite_that_arrives_before_its_deadline_is_waited_for_and_not_reported() -> None:
    present = False

    async def arrive() -> None:
        nonlocal present
        await asyncio.sleep(0.1)
        present = True

    reporter = Reporter()
    arriving = asyncio.create_task(arrive())

    missed = await wait_for([prerequisite("lidar", 5.0, lambda: present)], report=reporter)

    await arriving
    assert missed == ()
    assert reporter.reported == []


async def test_a_prerequisite_that_never_arrives_is_reported_and_missed() -> None:
    reporter = Reporter()

    missed = await wait_for([prerequisite("lidar", 0.05, lambda: False)], report=reporter)

    assert [each.binding for each in missed] == ["lidar"]
    assert reporter.reported == ["lidar"]


async def test_each_prerequisite_fires_at_its_own_deadline_and_the_verdict_is_the_last() -> None:
    reporter = Reporter()
    prerequisites = [
        prerequisite("camera", 0.3, lambda: False),
        prerequisite("lidar", 0.05, lambda: False),
    ]

    loop = asyncio.get_running_loop()
    began = loop.time()
    missed = await wait_for(prerequisites, report=reporter)
    took = loop.time() - began

    assert reporter.reported == ["lidar", "camera"]
    assert [each.binding for each in missed] == ["lidar", "camera"]
    assert took >= 0.3


async def test_one_that_arrives_does_not_hold_up_the_verdict_of_one_that_did_not() -> None:
    reporter = Reporter()
    prerequisites = [
        prerequisite("lidar", 0.05, lambda: False),
        prerequisite("camera", 5.0, lambda: True),
    ]

    missed = await wait_for(prerequisites, report=reporter)

    assert [each.binding for each in missed] == ["lidar"]
    assert reporter.reported == ["lidar"]
