# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Drivers: which service, which encoding, which topic, and no state machine.

A Driver is the part of a lifecycle Binding that knows how one flavour of coordinator is commanded.
The mapping onto Skill states is shared and lives in :mod:`opensmi.ros2.server.lifecycle`, so
Components driven by different Drivers present the same OPC UA surface. Two ship:

* :class:`StandardLifecycleDriver` is the ROS2 standard, ``lifecycle_msgs``, one node at a time via
  ``<node>/change_state``.
* :class:`Nav2ManagerDriver` is a nav2-style coordinator that drives a set of nodes with one
  ``<manager>/manage_nodes`` call.

Observing is common to both, because a nav2-managed node is an ordinary lifecycle node. Every
Driver gates each transition on what the node reports as available. An invalid change request kills
an rclpy node (Jazzy 7.1.11: the ``RCLError`` propagates out of ``rclpy.spin()``), and a
``finalized`` node offers nothing, so a node that does not offer a transition is skipped. Discovery
uses each node's ``transition_event`` subscription (transitions and publisher matches) instead of
polling the graph. A third flavour subclasses :class:`LifecycleDriver`, implements
:meth:`~LifecycleDriver.send` and is given as the Binding's ``driver``.

Nothing here imports ``rclpy`` or touches OPC UA, so a Driver is tested with the fake Runtime.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, ClassVar, Final

from opensmi.ros2.runtime.errors import ServiceCallTimeoutError, ServiceUnavailableError
from opensmi.ros2.server.lifecycle import TRANSITIONS_FOR, UNDONE_BY, LifecycleNodeState, LifecycleNodeTransition

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Iterable, Mapping, Sequence

    from opensmi.server.mixins.skill_logic_mixins import TRIGGER

    from opensmi.ros2.runtime import Message, RuntimeInterface, ServiceTypes, Subscription
    from opensmi.ros2.server.bindings import LifecycleBinding

__all__ = [
    "CHANGE_STATE_TYPE",
    "DRIVERS",
    "GET_AVAILABLE_TRANSITIONS_TYPE",
    "GET_STATE_TYPE",
    "MANAGE_NODES_TYPE",
    "NAV2_COMMANDS",
    "TRANSITION_EVENT_TYPE",
    "LifecycleDriver",
    "Nav2ManagerDriver",
    "NodeTransition",
    "StandardLifecycleDriver",
]

TRANSITION_EVENT_TYPE: Final = "lifecycle_msgs/msg/TransitionEvent"
"""The topic type every managed node announces its transitions on, whoever coordinates it."""

GET_STATE_TYPE: Final = "lifecycle_msgs/srv/GetState"
"""The service type behind ``<node>/get_state``: one query, answered by the node itself."""

CHANGE_STATE_TYPE: Final = "lifecycle_msgs/srv/ChangeState"
"""The service type behind ``<node>/change_state``, which only the standard flavour sends."""

GET_AVAILABLE_TRANSITIONS_TYPE: Final = "lifecycle_msgs/srv/GetAvailableTransitions"
"""The service type behind ``<node>/get_available_transitions``, which is the gate."""

MANAGE_NODES_TYPE: Final = "nav2_msgs/srv/ManageLifecycleNodes"
"""The service type a nav2-style manager takes: one ``uint8`` command covering a whole set.

Its interface package is nav2's, so naming this flavour without nav2 installed is fatal at init.
"""


NAV2_COMMANDS: Final[Mapping[LifecycleNodeTransition, int]] = {
    LifecycleNodeTransition.CONFIGURE: 5,  # ManageLifecycleNodes.CONFIGURE
    LifecycleNodeTransition.CLEANUP: 6,  # ManageLifecycleNodes.CLEANUP
    LifecycleNodeTransition.ACTIVATE: 2,  # ManageLifecycleNodes.RESUME
    LifecycleNodeTransition.DEACTIVATE: 1,  # ManageLifecycleNodes.PAUSE
}
"""Which nav2 manager command sends which lifecycle transition at the set it drives.

The numbers are spelled out because the constants live on a generated nav2 class that cannot be
imported without nav2. ``RESUME`` and ``PAUSE`` are the manager's words for ``activate`` and
``deactivate``. ``STARTUP`` and ``RESET`` are compounds already expressed in
:data:`opensmi.ros2.server.lifecycle.TRANSITIONS_FOR`, and ``SHUTDOWN`` ends in ``finalized``,
which this project never commands.
"""


class NodeTransition:
    """One transition a node announced: the node's path and its start and goal state ids.

    The event's ``transition`` and ``timestamp`` fields are not kept, because ``rcl_lifecycle``
    leaves them empty.
    """

    __slots__ = ("goal", "path", "start")

    def __init__(self, *, path: str, start: int, goal: int) -> None:
        """Record one node's move between two lifecycle states."""
        self.path = path
        self.start = start
        self.goal = goal

    def __repr__(self) -> str:
        """Return a representation naming the node and both ends of the move."""
        return f"<NodeTransition {self.path} {_state_name(self.start)} -> {_state_name(self.goal)}>"


class LifecycleDriver(ABC):
    """One coordinator flavour, commanded and observed. Holds no state machine.

    Constructed by the StartupSkill the Binding lands on. A declaration may name a subclass to add
    a third flavour.

    :cvar flavour: The name a declaration selects this Driver by, and its key in :data:`DRIVERS`.
    :cvar coordinates_a_set: Whether ``path`` names a coordinator other than the node it drives,
        so that the nodes need to be listed. True for a nav2 manager, false for the standard flavour.
    """

    flavour: ClassVar[str]
    coordinates_a_set: ClassVar[bool] = False

    def __init__(self, *, binding: LifecycleBinding, runtime: RuntimeInterface) -> None:
        """Resolve every ROS2 type this flavour needs, before anything is subscribed or called.

        A missing interface package is a declaration error and fatal at init.

        :param binding: What to drive.
        :param runtime: The Machine's Runtime, reached through the Component.
        :raises MessageTypeError: When an interface package this flavour needs is not installed.
        """
        self._binding = binding
        self._runtime = runtime
        self._event_type = runtime.describe_message(TRANSITION_EVENT_TYPE)
        self._get_state: ServiceTypes = runtime.describe_service(GET_STATE_TYPE)
        self._get_available: ServiceTypes = runtime.describe_service(GET_AVAILABLE_TRANSITIONS_TYPE)
        self._subscriptions: dict[str, Subscription] = {}

    def __repr__(self) -> str:
        """Return a representation naming the flavour and what it drives."""
        return f"<{type(self).__name__} {self._binding.path} driving {', '.join(self.paths)}>"

    @property
    def binding(self) -> LifecycleBinding:
        """Return the declaration this Driver was built from."""
        return self._binding

    @property
    def paths(self) -> tuple[str, ...]:
        """Return the lifecycle nodes this Driver observes, in declaration order."""
        return self._binding.observed

    async def open(self) -> None:
        """Subscribe to every node's ``transition_event``.

        Subscribing is local, so a Component whose robot is off still finishes building.
        The first question goes out later, when the match events say a node is there.
        """
        for path in self.paths:
            self._subscriptions[path] = await self._runtime.subscribe(
                topic=f"{path}/transition_event",
                message_type=self._event_type,
                depth=self._binding.depth,
            )

    async def close(self) -> None:
        """Close every subscription this Driver opened. Idempotent."""
        opened, self._subscriptions = self._subscriptions, {}
        for subscription in opened.values():
            await subscription.close()

    async def transitions(self, path: str) -> AsyncIterator[NodeTransition]:
        """Yield every transition one node announces, until its subscription closes."""
        async for message in self._subscriptions[path]:
            yield NodeTransition(
                path=path,
                start=_state_id(message, "start_state"),
                goal=_state_id(message, "goal_state"),
            )

    async def appearances(self, path: str) -> AsyncIterator[int]:
        """Yield how many publishers one node's ``transition_event`` has, now and on each change.

        The current count arrives at once, so a Driver opened against a node that is already there
        looks immediately.
        """
        async for count in self._subscriptions[path].publisher_counts():
            yield count

    async def read_state(self, path: str) -> LifecycleNodeState | None:
        """Ask one node what state it is in. One call, never a poll.

        :return: The state, or ``None`` when nobody answered (the node is off).
        """
        answer = await self._call(f"{path}/get_state", self._get_state, self._get_state.request())
        if answer is None:
            return None
        return _as_state(_state_id(answer, "current_state"))

    async def offered_transitions(self, path: str) -> frozenset[LifecycleNodeTransition] | None:
        """Ask one node which transitions it will accept right now.

        This is the gate. It is read from the robot, because a stale belief about the state would
        kill an rclpy node. The name avoids "available", which this project does not use for a state of the robot.

        :return: The transitions this project can command among those the node offers. The set is
            empty for a ``finalized`` node, and ``None`` when nobody answered. The two differ: a node
            that offers nothing is skipped, a node that is not there is a fault.
        """
        answer = await self._call(
            f"{path}/get_available_transitions", self._get_available, self._get_available.request()
        )
        if answer is None:
            return None
        # Reached dynamically: message dataclasses are built at run time and lifecycle_msgs has no stubs.
        offered: Any = getattr(answer, "available_transitions", ()) or ()
        known = {each.value for each in LifecycleNodeTransition}
        return frozenset(
            LifecycleNodeTransition(int(described.transition.id))
            for described in offered
            if int(described.transition.id) in known
        )

    async def apply(
        self, trigger: TRIGGER, ledger: dict[str, set[LifecycleNodeTransition]] | None = None
    ) -> tuple[str, ...]:
        """Drive every node toward what one Skill trigger means, and return what went wrong.

        The trigger's transitions run in order (:data:`TRANSITIONS_FOR`), each gated per node on
        what that node offers at that moment. The gate is read before the first transition and
        again between transitions, because deactivating is what makes cleaning up available.

        A node that answered and does not offer the transition is skipped without a fault, which
        makes the handlers idempotent. A node that did not answer is a fault, so that a robot that
        is off cannot let ``Start`` reach ``Running``; it is asked once per trigger.

        The ledger records what the Skill performed itself, and ``halt`` rolls back those
        transitions and no others. A node that was already ``active`` before ``Start``
        has an empty ledger and is not touched.

        :param trigger: The Skill trigger to carry out.
        :param ledger: What the Skill performed on each node, by path. Updated in place. ``None``
            is an empty ledger.
        :return: One fault per node or coordinator that did not answer or said no. Empty is
            success, including having had nothing to do.
        """
        if ledger is None:
            ledger = {}
        rolling_back = trigger == "halt"
        stuck: set[str] = set()
        faults: list[str] = []
        asked = tuple(path for path in self.paths if ledger.get(path)) if rolling_back else self.paths
        offered, unanswered = await self._gate(asked)
        faults.extend(unanswered)
        for index, transition in enumerate(TRANSITIONS_FOR[trigger]):
            if index:
                offered, unanswered = await self._gate(offered)
                faults.extend(unanswered)
            undone = UNDONE_BY.get(transition) if rolling_back else None
            if rolling_back and undone is None:  # pragma: no cover -- every halt transition undoes one
                continue
            ours = tuple(path for path in offered if undone is None or undone in ledger.get(path, ()))
            wanted = tuple(path for path in ours if transition in offered[path])
            refused = await self.send(transition, wanted) if wanted else ()
            faults.extend(refused)
            if undone is None:
                for path in wanted:
                    ledger.setdefault(path, set()).add(transition)
                continue
            for path in ours:
                # Kept once a node refused: the Skill's work on it still stands.
                if refused and path in wanted:
                    stuck.add(path)
                if path not in stuck:
                    ledger[path].discard(undone)
        return tuple(faults)

    async def _gate(self, paths: Iterable[str]) -> tuple[dict[str, frozenset[LifecycleNodeTransition]], list[str]]:
        """Ask each node which transitions it offers, keeping silence apart from none.

        A node that offered nothing is carried with an empty set. A node that did not answer is a
        fault and is dropped from the set the following transitions gate on.

        :param paths: The nodes to ask, in the order their faults should be reported in.
        :return: What each node that answered offers, and one fault per node that did not.
        """
        offered: dict[str, frozenset[LifecycleNodeTransition]] = {}
        faults: list[str] = []
        for path in paths:
            answered = await self.offered_transitions(path)
            if answered is None:
                faults.append(f"{path} did not answer when asked which transitions it offers")
            else:
                offered[path] = answered
        return offered, faults

    @abstractmethod
    async def send(self, transition: LifecycleNodeTransition, paths: tuple[str, ...]) -> Sequence[str]:
        """Command one transition at the nodes that will accept it, the way this flavour does.

        Called only for a transition at least one of ``paths`` reported as available, so an
        implementation does not gate again.

        :param transition: What to command.
        :param paths: The nodes that offered it, a non-empty subset of :attr:`paths`.
        :return: One human-readable fault per refusal or missing answer; empty on success. Faults
            are returned so that an operator sees every cause, not the first.
        """

    async def _call(self, service: str, types: ServiceTypes, request: Message) -> Message | None:
        """Call one ROS2 service, answering ``None`` when the call does not happen.

        A node that is off is a runtime condition, not a fault. The commanding path
        turns the missing answer into a message.
        """
        try:
            return await self._runtime.call_service(
                service=service,
                service_type=types,
                request=request,
                server_timeout=self._binding.server_timeout,
                call_timeout=self._binding.call_timeout,
            )
        except (ServiceUnavailableError, ServiceCallTimeoutError):
            return None


class StandardLifecycleDriver(LifecycleDriver):
    """The ROS2 standard: ``lifecycle_msgs``, one ``change_state`` call per node.

    The default flavour. Its ``path`` is the node itself.
    """

    flavour: ClassVar[str] = "lifecycle"

    def __init__(self, *, binding: LifecycleBinding, runtime: RuntimeInterface) -> None:
        """Resolve the standard services, and the ``ChangeState`` request this flavour builds."""
        super().__init__(binding=binding, runtime=runtime)
        self._change: ServiceTypes = runtime.describe_service(CHANGE_STATE_TYPE)
        self._request_type: Any = self._change.request
        self._transition_type: Any = self._change.request.ros2_nested["transition"]

    async def send(self, transition: LifecycleNodeTransition, paths: tuple[str, ...]) -> Sequence[str]:
        """Call ``<node>/change_state`` once per node, and report every node that said no."""
        request = self._request_type(transition=self._transition_type(id=int(transition), label=""))
        faults: list[str] = []
        for path in paths:
            answer: Any = await self._call(f"{path}/change_state", self._change, request)
            if answer is None:
                faults.append(f"{path} did not answer a {transition.name.lower()} request")
            elif not answer.success:
                faults.append(f"{path} refused to {transition.name.lower()}")
        return faults


class Nav2ManagerDriver(LifecycleDriver):
    """A nav2-style manager: one ``manage_nodes`` call covering the whole set it drives.

    The declaration's ``path`` is the manager and ``nodes`` are the lifecycle nodes it drives,
    which are observed and gated like the standard flavour's. The manager commands its whole
    set and this Driver cannot narrow that: if some nodes are ``active`` and some ``inactive``,
    the manager answers ``success=False`` for the half that could not take the transition, and
    that is reported. A halt also reaches nodes the Skill did not start. The gate still prevents a
    call when no node in the set can take the transition.
    """

    flavour: ClassVar[str] = "nav2"
    coordinates_a_set: ClassVar[bool] = True

    def __init__(self, *, binding: LifecycleBinding, runtime: RuntimeInterface) -> None:
        """Resolve the standard observation services, and nav2's own management service."""
        super().__init__(binding=binding, runtime=runtime)
        self._manage: ServiceTypes = runtime.describe_service(MANAGE_NODES_TYPE)
        self._request_type: Any = self._manage.request

    async def send(self, transition: LifecycleNodeTransition, paths: tuple[str, ...]) -> Sequence[str]:
        """Call ``<manager>/manage_nodes`` once, with the command that sends this transition."""
        manager = self._binding.path
        request = self._request_type(command=NAV2_COMMANDS[transition])
        answer: Any = await self._call(f"{manager}/manage_nodes", self._manage, request)
        listed = ", ".join(paths)
        if answer is None:
            return [f"the manager {manager} did not answer a {transition.name.lower()} request for {listed}"]
        if not answer.success:
            return [f"the manager {manager} refused to {transition.name.lower()} {listed}"]
        return []


DRIVERS: Final[Mapping[str, type[LifecycleDriver]]] = {
    StandardLifecycleDriver.flavour: StandardLifecycleDriver,
    Nav2ManagerDriver.flavour: Nav2ManagerDriver,
}
"""Which coordinator flavour a declaration's ``driver=`` selects.

A fixed table, so that a Binding file chooses among these by name and cannot introduce a class.
A third flavour subclasses :class:`LifecycleDriver` and is passed as the class.
"""


def _state_id(message: object, field: str) -> int:
    """Return the ``id`` of one ``lifecycle_msgs/msg/State`` field, or ``UNKNOWN`` if it is missing.

    ``UNKNOWN`` reads as no opinion instead of a traceback on a later task.
    """
    return int(getattr(getattr(message, field, None), "id", LifecycleNodeState.UNKNOWN))


def _as_state(identifier: object) -> LifecycleNodeState:
    """Return the lifecycle state one id names, or ``UNKNOWN`` for a number ROS2 has not defined."""
    try:
        return LifecycleNodeState(int(identifier))  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return LifecycleNodeState.UNKNOWN


def _state_name(identifier: int) -> str:
    """Return a readable name for one lifecycle state id, for a representation or a log line."""
    return _as_state(identifier).name.lower()
