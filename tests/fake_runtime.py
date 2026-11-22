# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The single test double: a Runtime with no ROS2 behind it.

It is not part of the distribution. It reuses the real Subscription, SubscriptionSink, GraphFacet
and QoSNegotiation, and exposes levers (`note_publishers`, `note_qos_incompatible`) in place of
middleware events. It cannot show the effect of a QoS profile on the wire; the Jazzy tests do that.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, final

from opensmi.ros2.runtime.errors import (
    ActionServerUnavailableError,
    FieldTypeMismatchError,
    LoopAffinityError,
    MissingInterfacePackageError,
    PublishFailedError,
    RuntimeAlreadyStartedError,
    RuntimeNotStartedError,
    ServiceCallTimeoutError,
    ServiceUnavailableError,
)
from opensmi.ros2.runtime.goal import Goal, GoalSink, rejected_outcome, settled_outcome
from opensmi.ros2.runtime.graph import EMPTY_GRAPH, POLL_INTERVAL, GraphFacet, GraphSnapshot
from opensmi.ros2.runtime.guard import require_sendable
from opensmi.ros2.runtime.messages import (
    ActionTypes,
    ArrayType,
    NestedType,
    PrimitiveType,
    SequenceType,
    ServiceTypes,
    is_byte_container,
)
from opensmi.ros2.runtime.outcomes import GoalStatus, StopOutcome
from opensmi.ros2.runtime.parameters import (
    ParameterDescription,
    ParameterFacet,
    ParameterReading,
    ParameterUpdate,
    ParameterValue,
    SetOutcome,
    absolute,
)
from opensmi.ros2.runtime.publisher import Publisher
from opensmi.ros2.runtime.qos import (
    DEFAULT_PROFILE,
    EndpointInfo,
    Policy,
    Provenance,
    QoSNegotiation,
    QoSProfile,
)
from opensmi.ros2.runtime.subscription import Subscription, SubscriptionSink, TypeHashMismatch
from opensmi.ros2.runtime.tf import TRANSFORM_CACHE, TFFacet, Transform

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator, Callable, Mapping

    from opensmi.ros2.runtime.messages import FieldType, Message


@final
class FakeRuntime:
    """A Runtime that never touches ROS2, and the lever a test drives it with."""

    def __init__(
        self,
        *,
        domain_id: int = 0,
        node_name: str = "fake_runtime",
        namespace: str | None = None,
        stop_timeout: float = 5.0,
        graph_poll_interval: float = POLL_INTERVAL,
        transform_cache: float = TRANSFORM_CACHE,
        message_types: Mapping[str, type[Message]] | None = None,
        action_types: Mapping[str, ActionTypes] | None = None,
        service_types: Mapping[str, ServiceTypes] | None = None,
        graph: GraphSnapshot = EMPTY_GRAPH,
        clock: tuple[int, int] = (0, 0),
    ) -> None:
        """Describe a fake Runtime.

        :param message_types: What a ROS2 type name resolves to, standing in for the installed
            interface packages. A name that is not here raises `MissingInterfacePackageError`, so a
            test of the fatal-at-init rule does not need a ROS2 installation to write.
        :param action_types: The same, for an action's trio of types.
        :param service_types: The same, for a service's pair of types.
        :param graph: What the graph facet reads. Moved by `set_graph`, which is the lever a test
            of the graph-driven case pulls.
        :param clock: What a Publisher stamps a header with, standing in for the ROS2 node's own
            clock. Moved by `set_clock`. It starts at zero, which means *unset* in ROS2 and is
            therefore the reading under which a stamp is left exactly as it arrived -- a test of
            the stamping has to say what time it is, the way a test of the graph has to say what
            is on it.
        """
        self._domain_id = domain_id
        self._node_name = node_name
        self._namespace = namespace
        self._stop_timeout = stop_timeout
        self._message_types = dict(message_types or {})
        self._action_types = dict(action_types or {})
        self._service_types = dict(service_types or {})
        self._started = False
        self._loop: asyncio.AbstractEventLoop | None = None
        self._sinks: dict[str, list[SubscriptionSink]] = {}
        self._negotiations: dict[SubscriptionSink, QoSNegotiation] = {}
        self._offers: dict[str, tuple[EndpointInfo, ...]] = {}
        self._subscriptions: list[Subscription] = []
        self._servers: dict[str, bool] = {}
        self._services: dict[str, Callable[[Message], Message] | None] = {}
        self._requests: list[tuple[str, Message]] = []
        self._goal_sinks: dict[str, list[GoalSink]] = {}
        self._goals: list[Goal] = []
        self._cancel_requests: list[str] = []
        self._sent_goals: list[tuple[str, Message]] = []
        self._graph_snapshot = graph
        self._graph = GraphFacet(read=self._read_graph, interval=graph_poll_interval)
        self._transform_cache = transform_cache
        self._transforms: dict[tuple[str, str], Transform] = {}
        self._fed: list[tuple[Message, bool]] = []
        self._clock = clock
        self._tf = TFFacet(
            subscribe=self.subscribe,
            advertise=self.advertise,
            feed=self._feed_transforms,
            lookup=self._lookup_transform,
            clock=lambda: self._clock,
        )
        self._publishers: list[Publisher] = []
        self._published: list[tuple[str, Message]] = []
        self._refused: dict[str, int | None] = {}
        self._held: dict[str, dict[str, ParameterValue | None]] = {}
        self._parameter_refusals: dict[tuple[str, str], str] = {}
        self._unsendable_parameters: dict[tuple[str, str], str] = {}
        self._parameter_reads: list[tuple[str, str]] = []
        self._parameter_writes: list[tuple[str, dict[str, ParameterValue]]] = []
        self._parameter_updates: asyncio.Queue[ParameterUpdate] = asyncio.Queue()
        self._parameter_write_gate: asyncio.Event | None = None
        self._descriptions: dict[tuple[str, str], ParameterDescription] = {}
        self._parameters = ParameterFacet(
            read=self._read_parameter,
            write=self._write_parameters,
            events=self._follow_parameter_events,
            list_names=self._list_parameters,
            describe=self._describe_parameter,
        )

    def __repr__(self) -> str:
        """Return a representation naming the node and its Domain ID."""
        state = "started" if self.started else "stopped"
        return f"<FakeRuntime {self._node_name} on domain {self._domain_id} ({state})>"

    @property
    def domain_id(self) -> int:
        """Return the ROS2 Domain ID this Runtime is confined to."""
        return self._domain_id

    @property
    def node_name(self) -> str:
        """Return the name of the single ``rclpy`` Node this Runtime owns."""
        return self._node_name

    @property
    def started(self) -> bool:
        """Return whether the Context, Node and executor thread are up."""
        return self._started

    @property
    def graph(self) -> GraphFacet:
        """Return the facet answering what is on the ROS2 graph."""
        return self._graph

    @property
    def parameters(self) -> ParameterFacet:
        """Return the facet answering what a ROS2 node's parameters are, and writing them."""
        return self._parameters

    @property
    def tf(self) -> TFFacet:
        """Return the facet answering what the TF frame network looks like, and where a frame sits."""
        return self._tf

    def serve_transform(self, *, target: str, source: str, transform: Transform | None) -> None:
        """Say what a composed lookup answers, which is the lever a test of an alternative pulls.

        Named for what it does here rather than for tf2's ``set_transform``, which *feeds a
        buffer*: there is no buffer in this fake and nothing pretends to be one, so a test says
        what a chain composes to instead of publishing the links it would compose from.
        """
        if transform is None:
            self._transforms.pop((target, source), None)
            return
        self._transforms[(target, source)] = transform

    def _feed_transforms(self, message: Message, static: bool) -> None:
        """Take a transform message the way the real Runtime hands one to its buffer."""
        self._fed.append((message, static))

    def _lookup_transform(self, target: str, source: str) -> Transform | None:
        """Answer a composed lookup out of what a test put there, or not at all."""
        return self._transforms.get((target, source))

    async def start(self) -> None:
        """Bring up the Context, the Node and the executor thread."""
        if self._started:
            message = f"{self._node_name} is already started on domain {self._domain_id}"
            raise RuntimeAlreadyStartedError(message)
        self._loop = asyncio.get_running_loop()
        self._started = True

    async def stop(self) -> StopOutcome:
        """Close every open subscription, then tear down the thread, the Node and the Context."""
        self._require_started()
        self._require_loop()
        await self._graph.close()
        await self._tf.close()
        await self._parameters.close()
        for goal in tuple(self._goals):
            await goal.close()
        self._goals.clear()
        self._goal_sinks.clear()
        for subscription in tuple(self._subscriptions):
            await subscription.close()
        self._subscriptions.clear()
        self._sinks.clear()
        for publisher in tuple(self._publishers):
            await publisher.close()
        self._publishers.clear()
        self._started = False
        self._loop = None
        return StopOutcome(faults=(), executor_thread_joined=True)

    async def subscribe(
        self,
        *,
        topic: str,
        message_type: str | type[Message],
        depth: int = 10,
        qos: QoSProfile | None = None,
    ) -> Subscription:
        """Subscribe to a ROS2 topic, and return the handle that owns the subscription."""
        self._require_started()
        self._require_loop()
        resolved = self._resolve(message_type) if isinstance(message_type, str) else message_type
        negotiation = QoSNegotiation(declared=qos, endpoints=self.publishers_on(topic))
        sink = SubscriptionSink(depth=depth, qos=negotiation.profile, provenance=negotiation.provenance)

        async def release() -> None:
            self._sinks.get(topic, []).remove(sink)
            self._negotiations.pop(sink, None)
            if handle in self._subscriptions:
                self._subscriptions.remove(handle)

        handle = Subscription(topic=topic, message_type=resolved, sink=sink, release=release)
        self._sinks.setdefault(topic, []).append(sink)
        self._negotiations[sink] = negotiation
        self._subscriptions.append(handle)
        return handle

    async def advertise(
        self,
        *,
        topic: str,
        message_type: str | type[Message],
        depth: int = 10,
        qos: QoSProfile | None = None,
    ) -> Publisher:
        """Advertise a ROS2 topic, and return the handle that owns the publisher."""
        self._require_started()
        self._require_loop()
        resolved = self._resolve(message_type) if isinstance(message_type, str) else message_type

        async def send(message: Message) -> None:
            self._refuse_if_asked(topic)
            self._published.append((topic, message))

        async def release() -> None:
            if handle in self._publishers:
                self._publishers.remove(handle)

        handle = Publisher(
            topic=topic,
            message_type=resolved,
            clock=lambda: self._clock,
            send=send,
            release=release,
            qos=qos if qos is not None else DEFAULT_PROFILE,
            provenance=Provenance.DECLARED if qos is not None else Provenance.DEFAULTED,
        )
        self._publishers.append(handle)
        return handle

    def publishers_on(self, topic: str) -> tuple[EndpointInfo, ...]:
        """Return what every publisher on a topic offers, as the QoS resolver takes it."""
        return self._offers.get(topic, ())

    def describe_message(self, message_type: str) -> type[Message]:
        """Return the message dataclass of a ROS2 message type."""
        return self._resolve(message_type)

    def default_message(self, message_type: type[Message]) -> Message:
        """Return a message of a type with every field at its type's zero.

        A fixture type has no interface definition to declare other defaults, so the zero is what
        its generated class would construct.
        """
        return _zero_message(message_type)

    def describe_action(self, action_type: str) -> ActionTypes:
        """Return the goal, feedback and result message dataclasses of a ROS2 action type."""
        return self._resolve_action(action_type)

    def describe_service(self, service_type: str) -> ServiceTypes:
        """Return the request and response message dataclasses of a ROS2 service type."""
        return self._resolve_service(service_type)

    async def call_service(
        self,
        *,
        service: str,
        service_type: str | ServiceTypes,
        request: Message,
        server_timeout: float = 10.0,
        call_timeout: float = 30.0,
    ) -> Message:
        """Call one ROS2 service and return the response it answered with."""
        self._require_started()
        self._require_loop()
        types = self._resolve_service(service_type) if isinstance(service_type, str) else service_type
        if not isinstance(request, types.request):
            message = (
                f"the request for {service} is a {type(request).__name__}, and {types.type_name} "
                f"takes a {types.request.__name__}"
            )
            raise TypeError(message)
        require_sendable(request)
        if service not in self._services:
            message = (
                f"no service server answered on {service} within {server_timeout} seconds, so the request was not sent"
            )
            raise ServiceUnavailableError(message)
        self._requests.append((service, request))
        answer = self._services[service]
        if answer is None:
            message = f"the service on {service} took the request and did not answer within {call_timeout} seconds"
            raise ServiceCallTimeoutError(message)
        return answer(request)

    async def send_goal(
        self,
        *,
        action: str,
        action_type: str | ActionTypes,
        goal: Message,
        depth: int = 10,
        server_timeout: float = 10.0,
    ) -> Goal:
        """Send one goal to a ROS2 action server, and return the handle that owns it."""
        self._require_started()
        self._require_loop()
        types = self._resolve_action(action_type) if isinstance(action_type, str) else action_type
        if not isinstance(goal, types.goal):
            message = (
                f"the goal for {action} is a {type(goal).__name__}, and {types.type_name} takes a {types.goal.__name__}"
            )
            raise TypeError(message)
        require_sendable(goal)
        if action not in self._servers:
            message = f"no action server answered on {action} within {server_timeout} seconds, so the goal was not sent"
            raise ActionServerUnavailableError(message)

        self._sent_goals.append((action, goal))
        sink = GoalSink(depth=depth)

        async def request_cancel() -> None:
            self._cancel_requests.append(action)

        async def release() -> None:
            if sink in self._goal_sinks.get(action, []):
                self._goal_sinks[action].remove(sink)
            if handle in self._goals:
                self._goals.remove(handle)

        handle = Goal(
            action=action,
            types=types,
            sink=sink,
            request_cancel=request_cancel,
            release=release,
        )
        if not self._servers[action]:
            sink.settle(rejected_outcome(action))
        self._goal_sinks.setdefault(action, []).append(sink)
        self._goals.append(handle)
        return handle

    # -- the test's side of the fake -------------------------------------------------------

    def deliver(self, topic: str, message: Message) -> int:
        """Hand a message to every subscription on a topic, and return how many got it."""
        sinks = self._sinks.get(topic, [])
        for sink in sinks:
            sink.deliver(message)
        return len(sinks)

    @property
    def subscribed_topics(self) -> tuple[str, ...]:
        """Return every topic something is subscribed to right now, in sorted order.

        The lever a test of a self-disabling facet reads: what makes one affordable is that it
        opens no subscription until something asks, and there is no other way to see that from
        outside the facet.
        """
        return tuple(sorted(topic for topic, sinks in self._sinks.items() if sinks))

    @property
    def advertised_topics(self) -> tuple[str, ...]:
        """Return every topic something is advertising right now, in sorted order.

        The outward mirror of `subscribed_topics`, and the lever a test of the TF facet's outward
        self-disable rule reads: nothing of ours appears on `/tf` as a publisher until a
        declaration says it publishes a frame, and a closed handle is off this list again.
        """
        return tuple(sorted(publisher.topic for publisher in self._publishers if not publisher.closed))

    def note_type_hash_mismatch(self, mismatch: TypeHashMismatch) -> None:
        """Record a type-hash disagreement on every subscription to the mismatch's topic."""
        for sink in self._sinks.get(mismatch.topic, []):
            sink.note_type_hash_mismatch(mismatch)

    def note_publishers(self, topic: str, count: int) -> None:
        """Stand in for a match event: say how many publishers a topic now has.

        It spends the one bootstrap tightening exactly as the real match callback does, so a test
        of "a latched publisher appeared after we subscribed" drives the production rule rather
        than a re-implementation of it.
        """
        for sink in self._sinks.get(topic, []):
            sink.note_publishers(count)
            if count <= 0:
                continue
            negotiation = self._negotiations.get(sink)
            if negotiation is None:
                continue
            tightened = negotiation.on_first_publisher(self.publishers_on(topic))
            if tightened is not None:
                sink.note_qos(tightened)

    def set_publisher_qos(self, topic: str, *profiles: QoSProfile) -> None:
        """Say what the publishers on a topic offer, which is what discovery resolves over.

        Deliberately separate from `note_publishers`: on a robot the graph read and the match
        event are two different things arriving at two different moments, and a test of the
        bootstrap needs to put a publisher on the graph *before* saying the subscription matched
        it. Passing no profile leaves the topic with no publishers.
        """
        self._offers[topic] = tuple(
            EndpointInfo(node_name=f"/publisher_{index}", qos=profile) for index, profile in enumerate(profiles)
        )

    def note_qos_incompatible(self, topic: str, policy: Policy) -> None:
        """Stand in for the middleware reporting that a publisher offers QoS we cannot accept.

        Drives the real rule: a discovered profile relaxes towards what `set_publisher_qos` says
        is there, a declared one does not move, and either way the incompatibility is recorded on
        the subscription for whatever is following it.
        """
        for sink in self._sinks.get(topic, []):
            negotiation = self._negotiations.get(sink)
            if negotiation is None:  # pragma: no cover -- every sink here was opened with one
                continue
            reported = negotiation.incompatibility(topic, policy, self.publishers_on(topic))
            sink.note_incompatibility(reported)
            if reported.adapted is not None:
                sink.note_qos(reported.adapted)

    @property
    def published(self) -> tuple[tuple[str, Message], ...]:
        """Return every message that went out, by topic, in the order it was published.

        The payload recorded is the one that reached the send step -- stamped and checked -- and
        not the one the caller handed over, because that is what a real subscriber would see.
        """
        return tuple(self._published)

    def refuse_publishing(self, topic: str, *, times: int | None = None) -> None:
        """Make publishing on a topic fail, as a middleware with nowhere to put a message does.

        `times=None` refuses for ever; a count refuses that many sends and then lets the next one
        through, which is what a test of "tolerates N consecutive failures and recovers" needs.
        Passing zero clears the refusal.
        """
        self._refused[topic] = times

    def set_clock(self, seconds: int, nanoseconds: int = 0) -> None:
        """Move the clock a Publisher stamps a header from, as time passing moves the real one."""
        self._clock = (seconds, nanoseconds)

    def set_graph(self, graph: GraphSnapshot) -> None:
        """Move the graph the facet reads, as a robot coming up moves the real one."""
        self._graph_snapshot = graph

    def serve_service(self, service: str, answer: Callable[[Message], Message] | None = None) -> None:
        """Say that a service server answers on a path, and with what.

        A service nobody has served here raises `ServiceUnavailableError`, which is the real
        Runtime's answer to a path with nothing on it. `answer=None` is the server that takes the
        request and never replies, which is the other transport failure and a different error.
        """
        self._services[service] = answer

    @property
    def requests(self) -> tuple[tuple[str, Message], ...]:
        """Return every request that reached a service, in the order it was sent."""
        return tuple(self._requests)

    def serve_action(self, action: str, *, accepted: bool = True) -> None:
        """Say that an action server answers on a path, and whether it takes the goals it gets.

        An action nobody has served here raises `ActionServerUnavailableError`, which is the
        real Runtime's answer to a path with nothing on it.
        """
        self._servers[action] = accepted

    def deliver_feedback(self, action: str, feedback: Message) -> int:
        """Hand a feedback message to every goal in flight on an action, and say how many got it."""
        sinks = [sink for sink in self._goal_sinks.get(action, []) if not sink.settled]
        for sink in sinks:
            sink.deliver(feedback)
        return len(sinks)

    def settle_goal(self, action: str, *, status: GoalStatus, result: Message | None = None) -> int:
        """End every goal in flight on an action, as a server answering with a result does."""
        sinks = [sink for sink in self._goal_sinks.get(action, []) if not sink.settled]
        for sink in sinks:
            sink.settle(settled_outcome(status, result))
        return len(sinks)

    @property
    def sent_goals(self) -> tuple[tuple[str, Message], ...]:
        """Return every goal that reached an action server, with its action, in the order it was sent."""
        return tuple(self._sent_goals)

    @property
    def cancel_requests(self) -> tuple[str, ...]:
        """Return the action of every cancel asked for, in the order they were asked."""
        return tuple(self._cancel_requests)

    def serve_parameters(self, node: str, values: Mapping[str, ParameterValue | None] | None = None) -> None:
        """Say that a ROS2 node's parameter services answer, and which parameters it holds.

        A node nobody has served here raises `ServiceUnavailableError`, which is the real
        Runtime's answer to a node that is not running. A name mapped to `None` is declared on
        the node and holds nothing, which is ROS2's `PARAMETER_NOT_SET`; a name that is absent
        from the mapping is a name the node does not have, and is what a typo in a declaration
        meets.
        """
        self._held[absolute(node)] = dict(values or {})

    def describe_parameter(self, node: str, name: str, description: ParameterDescription) -> None:
        """Say what a node answers `describe_parameters` with for one of its parameters.

        A lever rather than something derived from `serve_parameters`, because a descriptor is
        prose and constraints the node wrote and nothing here could invent them. A name served
        but never described answers with its declared type and no prose, which is what a node
        that wrote no descriptor answers.
        """
        self._descriptions[(absolute(node), name)] = description

    def stop_parameters(self, node: str) -> None:
        """Say that a ROS2 node's parameter services no longer answer, as a stopped node does.

        The mirror image of `serve_parameters`, and the lever behind the *call never happened*
        half of the parameter split: every call to this node raises from here on.
        """
        self._held.pop(absolute(node), None)

    def refuse_parameter(self, node: str, name: str, reason: str) -> None:
        """Make a node refuse writes to one parameter, with its own reason text.

        This is the answered-and-said-no half of the parameter split: the call happened, so nothing
        raises, and the refusal comes back as one parameter's outcome.
        """
        self._parameter_refusals[(absolute(node), name)] = reason

    def refuse_parameter_value(self, node: str, name: str, reason: str) -> None:
        """Make the guard behind the seam refuse a write of one parameter, before it is sent.

        The third of the parameter failures, and the one the node has nothing to do with: the real
        Runtime builds the request and runs `require_sendable` over it inside `call_service`, so
        a value the declared ROS2 type cannot carry raises there and no call goes out. Modelled
        as a lever rather than by re-deriving the guard's rules here, so that this fake cannot
        drift from `runtime.guard` by agreeing with a stale copy of it.
        """
        self._unsendable_parameters[(absolute(node), name)] = reason

    def hold_parameter_writes(self) -> asyncio.Event:
        """Leave every `set_parameters` answer in flight until the returned event is set.

        A real node answers a write after it has already published the change, so a test that
        wants an event to land *while* an answer is on its way has to be able to hold the
        answer. Handed back rather than kept, so that a test reads as "hold, then release".
        """
        gate = asyncio.Event()
        self._parameter_write_gate = gate
        return gate

    def deliver_parameter_event(self, update: ParameterUpdate) -> None:
        """Announce a parameter change on `/parameter_events`, as a node changing one does.

        Never fired by `set` itself, for the reason publishing here does not deliver to this
        fake's own subscriptions: what a test drives and what a test observes stay separate.
        """
        self._parameter_updates.put_nowait(update)

    @property
    def parameter_reads(self) -> tuple[tuple[str, str], ...]:
        """Return every parameter read, as node and name, in the order they were asked.

        One entry per name is the assertion the one-per-request rule is checked with.
        """
        return tuple(self._parameter_reads)

    @property
    def parameter_writes(self) -> tuple[tuple[str, Mapping[str, ParameterValue]], ...]:
        """Return every `set_parameters` call, as node and batch, in the order they were made."""
        return tuple(self._parameter_writes)

    async def _read_parameter(self, node: str, name: str) -> ParameterReading:
        self._parameter_reads.append((node, name))
        held = self._held.get(node)
        if held is None:
            message = f"no service server answered on {node}/get_parameters, so the request was not sent"
            raise ServiceUnavailableError(message)
        if name not in held:
            return ParameterReading(name=name, present=False)
        return ParameterReading(name=name, present=True, value=held[name])

    async def _list_parameters(self, node: str) -> tuple[str, ...]:
        held = self._held.get(node)
        if held is None:
            message = f"no service server answered on {node}/list_parameters, so the request was not sent"
            raise ServiceUnavailableError(message)
        return tuple(held)

    async def _describe_parameter(self, node: str, name: str) -> ParameterDescription:
        self._parameter_reads.append((node, name))
        held = self._held.get(node)
        if held is None:
            message = f"no service server answered on {node}/describe_parameters, so the request was not sent"
            raise ServiceUnavailableError(message)
        if name not in held:
            return ParameterDescription(name=name, present=False)
        described = self._descriptions.get((node, name))
        if described is not None:
            return described
        value = held[name]
        return ParameterDescription(name=name, present=True, type=None if value is None else value.type)

    async def _write_parameters(
        self,
        node: str,
        values: Mapping[str, ParameterValue],
    ) -> Mapping[str, SetOutcome]:
        for name in values:
            unsendable = self._unsendable_parameters.get((node, name))
            if unsendable is not None:
                # Recorded after the guard: in the real Runtime a refused request never reaches the wire.
                raise FieldTypeMismatchError(unsendable)
        self._parameter_writes.append((node, dict(values)))
        held = self._held.get(node)
        if held is None:
            message = f"no service server answered on {node}/set_parameters, so the request was not sent"
            raise ServiceUnavailableError(message)
        outcomes: dict[str, SetOutcome] = {}
        for name, value in values.items():
            reason = self._parameter_refusals.get((node, name))
            if reason is not None:
                outcomes[name] = SetOutcome(successful=False, reason=reason)
            elif name not in held:
                outcomes[name] = SetOutcome(successful=False, reason=f"Parameter '{name}' is not declared")
            else:
                held[name] = value
                outcomes[name] = SetOutcome(successful=True)
        if self._parameter_write_gate is not None:
            # Held after the node took the values and before the answer: the window a /parameter_events change lands in.
            await self._parameter_write_gate.wait()
        return outcomes

    async def _follow_parameter_events(self) -> AsyncGenerator[ParameterUpdate, None]:
        while True:
            yield await self._parameter_updates.get()

    def _refuse_if_asked(self, topic: str) -> None:
        """Raise the fault the real Runtime wraps an rclpy refusal in, if this topic is refusing."""
        remaining = self._refused.get(topic)
        if remaining is None and topic not in self._refused:
            return
        if remaining == 0:
            return
        if remaining is not None:
            self._refused[topic] = remaining - 1
        message = f"rclpy refused to publish on {topic}: this fake Runtime was told to refuse"
        raise PublishFailedError(message)

    async def _read_graph(self) -> GraphSnapshot:
        return self._graph_snapshot

    def _resolve(self, type_name: str) -> type[Message]:
        try:
            return self._message_types[type_name]
        except KeyError as error:
            message = f"no interface package for {type_name!r} was given to this fake Runtime"
            raise MissingInterfacePackageError(message) from error

    def _resolve_action(self, type_name: str) -> ActionTypes:
        try:
            return self._action_types[type_name]
        except KeyError as error:
            message = f"no interface package for the action {type_name!r} was given to this fake Runtime"
            raise MissingInterfacePackageError(message) from error

    def _resolve_service(self, type_name: str) -> ServiceTypes:
        try:
            return self._service_types[type_name]
        except KeyError as error:
            message = f"no interface package for the service {type_name!r} was given to this fake Runtime"
            raise MissingInterfacePackageError(message) from error

    def _require_started(self) -> None:
        if not self._started:
            message = f"{self._node_name} is not started; call start() before using the Runtime"
            raise RuntimeNotStartedError(message)

    def _require_loop(self) -> None:
        if self._loop is not None and self._loop is not asyncio.get_running_loop():
            message = (
                f"{self._node_name} is affine to the event loop it was started on; this call is "
                f"on another one, which would receive none of its messages"
            )
            raise LoopAffinityError(message)


def _zero_message(message_type: type[Message]) -> Message:
    """Return a message whose every field holds its type's zero, nested messages included."""
    return message_type(
        **{
            field.name: _zero(field.field_type, message_type.ros2_nested.get(field.name))
            for field in message_type.ros2.fields
        }
    )


def _zero(field_type: FieldType, nested: type[Message] | None) -> object:
    """Return the zero of one field: ``0``, ``0.0``, ``False``, an empty string or sequence."""
    if is_byte_container(field_type):
        return bytes(field_type.length) if isinstance(field_type, ArrayType) else b""
    match field_type:
        case ArrayType(element=element, length=length):
            return tuple(_zero(element, nested) for _ in range(length))
        case SequenceType():
            return ()
        case NestedType():
            assert nested is not None
            return _zero_message(nested)
        case PrimitiveType(name="bool"):
            return False
        case PrimitiveType(name="float32" | "float64"):
            return 0.0
        case PrimitiveType(name="char" | "wchar"):
            return chr(0)
        case PrimitiveType():
            return 0
        case _:
            return ""
