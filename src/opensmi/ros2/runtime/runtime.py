# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Runtime: the sole owner of ROS2 access for one Machine.

This is the only module in the package that imports ``rclpy``. One Runtime owns one ``rclpy``
Context on its own Domain ID, one Node and one executor thread. There is no registry and no
process global, so two Machines on two Domain IDs run in one process without discovering each
other.

No ``rclpy`` type crosses this interface. Every operation is async and must be awaited on the
loop :meth:`Runtime.start` ran on (otherwise it raises ``LoopAffinityError``). Callbacks that
``rclpy`` invokes on the executor thread only hop to the loop with ``call_soon_threadsafe``;
decoding and buffering happen there. Blocking ``rclpy`` calls run through ``asyncio.to_thread``.
QoS adaptation lives here too (see :mod:`opensmi.ros2.runtime.qos`).
"""

from __future__ import annotations

import asyncio
import threading
import warnings
from typing import TYPE_CHECKING, Any, Final, final

import rclpy.context
import rclpy.executors
import rclpy.node
import rclpy.qos
import structlog
import tf2_ros
from rclpy.action import ActionClient, get_action_names_and_types
from rclpy.duration import Duration
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.time import Time

from opensmi.ros2.runtime._decode import decode
from opensmi.ros2.runtime._encode import encode
from opensmi.ros2.runtime._parser import import_action_class, import_interface_class, import_service_class
from opensmi.ros2.runtime.errors import (
    ActionServerUnavailableError,
    LoopAffinityError,
    PublishFailedError,
    RuntimeAlreadyStartedError,
    RuntimeNotStartedError,
    ServiceCallTimeoutError,
    ServiceUnavailableError,
)
from opensmi.ros2.runtime.goal import Goal, GoalSink, rejected_outcome, settled_outcome
from opensmi.ros2.runtime.graph import POLL_INTERVAL, GraphFacet, GraphSnapshot
from opensmi.ros2.runtime.guard import require_sendable
from opensmi.ros2.runtime.messages import (
    ActionTypes,
    ServiceTypes,
    action_types,
    message_dataclass,
    service_types,
)
from opensmi.ros2.runtime.outcomes import GoalOutcome, GoalStatus, StopOutcome
from opensmi.ros2.runtime.parameters import (
    CALL_TIMEOUT,
    DESCRIBE_PARAMETERS_TYPE,
    EVENT_DEPTH,
    GET_PARAMETERS_TYPE,
    LIST_PARAMETERS_TYPE,
    PARAMETER_EVENT_TYPE,
    PARAMETER_EVENTS_TOPIC,
    SERVER_TIMEOUT,
    SET_PARAMETERS_TYPE,
    ParameterDescription,
    ParameterFacet,
    ParameterReading,
    ParameterValue,
    SetOutcome,
    describe_request,
    description_of,
    get_request,
    list_request,
    names_of,
    outcomes_of,
    reading_of,
    set_request,
    update_of,
)
from opensmi.ros2.runtime.publisher import Publisher
from opensmi.ros2.runtime.qos import (
    DEFAULT_PROFILE,
    FOREVER,
    Durability,
    EndpointInfo,
    Liveliness,
    Policy,
    Provenance,
    QoSNegotiation,
    QoSProfile,
    Reliability,
)
from opensmi.ros2.runtime.subscription import (
    Subscription,
    SubscriptionSink,
    TypeHashMismatch,
    TypeHashMismatchWarning,
)
from opensmi.ros2.runtime.tf import TRANSFORM_CACHE, TFFacet, Transform

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator, Awaitable, Callable, Iterable, Mapping

    from opensmi.ros2.runtime.messages import Message
    from opensmi.ros2.runtime.parameters import ParameterUpdate

__all__ = ["Runtime"]

# rclpy renders an unknown type hash as this literal; comparing against it would report every
# endpoint on a middleware that does not distribute hashes as drift.
_UNKNOWN_TYPE_HASH = "INVALID"

_LOGGER = structlog.get_logger("opensmi.ros2.runtime.runtime")


@final
class Runtime:
    """All ROS2 access for one Machine, which is the unit of one Domain ID."""

    def __init__(
        self,
        *,
        domain_id: int,
        node_name: str = "opensmi_ros2",
        namespace: str | None = None,
        stop_timeout: float = 5.0,
        graph_poll_interval: float = POLL_INTERVAL,
        transform_cache: float = TRANSFORM_CACHE,
    ) -> None:
        """Describe a Runtime without starting anything; construction touches no ROS2.

        :param domain_id: The ROS2 Domain ID this Runtime's Context is confined to.
        :param node_name: The name of the single ``rclpy`` Node this Runtime owns.
        :param namespace: The ROS2 namespace for that node, or ``None`` for the root.
        :param stop_timeout: Seconds :meth:`stop` waits for the executor thread.
        :param graph_poll_interval: Seconds between graph reads while the graph facet is watched.
        :param transform_cache: Seconds of transform history the TF buffer keeps.
        """
        self._domain_id = domain_id
        self._node_name = node_name
        self._namespace = namespace
        self._stop_timeout = stop_timeout
        self._context: rclpy.context.Context | None = None
        self._node: rclpy.node.Node | None = None
        self._executor: rclpy.executors.SingleThreadedExecutor | None = None
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._subscriptions: list[Subscription] = []
        self._publishers: list[Publisher] = []
        self._goals: list[Goal] = []
        self._transform_cache = transform_cache
        self._buffer: tf2_ros.Buffer | None = None
        self._graph = GraphFacet(read=self._read_graph, interval=graph_poll_interval)
        self._tf = TFFacet(
            subscribe=self.subscribe,
            advertise=self.advertise,
            feed=self._feed_transforms,
            lookup=self._lookup_transform,
            clock=self._ros2_clock,
        )
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
        return f"<Runtime {self._node_name} on domain {self._domain_id} ({state})>"

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
        return self._context is not None

    @property
    def graph(self) -> GraphFacet:
        """Return the facet answering what is on the ROS2 graph (it reads nothing until watched)."""
        return self._graph

    @property
    def parameters(self) -> ParameterFacet:
        """Return the facet for the parameters of ROS2 nodes (``/parameter_events`` only while watched)."""
        return self._parameters

    @property
    def tf(self) -> TFFacet:
        """Return the facet answering where a TF frame sits (``/tf`` is subscribed once a Binding asks)."""
        return self._tf

    async def start(self) -> None:
        """Bring up the Context, the Node and the executor thread on the running loop, which stays the Runtime's.

        :raises RuntimeAlreadyStartedError: When the Runtime is already up.
        """
        if self.started:
            message = f"{self._node_name} is already started on domain {self._domain_id}"
            raise RuntimeAlreadyStartedError(message)
        loop = asyncio.get_running_loop()
        await asyncio.to_thread(self._open)
        self._loop = loop

    def _open(self) -> None:
        """Create and spin the ROS2 objects (blocking, so called off the loop)."""
        context = rclpy.context.Context()
        try:
            context.init(args=None, domain_id=self._domain_id)
            node = rclpy.node.Node(self._node_name, context=context, namespace=self._namespace)
            executor = rclpy.executors.SingleThreadedExecutor(context=context)
            executor.add_node(node)
        except BaseException:
            context.try_shutdown()
            raise
        # A daemon thread, so a process that never stops its Runtime still exits.
        thread = threading.Thread(
            target=executor.spin,
            name=f"ros2-runtime-domain-{self._domain_id}",
            daemon=True,
        )
        thread.start()
        self._context = context
        self._node = node
        self._executor = executor
        self._thread = thread
        # No `TransformListener`: it would spin this Node on a second executor thread. A bare
        # `Buffer` is fed by the TF facet's own subscriptions.
        self._buffer = tf2_ros.Buffer(cache_time=Duration(seconds=self._transform_cache))

    async def stop(self) -> StopOutcome:
        """Close every open endpoint, then tear down the thread, the Node and the Context.

        Teardown faults are returned in the outcome, not raised: the Runtime is stopped either way.

        :raises RuntimeNotStartedError: When the Runtime was never started, or is already stopped.
        :raises LoopAffinityError: When awaited on a loop other than the Runtime's.
        """
        self._require_started()
        self._require_loop()
        await self._graph.close()
        # Before the subscriptions below: these facets own some of them.
        await self._tf.close()
        await self._parameters.close()
        for goal in tuple(self._goals):
            await goal.close()
        self._goals.clear()
        for subscription in tuple(self._subscriptions):
            await subscription.close()
        self._subscriptions.clear()
        for publisher in tuple(self._publishers):
            await publisher.close()
        self._publishers.clear()
        return await asyncio.to_thread(self._close)

    def _close(self) -> StopOutcome:
        """Tear the ROS2 objects down (blocking, so called off the loop), collecting faults."""
        executor, node, context, thread = self._executor, self._node, self._context, self._thread
        self._executor = self._node = self._context = self._thread = self._loop = None
        self._buffer = None
        faults: list[str] = []
        joined = False
        if executor is not None:
            faults += _faults_from(
                "shutting the executor down",
                lambda: executor.shutdown(timeout_sec=self._stop_timeout),
            )
        if thread is not None:
            thread.join(timeout=self._stop_timeout)
            joined = not thread.is_alive()
            if not joined:
                faults.append(f"the executor thread did not end within {self._stop_timeout} seconds")
        if node is not None:
            faults += _faults_from("destroying the node", node.destroy_node)
        if context is not None:
            faults += _faults_from("shutting the context down", context.try_shutdown)
        return StopOutcome(faults=tuple(faults), executor_thread_joined=joined)

    async def subscribe(
        self,
        *,
        topic: str,
        message_type: str | type[Message],
        depth: int = 10,
        qos: QoSProfile | None = None,
    ) -> Subscription:
        """Subscribe to a ROS2 topic, and return the handle that owns the subscription.

        A declared profile is honoured exactly; a discovered one relaxes as the topic's publishers
        require. The call does not wait for a publisher: it opens against whatever the graph shows
        now, possibly nothing.

        :param topic: The ROS2 topic name.
        :param message_type: The ROS2 type name (``package/Name`` or ``package/kind/Name``) or an
            already-resolved message dataclass.
        :param depth: The handle's buffer size, and the ROS2 queue depth.
        :param qos: The profile to request, or ``None`` to discover one: the most permissive
            request compatible with every publisher now, re-resolved once on the first publisher
            when there is none yet, and relaxed whenever the middleware reports an incompatibility.
        :raises RuntimeNotStartedError: When the Runtime is not up.
        :raises LoopAffinityError: When awaited on a loop other than the Runtime's.
        :raises MissingInterfacePackageError: When the type's interface package is not installed.
        """
        node = self._require_started()
        loop = self._require_loop()
        dataclass_type = self.describe_message(message_type) if isinstance(message_type, str) else message_type
        interface_class = import_interface_class(dataclass_type.ros2.type_name)

        negotiation = QoSNegotiation(declared=qos, endpoints=self.publishers_on(topic))
        sink = SubscriptionSink(depth=depth, qos=negotiation.profile, provenance=negotiation.provenance)

        async def release() -> None:
            await adaptive.close()
            if handle in self._subscriptions:
                self._subscriptions.remove(handle)

        handle = Subscription(topic=topic, message_type=dataclass_type, sink=sink, release=release)

        def on_loop(raw: Any) -> None:
            sink.deliver(decode(raw, dataclass_type))

        def on_message(raw: Any) -> None:
            # Executor thread: anything heavier than the hop stalls the wait set.
            loop.call_soon_threadsafe(on_loop, raw)

        def on_matched(info: Any) -> None:
            loop.call_soon_threadsafe(adaptive.matched, int(info.current_count))

        def on_incompatible(info: Any) -> None:
            # `last_policy_kind` is only the last clash the middleware found.
            loop.call_soon_threadsafe(adaptive.incompatible, _policy_of(info.last_policy_kind))

        async def create(profile: QoSProfile) -> Any:
            return await asyncio.to_thread(
                node.create_subscription,
                interface_class,
                topic,
                on_message,
                _rclpy_profile(profile, depth),
                event_callbacks=SubscriptionEventCallbacks(matched=on_matched, incompatible_qos=on_incompatible),
            )

        async def destroy(ros_subscription: Any) -> None:
            await asyncio.to_thread(self._destroy, "subscription", ros_subscription)

        adaptive = _AdaptiveSubscription(
            topic=topic,
            negotiation=negotiation,
            sink=sink,
            loop=loop,
            create=create,
            destroy=destroy,
            publishers=lambda: self.publishers_on(topic),
        )
        await adaptive.open()
        self._subscriptions.append(handle)
        self._report_type_hash_mismatches(topic, sink)
        return handle

    def publishers_on(self, topic: str) -> tuple[EndpointInfo, ...]:
        """Return what every publisher on a topic offers, as the QoS resolver takes it.

        This reads the node's local graph cache on the loop: callers decide synchronously whether
        to adapt, and an ``await`` in between would let two match events spend the one bootstrap
        tightening twice. Returns an empty tuple when the Runtime is not up, because middleware
        callbacks call it during teardown.
        """
        node = self._node
        if node is None:
            return ()
        return tuple(
            EndpointInfo(
                node_name=f"{info.node_namespace.rstrip('/')}/{info.node_name}",
                qos=_our_profile(info.qos_profile),
            )
            for info in node.get_publishers_info_by_topic(topic)
        )

    async def advertise(
        self,
        *,
        topic: str,
        message_type: str | type[Message],
        depth: int = 10,
        qos: QoSProfile | None = None,
    ) -> Publisher:
        """Advertise a ROS2 topic, and return the handle that owns the publisher.

        Nothing goes on the wire until :meth:`~opensmi.ros2.runtime.publisher.Publisher.publish`.
        The profile is declared or defaulted, never discovered or adapted, since this call is how
        a publisher on the topic comes to exist.

        :param topic: The ROS2 topic name.
        :param message_type: The ROS2 type name (``package/Name`` or ``package/kind/Name``) or an
            already-resolved message dataclass.
        :param depth: The ROS2 queue depth.
        :param qos: The profile to offer, or ``None`` for
            :data:`~opensmi.ros2.runtime.qos.DEFAULT_PROFILE`.
        :raises RuntimeNotStartedError: When the Runtime is not up.
        :raises LoopAffinityError: When awaited on a loop other than the Runtime's.
        :raises MissingInterfacePackageError: When the type's interface package is not installed.
        """
        node = self._require_started()
        self._require_loop()
        dataclass_type = self.describe_message(message_type) if isinstance(message_type, str) else message_type
        interface_class = import_interface_class(dataclass_type.ros2.type_name)
        profile = qos if qos is not None else DEFAULT_PROFILE
        provenance = Provenance.DECLARED if qos is not None else Provenance.DEFAULTED
        ros_publisher: Any = None

        def put(message: Message) -> None:
            ros_publisher.publish(encode(message))

        async def send(message: Message) -> None:
            # Off the loop: encoding a large occupancy grid would stall the OPC UA endpoint.
            # Faults are wrapped because rclpy spells "the publish did not happen" several ways.
            try:
                await asyncio.to_thread(put, message)
            except Exception as error:
                text = f"rclpy refused to publish on {topic}: {error}"
                raise PublishFailedError(text) from error

        async def release() -> None:
            if ros_publisher is not None:
                await asyncio.to_thread(self._destroy, "publisher", ros_publisher)
            if handle in self._publishers:
                self._publishers.remove(handle)

        handle = Publisher(
            topic=topic,
            message_type=dataclass_type,
            clock=self._ros2_clock,
            send=send,
            release=release,
            qos=profile,
            provenance=provenance,
        )
        ros_publisher = await asyncio.to_thread(
            node.create_publisher,
            interface_class,
            topic,
            _rclpy_profile(profile, depth),
        )
        self._publishers.append(handle)
        return handle

    def _feed_transforms(self, message: Message, static: bool) -> None:
        """Put every transform of one message into the buffer, on the loop that received it.

        The authority is this Runtime's node name, as a ``TransformListener`` would have written
        it. Nothing is raised: tf2 refuses a non-finite value or an empty frame id, and that is no
        reason to stop reading the topic.
        """
        buffer = self._buffer
        if buffer is None:  # pragma: no cover -- the facet is closed before the buffer is dropped
            return
        put = buffer.set_transform_static if static else buffer.set_transform
        for transform in getattr(encode(message), "transforms", ()):
            try:
                put(transform, self._node_name)
            except Exception as error:
                _LOGGER.debug("tf2 refused a transform", error=str(error))

    def _lookup_transform(self, target: str, source: str) -> Transform | None:
        """Return where one frame sits in another, out of what the buffer already holds.

        Non-blocking: ``lookup_transform`` at zero time returns the latest cached transform or
        raises. The ``_async`` variant is avoided because awaiting its future on this loop
        busy-spins until another message feeds the buffer.

        :return: The composed transform, or ``None`` when the buffer cannot complete the chain
            (unpublished frame, two trees that do not meet, or a link older than the cache).
        """
        buffer = self._buffer
        if buffer is None:  # pragma: no cover -- the facet is closed before the buffer is dropped
            return None
        try:
            stamped = buffer.lookup_transform(target, source, Time())
        except tf2_ros.TransformException:
            return None
        return Transform(
            translation=(
                float(stamped.transform.translation.x),
                float(stamped.transform.translation.y),
                float(stamped.transform.translation.z),
            ),
            rotation=(
                float(stamped.transform.rotation.x),
                float(stamped.transform.rotation.y),
                float(stamped.transform.rotation.z),
                float(stamped.transform.rotation.w),
            ),
        )

    def _ros2_clock(self) -> tuple[int, int]:
        """Return the Node's clock (not the wall clock, so right under ``use_sim_time``) as seconds and nanoseconds."""
        seconds, nanoseconds = self._require_started().get_clock().now().seconds_nanoseconds()
        return int(seconds), int(nanoseconds)

    def describe_message(self, message_type: str) -> type[Message]:
        """Return the message dataclass of a ROS2 message type.

        Usable before :meth:`start`.

        :param message_type: ``package/Name`` or ``package/msg/Name``.
        :raises InvalidMessageTypeNameError: When the name is not a message name.
        :raises MissingInterfacePackageError: When the interface package is not installed.
        :raises UnusableMessageTypeError: When the package has no such message.
        """
        return message_dataclass(message_type)

    def default_message(self, message_type: type[Message]) -> Message:
        """Return a message of a type as its generated ROS2 class constructs it, with no arguments.

        Every field holds the default its interface definition declares, or its type's zero. Usable
        before :meth:`start`.

        :param message_type: A message dataclass, as :meth:`describe_message` returns it.
        """
        return decode(import_interface_class(message_type.ros2.type_name)(), message_type)

    def describe_action(self, action_type: str) -> ActionTypes:
        """Return the goal, feedback and result message dataclasses of a ROS2 action type.

        Usable before :meth:`start`, so a missing interface package fails while a Component is
        constructed. It lives on the Runtime so that tests can substitute a double.

        :param action_type: ``package/action/Name``.
        :raises InvalidMessageTypeNameError: When the name is not a three-part action name.
        :raises MissingInterfacePackageError: When the interface package is not installed.
        :raises UnusableMessageTypeError: When the package has no such action.
        """
        return action_types(action_type)

    def describe_service(self, service_type: str) -> ServiceTypes:
        """Return the request and response message dataclasses of a ROS2 service type.

        Usable before :meth:`start`, like :meth:`describe_action`.

        :param service_type: ``package/srv/Name``.
        :raises InvalidMessageTypeNameError: When the name is not a three-part service name.
        :raises MissingInterfacePackageError: When the interface package is not installed.
        :raises UnusableMessageTypeError: When the package has no such service.
        """
        return service_types(service_type)

    async def call_service(
        self,
        *,
        service: str,
        service_type: str | ServiceTypes,
        request: Message,
        server_timeout: float = 10.0,
        call_timeout: float = 30.0,
    ) -> Message:
        """Call one ROS2 service and return the response it answered with.

        A refusal is in the response message. Only two cases raise: nobody serving the path, and
        a server that took the request and never answered. They have separate timeouts so that
        "the robot is off" and "the robot is stuck" stay distinguishable.

        :param service: The ROS2 service name.
        :param service_type: The service type (``package/srv/Name``) or the already-resolved
            pair of message dataclasses.
        :param request: The request, as an instance of the service's request message dataclass.
        :param server_timeout: Seconds to wait for a service server to appear on the path.
        :param call_timeout: Seconds to wait for the answer once the request has gone.
        :raises RuntimeNotStartedError: When the Runtime is not up.
        :raises LoopAffinityError: When awaited on a loop other than the Runtime's.
        :raises FieldTypeMismatchError: When a field of the request holds a value of the wrong
            Python type. Refused here because ``rclpy`` aborts the process.
        :raises FieldValueOutOfRangeError: When a field of the request is past its width or bound.
        :raises ServiceUnavailableError: When no service server answers on the path.
        :raises ServiceCallTimeoutError: When the request went and no answer came back in time.
        :raises MissingInterfacePackageError: When the service's interface package is absent.
        """
        node = self._require_started()
        loop = self._require_loop()
        types = self.describe_service(service_type) if isinstance(service_type, str) else service_type
        _require_the_payload_type("request", service, request, types.request, types.type_name)
        require_sendable(request)

        service_class = import_service_class(types.type_name)
        client = await asyncio.to_thread(node.create_client, service_class, service)
        try:
            if not await asyncio.to_thread(client.wait_for_service, server_timeout):
                _refuse_an_unserved_service(service, server_timeout)
            pending = await asyncio.to_thread(client.call_async, encode(request))
            try:
                raw = await asyncio.wait_for(_resolved(pending, loop), timeout=call_timeout)
            except TimeoutError as error:
                message = f"the service on {service} took the request and did not answer within {call_timeout} seconds"
                raise ServiceCallTimeoutError(message) from error
        finally:
            await asyncio.to_thread(self._destroy, "client", client)
        return decode(raw, types.response)

    async def send_goal(
        self,
        *,
        action: str,
        action_type: str | ActionTypes,
        goal: Message,
        depth: int = 10,
        server_timeout: float = 10.0,
    ) -> Goal:
        """Send one goal to a ROS2 action server, and return the handle that owns it.

        The handle is returned once the server has answered the goal request. A refusal comes back
        as an already settled handle; only a path nobody serves raises.

        :param action: The ROS2 action name.
        :param action_type: The action type (``package/action/Name``) or the already-resolved
            trio of message dataclasses.
        :param goal: The goal, as an instance of the action's goal message dataclass.
        :param depth: How many feedback messages the handle buffers.
        :param server_timeout: Seconds to wait for an action server to appear on the path.
        :raises RuntimeNotStartedError: When the Runtime is not up.
        :raises LoopAffinityError: When awaited on a loop other than the Runtime's.
        :raises FieldTypeMismatchError: When a field of the goal holds a value of the wrong
            Python type. Refused here because ``rclpy`` aborts the process instead of raising.
        :raises FieldValueOutOfRangeError: When a field of the goal is past its width or bound.
        :raises ActionServerUnavailableError: When no action server answers on the path.
        :raises MissingInterfacePackageError: When the action's interface package is absent.
        """
        node = self._require_started()
        loop = self._require_loop()
        types = self.describe_action(action_type) if isinstance(action_type, str) else action_type
        _require_the_payload_type("goal", action, goal, types.goal, types.type_name)
        require_sendable(goal)

        action_class = import_action_class(types.type_name)
        sink = GoalSink(depth=depth)
        client = await asyncio.to_thread(ActionClient, node, action_class, action)
        try:
            if not await asyncio.to_thread(client.wait_for_server, server_timeout):
                _refuse_an_unserved_action(action, server_timeout)
            handle = await self._await_acceptance(
                action=action,
                types=types,
                goal=goal,
                sink=sink,
                client=client,
                loop=loop,
            )
        except BaseException:
            await asyncio.to_thread(self._destroy_action_client, client)
            raise
        self._goals.append(handle)
        return handle

    async def _await_acceptance(
        self,
        *,
        action: str,
        types: ActionTypes,
        goal: Message,
        sink: GoalSink,
        client: Any,
        loop: asyncio.AbstractEventLoop,
    ) -> Goal:
        """Send the goal, wait for the server's answer, and build the handle around it."""

        def on_feedback(raw: Any) -> None:
            # `raw` is the generated FeedbackMessage wrapper; the feedback is inside it.
            loop.call_soon_threadsafe(_deliver, raw.feedback)

        def _deliver(raw: Any) -> None:
            sink.deliver(decode(raw, types.feedback))

        ros_goal = encode(goal)
        goal_handle = await _resolved(
            await asyncio.to_thread(client.send_goal_async, ros_goal, on_feedback),
            loop,
        )

        async def request_cancel() -> None:
            await _resolved(await asyncio.to_thread(goal_handle.cancel_goal_async), loop)

        async def release() -> None:
            await asyncio.to_thread(self._destroy_action_client, client)
            if handle in self._goals:
                self._goals.remove(handle)

        handle = Goal(
            action=action,
            types=types,
            sink=sink,
            request_cancel=request_cancel,
            release=release,
        )
        if not goal_handle.accepted:
            sink.settle(rejected_outcome(action))
            return handle

        def on_result(future: Any) -> None:
            loop.call_soon_threadsafe(_settle, future)

        def _settle(future: Any) -> None:
            try:
                response = future.result()
                outcome = settled_outcome(GoalStatus(response.status), decode(response.result, types.result))
            except Exception as error:
                # An unreadable result still ends the goal; report it as an outcome, since this
                # callback has nobody to raise to.
                outcome = GoalOutcome(
                    faults=(f"the result of the goal on {action} could not be read: {error!r}",),
                    accepted=True,
                    status=GoalStatus.UNKNOWN,
                    result=None,
                )
            sink.settle(outcome)

        (await asyncio.to_thread(goal_handle.get_result_async)).add_done_callback(on_result)
        return handle

    def _destroy_action_client(self, client: Any) -> None:
        """Destroy one ``rclpy`` action client and wake the executor so its wait set is rebuilt."""
        client.destroy()
        self._wake()

    async def _read_graph(self) -> GraphSnapshot:
        """Read every name on the graph once, off the loop; this is what :attr:`graph` polls with.

        :raises RuntimeNotStartedError: When the Runtime is not up (a poll racing teardown).
        """
        node = self._require_started()
        return await asyncio.to_thread(_snapshot_of, node)

    async def _call_parameter_service(
        self,
        node: str,
        service: str,
        service_type: str,
        build: Callable[[type[Message]], Message],
    ) -> Message:
        """Call one parameter service of a ROS2 node.

        :raises ServiceUnavailableError: When the node's service does not answer in time.
        :raises ServiceCallTimeoutError: When the request went and no answer came back.
        """
        types = self.describe_service(service_type)
        return await self.call_service(
            service=f"{node}/{service}",
            service_type=types,
            request=build(types.request),
            server_timeout=SERVER_TIMEOUT,
            call_timeout=CALL_TIMEOUT,
        )

    async def _read_parameter(self, node: str, name: str) -> ParameterReading:
        """Read one parameter of one ROS2 node (one name per call, see ``ParameterFacet``)."""
        response = await self._call_parameter_service(
            node, "get_parameters", GET_PARAMETERS_TYPE, lambda request: get_request(request, name)
        )
        return reading_of(name, response)

    async def _list_parameters(self, node: str) -> tuple[str, ...]:
        """Ask one ROS2 node what parameters it declares, recursively."""
        return names_of(await self._call_parameter_service(node, "list_parameters", LIST_PARAMETERS_TYPE, list_request))

    async def _describe_parameter(self, node: str, name: str) -> ParameterDescription:
        """Ask one ROS2 node about one parameter (one name per call: a batch with an undeclared name answers empty)."""
        response = await self._call_parameter_service(
            node, "describe_parameters", DESCRIBE_PARAMETERS_TYPE, lambda request: describe_request(request, name)
        )
        return description_of(name, response)

    async def _write_parameters(self, node: str, values: Mapping[str, ParameterValue]) -> Mapping[str, SetOutcome]:
        """Write a batch of parameters to one ROS2 node, with one outcome per parameter.

        Uses ``set_parameters``, not the atomic variant, which discards the whole batch when one
        name is undeclared.
        """
        response = await self._call_parameter_service(
            node, "set_parameters", SET_PARAMETERS_TYPE, lambda request: set_request(request, values)
        )
        return outcomes_of(tuple(values), response)

    async def _follow_parameter_events(self) -> AsyncGenerator[ParameterUpdate, None]:
        """Yield every parameter change on the graph from one ``/parameter_events`` subscription.

        The facet demultiplexes by node. The subscription closes with the generator.
        """
        subscription = await self.subscribe(
            topic=PARAMETER_EVENTS_TOPIC,
            message_type=PARAMETER_EVENT_TYPE,
            depth=EVENT_DEPTH,
        )
        try:
            async for message in subscription:
                yield update_of(message)
        finally:
            await subscription.close()

    def _destroy(self, kind: str, endpoint: Any) -> None:
        """Destroy one ``rclpy`` subscription, publisher or client, and wake the executor."""
        if self._node is not None:
            getattr(self._node, f"destroy_{kind}")(endpoint)
        self._wake()

    def _wake(self) -> None:
        """Wake the executor so its wait set is rebuilt after an endpoint went away."""
        if self._executor is not None:
            self._executor.wake()

    def _report_type_hash_mismatches(self, topic: str, sink: SubscriptionSink) -> None:
        """Compare every publisher's type hash against our own, and warn about each new one.

        A warning, not a refusal: a changed hash does not mean the message is undecodable.
        """
        node = self._node
        if node is None:
            return
        expected = _own_type_hash(node, topic)
        if expected is None:
            return
        for info in node.get_publishers_info_by_topic(topic):
            published = str(info.topic_type_hash)
            if published in (_UNKNOWN_TYPE_HASH, expected):
                continue
            mismatch = TypeHashMismatch(
                topic=topic,
                node_name=f"{info.node_namespace.rstrip('/')}/{info.node_name}",
                topic_type=info.topic_type,
                published_hash=published,
                expected_hash=expected,
            )
            sink.note_type_hash_mismatch(mismatch)
            warnings.warn(
                f"{mismatch.node_name} publishes {mismatch.topic_type} on {topic} with type hash "
                f"{published}, but this adapter was built against {expected}",
                TypeHashMismatchWarning,
                stacklevel=2,
            )

    def _require_started(self) -> rclpy.node.Node:
        """Return the node, or refuse because there is none."""
        if self._node is None:
            message = f"{self._node_name} is not started; call start() before using the Runtime"
            raise RuntimeNotStartedError(message)
        return self._node

    def _require_loop(self) -> asyncio.AbstractEventLoop:
        """Return the Runtime's loop, or refuse because this is not it."""
        loop = self._loop
        if loop is None:
            message = f"{self._node_name} is not started; call start() before using the Runtime"
            raise RuntimeNotStartedError(message)
        if loop is not asyncio.get_running_loop():
            message = (
                f"{self._node_name} is affine to the event loop it was started on; this call is "
                f"on another one, which would receive none of its messages"
            )
            raise LoopAffinityError(message)
        return loop


@final
class _AdaptiveSubscription:
    """One topic's QoS negotiation, and the ``rclpy`` subscription it is currently expressed as.

    The two event methods run on the Runtime's loop and decide synchronously, before any ``await``,
    whether to swap, so a second match event cannot spend the same bootstrap tightening twice. The
    swap itself is a task because ``rclpy`` calls block.
    """

    def __init__(
        self,
        *,
        topic: str,
        negotiation: QoSNegotiation,
        sink: SubscriptionSink,
        loop: asyncio.AbstractEventLoop,
        create: Callable[[QoSProfile], Awaitable[Any]],
        destroy: Callable[[Any], Awaitable[None]],
        publishers: Callable[[], tuple[EndpointInfo, ...]],
    ) -> None:
        """Bind one negotiation to the two ``rclpy`` steps and the graph read the Runtime owns."""
        self._topic = topic
        self._negotiation = negotiation
        self._sink = sink
        self._loop = loop
        self._create = create
        self._destroy = destroy
        self._publishers = publishers
        self._subscription: Any = None
        self._swapping = asyncio.Lock()
        self._tasks: set[asyncio.Task[None]] = set()
        self._closed = False

    async def open(self) -> None:
        """Create the first ``rclpy`` subscription, with whatever the negotiation opened at."""
        self._subscription = await self._create(self._negotiation.profile)

    def matched(self, count: int) -> None:
        """Record the publisher count, and spend the one bootstrap tightening when it is due."""
        self._sink.note_publishers(count)
        if count <= 0:
            return
        wanted = self._negotiation.on_first_publisher(self._publishers())
        if wanted is None:
            return
        _LOGGER.info(
            "Re-resolving a discovered QoS profile against the first publisher on the topic",
            topic=self._topic,
            profile=str(wanted),
        )
        self._swap(wanted)

    def incompatible(self, policy: Policy) -> None:
        """Report one incompatibility, and relax only if this profile was discovered."""
        reported = self._negotiation.incompatibility(self._topic, policy, self._publishers())
        if not self._sink.note_incompatibility(reported):
            # Already reported and unchanged: a flapping publisher must not cause a log storm.
            return
        _LOGGER.error(
            "A publisher offers QoS this subscription cannot accept",
            topic=self._topic,
            policy=policy.name,
            provenance=reported.provenance.value,
            requested=str(reported.request),
            adapted=str(reported.adapted) if reported.adapted is not None else None,
        )
        if reported.adapted is not None:
            self._swap(reported.adapted)

    def _swap(self, profile: QoSProfile) -> None:
        """Start the re-subscription as a task that :meth:`close` can wait for."""
        if self._closed:
            return
        task = self._loop.create_task(
            self._resubscribe(profile),
            name=f"ros2-qos-swap{self._topic}",
        )
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)

    async def _resubscribe(self, profile: QoSProfile) -> None:
        """Destroy the current ``rclpy`` subscription and create one asking for ``profile``.

        The old subscription goes first, so two never report match events into one sink; the cost
        is a few milliseconds of missed messages. The publisher count goes back to zero and is
        rebuilt from the new subscription's match events, because destroying a subscription
        discards its match state without an event; a carried-over count could leave a variable
        reading ``Good`` after its publisher went away, at the price of a momentary
        ``BadNoCommunication``.
        """
        async with self._swapping:
            if self._closed:
                return
            held = self._sink.qos
            current, self._subscription = self._subscription, None
            if current is not None:
                await self._destroy(current)
            self._sink.note_publishers(0)
            if await self._recreate(profile):
                self._sink.note_qos(profile)
            elif not await self._recreate(held):
                # Both profiles failed. Nobody to raise to on this task, so log; the zeroed
                # count leaves the variables reading `BadNoCommunication`, not a stale `Good`.
                _LOGGER.error(
                    "Could not re-subscribe after a QoS change, on either profile; this topic is "
                    "now unsubscribed and its variables read BadNoCommunication",
                    topic=self._topic,
                    wanted=str(profile),
                    held=str(held),
                )

    async def _recreate(self, profile: QoSProfile) -> bool:
        """Try to create the ``rclpy`` subscription with one profile, saying whether it worked."""
        try:
            self._subscription = await self._create(profile)
        except Exception:
            _LOGGER.exception(
                "Could not create a subscription with this QoS profile",
                topic=self._topic,
                profile=str(profile),
            )
            return False
        return True

    async def close(self) -> None:
        """Let any swap in flight finish, then destroy the ``rclpy`` subscription, once.

        A swap in flight is waited out, not cancelled: cancelling an ``asyncio.to_thread`` call
        stops the await but not the thread, which would leave a subscription nobody holds.
        """
        self._closed = True
        async with self._swapping:
            current, self._subscription = self._subscription, None
        for task in tuple(self._tasks):
            await asyncio.gather(task, return_exceptions=True)
        if current is not None:
            await self._destroy(current)


_INFINITE_NANOSECONDS: Final = 9223372036854775807
"""How a live Jazzy graph reports an infinite duration (``INT64_MAX``); a fresh profile says zero. Both mean FOREVER."""

_RELIABILITY: Final = {
    Reliability.RELIABLE: rclpy.qos.ReliabilityPolicy.RELIABLE,
    Reliability.BEST_EFFORT: rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
}
_DURABILITY: Final = {
    Durability.TRANSIENT_LOCAL: rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
    Durability.VOLATILE: rclpy.qos.DurabilityPolicy.VOLATILE,
}
_LIVELINESS: Final = {
    Liveliness.MANUAL_BY_TOPIC: rclpy.qos.LivelinessPolicy.MANUAL_BY_TOPIC,
    Liveliness.AUTOMATIC: rclpy.qos.LivelinessPolicy.AUTOMATIC,
}
"""This package's policy values in ``rclpy``'s vocabulary (``SYSTEM_DEFAULT`` is never requested)."""

_RELIABILITY_IN: Final = {int(theirs): ours for ours, theirs in _RELIABILITY.items()}
_DURABILITY_IN: Final = {int(theirs): ours for ours, theirs in _DURABILITY.items()}
_LIVELINESS_IN: Final = {int(theirs): ours for ours, theirs in _LIVELINESS.items()}
"""The same read backwards, by the integer a publisher reports.

``SYSTEM_DEFAULT``, ``UNKNOWN`` and ``BEST_AVAILABLE`` are absent and fall through to the most
permissive value, so an offer we cannot interpret never makes us ask for more than it.
"""


def _rclpy_profile(profile: QoSProfile, depth: int) -> rclpy.qos.QoSProfile:
    """Return one of our profiles as the ``rclpy`` object an endpoint is created with, joined with its depth."""
    return rclpy.qos.QoSProfile(
        depth=depth,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        reliability=_RELIABILITY[profile.reliability],
        durability=_DURABILITY[profile.durability],
        deadline=_duration_of(profile.deadline),
        liveliness=_LIVELINESS[profile.liveliness],
        liveliness_lease_duration=_duration_of(profile.lease),
    )


def _our_profile(profile: Any) -> QoSProfile:
    """Return what one endpoint offers; anything not modelled reads as the most permissive value."""
    return QoSProfile(
        reliability=_RELIABILITY_IN.get(int(profile.reliability), Reliability.BEST_EFFORT),
        durability=_DURABILITY_IN.get(int(profile.durability), Durability.VOLATILE),
        deadline=_seconds_of(profile.deadline),
        liveliness=_LIVELINESS_IN.get(int(profile.liveliness), Liveliness.AUTOMATIC),
        lease=_seconds_of(profile.liveliness_lease_duration),
    )


def _duration_of(seconds: float) -> Duration:
    """Return a number of seconds as the ``rclpy`` duration a QoS policy takes."""
    if seconds == FOREVER:
        return Duration(nanoseconds=_INFINITE_NANOSECONDS)
    return Duration(nanoseconds=int(seconds * 1_000_000_000))


def _seconds_of(duration: Any) -> float:
    """Return an ``rclpy`` duration as seconds, with both spellings of *no duration* as FOREVER."""
    nanoseconds = int(duration.nanoseconds)
    if nanoseconds <= 0 or nanoseconds >= _INFINITE_NANOSECONDS:
        return FOREVER
    return nanoseconds / 1_000_000_000


def _policy_of(kind: Any) -> Policy:
    """Return the policy an incompatibility event named; an unmodelled kind is ``Policy.INVALID``, not an exception."""
    try:
        return Policy(int(kind))
    except ValueError:
        return Policy.INVALID


def _snapshot_of(node: rclpy.node.Node) -> GraphSnapshot:
    """Return every topic, service and action on the graph, with the types on each name.

    A topic only this node subscribes to is included; match events, not this poll, say whether
    anything publishes on it.
    """
    return GraphSnapshot(
        topics=_by_name(node.get_topic_names_and_types()),
        services=_by_name(node.get_service_names_and_types()),
        actions=_by_name(get_action_names_and_types(node)),
    )


def _by_name(listed: Iterable[tuple[str, list[str]]]) -> dict[str, tuple[str, ...]]:
    """Return one of rclpy's name-and-types lists as the mapping a snapshot holds."""
    return {name: tuple(types) for name, types in listed}


def _own_type_hash(node: rclpy.node.Node, topic: str) -> str | None:
    """Return the type hash of this node's own subscription on a topic (what the dataclass was built against)."""
    ours = node.get_name(), node.get_namespace()
    for info in node.get_subscriptions_info_by_topic(topic):
        if (info.node_name, info.node_namespace) == ours:
            hash_text = str(info.topic_type_hash)
            return None if hash_text == _UNKNOWN_TYPE_HASH else hash_text
    return None


def _require_the_payload_type(
    what: str,
    path: str,
    payload: Message,
    expected: type[Message],
    type_name: str,
) -> None:
    """Refuse a payload that is not the message dataclass its ROS2 path takes.

    :raises TypeError: When the payload is not an instance of the expected dataclass.
    """
    if isinstance(payload, expected):
        return
    message = f"the {what} for {path} is a {type(payload).__name__}, and {type_name} takes a {expected.__name__}"
    raise TypeError(message)


def _refuse_an_unserved_action(action: str, server_timeout: float) -> None:
    """Refuse a goal for a path no action server answers on."""
    message = f"no action server answered on {action} within {server_timeout} seconds, so the goal was not sent"
    raise ActionServerUnavailableError(message)


def _refuse_an_unserved_service(service: str, server_timeout: float) -> None:
    """Refuse a call on a path no service server answers on."""
    message = f"no service server answered on {service} within {server_timeout} seconds, so the request was not sent"
    raise ServiceUnavailableError(message)


async def _resolved(future: Any, loop: asyncio.AbstractEventLoop) -> Any:
    """Await an ``rclpy`` future (not awaitable, completes on the executor thread) on the Runtime's loop."""
    waiter: asyncio.Future[Any] = loop.create_future()

    def on_done(done: Any) -> None:
        loop.call_soon_threadsafe(_settle, done)

    def _settle(done: Any) -> None:
        # The waiter was cancelled (e.g. a timed-out service call) and the future resolved later.
        if waiter.done():
            return
        try:
            waiter.set_result(done.result())
        except Exception as error:
            waiter.set_exception(error)

    future.add_done_callback(on_done)
    return await waiter


def _faults_from(what: str, step: Callable[[], object]) -> Iterable[str]:
    """Run one teardown step, returning a fault line instead of raising, so a failed step cannot keep the Context up."""
    try:
        step()
    except Exception as error:
        return [f"{what} failed: {error!r}"]
    return []
