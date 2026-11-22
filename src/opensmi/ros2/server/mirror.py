# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The inward mirror: ROS2 messages arrive, and a container's variables follow.

A Mirror keeps a UA variable container in sync with a foreign source, and a Binding is the
declaration that says which source. This mixin is the ROS2 to OPC UA direction, split by direction
and not by container, so a topic read into Monitoring and an action's feedback read into the same
place are one job on two sources. For each Binding it opens one subscription through the Runtime,
asks the mutation chokepoint for the Binding's nodes from the Projection's plan, writes what
arrives, derives the status code from the subscription's publisher count, and raises one
BindingEvent per QoS incompatibility. It holds no QoS (an adaptation swaps the ``rclpy``
subscription beneath a handle this mirror keeps) and no Nodes (it keeps the rendered plan, which knows
where each value goes).

A Binding has its nodes at boot even when its topic is absent from the graph, because the shape
comes from the declared type and a node that appears later is one a client's subscription never
recovers. Before the first message a client reads the plan's typed zero under
``BadNoCommunication`` (nothing publishes) or ``BadWaitingForInitialData`` (a publisher, no sample);
the rule is :mod:`opensmi.ros2.server.status`'s. A Binding that declares no type cannot subscribe,
so it has no nodes at boot and waits on the Runtime's graph facet. When its topic appears it
subscribes and gains its nodes through the chokepoint, and the watch closes when the last such
Binding is bound.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Final

from opensmi.ros2.runtime.errors import ROS2RuntimeError
from opensmi.ros2.runtime.selection import resolve_selection
from opensmi.ros2.server.errors import DeclarationError, DuplicateBindingNameError
from opensmi.ros2.server.events import Severity
from opensmi.ros2.server.mutation import NodeAddition, RenderedPlan, ROS2MutationMixin, ended
from opensmi.ros2.server.projection import ProjectionContext
from opensmi.ros2.server.projections import DEFAULT_REGISTRY
from opensmi.ros2.server.provenance import QOS_INCOMPATIBLE, BindingQoS, qos_clash_text
from opensmi.ros2.server.status import status_code

if TYPE_CHECKING:
    from collections.abc import Iterable, Mapping

    from asyncua import ua

    from opensmi.ros2.runtime.graph import GraphWatch
    from opensmi.ros2.runtime.interface import RuntimeInterface
    from opensmi.ros2.runtime.messages import Message
    from opensmi.ros2.runtime.subscription import Subscription
    from opensmi.ros2.server.bindings import MonitoringBinding
    from opensmi.ros2.server.projection import Projection, ProjectionRegistry

__all__ = ["AMBIGUOUS_TOPIC_TYPE", "PAYLOAD_CAPPED", "UNUSABLE_TOPIC_TYPE", "ROS2InwardMirrorMixin"]

AMBIGUOUS_TOPIC_TYPE: Final = "AMBIGUOUS_TOPIC_TYPE"
"""What an undeclared-type Binding reports when its topic carries more than one type.

It is reported and waited out, not resolved. The Binding stays unbound and binds if the disagreement
goes away.
"""

PAYLOAD_CAPPED: Final = "PAYLOAD_CAPPED"
"""What a Binding reports when a bulk payload reached this server's byte-string ceiling.

The cap itself is applied by the Projection. The event is reported once per Binding, so a 100 Hz
stream does not become an event storm.
"""

UNUSABLE_TOPIC_TYPE: Final = "UNUSABLE_TOPIC_TYPE"
"""What an undeclared-type Binding reports when the type its topic carries cannot be bound.

For a declared type these faults (a missing interface package, a selection matching nothing, a field
with no rendering) are fatal at init. Here the type arrived at run time, so the fault is reported
and the Binding is given up on.
"""


@dataclass(slots=True)
class _Mirrored:
    """One Binding, live: its subscription, what to write where, and the tasks doing it."""

    binding: MonitoringBinding
    subscription: Subscription
    projection: Projection
    context: ProjectionContext
    nodes: RenderedPlan
    # The last value of each entry of the plan, so a message is written whole and a status change with a typed
    # value. Seeded with the plan's initials.
    values: dict[str, object] = field(default_factory=dict)
    writing: asyncio.Lock = field(default_factory=asyncio.Lock)
    task: asyncio.Task[None] | None = None
    status_task: asyncio.Task[None] | None = None
    qos_task: asyncio.Task[None] | None = None
    satisfied: bool = False
    sampled: bool = False
    reported_unknown_name: bool = False
    reported_capped_payload: bool = False

    def __post_init__(self) -> None:
        """Start out holding the value each variable was declared with."""
        self.values = self.nodes.initial

    @property
    def status(self) -> ua.StatusCode:
        """Return the status code every variable of this Binding currently reads."""
        return status_code(satisfied=self.satisfied, sampled=self.sampled)

    @property
    def qos(self) -> BindingQoS:
        """Return this Binding's provenance entry, built from the subscription it is about."""
        return BindingQoS.of(
            self.binding.name,
            self.binding.path,
            self.subscription,
            self.subscription.qos_incompatibilities,
        )

    def cancel(self) -> None:
        """Ask this Binding's three tasks to stop, without waiting for any of them."""
        for task in (self.task, self.status_task, self.qos_task):
            if task is not None:
                task.cancel()


async def _stopped(mirrored: _Mirrored) -> None:
    """Stop following one Binding and close its subscription. Idempotent."""
    mirrored.cancel()
    await ended(mirrored.task)
    await ended(mirrored.status_task)
    await ended(mirrored.qos_task)
    mirrored.task = mirrored.status_task = mirrored.qos_task = None
    await mirrored.subscription.close()


class ROS2InwardMirrorMixin(ROS2MutationMixin):
    """Keeps a machinery item's Monitoring variables in sync with the topics it declared.

    Creating nodes is the mutation chokepoint's job, which this extends. Writing to them is this
    mixin's.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Start out mirroring nothing, and pass every argument to opensmi's constructor."""
        super().__init__(*args, **kwargs)
        self._mirrored: list[_Mirrored] = []
        self._deferred: list[MonitoringBinding] = []
        self._graph_watch: GraphWatch | None = None
        self._graph_task: asyncio.Task[None] | None = None
        self._runtime: RuntimeInterface | None = None
        self._registry: ProjectionRegistry = DEFAULT_REGISTRY

    @property
    def ros2_subscriptions(self) -> tuple[Subscription, ...]:
        """Return the subscription each bound Binding on this item opened, in the order they bound."""
        return tuple(each.subscription for each in self._mirrored)

    @property
    def ros2_monitoring_presence(self) -> Mapping[str, bool]:
        """Return, per Binding name, whether its topic has a publisher right now.

        This is Satisfied for a topic read into Monitoring: presence, never first data. It is the
        count the status code is written from, so a required Binding and its variables agree.
        A Binding that declared no type and whose topic has not appeared is ``False``,
        not missing.
        """
        deferred = {binding.name: False for binding in self._deferred}
        return {**deferred, **{each.binding.name: each.satisfied for each in self._mirrored}}

    @property
    def ros2_qos_provenance(self) -> Mapping[str, BindingQoS]:
        """Return, per Binding name, what its subscription asks for and where that came from.

        A Binding still waiting for its topic is absent, because it has no endpoint yet.
        """
        return {each.binding.name: each.qos for each in self._mirrored}

    @property
    def ros2_unbound_bindings(self) -> tuple[MonitoringBinding, ...]:
        """Return the Bindings still waiting for their topic to appear on the graph.

        Empty on a Machine whose declarations all name their type, which then never reads the graph.
        """
        return tuple(self._deferred)

    async def bind_topic_now(self, binding: MonitoringBinding, *, message_type: str) -> bool:
        """Open one Binding while the server runs, and give it its nodes.

        This is the client-driven exception. It lives here and not on the introspection
        Component, so that an ad-hoc Binding is built exactly like a declared one.

        :param binding: What to bind. Its ``name`` is what its variables are named for. It lives
            until the server restarts.
        :param message_type: The ROS2 type to subscribe with. It is required, because the caller has
            just read it off the graph and a run-time Binding has no init at which a declared type
            could be refused.
        :return: Whether it happened. ``False`` means a mutation cap refused the nodes and the
            subscription was closed again.
        :raises DuplicateBindingNameError: When this item already has a Binding of that name.
        :raises ROS2RuntimeError: When the type cannot be resolved or the subscription cannot be
            opened.
        :raises DeclarationError: When the Binding cannot be shaped (a selection matching nothing, a
            field with no rendering).
        """
        self._require_an_unused_binding_name(binding.name)
        opened = await self._open_binding(binding, message_type=message_type, at_init=False)
        if opened is None:
            return False
        self._mirrored.append(opened)
        await self._start_following(opened)
        return True

    def _require_an_unused_binding_name(self, name: str) -> None:
        """Refuse a second Binding of one name on this item.

        :raises DuplicateBindingNameError: When one is already bound or still waiting for its topic.
            Without this check the chokepoint would refuse the first colliding variable and name a
            field path instead of the Binding.
        """
        taken = {each.binding.name for each in self._mirrored} | {each.name for each in self._deferred}
        if name not in taken:
            return
        message = (
            f"{type(self).__name__} already has a Binding called {name!r}; a Binding's name is "
            f"what its variables are named for, so two of one name would be one set of nodes "
            f"with two things writing to it"
        )
        raise DuplicateBindingNameError(message)

    async def _init_monitoring_mirror(
        self,
        bindings: Iterable[MonitoringBinding],
        *,
        runtime: RuntimeInterface,
        registry: ProjectionRegistry = DEFAULT_REGISTRY,
    ) -> None:
        """Open, shape and start every Binding that declared a type, in declaration order.

        Anything wrong with such a declaration is fatal at init (a missing interface package, a
        pattern matching nothing, a field with no rendering, a name written twice), and whatever was
        already opened is closed first. A Binding that declared no type is put aside, and one watch
        on the graph facet covers all of them.
        """
        self._runtime = runtime
        self._registry = registry
        try:
            for binding in bindings:
                if binding.needs_the_graph:
                    self._deferred.append(binding)
                    continue
                assert binding.message_type is not None, "only a Binding without a type needs the graph"
                opened = await self._open_binding(binding, message_type=binding.message_type, at_init=True)
                if opened is not None:
                    self._mirrored.append(opened)
        except BaseException:
            await self._stop_monitoring_mirror()
            raise
        for mirrored in self._mirrored:
            await self._start_following(mirrored)
        self._start_watching_the_graph()

    async def _stop_monitoring_mirror(self) -> None:
        """Stop following every Binding, close its subscription, and let go of the graph.

        Idempotent. It runs before the Runtime stops, because a Component torn down on its own still
        has to stop reading.
        """
        await ended(self._graph_task)
        self._graph_task = None
        if self._graph_watch is not None:
            await self._graph_watch.close()
            self._graph_watch = None
        self._deferred = []
        # Cancelled in one pass before any is awaited, so teardown does not wait out each Binding's tasks in
        # turn.
        for mirrored in self._mirrored:
            mirrored.cancel()
        for mirrored in self._mirrored:
            await _stopped(mirrored)
        self._mirrored = []

    async def _start_following(self, mirrored: _Mirrored) -> None:
        """Put the first status on one Binding's variables, then start its tasks.

        The status is written here and not left to the publisher task, so the variables carry
        ``BadNoCommunication`` from the moment they exist and never the ``Good`` that node creation
        leaves.
        """
        await self._write_status(mirrored)
        name = mirrored.binding.name
        mirrored.task = asyncio.create_task(self._follow_binding(mirrored), name=f"ros2-monitoring-{name}")
        mirrored.status_task = asyncio.create_task(self._follow_publishers(mirrored), name=f"ros2-publishers-{name}")
        mirrored.qos_task = asyncio.create_task(self._follow_qos(mirrored), name=f"ros2-qos-{name}")

    def _start_watching_the_graph(self) -> None:
        """Open one watch covering every Binding still waiting for its topic, if there are any.

        With none, nothing is opened: the facet reads the graph only while a watch is open.
        """
        if not self._deferred or self._runtime is None:
            return
        self._graph_watch = self._runtime.graph.watch()
        self._graph_task = asyncio.create_task(self._follow_graph(), name="ros2-monitoring-graph")

    async def _follow_graph(self) -> None:
        """Bind each waiting Binding as its topic appears, then stop watching."""
        watch = self._graph_watch
        if watch is None:  # pragma: no cover -- the caller opens it before starting this task
            return
        async for delta in watch:
            for binding in tuple(self._deferred):
                # Appearing and being retyped both answer the Binding's question, what its topic carries. A
                # cleared `AMBIGUOUS_TOPIC_TYPE` arrives as the second.
                types = delta.topics.types_now_on(binding.path)
                if types is None:
                    continue
                try:
                    await self._bind_from_the_graph(binding, types)
                except Exception:
                    # A runtime condition is never fatal, and an unreported failure on this
                    # unawaited task would end the poll silently.
                    self.logger.exception(
                        "Could not bind a topic that appeared on the graph",
                        binding=binding.name,
                        topic=binding.path,
                    )
            if not self._deferred:
                # Nothing is waiting any more, so nothing needs the graph read again.
                await watch.close()
                return

    async def _bind_from_the_graph(self, binding: MonitoringBinding, types: tuple[str, ...]) -> None:
        """Give one waiting Binding its subscription and its nodes, or report why it has neither."""
        if len(types) != 1:
            listed = ", ".join(types)
            await self.emit_binding_event(
                binding=binding.name,
                path=binding.path,
                code=AMBIGUOUS_TOPIC_TYPE,
                severity=Severity.WARNING,
                text=f"{binding.path} carries more than one type ({listed}), so its shape is not decidable",
            )
            return
        try:
            opened = await self._open_binding(binding, message_type=types[0], at_init=False)
        except (ROS2RuntimeError, DeclarationError) as error:
            self._deferred.remove(binding)
            await self.emit_binding_event(
                binding=binding.name,
                path=binding.path,
                code=UNUSABLE_TOPIC_TYPE,
                severity=Severity.ERROR,
                text=f"{binding.path} carries {types[0]}, which this Binding cannot be built from: {error}",
            )
            return
        self._deferred.remove(binding)
        if opened is None:
            return
        self._mirrored.append(opened)
        await self._start_following(opened)

    async def _open_binding(
        self,
        binding: MonitoringBinding,
        *,
        message_type: str,
        at_init: bool,
    ) -> _Mirrored | None:
        """Subscribe, shape the plan, and ask the chokepoint for the nodes of one Binding.

        :return: The live Binding, or ``None`` when a cap refused its nodes (the subscription is
            closed again).
        """
        runtime = self._require_runtime()
        subscription = await runtime.subscribe(
            topic=binding.path,
            message_type=message_type,
            depth=binding.depth,
            qos=binding.qos,
        )
        try:
            resolved_type = subscription.message_type
            context = ProjectionContext(
                registry=self._registry,
                selected=resolve_selection(resolved_type, binding.selection),
                qualifier=self.ros2_projection_qualifier,
                byte_string_ceiling=self._byte_string_ceiling,
            )
            projection = self._registry.find(resolved_type.ros2.type_name)
            [nodes] = await self.mutate_address_space(
                [
                    NodeAddition(
                        binding=binding.name,
                        path=binding.path,
                        container=self.monitoring,
                        plan=projection.shape(resolved_type, context),
                        message_type=resolved_type,
                        root=binding.name,
                        group=binding.group,
                    )
                ],
                at_init=at_init,
            )
        except BaseException:
            await subscription.close()
            raise
        if nodes is None:
            await subscription.close()
            return None
        return _Mirrored(
            binding=binding,
            subscription=subscription,
            projection=projection,
            context=context,
            nodes=nodes,
        )

    def _require_runtime(self) -> RuntimeInterface:
        """Return the Runtime this mirror was initialised with.

        :raises DeclarationError: When there is none (a wiring fault).
        """
        if self._runtime is None:  # pragma: no cover -- every path here runs after init
            message = f"{type(self).__name__} has no Runtime; its Monitoring was never initialised"
            raise DeclarationError(message)
        return self._runtime

    async def _follow_binding(self, mirrored: _Mirrored) -> None:
        """Write the messages of one Binding until the subscription closes, throttled by ``max_rate``."""
        if mirrored.binding.max_rate is not None:
            await self._follow_binding_limited(mirrored, 1.0 / mirrored.binding.max_rate)
            return
        async for message in mirrored.subscription:
            await self._write_message_safely(mirrored, message)

    async def _follow_binding_limited(self, mirrored: _Mirrored, window: float) -> None:
        """Write at most one message per ``window`` seconds: the newest, never losing the last.

        The first message after a quiet spell is written at once. A message arriving inside a window
        replaces the one waiting, and the waiting one is written when the window ends. The throttle
        sits in front of the Projection, so a replaced message is never decoded.
        """
        loop = asyncio.get_running_loop()
        messages = aiter(mirrored.subscription)
        arrival = asyncio.ensure_future(anext(messages))
        waiting: Message | None = None
        open_at = 0.0  # the earliest moment the next write may happen
        try:
            while True:
                timeout = None if waiting is None else max(0.0, open_at - loop.time())
                done, _ = await asyncio.wait({arrival}, timeout=timeout)
                if arrival in done:
                    try:
                        message = arrival.result()
                    except StopAsyncIteration:
                        return
                    arrival = asyncio.ensure_future(anext(messages))
                    if loop.time() < open_at:
                        waiting = message
                        continue
                elif waiting is not None:
                    message = waiting
                else:  # pragma: no cover -- no timeout means no waiting message, and then arrival is all we wait on
                    continue
                waiting = None
                open_at = loop.time() + window
                await self._write_message_safely(mirrored, message)
        finally:
            arrival.cancel()

    async def _write_message_safely(self, mirrored: _Mirrored, message: Message) -> None:
        """Write one message, and survive whatever it could not be written as."""
        try:
            await self._write_message(mirrored, message)
        except Exception:
            # A runtime condition is never fatal. One unwritable value must not end the stream.
            self.logger.exception(
                "Could not write a message to its Monitoring variables",
                binding=mirrored.binding.name,
                topic=mirrored.binding.path,
            )

    async def _follow_publishers(self, mirrored: _Mirrored) -> None:
        """Write the status code every time the topic gains or loses its publishers.

        The count arrives on the subscription's DDS match events, without reading the graph. The
        first one arrives at once, so a Binding nobody publishes starts at ``BadNoCommunication``.
        """
        async for count in mirrored.subscription.publisher_counts():
            mirrored.satisfied = count > 0
            try:
                await self._write_status(mirrored)
            except Exception:
                self.logger.exception(
                    "Could not write a status code to its Monitoring variables",
                    binding=mirrored.binding.name,
                    topic=mirrored.binding.path,
                )

    async def _follow_qos(self, mirrored: _Mirrored) -> None:
        """Raise one BindingEvent for every QoS incompatibility the middleware reports.

        Any adaptation has already happened beneath the handle. The event tells a client that it
        happened, on which policy, and whether anything will change.
        """
        async for incompatibility in mirrored.subscription.follow_qos_incompatibilities():
            try:
                await self.emit_binding_event(
                    binding=mirrored.binding.name,
                    path=mirrored.binding.path,
                    code=QOS_INCOMPATIBLE,
                    severity=Severity.WARNING if incompatibility.adapted is not None else Severity.ERROR,
                    text=qos_clash_text(mirrored.binding.path, incompatibility),
                )
            except Exception:
                # A runtime condition is never fatal; a failed event must not end the stream.
                self.logger.exception(
                    "Could not report a QoS incompatibility",
                    binding=mirrored.binding.name,
                    topic=mirrored.binding.path,
                )

    async def _write_message(self, mirrored: _Mirrored, message: Message) -> None:
        """Decode one message and write it once, to the nodes of its Binding."""
        async with mirrored.writing:
            mirrored.sampled = True
            for name, value in mirrored.projection.decode(message, mirrored.context).items():
                if name not in mirrored.nodes.entries:
                    self._report_unknown_name(mirrored, name)
                    continue
                mirrored.values[name] = value
                await self._report_a_capped_payload(mirrored, name, value)
            await mirrored.nodes.write(mirrored.values, mirrored.status)

    async def _write_status(self, mirrored: _Mirrored) -> None:
        """Re-write every node of one Binding under the status its publishers now imply.

        The value goes with it, because a status written alone leaves the variant null.
        """
        async with mirrored.writing:
            await mirrored.nodes.write(mirrored.values, mirrored.status)

    async def _report_a_capped_payload(self, mirrored: _Mirrored, name: str, value: object) -> None:
        """Say once, as a BindingEvent, that a bulk payload reached this server's ceiling.

        The test is the length against the ceiling, because the cap is applied in a pure function
        that cannot report. A buffer as long as the ceiling is a cut one.
        """
        ceiling = self._byte_string_ceiling
        if mirrored.reported_capped_payload or not ceiling:
            return
        if not isinstance(value, (bytes, bytearray)) or len(value) < ceiling:
            return
        mirrored.reported_capped_payload = True
        await self.emit_binding_event(
            binding=mirrored.binding.name,
            path=mirrored.binding.path,
            code=PAYLOAD_CAPPED,
            severity=Severity.WARNING,
            text=(
                f"{name} reached this server's byte-string ceiling of {ceiling} bytes; a payload "
                f"larger than that is cut to it, and the ceiling is on the server's "
                f"MaxByteStringLength"
            ),
        )

    def _report_unknown_name(self, mirrored: _Mirrored, name: str) -> None:
        """Report, once per Binding, a Projection whose decode names what its shape never created.

        Unreachable with the shipped Projection. A curated one whose halves disagree is exposed only
        by a message, so it is reported instead of raised, and the value is not written.
        """
        if mirrored.reported_unknown_name:
            return
        mirrored.reported_unknown_name = True
        self.logger.error(
            "A Projection decoded a name its shape never created; those values are not written",
            binding=mirrored.binding.name,
            projection=mirrored.projection.type_name,
            name=name,
        )
