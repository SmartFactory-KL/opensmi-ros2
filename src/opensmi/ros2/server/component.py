# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Component side of the declarative API: a list of Bindings, and nothing overridden.

Mixed in ahead of opensmi's ``BaseComponent``, this turns the ``bindings`` class attribute into an
address space:

.. code-block:: python

    class Platform(ROS2ComponentMixin, BaseComponent):
        bindings = (
            MonitoringBinding(
                name="CurrentPose",
                path="/current_pose",
                message_type="geometry_msgs/msg/PoseStamped",
                exclude=("header/stamp",),
            ),
        )

        async def _write_identification(self) -> None: ...

A constructor may compute ``bindings`` (``self.bindings = [...]``), and ``bindings_toml`` adds
declarations from a file; the two sources union and a name collision is fatal
(:func:`opensmi.ros2.server.bindings.resolve_bindings`). A ``projections`` attribute lists the
recipes this Component renders its types with, ahead of its Machine's.

Each kind of Binding becomes a different part of the Component: Monitoring variables for a
``MonitoringBinding`` and a ``KeyedBinding`` (:mod:`opensmi.ros2.server.keyed`), a Method for a
``MethodBinding`` (:mod:`opensmi.ros2.server.method`), a finite Skill for an ``ActionBinding``
(:mod:`opensmi.ros2.server.skill`), ParameterSet entries for a ``ROS2ParameterBinding``
(:mod:`opensmi.ros2.server.parameters`) and a ``SpatialObject`` for a ``TFBinding``
(:mod:`opensmi.ros2.server.transforms`). The Component also gets a Notification for the events its
Bindings raise. A ``LifecycleBinding`` becomes the StartupSkill. Any declaration may be
marked ``required``: the Component then gets a StartupSkill that waits for it and halts if it is not
satisfied by its deadline (:mod:`opensmi.ros2.server.required`). A variable that is not a
message field is added the opensmi way, by overriding ``_init``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from opensmi.server.base_startup_skill import BaseStartupSkill

from opensmi.ros2.server.bindings import (
    ActionBinding,
    KeyedBinding,
    LifecycleBinding,
    MethodBinding,
    MonitoringBinding,
    PublisherBinding,
    ROS2ParameterBinding,
    is_required,
    resolve_bindings,
)
from opensmi.ros2.server.config import ROS2Config
from opensmi.ros2.server.errors import DeclarationError, ImplementationClassError
from opensmi.ros2.server.events import root_of
from opensmi.ros2.server.keyed import ROS2KeyedMirrorMixin
from opensmi.ros2.server.method import ROS2ServiceMethod
from opensmi.ros2.server.mirror import ROS2InwardMirrorMixin
from opensmi.ros2.server.parameters import ROS2ParameterMirrorMixin
from opensmi.ros2.server.projection import ProjectionContext
from opensmi.ros2.server.projections import DEFAULT_REGISTRY
from opensmi.ros2.server.publisher import ROS2PublisherSkill
from opensmi.ros2.server.required import ROS2StartupSkill, close_startup_skills
from opensmi.ros2.server.skill import ROS2ActionSkill
from opensmi.ros2.server.startup import ROS2LifecycleStartupSkill
from opensmi.ros2.server.transforms import ROS2SpatialMixin

if TYPE_CHECKING:
    from collections.abc import Sequence
    from pathlib import Path

    from opensmi.ros2.runtime.interface import RuntimeInterface
    from opensmi.ros2.server.bindings import Binding
    from opensmi.ros2.server.projection import Projection, ProjectionRegistry

__all__ = ["ROS2ComponentMixin"]


class ROS2ComponentMixin(
    ROS2ParameterMirrorMixin,
    ROS2InwardMirrorMixin,
    ROS2KeyedMirrorMixin,
    ROS2SpatialMixin,
):
    """What a Component declares, and what becomes of it.

    :cvar bindings: The Bindings this Component exposes. An instance may replace it in its
        constructor.
    :cvar bindings_toml: A Binding file whose declarations union with ``bindings``, or ``None``.
    :cvar projections: The Projections this Component curates its own types with, ahead of its
        Machine's. An instance may replace it in its constructor.
    :cvar startup_skill_type: A subclass of the stock StartupSkill these declarations ask for, or
        ``None`` for the stock one.
    """

    _resolved_bindings: tuple[Binding, ...] | None = None
    _resolved_projections: ProjectionRegistry | None = None
    _publisher_skills: tuple[ROS2PublisherSkill, ...] = ()

    bindings: Sequence[Binding] = ()
    bindings_toml: str | Path | None = None
    projections: Sequence[Projection] = ()
    startup_skill_type: type[ROS2StartupSkill] | None = None

    @property
    def runtime(self) -> RuntimeInterface:
        """Return the Runtime this Component's Machine was injected with.

        :raises DeclarationError: When the Machine carries no Runtime, which means it does
            not use :class:`opensmi.ros2.server.machine.ROS2MachineMixin`.
        """
        runtime = getattr(root_of(self), "runtime", None)
        if runtime is None:
            message = (
                f"{type(self).__name__} declares Bindings but its Machine carries no Runtime; a "
                f"Machine gains one by mixing in ROS2MachineMixin and being constructed with "
                f"runtime="
            )
            raise DeclarationError(message)
        return runtime

    @property
    def ros2_bindings(self) -> tuple[Binding, ...]:
        """Return the Bindings this Component resolved, both sources unioned.

        Resolved once, on first use, and held, so a name collision is fatal at init and the
        declarations cannot change under a running server.
        """
        if self._resolved_bindings is None:
            self._resolved_bindings = resolve_bindings(self.bindings, self.bindings_toml)
        return self._resolved_bindings

    @property
    def ros2_projections(self) -> ProjectionRegistry:
        """Return the registry this Component renders its types through.

        Derived from the Machine's registry with this Component's own entries ahead of it, so
        precedence runs Component, Machine, shipped, generic walk. A Component whose Machine is
        not a ROS2 one derives from the shipped catalogue; it cannot render anything anyway,
        because its ``_init`` raises :class:`~opensmi.ros2.server.errors.DeclarationError`.
        Resolved once and held, like ``ros2_bindings``.

        :raises DuplicateProjectionError: When an entry covers a type the Machine already covers
            and does not say ``override=True``.
        """
        if self._resolved_projections is None:
            inherited: ProjectionRegistry = getattr(root_of(self), "ros2_projections", DEFAULT_REGISTRY)
            self._resolved_projections = inherited.extended(self.projections)
        return self._resolved_projections

    @property
    def ros2_config(self) -> ROS2Config:
        """Return the ROS2 settings of the deployment this Component is part of.

        A Component whose Machine is not a ROS2 one answers with the defaults.
        """
        return getattr(root_of(self), "ros2_config", None) or ROS2Config()

    @property
    def ros2_projection_context(self) -> ProjectionContext:
        """Return the Projection context this Component renders through, with nothing selected.

        A surface narrows it to the field paths its own selection kept.
        """
        return ProjectionContext(
            registry=self.ros2_projections,
            selected=frozenset(),
            qualifier=self.ros2_projection_qualifier,
            byte_string_ceiling=self._byte_string_ceiling,
        )

    async def _init(self) -> None:
        """Let opensmi initialise the Component, then create its Binding nodes.

        Every way a declaration can be wrong is fatal on this path. The StartupSkill comes first,
        so it browses first in the SkillSet, then Methods, Skills, publishers, Monitoring
        variables, keyed members and parameters. The SpatialObject is last, because a companion
        specification's AddIn extends what the Component already is.
        """
        await super()._init()  # pyright: ignore[reportAttributeAccessIssue]
        await self._init_startup()
        await self._init_methods()
        await self._init_skills()
        await self._init_publishers()
        topics = [each for each in self.ros2_bindings if isinstance(each, MonitoringBinding)]
        keyed = [each for each in self.ros2_bindings if isinstance(each, KeyedBinding)]
        if topics:
            await self._init_monitoring_mirror(topics, runtime=self.runtime, registry=self.ros2_projections)
        if keyed:
            await self._init_keyed_mirror(keyed, runtime=self.runtime, registry=self.ros2_projections)
        tuned = [each for each in self.ros2_bindings if isinstance(each, ROS2ParameterBinding)]
        if tuned:
            await self._init_parameter_mirror(tuned, runtime=self.runtime)
        await self._init_ros2_spatial_object()

    async def _init_startup(self) -> None:
        """Give this Component the one StartupSkill its declarations ask for, if any do.

        A lifecycle Binding is the StartupSkill and a required Binding needs one to
        wait for it. A Component declaring both gets the lifecycle class, which adds
        the managed nodes to the other. Built here, not at the mutation chokepoint, because its
        nodes exist from init and never move.

        :raises DeclarationError: When this Component already carries a StartupSkill of its own.
        :raises ImplementationClassError: When ``startup_skill_type`` names a class that is not a
            subclass of the stock one these declarations ask for.
        """
        lifecycles = [each for each in self.ros2_bindings if isinstance(each, LifecycleBinding)]
        required = [each for each in self.ros2_bindings if is_required(each)]
        if not lifecycles and not required:
            return
        # One branch: which declaration asked decides the stock class, the count and the wording.
        stock: type[ROS2StartupSkill]
        if lifecycles:
            stock, counted, asked = ROS2LifecycleStartupSkill, len(lifecycles), "lifecycle"
            because = "a lifecycle Binding is the StartupSkill"
        else:
            stock, counted, asked = ROS2StartupSkill, len(required), "required"
            because = "a required Binding is what the StartupSkill waits for"
        self._require_no_startup_skill_yet(asked, counted, because)
        skill_type: Any = self._require_a_startup_skill_class(stock, asked)
        level = self.ros2_config.skill_access_level
        if lifecycles:
            skill = skill_type(bindings=tuple(lifecycles), runtime=self.runtime, minimum_access_level=level)
        else:
            skill = skill_type(minimum_access_level=level)
        await self.add_skill(skill)  # pyright: ignore[reportAttributeAccessIssue]

    def _require_no_startup_skill_yet(self, asked: str, counted: int, because: str) -> None:
        """Refuse a Component that registered a StartupSkill of its own and then declared one.

        :param asked: What to call the declarations that asked for one, in the message.
        :param counted: How many of them there are.
        :param because: Why that declaration lands on the Skill this Component already has.
        :raises DeclarationError: When one is already registered.
        """
        skills: Any = self.skill_set  # pyright: ignore[reportAttributeAccessIssue]
        if BaseStartupSkill.NAME not in skills:
            return
        existing = type(skills[BaseStartupSkill.NAME]).__name__
        message = (
            f"{type(self).__name__} declares {counted} {asked} Binding(s) and has already "
            f"registered {existing} as its {BaseStartupSkill.NAME}; {because}, so add behaviour "
            f"by naming a subclass in startup_skill_type rather than by registering a second one"
        )
        raise DeclarationError(message)

    def _require_a_startup_skill_class(self, stock: type[ROS2StartupSkill], asked: str) -> type[ROS2StartupSkill]:
        """Return the class to build this Component's StartupSkill from: its own, or the stock one.

        :raises ImplementationClassError: When the named class is not a subclass of the stock one.
        """
        named = self.startup_skill_type
        if named is None:
            return stock
        if not isinstance(named, type) or not issubclass(named, stock):
            shown = getattr(named, "__name__", repr(named))
            message = (
                f"{type(self).__name__} declares {asked} Binding(s) and names {shown} as its "
                f"startup_skill_type, and it is not a subclass of {stock.__name__}; a custom "
                f"class adds behaviour to the stock one rather than replacing it"
            )
            raise ImplementationClassError(message)
        return named

    async def _init_methods(self) -> None:
        """Add one Method per service declared, before any subscription is open."""
        services = [each for each in self.ros2_bindings if isinstance(each, MethodBinding)]
        if not services:
            return
        runtime = self.runtime
        context = self.ros2_projection_context
        level = self.ros2_config.skill_access_level
        for binding in services:
            method_type = binding.method_type or ROS2ServiceMethod
            method = method_type(binding=binding, runtime=runtime, context=context, minimum_access_level=level)
            await self.add_method(method)  # pyright: ignore[reportAttributeAccessIssue]

    async def _init_skills(self) -> None:
        """Add one finite Skill per action declared, before any subscription is open."""
        actions = [each for each in self.ros2_bindings if isinstance(each, ActionBinding)]
        if not actions:
            return
        runtime = self.runtime
        context = self.ros2_projection_context
        config = self.ros2_config
        for binding in actions:
            skill_type = binding.skill_type or ROS2ActionSkill
            skill = skill_type(
                binding=binding,
                runtime=runtime,
                context=context,
                config=config,
                minimum_access_level=config.skill_access_level,
            )
            await self.add_skill(skill)  # pyright: ignore[reportAttributeAccessIssue]

    async def _init_publishers(self) -> None:
        """Add one Skill per published topic declared, continuous or finite as it says.

        The Skill advertises its publisher itself, last, so a refused declaration leaves no
        endpoint behind.
        """
        publishers = [each for each in self.ros2_bindings if isinstance(each, PublisherBinding)]
        if not publishers:
            return
        runtime = self.runtime
        context = self.ros2_projection_context
        level = self.ros2_config.skill_access_level
        for binding in publishers:
            skill_type = binding.skill_type or binding.stock_skill_type
            skill = skill_type(binding=binding, runtime=runtime, context=context, minimum_access_level=level)
            await self.add_skill(skill)  # pyright: ignore[reportAttributeAccessIssue]
            self._publisher_skills = (*self._publisher_skills, skill)

    async def _shutdown(self) -> None:
        """Stop following this Component's nodes and topics, then let opensmi shut it down.

        A Component torn down inside a process that goes on serving has to release its own
        endpoints; stopping the Runtime would release all of them.
        """
        await self._stop_parameter_mirror()
        await self._stop_monitoring_mirror()
        await self._stop_keyed_mirror()
        await self._stop_spatial_object()
        for skill in self._publisher_skills:
            await skill.close_publisher()
        await close_startup_skills(self)
        await super()._shutdown()  # pyright: ignore[reportAttributeAccessIssue]
