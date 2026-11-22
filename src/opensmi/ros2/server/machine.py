# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Machine side: one injected Runtime, started at init and stopped at shutdown.

A mixin that goes ahead of opensmi's ``BaseMachine``:

.. code-block:: python

    class Robot(ROS2MachineMixin, BaseMachine):
        async def _init(self) -> None:
            await super()._init()
            await self.add_component(Platform(name="Platform"))


    config = ROS2Config.from_toml("ros2.toml")
    runtime = Runtime(domain_id=config.domain_id, node_name=config.node_name)
    await server.add_machine(Robot(runtime=runtime, ros2_config=config))

The application constructs the (unstarted) Runtime, because a Runtime is the unit of one Domain ID
and tests need to pass a double; the mixin owns its lifetime. It starts before the Machine's
``_init``, since Components subscribe while building their nodes, and stops after its
``_shutdown``. A Machine that fails to build stops its Runtime too.

The Machine also adds the StartupSkill opensmi expects (default
:class:`~opensmi.ros2.server.required.ROS2StartupSkill`), which starts each
Component's StartupSkill so that startup completes only when every Required Binding is satisfied.
It holds the default Projection registry that each Component derives its own from, and the
byte-string ceiling, which is published before any Component is built
(:mod:`opensmi.ros2.server.limits`). Through :class:`opensmi.ros2.server.events.ROS2ModelChangeMixin`
it carries the BindingEvent and the standard model-change event, and the Machine-wide cap on
mutations. A Machine may declare a TF Binding of its own and runs the TF poll for every
SpatialObject on it (:mod:`opensmi.ros2.server.transforms`).
"""

from __future__ import annotations

from collections.abc import AsyncGenerator
from typing import TYPE_CHECKING, Any

from opensmi.core.lifecycle_mixin import lifecycle
from opensmi.server.base_machinery_item import BaseMachineryItem
from opensmi.server.base_startup_skill import BaseStartupSkill

from opensmi.ros2.server.bindings import TFBinding, resolve_bindings
from opensmi.ros2.server.config import ROS2Config
from opensmi.ros2.server.errors import DeclarationError
from opensmi.ros2.server.events import ROS2ModelChangeMixin
from opensmi.ros2.server.introspection import ROS2IntrospectionComponent
from opensmi.ros2.server.limits import publish_byte_string_ceiling
from opensmi.ros2.server.projections import DEFAULT_REGISTRY
from opensmi.ros2.server.required import ROS2StartupSkill, close_startup_skills
from opensmi.ros2.server.transforms import ROS2TransformMirrorMixin

if TYPE_CHECKING:
    from collections.abc import Sequence
    from pathlib import Path

    from opensmi.ros2.runtime.interface import RuntimeInterface
    from opensmi.ros2.server.bindings import Binding
    from opensmi.ros2.server.projection import Projection, ProjectionRegistry

__all__ = ["ROS2MachineMixin"]


class ROS2MachineMixin(ROS2ModelChangeMixin, ROS2TransformMirrorMixin):
    """Gives a Machine its Runtime, and its Runtime a lifetime.

    :cvar introspection_component_type: The Component that answers what is on the ROS2 graph and
        what this Machine is bound to. It is always added, with no flag to turn it off;
        the access level on its mutators is the safeguard. Replaceable to add listings.
    :cvar startup_skill_type: The StartupSkill added when the Machine registers none of its own.
        Replace it with a subclass of the stock one.
    :cvar projections: The Projections this Machine adds to the shipped catalogue. Every Component
        on it derives its own registry from the result.
    :cvar bindings: TF Bindings only: the one frame the Machine is. Any other kind is
        refused, because it would land nowhere.
    :cvar bindings_toml: A Binding file whose declarations union with ``bindings``, or ``None``.
    """

    introspection_component_type: type[ROS2IntrospectionComponent] = ROS2IntrospectionComponent
    projections: Sequence[Projection] = ()
    bindings: Sequence[Binding] = ()
    bindings_toml: str | Path | None = None
    startup_skill_type: type[ROS2StartupSkill] = ROS2StartupSkill

    def __init__(
        self,
        *args: Any,
        runtime: RuntimeInterface,
        ros2_config: ROS2Config | None = None,
        **kwargs: Any,
    ) -> None:
        """Take the Runtime this Machine owns, and pass everything else to opensmi.

        :param runtime: The Runtime, constructed and not yet started.
        :param ros2_config: The ROS2 settings this deployment loaded, or ``None`` for the
            defaults. The Components read the settings that are not the Runtime's from it.
        """
        super().__init__(*args, **kwargs)
        self._ros2_runtime = runtime
        self._ros2_config = ros2_config or ROS2Config()
        self._resolved_projections: ProjectionRegistry | None = None
        self._resolved_bindings: tuple[Binding, ...] | None = None
        self._ros2_byte_string_ceiling = 0

    @property
    def runtime(self) -> RuntimeInterface:
        """Return this Machine's Runtime, which is where all its ROS2 access goes."""
        return self._ros2_runtime

    @property
    def ros2_config(self) -> ROS2Config:
        """Return the ROS2 settings this Machine was built with, defaults included."""
        return self._ros2_config

    @property
    def ros2_bindings(self) -> tuple[Binding, ...]:
        """Return the Bindings this Machine declared, which may only be TF Bindings.

        Resolved once and held.

        :raises DeclarationError: When a kind other than :class:`TFBinding` is declared.
        :raises DuplicateBindingNameError: When one name appears twice, in either source.
        """
        if self._resolved_bindings is None:
            resolved = resolve_bindings(self.bindings, self.bindings_toml)
            _require_only_frames(self, resolved)
            self._resolved_bindings = resolved
        return self._resolved_bindings

    @property
    def ros2_projections(self) -> ProjectionRegistry:
        """Return this Machine's default registry: the shipped catalogue plus its own entries.

        Resolved once and held. Every Component derives its own registry from this one; nothing
        discovers an entry, and no Component's curation reaches another's.

        :raises DuplicateProjectionError: When an entry covers a type the shipped catalogue
            already covers and does not say ``override=True``.
        """
        if self._resolved_projections is None:
            self._resolved_projections = DEFAULT_REGISTRY.extended(self.projections)
        return self._resolved_projections

    @property
    def ros2_byte_string_ceiling(self) -> int:
        """Return how many bytes a bulk payload on this Machine may carry, or ``0`` for no limit.

        Published in the server's capability variable at init and applied by the Projection that
        produces the bytes. Zero before init.
        """
        return self._ros2_byte_string_ceiling

    async def _init(self) -> None:
        """Let opensmi initialise the Machine, then add the introspection Component.

        The introspection Component comes first, so it browses ahead of whatever this Machine
        declares. The Machine's own SpatialObject, if it declared a TF Binding, is created next so
        that it joins the list before any Component's.
        """
        await super()._init()  # pyright: ignore[reportAttributeAccessIssue]
        add_component: Any = getattr(self, "add_component")  # noqa: B009 -- opensmi's, not ours
        await add_component(self.introspection_component_type())
        await self._init_ros2_spatial_object()

    async def _shutdown(self) -> None:
        """Close the Machine's TF watch, then let opensmi shut the Machine down."""
        await self._stop_spatial_object()
        await super()._shutdown()  # pyright: ignore[reportAttributeAccessIssue]

    @lifecycle(after=BaseMachineryItem.lifecycle_machinery_item)
    async def lifecycle_ros2_startup(self) -> AsyncGenerator[None]:
        """Add the default StartupSkill unless the Machine registered one; release it at the end.

        Runs after the Machine's ``_init`` so that a StartupSkill registered there is seen however
        it is ordered against ``super()._init()``. The Skill gets the ``skill_access_level`` of
        the ROS2 settings, so the orchestrator can start the Machine with no operator.
        It is released before the Components are shut down, because it holds a task and a watch.
        """
        skills: Any = getattr(self, "skill_set")  # noqa: B009 -- opensmi's, not ours
        if BaseStartupSkill.NAME not in skills:
            add_skill: Any = getattr(self, "add_skill")  # noqa: B009 -- opensmi's, not ours
            await add_skill(self.startup_skill_type(minimum_access_level=self._ros2_config.skill_access_level))
        yield
        await close_startup_skills(self)

    @lifecycle(before=BaseMachineryItem.lifecycle_machinery_item)
    async def lifecycle_ros2_runtime(self) -> AsyncGenerator[None]:
        """Start the Runtime and publish the byte-string ceiling; stop the Runtime at the end.

        Runs before the Machine's ``_init`` and after its ``_shutdown``. opensmi rolls a failed
        init back through this step too, so a Machine that refuses to boot leaves no rclpy
        Context or executor thread behind. A stop that was not clean is logged, not raised.

        :raises RuntimeAlreadyStartedError: When the injected Runtime is already up.
        """
        await self._ros2_runtime.start()
        try:
            self._ros2_byte_string_ceiling = await publish_byte_string_ceiling(self.server.ua_server)
        except BaseException:
            await self._ros2_runtime.stop()
            raise
        yield
        outcome = await self._ros2_runtime.stop()
        if not outcome.ok:
            self.logger.warning("The Runtime did not stop cleanly", faults=outcome.faults)


def _require_only_frames(machine: ROS2MachineMixin, declared: tuple[Binding, ...]) -> None:
    """Refuse a Machine-level declaration that is not about where this Machine is.

    :raises DeclarationError: On any kind but :class:`TFBinding`.
    """
    wrong = [each for each in declared if not isinstance(each, TFBinding)]
    if not wrong:
        return
    listed = ", ".join(f"{each.name!r} ({type(each).__name__})" for each in wrong)
    message = (
        f"{type(machine).__name__} declares {listed} on the Machine; a Machine exposes nothing of "
        f"its own but where it is in space, so the only kind it may declare is a TFBinding -- "
        f"every other kind lands on a Component's Monitoring, ParameterSet, MethodSet or SkillSet"
    )
    raise DeclarationError(message)
