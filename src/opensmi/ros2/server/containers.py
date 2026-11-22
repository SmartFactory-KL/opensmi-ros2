# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The variable containers a Skill and a Method both fill from a Projection plan.

opensmi gives a Skill and a Method a ParameterSet, a Monitoring and a FinalResultData container.
This module renders their variables from a plan, writes them from a message and resets them to
their declared values. It knows nothing about which ROS2 message lands in which container; that
belongs to :mod:`opensmi.ros2.server.method` and the Skills.

A top-level field of the message is a child of the container, named by its own browse name
without the Binding prefix, because the callable's node already carries the Binding's name
(``SetMode/Mode``). A nested message is a complex Variable. The variables are declared
once at init, so they bypass the capped mutation chokepoint but share its
:meth:`~opensmi.ros2.server.mutation.ROS2MutationMixin.render_plan`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, ClassVar

from opensmi.ros2.server.errors import DeclarationError
from opensmi.ros2.server.status import GOOD, status_value

if TYPE_CHECKING:
    from collections.abc import Mapping, Sequence

    from asyncua import ua
    from opensmi.server import UaVariable
    from opensmi.server.ua_variable_containers import UaVariableContainer
    from structlog.stdlib import BoundLogger

    from opensmi.ros2.runtime.messages import Message
    from opensmi.ros2.server.mutation import RenderedPlan
    from opensmi.ros2.server.projection import PlanEntry

__all__ = ["ROS2ContainerMixin", "VariableContainer"]


@dataclass(slots=True, kw_only=True)
class VariableContainer:
    """One of a callable's variable containers, as everything a write to it needs.

    opensmi's container holds the variables by name only, so this adds what the plan rendered into
    it and the value of every entry, since a message is written whole.

    :ivar name: What the container is called, for a refusal a reader has to act on.
    :ivar container: opensmi's own container, shared and not copied.
    :ivar rendered: The nodes the plan became, once they are created.
    :ivar values: The value of each entry by its plan name, as last written.
    """

    name: str
    container: UaVariableContainer
    rendered: RenderedPlan | None = None
    values: dict[str, object] = field(default_factory=dict)

    @property
    def nodes(self) -> RenderedPlan:
        """Return the rendered plan.

        :raises DeclarationError: When the container was not created yet.
        """
        if self.rendered is None:  # pragma: no cover -- every callable creates its containers at init
            message = f"the {self.name} was read or written before its variables were created"
            raise DeclarationError(message)
        return self.rendered


class ROS2ContainerMixin:
    """How a Skill and a Method build, write and clear their variable containers.

    Mixed in ahead of opensmi's callable class. The annotated names below are what it expects to
    find on that class.

    :cvar callable_kind: What a refusal calls the owner, ``Skill`` or ``Method``.
    """

    callable_kind: ClassVar[str]

    if TYPE_CHECKING:

        @property
        def name(self) -> str:
            """Return the callable's name, which is its Binding's."""
            ...

        @property
        def logger(self) -> BoundLogger:
            """Return the logger opensmi gives every object."""
            ...

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Pass every argument on to opensmi's callable.

        Only here for the type checker: without it ``super().__init__(name=...)`` in a Skill or
        Method resolves to this mixin and reports unknown keywords.
        """
        super().__init__(*args, **kwargs)

    async def _create(
        self,
        plan: Sequence[PlanEntry],
        container: VariableContainer,
        *,
        message_type: type[Message],
        writable: bool = False,
    ) -> None:
        """Render one plan in one container, and record it on it.

        Every name is checked before any variable is made, so a failing plan leaves nothing
        half-built.

        :param message_type: The ROS2 type the plan was shaped from.
        :raises UnprojectableFieldError: When an entry is an Object.
        :raises DuplicateVariableNameError: When a name is already taken in that container.
        :raises DeclarationError: When the machinery item cannot render a plan, because it does not
            carry the ROS2 mixins.
        """
        item: Any = getattr(self, "parent", None)
        render = getattr(item, "render_plan", None)
        if render is None:
            message = (
                f"the {self.callable_kind} {self.name!r} renders nodes of its own, and the Component it is "
                f"on cannot create them; a Component carrying ROS2 Bindings mixes in ROS2ComponentMixin"
            )
            raise DeclarationError(message)
        rendered: RenderedPlan = await render(
            container.container,
            plan,
            message_type=message_type,
            qualifier=f"{item.ros2_projection_qualifier}.{self.name}.{container.name}",
            where=self._named(container),
            writable=writable,
        )
        container.rendered = rendered
        container.values = rendered.initial

    async def _write_values(self, container: VariableContainer, values: Mapping[str, object]) -> None:
        """Write a name-to-value map into one container, skipping names it has no variable for."""
        nodes = container.nodes
        for name, value in values.items():
            if name not in nodes.entries:
                self._report_unknown_name(container, name)
                continue
            container.values[name] = value
        await nodes.write(container.values)

    async def _reset_values(self, container: VariableContainer) -> None:
        """Put every variable of one container back to the plan's ``initial`` value.

        That is a typed zero only where the declaration says so; a curated Projection may choose
        another value.
        """
        container.values = container.nodes.initial
        await container.nodes.write(container.values)

    async def _read_values(self, container: VariableContainer) -> dict[str, object]:
        """Read one container into the plan's values, by name.

        Elements of a structure array come back as maps of their fields, the form a Projection
        builds a message from. Raises ``ValueError`` for a value that is not an array of the
        variable's structure, which the write check normally prevents.
        """
        return await container.nodes.read()

    async def _write(self, variable: UaVariable[Any], value: object, variant_type: ua.VariantType) -> None:
        """Write one value to a variable's node with the variant type the plan decided.

        Written on the node, so no write check runs: the value comes from ROS2 or the declaration.
        """
        await variable.ua_node.write_value(status_value(value, variant_type, GOOD))

    def _named(self, container: VariableContainer) -> str:
        """Return how a refusal names one of these containers, from a reader's side."""
        return f"the {container.name} of the {self.callable_kind} {self.name!r}"

    def _report_unknown_name(self, container: VariableContainer, name: str) -> None:
        """Log a decoded name its shape never created, and skip the value.

        Unreachable with the generic Projection. A curated one whose halves disagree only shows
        up with a message, so it is logged and not raised.
        """
        self.logger.error(
            "A Projection decoded a name its shape never created; that value was not written",
            binding=self.name,
            container=container.name,
            name=name,
        )
