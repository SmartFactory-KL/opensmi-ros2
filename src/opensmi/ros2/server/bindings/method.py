# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A ROS2 service called as a Method."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import ClassVar, TypeAlias

from opensmi.ros2.runtime.messages import Message
from opensmi.ros2.runtime.selection import FieldSelection
from opensmi.ros2.server.bindings._base import AwaitedBinding
from opensmi.ros2.server.projection import ProjectionContext

RequestFunction: TypeAlias = Callable[[type[Message], Mapping[str, object], ProjectionContext], Message]
"""Builds a service request from a Method's Parameters, as a pure function.

The default is :func:`opensmi.ros2.server.method.default_request_from`.
"""

ResponseFunction: TypeAlias = Callable[[Message, ProjectionContext], Mapping[str, object]]
"""Turns a service response into the values of a Method's FinalResultData, by variable name.

The default is :func:`opensmi.ros2.server.method.default_response_to`.
"""


@dataclass(frozen=True, slots=True, kw_only=True)
class MethodBinding(AwaitedBinding):
    """A ROS2 service exposed as a Method on the declaring Component.

    The request becomes the Method's Parameters (written before ``Call``) and the response its
    FinalResultData. There is no Monitoring, since a service has no feedback. opensmi's ``Call`` takes no
    input arguments and returns a bare status code, so anything richer than a status is in FinalResultData
    when the call returns.

    Two failures reach a client differently. A transport failure (nobody serving the path, or a server
    that went quiet) means the call did not happen and returns a real UA status code. An application
    failure (the service answered no) is ``Good`` plus the response in FinalResultData, unless
    ``success_field`` names the field carrying the verdict.

    :ivar name: The Binding's name, also the Method's name. Its variables are named for the field path
        alone.
    :ivar path: The ROS2 service.
    :ivar service_type: The ROS2 service type, ``package/srv/Name``. Required, so the address space does
        not depend on start-up timing. A type whose package is not installed is fatal at init.
    :ivar include: Field patterns narrowing the request; empty means every field.
    :ivar exclude: Field patterns removing request fields, after ``include`` has narrowed. A selection
        that leaves the request unbuildable is refused at init instead of filled from a default.
    :ivar rename: New browse names by field path, as on :class:`MonitoringBinding`.
    :ivar response_include: Field patterns narrowing the response.
    :ivar response_exclude: Field patterns removing response fields.
    :ivar response_rename: New browse names for response fields, as :attr:`rename` does.
    :ivar success_field: The name of a ``bool`` response field carrying the service's own verdict, or
        ``None``. Never auto-detected: ROS2 has no standard success field. A field the response lacks, or
        one that is not a ``bool``, is fatal at init.
    :ivar server_timeout: Seconds ``Call`` waits for a service server before giving up.
    :ivar call_timeout: Seconds to wait for the answer once the request has gone. Separate from
        ``server_timeout`` because "nothing is there" and "it is stuck" reach a client as different
        status codes.
    :ivar method_type: A subclass of :class:`opensmi.ros2.server.method.ROS2ServiceMethod`, or ``None``
        for the stock one. Python-only; refused when the Binding is constructed if it is not a
        subclass.
    :ivar request_from: Builds the request from the Parameters, or ``None`` for the default. Python-only.
    :ivar response_to: Turns the response into FinalResultData values, or ``None`` for the default.
        Python-only.
    :ivar required: Whether the StartupSkill waits for the service server to be discovered before
        ``Running``.
    :ivar startup_timeout: Seconds that wait lasts, or ``None`` for the deployment's default. Refused when the
        Binding is constructed without ``required``.
    """

    kind: ClassVar[str] = "Method Binding"

    path: str
    service_type: str
    include: tuple[str, ...] = ()
    exclude: tuple[str, ...] = ()
    rename: dict[str, str] = field(default_factory=dict)
    response_include: tuple[str, ...] = ()
    response_exclude: tuple[str, ...] = ()
    response_rename: dict[str, str] = field(default_factory=dict)
    success_field: str | None = None
    server_timeout: float = 10.0
    call_timeout: float = 30.0
    method_type: type | None = None
    request_from: RequestFunction | None = None
    response_to: ResponseFunction | None = None

    def __post_init__(self) -> None:
        """Refuse a custom implementation class that is not a subclass of the stock one.

        :raises ImplementationClassError: When ``method_type`` does not derive from
            :class:`opensmi.ros2.server.method.ROS2ServiceMethod`.
        :raises RequiredDeclarationError: When ``startup_timeout`` is declared without ``required``, or
            is not positive.
        """
        self._require_a_name_that_can_be_browsed()
        self._require_new_names_that_follow_the_rule("rename", self.rename)
        self._require_new_names_that_follow_the_rule("response_rename", self.response_rename)
        self._require_a_deadline_that_means_something()
        if self.method_type is None:
            return
        # Imported here: the Method annotates against this declaration.
        from opensmi.ros2.server.method import ROS2ServiceMethod

        self._require_a_subclass(
            self.method_type,
            ROS2ServiceMethod,
            "a custom class adds behaviour to the stock one rather than replacing it, "
            "and a TOML file cannot name one at all",
        )

    @property
    def selection(self) -> FieldSelection:
        """Return the request's field selection."""
        return FieldSelection(include=self.include, exclude=self.exclude, rename=self.rename)

    @property
    def response_selection(self) -> FieldSelection:
        """Return the response's field selection."""
        return FieldSelection(include=self.response_include, exclude=self.response_exclude, rename=self.response_rename)

    @property
    def needs_the_graph(self) -> bool:
        """Return ``False``: the declaration carries its type."""
        return False
