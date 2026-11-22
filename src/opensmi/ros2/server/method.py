# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""A service is a Method: a client writes the request, calls, and reads what came back.

======================  ========================================================================
``ParameterSet``        The request's fields, writable in any state. They are the Method's
                        arguments, because opensmi's ``Call`` takes none of its own.
``Call``                Read the Parameters, build the request, send it, write the response.
``FinalResultData``     The response's fields, written when the call returns.
======================  ========================================================================

A Method has no Monitoring, because a service has no feedback.

opensmi's ``Call`` checks the access level (the Skill level, ``skill_access_level``, default
:data:`~opensmi.ros2.server.access.SKILL_ACCESS_LEVEL`, because a service call commands the robot),
the Lock and a completed startup, then runs :meth:`ROS2ServiceMethod.execute_method`. A
``UaStatusCodeError`` raised there becomes the returned status code; anything else becomes
``BadInternalError``.

Two kinds of failure are told apart. A transport failure means the call did not happen: no server
on the path gives ``BadNotConnected``, a server that took the request and went quiet gives
``BadTimeout``. This is the one place where the UA layer translates an error itself, because a
missing service server reported as an internal error would blame the wrong party. An application
failure (the service ran and refused) answers ``Good`` with the response in FinalResultData. A
declaration may name the verdict field (``success_field``); a ``False`` in it answers
``BadRequestNotAllowed`` with the response still written. No field is detected automatically,
since ROS2 has no standard success field; a wrong name fails at init.

FinalResultData and the Parameters are never reset, so FinalResultData holds the last response
that arrived; the status code is the verdict on the latest call. Post-call work goes into an
override of ``execute_method`` that calls ``super()`` first.

:func:`default_request_from` and :func:`default_response_to` are pure functions of a message and
its Projection context. A declaration may name its own pair; a replacement that needs the
instance overrides :meth:`ROS2ServiceMethod.build_request` or
:meth:`ROS2ServiceMethod.read_response`. The containers are built in
:mod:`opensmi.ros2.server.containers`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, ClassVar, Final

from asyncua import ua
from asyncua.ua.status_codes import StatusCodes
from opensmi.server import BaseMachineryItem, BaseMethod
from opensmi.server.mixins import FinalResultDataMixin, ParameterSetMixin, ParentMixin

from opensmi.ros2.runtime.errors import ServiceCallTimeoutError, ServiceUnavailableError
from opensmi.ros2.runtime.guard import require_sendable
from opensmi.ros2.runtime.messages import PrimitiveType
from opensmi.ros2.runtime.selection import SEPARATOR
from opensmi.ros2.server.access import SKILL_ACCESS_LEVEL
from opensmi.ros2.server.containers import ROS2ContainerMixin, VariableContainer
from opensmi.ros2.server.errors import SuccessFieldError
from opensmi.ros2.server.projection import narrowed, require_invertible, shaped

if TYPE_CHECKING:
    from collections.abc import Mapping

    from opensmi.ros2.runtime.interface import RuntimeInterface
    from opensmi.ros2.runtime.messages import Message, ServiceTypes
    from opensmi.ros2.server.bindings import MethodBinding
    from opensmi.ros2.server.projection import ProjectionContext

__all__ = [
    "REFUSED_BY_THE_SERVICE",
    "TRANSPORT_STATUS",
    "ROS2ServiceMethod",
    "default_request_from",
    "default_response_to",
]

TRANSPORT_STATUS: Final[Mapping[type[Exception], int]] = {
    ServiceUnavailableError: StatusCodes.BadNotConnected,
    ServiceCallTimeoutError: StatusCodes.BadTimeout,
}
"""The two ways a call does not happen, and the UA status code each becomes.

They differ for an operator: nothing serves the path, or something serves it and is stuck.
"""


_TRANSPORT_FAILURES: Final = tuple(TRANSPORT_STATUS)
"""What :meth:`ROS2ServiceMethod._call_the_service` catches, derived from the table."""


REFUSED_BY_THE_SERVICE: Final = StatusCodes.BadRequestNotAllowed
"""What a Method answers when the field named by ``success_field`` came back ``False``.

``BadInvalidState`` would assert why the service refused; only the service's own reason text,
which is in FinalResultData beside this code, can say that.
"""


def default_request_from(
    request_type: type[Message],
    values: Mapping[str, object],
    context: ProjectionContext,
) -> Message:
    """Build a request message out of what a client wrote into a Method's Parameters.

    The Projection covering the request type, run backwards. A pure function: no server,
    ``rclpy`` or event loop.

    :raises NotInvertibleError: When a field of the request has no value, because the selection
        removed it or no Parameter carries it. Such a field is refused instead of filled with a
        default.
    """
    return context.registry.find(request_type.ros2.type_name).inverse_of(request_type, values, context)


def default_response_to(response: Message, context: ProjectionContext) -> Mapping[str, object]:
    """Read a response message into the FinalResultData variables its type was shaped into."""
    return context.registry.find(type(response).ros2.type_name).decode(response, context)


class ROS2ServiceMethod(
    ROS2ContainerMixin,
    ParentMixin[BaseMachineryItem],
    FinalResultDataMixin,
    ParameterSetMixin,
    BaseMethod,
):
    """The Method one :class:`~opensmi.ros2.server.bindings.MethodBinding` becomes.

    Constructed by the declaring Component. A declaration may name a subclass of this class to add
    behaviour; the subclass is checked when the declaration is read.
    """

    callable_kind: ClassVar[str] = "Method"

    def __init__(
        self,
        *,
        binding: MethodBinding,
        runtime: RuntimeInterface,
        context: ProjectionContext,
        minimum_access_level: int = SKILL_ACCESS_LEVEL,
    ) -> None:
        """Resolve everything about the declaration that can be wrong, before any node exists.

        A missing interface package, a pattern that matches nothing, a request that cannot be
        built backwards or a bad ``success_field`` fails here instead of on the first ``Call``.

        :param binding: What to build.
        :param runtime: Where the request is sent. The Machine's, reached through the Component.
        :param context: The Projection context of the Component, with nothing selected yet. The
            Method narrows it once per surface (request, response).
        :param minimum_access_level: The level a caller needs.
        :raises MessageTypeError: When the service type cannot be resolved.
        :raises FieldSelectionError: When a selection pattern matches nothing.
        :raises NotInvertibleError: When the request type reaches a Projection that cannot run
            backwards.
        :raises SuccessFieldError: When ``success_field`` names a field the response does not
            have, or one that is not a ``bool``.
        """
        # A service call commands the robot as a Skill does, so it asks for the Skill level.
        super().__init__(name=binding.name, minimum_access_level=minimum_access_level)
        self._binding = binding
        self._runtime = runtime
        self._types: ServiceTypes = runtime.describe_service(binding.service_type)
        self._request_context = narrowed(context, self._types.request, binding.selection)
        self._response_context = narrowed(context, self._types.response, binding.response_selection)
        require_invertible(self._types.request, self._request_context)
        _require_a_bool_field(self._types.response, binding)
        self._request_plan = shaped(self._types.request, self._request_context)
        self._response_plan = shaped(self._types.response, self._response_context)
        # opensmi's containers exist from first use; `_init` fills these records.
        self._parameters = VariableContainer(name="ParameterSet", container=self.parameter_set)
        self._results = VariableContainer(name="FinalResultData", container=self.final_result_data)

    @property
    def binding(self) -> MethodBinding:
        """Return the declaration this Method was built from."""
        return self._binding

    @property
    def service_types(self) -> ServiceTypes:
        """Return the request and response types of the service this Method is bound to."""
        return self._types

    async def _init(self) -> None:
        """Let opensmi register the Call, create the variables, and prove the defaults can be sent.

        Building a request from the initial Parameters must work, so a value ROS2 would abort on
        fails here, on the machine of whoever wrote the declaration.
        """
        await super()._init()
        await self._create(self._request_plan, self._parameters, message_type=self._types.request, writable=True)
        await self._create(self._response_plan, self._results, message_type=self._types.response)
        require_sendable(await self.build_request(await self.read_parameters()))

    async def read_parameters(self) -> Mapping[str, object]:
        """Return what the ParameterSet holds now, by variable name."""
        return await self._read_values(self._parameters)

    async def read_results(self) -> Mapping[str, object]:
        """Return what the FinalResultData holds now, by variable name."""
        return await self._read_values(self._results)

    async def build_request(self, values: Mapping[str, object]) -> Message:
        """Return the request message to send, given what a client wrote into the Parameters.

        Override on a custom class when building the request needs the instance. The declaration's
        ``request_from`` is the pure-function form, and :func:`default_request_from` is the
        default. An override must also work with the declaration's defaults, because ``_init``
        calls it once with them. A Parameter a subclass adds after ``super()._init()`` does not
        exist yet at that point, so an override reading it must tolerate its absence.
        """
        build = self._binding.request_from or default_request_from
        return build(self._types.request, values, self._request_context)

    async def read_response(self, response: Message) -> Mapping[str, object]:
        """Return the FinalResultData values one response message carries, by variable name.

        Overridable like :meth:`build_request`; the declaration's ``response_to`` is the
        pure-function form and :func:`default_response_to` the default.
        """
        read = self._binding.response_to or default_response_to
        return read(response, self._response_context)

    async def execute_method(self) -> None:
        """Call the service and write what answered.

        opensmi's only hook, called after the access level, Lock and startup checks.

        :raises UaStatusCodeError: The code :data:`TRANSPORT_STATUS` gives a transport failure,
            or :data:`REFUSED_BY_THE_SERVICE` when the ``success_field`` came back ``False``
            (after the response has been written).
        """
        request = await self.build_request(await self.read_parameters())
        response = await self._call_the_service(request)
        await self._write_values(self._results, await self.read_response(response))
        self._require_the_service_agreed(response)

    async def _call_the_service(self, request: Message) -> Message:
        """Send one request, turning the two transport failures into UA status codes."""
        try:
            return await self._runtime.call_service(
                service=self._binding.path,
                service_type=self._types,
                request=request,
                server_timeout=self._binding.server_timeout,
                call_timeout=self._binding.call_timeout,
            )
        except _TRANSPORT_FAILURES as error:
            # Logged too: the status code says that the call did not happen, never which service.
            self.logger.exception(
                "The ROS2 service call did not happen",
                binding=self._binding.name,
                service=self._binding.path,
            )
            raise ua.UaStatusCodeError(_transport_status(error)) from error

    def _require_the_service_agreed(self, response: Message) -> None:
        """Refuse a call the service answered with no, if the declaration named the field.

        Read off the response message, not FinalResultData, so a selection that hides the field
        does not hide the refusal.

        :raises UaStatusCodeError: When the named field came back ``False``.
        """
        named = self._binding.success_field
        if named is None or getattr(response, named):
            return
        raise ua.UaStatusCodeError(REFUSED_BY_THE_SERVICE)


def _transport_status(error: Exception) -> int:
    """Return the UA status code one transport failure becomes.

    The fallback is unreachable today. It keeps an error class added later without a table entry
    on opensmi's own answer instead of a ``StopIteration`` inside an exception handler.
    """
    return next(
        (code for kind, code in TRANSPORT_STATUS.items() if isinstance(error, kind)), StatusCodes.BadInternalError
    )


def _require_a_bool_field(response_type: type[Message], binding: MethodBinding) -> None:
    """Refuse a ``success_field`` the response has no bool field for.

    :raises SuccessFieldError: When the field is absent, or is present and is not a ``bool``.
    """
    named = binding.success_field
    if named is None:
        return
    if SEPARATOR in named:
        message = (
            f"the Binding {binding.name!r} names {named!r} as the field carrying the verdict of "
            f"{response_type.ros2.type_name}, and a verdict is read off a field of the response "
            f"itself rather than down a path; name a top-level bool field"
        )
        raise SuccessFieldError(message)
    described = next((each for each in response_type.ros2.fields if each.name == named), None)
    if described is not None and described.field_type == PrimitiveType(name="bool"):
        return
    listed = ", ".join(each.name for each in response_type.ros2.fields) or "no fields at all"
    found = "is not there" if described is None else f"is a {described.ros2_text}, not a bool"
    message = (
        f"the Binding {binding.name!r} names {named!r} as the field carrying the verdict of "
        f"{response_type.ros2.type_name}, and it {found}; that response has: {listed}"
    )
    raise SuccessFieldError(message)
