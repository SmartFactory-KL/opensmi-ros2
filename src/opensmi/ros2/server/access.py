# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Access control for this half: three levels and two overrides.

Every write and every call passes two separate gates: an access level and ownership of the Machine's
Lock. Reading needs neither. opensmi answers a refusal with ``BadUserAccessDenied`` for the level and
``BadRequiresLock`` for a call without the Lock.

=============================================  =========================================================
Introspection listings and the report          :data:`LISTING_ACCESS_LEVEL`, Lock bypassed
Skills, including the StartupSkill             :data:`SKILL_ACCESS_LEVEL`, plus the Lock
Methods backed by a ROS2 service               :data:`SKILL_ACCESS_LEVEL`, plus the Lock
Mutators such as ``BindTopic``                 :data:`MUTATOR_ACCESS_LEVEL` by default, plus the Lock
=============================================  =========================================================

The numbers are opensmi's default user table (0 visitor, 10 orchestrator, 20 operator). A site with a
different table overrides them in :class:`~opensmi.ros2.server.config.ROS2Config`.

The listings skip the Lock through :class:`ROS2GatedMethod` (overrides the callable's access
condition) and :class:`ROS2ArgumentVariable` (overrides the variable's access check).

opensmi releases the Lock after ten minutes without a call to ``InitLock``, ``BreakLock`` or
``RenewLock``; a variable write does not count. A client holding the Lock calls ``RenewLock`` at
least every ten minutes.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, ClassVar, Final

from asyncua import ua
from asyncua.crypto.permission_rules import UserRole
from asyncua.ua.status_codes import StatusCodes
from opensmi.server import BaseMethod, UaVariable
from typing_extensions import override

if TYPE_CHECKING:
    from opensmi.server.protocols import UserAuthorization

__all__ = [
    "LISTING_ACCESS_LEVEL",
    "MUTATOR_ACCESS_LEVEL",
    "SKILL_ACCESS_LEVEL",
    "ROS2ArgumentVariable",
    "ROS2GatedMethod",
]

LISTING_ACCESS_LEVEL: Final = 0
"""What a read-only introspection listing requires: nothing beyond being authenticated."""

SKILL_ACCESS_LEVEL: Final = 10
"""What a Skill, a service Method and a publishing Binding's Parameters require: the orchestrator."""

MUTATOR_ACCESS_LEVEL: Final = 20
"""What changing the address space from a client requires by default: the operator."""


class ROS2GatedMethod(BaseMethod):
    """A Method that can answer without the Lock.

    opensmi checks a callable's level and the Lock in one condition and offers no bypass, so this class
    overrides that condition. A subclass that leaves :attr:`bypasses_the_lock` alone behaves like
    ``BaseMethod``.

    :cvar bypasses_the_lock: Whether the Machine's Lock is skipped. ``True`` only on the read-only
        introspection listings: nothing they expose changes the robot, the address space or the
        server state.
    """

    bypasses_the_lock: ClassVar[bool] = False

    @override
    async def _condition_access_allowed(self, user: UserAuthorization) -> bool:
        """Check the level, and the Lock unless this Method bypasses it.

        :raises UaStatusCodeError: ``BadUserAccessDenied`` when the level is too low, and, where the
            Lock is not bypassed, ``BadRequiresLock`` when the caller does not hold it.
        """
        if not self.bypasses_the_lock:
            return await super()._condition_access_allowed(user)
        if user.role == UserRole.Admin or user.current_access_level >= self.minimum_access_level:
            return True
        await self.ua_log_error(
            f"Denied, user '{user.name}' has no access to '{self.path}' "
            f"(minimum access level of {self.minimum_access_level} required)!"
        )
        raise ua.UaStatusCodeError(StatusCodes.BadUserAccessDenied)


class ROS2ArgumentVariable(UaVariable[Any]):
    """An argument of a Lock-bypassing Method, writable at its own level and without the Lock.

    Arguments are Parameters, so using a listing is a write and then a call. Without this class the
    write would go through opensmi's variable check, which asks for the Lock, and a visitor could not
    use the listings. An Admin always passes, anyone else needs this variable's own minimum level.
    Writing a scratch input on a read-only Method changes nothing about the robot or the server.

    Two clients using one listing at once overwrite each other's arguments and read each other's
    answers. That holds for every opensmi Method, which has one ParameterSet and one FinalResultData,
    not one per session.
    """

    @override
    def access_allowed(self, user: UserAuthorization, *, lock_required: bool | None = None) -> bool:
        """Let an Admin through always, and anyone else at this variable's own level."""
        del lock_required  # this variable is never behind the Lock, whatever the caller asks
        return user.role == UserRole.Admin or user.current_access_level >= self.minimum_access_level
