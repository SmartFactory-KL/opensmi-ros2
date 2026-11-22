# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Errors for declarations the server half refuses to boot with.

A declaration error is fatal at ``init`` and a runtime condition never is. Each error here
can be known before a node exists, independent of the ROS2 graph. They form their own family, not
leaves of :class:`opensmi.ros2.runtime.errors.ROS2RuntimeError`, so an application catching the
Runtime's errors does not also catch a wrong declaration. The field-selection faults are the
exception: a glob over ROS2 field paths is resolved in the runtime half, so its error lives there.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Iterable

__all__ = [
    "BindingNameError",
    "DeclarationError",
    "DuplicateBindingNameError",
    "DuplicateProjectionError",
    "DuplicateVariableNameError",
    "FileDeclarationError",
    "FrameDirectionError",
    "ImplementationClassError",
    "NotInvertibleError",
    "ParameterDeclarationError",
    "RequiredDeclarationError",
    "SuccessFieldError",
    "UnprojectableFieldError",
]


class DeclarationError(Exception):
    """A declaration this half refuses to boot with. The message names the case.

    Also raised for a wiring fault (no Runtime injected, no event node), a contradicting keyword
    (continuity, ``max_rate``, a reserved name, a Machine-wide setting on a Binding) and a duplicate
    lifecycle node. Only the classes below are told apart.
    """


class FileDeclarationError(DeclarationError):
    """A machine file has one or more mistakes in it.

    A file is checked whole before anything is built, so this is the one declaration error that
    carries several problems. Each names its place as a path through the file, such as
    ``component[1].monitoring[2].message_type``.

    :ivar problems: ``(path, message)`` for each mistake, in file order. A mistake about the file as a
        whole, such as TOML that does not parse, has the empty path.
    """

    def __init__(self, problems: Iterable[tuple[str, str]], *, source: str | None = None) -> None:
        """Keep the problems, and print each of them under a heading that counts them."""
        self.problems = tuple(problems)
        self.source = source
        count = len(self.problems)
        head = f"{source}: " if source else ""
        head += f"{count} problem{'s' if count != 1 else ''} in the machine file"
        lines = [f"  {path}: {message}" if path else f"  {message}" for path, message in self.problems]
        super().__init__("\n".join([head, *lines]))


class DuplicateBindingNameError(DeclarationError):
    """Two Bindings on one Component carry the same name.

    Raised within one source and between the class attribute and the TOML source.
    """


class DuplicateVariableNameError(DeclarationError):
    """Two variables on one container would take the same name.

    An opensmi container files a variable with ``setattr`` without looking anything up, so the second
    would replace the first. This check names the Binding and field path that collided.
    """


class UnprojectableFieldError(DeclarationError):
    """A field has no rendering under the Projection that covers its type.

    A repeated message is not one of these: it renders as one array Variable whose element is a
    structure derived from the message type. This error is for what still has no answer:
    an element type whose Projection renders a browsable Object, a field of an element whose type no
    structure field can carry, and a plan that asks for an Object anywhere, since a message nests as
    complex Variables and not as Objects.
    """


class DuplicateProjectionError(DeclarationError):
    """Two Projections in one registry cover the same ROS2 type and neither says which wins.

    Shadowing is allowed (a Component's entry beats the Machine's), but it must be explicit: an entry
    that replaces another says ``override=True``.
    """


class NotInvertibleError(DeclarationError):
    """A Projection that cannot run backwards was asked to.

    The inverse is derived from the recipe: the drop recipe never inverts, a blob inverts
    only if it is byte-identity, and any other inverse is supplied explicitly. Nothing is filled in
    from defaults, since an invented field would be a value nobody sent.
    """


class ImplementationClassError(DeclarationError):
    """A declaration names a custom implementation class that is not a subclass of the stock class.

    The check lives on the declaration because a TOML file can only produce a string, and refusing it
    there keeps an editable configuration file from choosing a Python class.
    """


class SuccessFieldError(DeclarationError):
    """A Method declaration names a response field that cannot carry a service's verdict.

    The response has no such field, or the field is not a ``bool``. Nothing looks for a field called
    ``success`` by itself, so a typo in ``success_field`` would otherwise give a Method that never
    reports a refusal.
    """


class ParameterDeclarationError(DeclarationError):
    """A ROS2-parameter declaration contradicts itself.

    Cases: a declaration that exposes no parameter, one that marks a name read-only without declaring
    it, and a retry interval that is not a positive number of seconds. Names and types are declared,
    never discovered, so there is nothing on the robot to correct them against.
    """


class BindingNameError(DeclarationError):
    """A name cannot be a browse name: it is empty, contains a ``/``, or does not follow the naming rule."""


class RequiredDeclarationError(DeclarationError):
    """A declaration's ``required`` mark and its startup timeout contradict each other.

    Either a ``startup_timeout`` on a Binding nothing waits for, or one that is not a positive number
    of seconds. Every Binding is optional unless marked required. A publishing Binding can
    never be required and carries neither field.
    """


class FrameDirectionError(DeclarationError):
    """One TF frame is declared both as read inward and as published outward.

    An outward frame's Location is where its pose is decided, an inward frame's Location is written
    from ``/tf``. A frame that were both would publish the client's pose, read it back from our own
    subscription and overwrite the client's value on the next tick. Refusing the declaration rules the
    loop out. Echo suppression is the answer for values a client and a robot genuinely share, such as
    ROS2 parameters; one frame has one publisher.
    """
