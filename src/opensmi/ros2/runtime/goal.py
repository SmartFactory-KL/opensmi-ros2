# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The Goal handle: one ROS2 goal's lifetime and its signals, in one object.

Like :mod:`opensmi.ros2.runtime.subscription`, the handle is read-only for a Skill author and a
separate sink is the write side. A goal has three signals:

* feedback is a stream that ends with the goal. The buffer is bounded and drops the oldest message,
  because the newest progress report is the one worth having;
* the result is a single value, awaited once and readable afterwards. It is a
  :class:`~opensmi.ros2.runtime.outcomes.GoalOutcome` and not an exception, because a goal that
  was rejected, aborted or cancelled did happen;
* cancel is a request. :meth:`Goal.cancel` returns when the server has answered; the actual ending
  arrives on :meth:`Goal.result`.

A server that rejects the goal answers, so the handle comes back settled with
:attr:`Goal.accepted` ``False``. A path with no action server raises
:class:`~opensmi.ros2.runtime.errors.ActionServerUnavailableError`. Nothing here imports ``rclpy``.
"""

from __future__ import annotations

import asyncio
from collections import deque
from typing import TYPE_CHECKING, final

from opensmi.ros2.runtime.errors import GoalClosedError, LoopAffinityError
from opensmi.ros2.runtime.outcomes import GoalOutcome, GoalStatus

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Awaitable, Callable

    from opensmi.ros2.runtime.messages import ActionTypes, Message

__all__ = ["Goal", "GoalSink", "rejected_outcome", "settled_outcome"]


def settled_outcome(status: GoalStatus, result: Message | None) -> GoalOutcome:
    """Return the outcome of an accepted goal that has finished; every ending but success has one fault."""
    faults = () if status is GoalStatus.SUCCEEDED else (f"the goal ended {status.name.lower()}",)
    return GoalOutcome(faults=faults, accepted=True, status=status, result=result)


def rejected_outcome(action: str) -> GoalOutcome:
    """Return the outcome of a goal the server refused, which therefore never ran."""
    return GoalOutcome(
        faults=(f"the action server on {action} rejected the goal",),
        accepted=False,
        status=GoalStatus.UNKNOWN,
        result=None,
    )


@final
class GoalSink:
    """The write side of a :class:`Goal`; every method runs on the Runtime's event loop."""

    def __init__(self, *, depth: int) -> None:
        """Open a sink buffering at most ``depth`` feedback messages, affine to the running loop.

        :param depth: How many feedback messages may wait for a consumer. When the buffer is
            full the oldest is dropped.
        :raises ValueError: When the depth is less than one.
        :raises RuntimeError: When constructed with no running event loop.
        """
        if depth < 1:
            message = f"a feedback depth must be at least 1, not {depth}"
            raise ValueError(message)
        # An `asyncio.Event` binds to the first loop that awaits it; a second loop would hang silently.
        self._loop = asyncio.get_running_loop()
        self._feedback: deque[Message] = deque(maxlen=depth)
        self._arrival = asyncio.Event()
        self._outcome: GoalOutcome | None = None
        self._settled = asyncio.Event()

    @property
    def settled(self) -> bool:
        """Return whether the goal has ended, however it ended."""
        return self._outcome is not None

    @property
    def outcome(self) -> GoalOutcome | None:
        """Return how the goal ended, or ``None`` while it is still running."""
        return self._outcome

    def deliver(self, feedback: Message) -> None:
        """Accept one decoded feedback message, dropping the oldest if the buffer is full."""
        self._feedback.append(feedback)
        self._arrival.set()

    def settle(self, outcome: GoalOutcome) -> None:
        """End the goal, waking everything waiting on it. The first settlement wins."""
        self.require_own_loop()
        if self._outcome is not None:
            return
        self._outcome = outcome
        self._settled.set()
        self._arrival.set()

    async def take_feedback(self) -> Message:
        """Await the next buffered feedback message.

        :raises GoalClosedError: When the goal has ended and no feedback is left buffered.
        :raises LoopAffinityError: When awaited on a loop other than this sink's.
        """
        self.require_own_loop()
        while not self._feedback:
            if self._outcome is not None:
                message = "this goal has ended and has no feedback left"
                raise GoalClosedError(message)
            # Nothing awaits between the clear and the wait, so no message can arrive in between.
            self._arrival.clear()
            await self._arrival.wait()
        return self._feedback.popleft()

    async def await_outcome(self) -> GoalOutcome:
        """Await how the goal ended.

        :raises LoopAffinityError: When awaited on a loop other than this sink's.
        """
        self.require_own_loop()
        await self._settled.wait()
        outcome = self._outcome
        if outcome is None:  # pragma: no cover -- the event is set only by `settle`
            message = "this goal was woken without an outcome"
            raise GoalClosedError(message)
        return outcome

    def require_own_loop(self) -> None:
        """Refuse an operation running on an event loop other than the one this sink opened on.

        :raises LoopAffinityError: When the running loop is not this sink's.
        """
        if asyncio.get_running_loop() is not self._loop:
            message = "this goal is affine to the event loop it was sent on; this call is on another one"
            raise LoopAffinityError(message)


@final
class Goal:
    """One goal sent to a ROS2 action server, from :meth:`Runtime.send_goal`."""

    def __init__(
        self,
        *,
        action: str,
        types: ActionTypes,
        sink: GoalSink,
        request_cancel: Callable[[], Awaitable[None]],
        release: Callable[[], Awaitable[None]],
    ) -> None:
        """Bind a handle to the sink and the two operations the Runtime prepared for it."""
        self._action = action
        self._types = types
        self._sink = sink
        self._request_cancel = request_cancel
        self._release = release
        self._released = False

    def __repr__(self) -> str:
        """Return a representation naming the action and where the goal has got to."""
        outcome = self._sink.outcome
        state = "running" if outcome is None else ("accepted" if outcome.accepted else "rejected")
        if outcome is not None and outcome.accepted:
            state = outcome.status.name.lower()
        return f"<Goal {self._action} {self._types.type_name} ({state})>"

    @property
    def accepted(self) -> bool:
        """Return whether the server took this goal (``False`` only if it refused)."""
        outcome = self._sink.outcome
        return outcome is None or outcome.accepted

    @property
    def outcome(self) -> GoalOutcome | None:
        """Return how this goal ended, or ``None`` while it is running (never waits)."""
        return self._sink.outcome

    async def receive_feedback(self) -> Message:
        """Await the next feedback message.

        :raises GoalClosedError: When the goal has ended and nothing is buffered.
        """
        return await self._sink.take_feedback()

    def __aiter__(self) -> AsyncIterator[Message]:
        """Return an iterator yielding feedback until the goal ends."""
        return self._stream()

    async def _stream(self) -> AsyncIterator[Message]:
        """Yield feedback until the goal ends, then stop cleanly."""
        while True:
            try:
                yield await self._sink.take_feedback()
            except GoalClosedError:
                return

    async def result(self) -> GoalOutcome:
        """Await how this goal ended; every ending (rejected, aborted, cancelled, succeeded) is returned."""
        return await self._sink.await_outcome()

    async def cancel(self) -> None:
        """Ask the server to cancel this goal.

        Returns when the request has been answered, not when the goal has stopped; the ending
        arrives on :meth:`result`. Does nothing for a goal that has already ended.
        """
        if self._sink.settled:
            return
        await self._request_cancel()

    async def close(self) -> None:
        """End this goal's handle, releasing its ROS2 resources. Idempotent.

        It does not cancel the goal; a goal still running is settled as cancelled from this side,
        so nothing waits on a result that will never arrive.
        """
        if self._released:
            return
        self._released = True
        self._sink.settle(
            self._sink.outcome
            or GoalOutcome(
                faults=(f"the handle on {self._action} was closed while the goal was still running",),
                accepted=True,
                status=GoalStatus.CANCELED,
                result=None,
            )
        )
        await self._release()
