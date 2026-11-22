# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The TF tree as RSL Locations."""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

from opensmi.ros2.server.bindings._base import AwaitedBinding
from opensmi.ros2.server.errors import (
    DeclarationError,
    FrameDirectionError,
)
from opensmi.ros2.server.spatial import excluded


@dataclass(frozen=True, slots=True, kw_only=True)
class PublishedFrame:
    """One frame this object puts on the ROS2 frame network, and the pose it starts at.

    The outward half of a TF Binding. Inward, which frames exist is discovered. Outward each transform an
    operator wants to assert is named here, which creates a client-writable Location and a link on
    ``/tf``. It reads like an alternative: where ``frame`` sits in ``base``, answered by a client for the
    robot.

    :ivar base: The frame this one's pose is relative to, becoming ``header.frame_id``. It is the frame's TF
        parent and what the Location's ``Base`` resolves through. Declared, since nobody else publishes
        this frame.
    :ivar frame: The frame this declaration owns, becoming ``child_frame_id``. It may not be declared
        twice, be its own parent, or be any object's head frame.
    :ivar static: Whether it goes on ``/tf_static`` instead of ``/tf``. A static frame is published when its
        value changes, not every tick. Our own publisher is used because
        ``tf2_ros.StaticTransformBroadcaster`` silently drops an update to a frame it already sent.
    :ivar position: Where the frame sits in its base until a client says otherwise, in metres. Declared so
        that "nobody wrote it yet" is distinguishable from "at the origin"; leaving it out means identity.
    :ivar orientation: The ``A``, ``B`` and ``C`` angles RSL holds, in degrees: roll about X, pitch about
        Y, yaw about Z. The conversion to the quaternion ROS2 wants is ours.
    """

    base: str
    frame: str
    static: bool = False
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True, slots=True, kw_only=True)
class TFBinding(AwaitedBinding):
    """The TF tree, rendered as RSL Locations inside the SpatialObject of whatever owns them.

    The second keyed Binding: one ROS2 path, many OPC UA objects, membership taken from the messages. It
    is not a :class:`KeyedBinding` because a frame is a ``SpatialLocationType`` Variable in a folder
    OPC 10000-210 names, with a ``Base`` pointing at another Location, and a companion specification does
    not belong in the generic mechanism. Named for its ROS2 side.

    One declaration per object, and its content is one frame name. The declaring machinery item gains a
    SpatialObject whose ``PositionFrame`` is ``head``, and every frame under that head belongs to it, down
    to the next declared head. A Machine declares one for itself and so does a Component; the
    Machine's mirror runs the whole partition.

    Where TF is read from and how often are facts about the Machine, declared once in
    :attr:`ROS2Config.transforms <opensmi.ros2.server.config.ROS2Config.transforms>`. The keywords
    ``path``, ``static_path``, ``depth``, ``interval`` and ``quiet_after`` stay here only so that writing
    one is refused with a message saying where it went.

    :ivar name: The Binding's name, used in BindingEvents and the mutation cap. It is not a prefix on any
        browse name: a frame's raw ``child_frame_id`` is the ``DisplayName`` and a sanitised form the
        BrowseName.
    :ivar head: The one frame this object is. It becomes the SpatialObject's mandatory ``PositionFrame``;
        ownership of everything else follows from the tree.
    :ivar alternatives: The composed poses this object publishes, each a ``(base, frame)`` pair. Each
        becomes a Location in ``AlternativeFrames`` holding where ``frame`` sits in ``base`` (OPC 10000-210,
        section 6.1.1). Declared, since nothing in a frame tree says which compositions are wanted.
    :ivar publishes: The frames this object puts on the frame network instead of reading, each a
        :class:`PublishedFrame`. None may also be read inward (:class:`FrameDirectionError`). A declaration
        that publishes nothing never advertises ``/tf``.
    :ivar exclude: ``fnmatch`` globs over whole frame ids that are not rendered. A frame removed from the
        middle of a chain re-bases its children onto the nearest surviving ancestor.
    :ivar required: Whether the StartupSkill waits for a publisher on a transform topic before ``Running``.
        Presence only, never a particular frame.
    :ivar startup_timeout: Seconds that wait lasts, or ``None`` for
        :attr:`~opensmi.ros2.server.config.ROS2Config.startup_timeout`.
    :ivar path: Machine-wide; refused here.
    :ivar static_path: Machine-wide; refused here.
    :ivar depth: Machine-wide; refused here.
    :ivar interval: Machine-wide; refused here.
    :ivar quiet_after: Machine-wide; refused here.
    """

    kind: ClassVar[str] = "TF Binding"

    head: str
    alternatives: tuple[tuple[str, str], ...] = ()
    publishes: tuple[PublishedFrame, ...] = ()
    exclude: tuple[str, ...] = ()
    # The five Machine-wide settings are fields only so that writing one gets a message saying where it
    # went. None is the only allowed value.
    path: str | None = None
    static_path: str | None = None
    depth: int | None = None
    interval: float | None = None
    quiet_after: float | None = None

    def __post_init__(self) -> None:
        """Refuse a contradictory declaration.

        :raises DeclarationError: When it names no head frame, excludes its own head, declares an
            alternative that is not a pair of two different frames, or publishes a frame that names no
            pair, is its own parent, is declared twice or is one of its own exclusions.
        :raises FrameDirectionError: When it publishes its own head frame, which is read inward.
        :raises DeclarationError: When it declares where TF is read from or how often it is
            polled.
        :raises RequiredDeclarationError: When ``startup_timeout`` is declared without ``required``, or
            is not positive.
        """
        self._require_a_deadline_that_means_something()
        self._require_nothing_the_machine_decides()
        if not self.head:
            message = (
                f"the TF Binding {self.name!r} names no head frame; the head is the one frame "
                f"this object *is*, and without it nothing in the tree belongs to it"
            )
            raise DeclarationError(message)
        if excluded(self.head, self.exclude):
            message = (
                f"the TF Binding {self.name!r} excludes its own head frame {self.head!r}; the "
                f"head is this object's mandatory PositionFrame, so excluding it asks for a "
                f"SpatialObject that OPC 10000-210 does not allow"
            )
            raise DeclarationError(message)
        for alternative in self.alternatives:
            self._require_a_composable_pair(alternative)
        self._require_publishable_frames()

    @property
    def broadcasts(self) -> bool:
        """Return whether this declaration puts anything on the frame network.

        A Machine that publishes no frame must not appear on ``/tf`` as a publisher, since every TF
        Binding reads that presence as a transform source being up.
        """
        return bool(self.publishes)

    @property
    def published_frames(self) -> frozenset[str]:
        """Return the frames this declaration publishes."""
        return frozenset(published.frame for published in self.publishes)

    def _require_publishable_frames(self) -> None:
        """Refuse outward frames that contradict each other or the declaration.

        :raises DeclarationError: When a published frame names no pair, is its own parent, is declared
            twice, or is excluded by this declaration.
        :raises FrameDirectionError: When a published frame is this object's own head.
        """
        seen: set[str] = set()
        for published in self.publishes:
            self._require_a_publishable_pair(published)
            if published.frame == self.head:
                message = (
                    f"the TF Binding {self.name!r} publishes {published.frame!r} and that is its "
                    f"own head frame; the head is read *inward* -- it is the SpatialObject's "
                    f"mandatory PositionFrame and the poll writes it from whatever the tree says "
                    f"-- so a frame is inward or outward and never both. Publish a frame of its "
                    f"own above or below the head instead"
                )
                raise FrameDirectionError(message)
            if excluded(published.frame, self.exclude):
                message = (
                    f"the TF Binding {self.name!r} publishes {published.frame!r} and also excludes "
                    f"it; an exclusion says this object does not render that frame and publishing "
                    f"it says this object decides where it is, and one declaration cannot mean both"
                )
                raise DeclarationError(message)
            if published.frame in seen:
                message = (
                    f"the TF Binding {self.name!r} publishes {published.frame!r} twice; a frame has "
                    f"one parent and one publisher, so two entries are two answers to one question "
                    f"and which of them reached the robot would be the order they were typed in"
                )
                raise DeclarationError(message)
            seen.add(published.frame)

    def _require_a_publishable_pair(self, published: PublishedFrame) -> None:
        """Refuse a published frame that is not a transform.

        :raises DeclarationError: When either name is empty, or the two are one frame.
        """
        if published.frame and published.base and published.frame != published.base:
            return
        if not published.frame or not published.base:
            message = (
                f"the TF Binding {self.name!r} publishes (base={published.base!r}, "
                f"frame={published.frame!r}), which names no transform; a published frame is "
                f"*where this frame sits in that base*, so both halves are frames"
            )
            raise DeclarationError(message)
        message = (
            f"the TF Binding {self.name!r} publishes {published.frame!r} relative to itself, so it "
            f"would be its own parent; tf2 gives a frame one parent and it is never the frame, and "
            f"there is no transform here to put on the wire"
        )
        raise DeclarationError(message)

    def _require_nothing_the_machine_decides(self) -> None:
        """Refuse a value that belongs to the whole Machine.

        :raises DeclarationError: When any of the five Machine-wide keywords carries a value.
        """
        written = {
            name: value
            for name, value in (
                ("path", self.path),
                ("static_path", self.static_path),
                ("depth", self.depth),
                ("interval", self.interval),
                ("quiet_after", self.quiet_after),
            )
            if value is not None
        }
        if not written:
            return
        listed = ", ".join(f"{name}={value!r}" for name, value in written.items())
        message = (
            f"the TF Binding {self.name!r} declares {listed}; where TF is read from, how often "
            f"the frame network is polled and how long a frame may go unpublished before its "
            f"Location says so are facts about the Machine -- one Runtime is one Domain ID and "
            f"one frame network, and one Machine runs one poll over the "
            f"partition every declared head takes part in -- so they are declared once, in the "
            f"[ros2.transforms] table of the machine file (ROS2Config.transforms). What stays here "
            f"is what is true of this object: its head frame, its exclusions, its alternatives"
        )
        raise DeclarationError(message)

    def _require_a_composable_pair(self, alternative: tuple[str, str]) -> None:
        """Refuse an alternative that is not two different, non-empty frame names.

        :raises DeclarationError: When the pair is invalid.
        """
        base, frame = alternative
        if base and frame and base != frame:
            return
        message = (
            f"the TF Binding {self.name!r} declares the alternative {alternative!r}; an "
            f"alternative is a (base, frame) pair asking where one frame sits in another, so the "
            f"two are different frames and neither of them is empty"
        )
        raise DeclarationError(message)

    @property
    def needs_the_graph(self) -> bool:
        """Return ``False``: both topics carry ``tf2_msgs/msg/TFMessage`` and the frames are content."""
        return False
