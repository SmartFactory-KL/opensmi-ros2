# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The SpatialObject of a machinery item that declared a TF Binding."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from opensmi.server.spatial_object import SpatialObject

from opensmi.ros2.server.bindings import TFBinding
from opensmi.ros2.server.config import ROS2Config, TransformConfig
from opensmi.ros2.server.errors import (
    DeclarationError,
)
from opensmi.ros2.server.mutation import ROS2MutationMixin
from opensmi.ros2.server.transforms.conversions import _ORIGIN
from opensmi.ros2.server.transforms.records import (
    _Head,
)

if TYPE_CHECKING:
    from collections.abc import Mapping

    from opensmi.ros2.runtime.interface import RuntimeInterface
    from opensmi.ros2.runtime.tf import TFWatch


class ROS2SpatialMixin(ROS2MutationMixin):
    """Gives a machinery item that declared a TF Binding its SpatialObject and its TF reading.

    Goes on the Machine and on Components, because OPC 10000-210 6.1.1 defines the SpatialObjectType
    as an AddIn extending an existing object. It also opens the item's TF watch. The facet
    reference-counts, so any number of items share one pair of subscriptions, and the watch must
    exist before a required TF Binding waits on it. Topics and depth come from the Machine's
    :class:`~opensmi.ros2.server.config.TransformConfig`, so every item reads the same frame
    network.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Start out with no SpatialObject, and pass every argument to opensmi."""
        super().__init__(*args, **kwargs)
        self._ros2_spatial_object: SpatialObject | None = None
        self._ros2_tf_watch: TFWatch | None = None

    @property
    def ros2_tf_binding(self) -> TFBinding | None:
        """Return the one TF Binding this item declared, or ``None``.

        A machinery item has one head frame, and OPC 10000-210 gives a SpatialObject one
        ``PositionFrame``.

        :raises DeclarationError: When this item declares more than one.
        """
        declared = [each for each in getattr(self, "ros2_bindings", ()) if isinstance(each, TFBinding)]
        if not declared:
            return None
        if len(declared) > 1:
            listed = ", ".join(repr(each.name) for each in declared)
            message = (
                f"{type(self).__name__} declares {len(declared)} TF Bindings ({listed}); a "
                f"machinery item is one thing in space, so it has one head frame and one "
                f"SpatialObject, and OPC 10000-210 gives that object exactly one PositionFrame"
            )
            raise DeclarationError(message)
        return declared[0]

    @property
    def ros2_spatial_object(self) -> SpatialObject | None:
        """Return this item's SpatialObject, or ``None`` when it declared no TF Binding."""
        return self._ros2_spatial_object

    @property
    def ros2_transform_presence(self) -> Mapping[str, bool]:
        """Return, per TF Binding name, whether anything is publishing transforms right now.

        Presence only, never a particular frame: a required TF Binding that waited for ``tool0``
        would keep the Machine from starting until somebody switched an arm on.
        """
        binding = self.ros2_tf_binding
        if binding is None:
            return {}
        return {binding.name: bool(self._ros2_runtime_for_tf().tf.satisfied)}

    async def _init_ros2_spatial_object(self) -> None:
        """Create this item's SpatialObject and open its TF watch, if it declared a TF Binding.

        The ``PositionFrame`` starts at zero metres on the World Frame; the first poll re-bases it
        onto its head frame's TF parent. The SpatialObject is created here instead of by opensmi's
        spatial-object mixin, because that mixin needs a typed-parent property a Component need not
        have.

        :raises DeclarationError: When the Machine has no spatial-object list.
        """
        binding = self.ros2_tf_binding
        if binding is None:
            return
        try:
            listed = self.ros2_root_machine.spatial_object_list
        except Exception as error:
            message = (
                f"{type(self).__name__} declares the TF Binding {binding.name!r} and its Machine "
                f"has no spatial object list; a SpatialObject is an AddIn on an object inside one"
            )
            raise DeclarationError(message) from error
        spatial = SpatialObject(position_frame=_Head(initial_pose=_ORIGIN, initial_base=listed.world_frame))
        spatial.parent = self
        await spatial.ua_create_node(self.ua_node)
        await spatial.init()
        await listed.add_spatial_object(spatial)
        self._ros2_spatial_object = spatial
        # The Machine's topics and depth, so every item on it reads the same frame network.
        settings = self.ros2_transform_settings
        self._ros2_tf_watch = await self._ros2_runtime_for_tf().tf.watch(
            topic=settings.path,
            static_topic=settings.static_path,
            depth=settings.depth,
            # Whether the topics are also advertised is decided here: `satisfied` discounts our own publisher,
            # so the count must be right before anything waits on it.
            broadcasting=binding.broadcasts,
        )

    async def _stop_spatial_object(self) -> None:
        """Close this item's TF watch. Idempotent.

        It runs before the Runtime stops, so an item torn down alone stops reading.
        """
        if self._ros2_tf_watch is None:
            return
        watch, self._ros2_tf_watch = self._ros2_tf_watch, None
        await watch.close()

    @property
    def ros2_transform_settings(self) -> TransformConfig:
        """Return where this item's Machine reads TF from, and how often it polls.

        A Component answers with its Machine's settings. An item whose Machine is not one of ours
        gets the defaults.
        """
        config: ROS2Config | None = getattr(self, "ros2_config", None)
        return (config or ROS2Config()).transforms

    def _ros2_runtime_for_tf(self) -> RuntimeInterface:
        """Return the Runtime this item reaches TF through.

        :raises DeclarationError: When there is none (a wiring fault).
        """
        runtime: RuntimeInterface | None = getattr(self, "runtime", None)
        if runtime is None:  # pragma: no cover -- every caller runs after the Runtime is injected
            message = f"{type(self).__name__} declares a TF Binding and carries no Runtime"
            raise DeclarationError(message)
        return runtime
