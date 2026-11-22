# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The TF Binding, live and both ways: SpatialObjects at init, Locations on a poll, frames out.

:class:`ROS2SpatialMixin` gives a machinery item that declared a TF Binding a SpatialObject whose
mandatory ``PositionFrame`` is the declared head frame (Machine and Components).
:class:`ROS2TransformMirrorMixin` goes on the Machine alone and polls a snapshot of the facet's
record of ``/tf`` and ``/tf_static``, rendering one Location per frame the robot publishes
. A Location is opensmi's ``CartesianFrame``: a ``3DFrame`` in metres and degrees whose
pose is the transform its TF parent published and whose ``Base`` is the node holding that parent, or
the World Frame at the tree's root. A frame is filed once and never moved. Locations are
created through :meth:`~opensmi.ros2.server.mutation.ROS2MutationMixin.mutate_address_space_with`,
and the mutation cap counts Locations, not nodes: a Location is about ten nodes.

Outward, a ``TFBinding`` may name :class:`~opensmi.ros2.server.bindings.PublishedFrame` entries.
Each gets a client-writable Location in ``AttachPoints``, created at init and not counted by the
cap. A frame is inward or outward and never both, because our own broadcast comes back through our
own subscription. Only the pose is writable. A client's write is broadcast once it is stored; the
poll re-asserts moving frames every tick and static frames when they differ from what last went out.
The static publisher is transient-local (:data:`~opensmi.ros2.runtime.tf.TF_STATIC_QOS`), so late
joiners get the frame.
"""

from opensmi.ros2.server.transforms.conversions import METRES, inverted
from opensmi.ros2.server.transforms.filing import ALTERNATIVE_BASE_ABSENT, FRAME_COLLISION
from opensmi.ros2.server.transforms.mirror import SEPARATE_FRAME_TREES, ROS2TransformMirrorMixin
from opensmi.ros2.server.transforms.spatial import ROS2SpatialMixin

__all__ = [
    "ALTERNATIVE_BASE_ABSENT",
    "FRAME_COLLISION",
    "METRES",
    "SEPARATE_FRAME_TREES",
    "ROS2SpatialMixin",
    "ROS2TransformMirrorMixin",
    "inverted",
]
