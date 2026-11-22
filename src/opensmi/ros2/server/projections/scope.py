# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""What is in scope, and how each type in it is rendered.

Every ROS2 type in :data:`SCOPE` is classified in :data:`CLASSIFIED` with one of four verdicts, and
a CI job checks the record against a live ROS2 installation in both directions: an unclassified type
fails the build, and so does a classification that no longer matches what the registry does. The
check runs from the ROS2 side only, because most of what the framework resolves (device health,
counters, identification, the state machines) has no ROS2 source.

* :data:`WALK`: the generic field-walk renders it.
* :data:`CURATED`: a shipped Projection renders it, because the walk would get it wrong.
* :data:`KEYED`: it repeats a message whose elements a client browses, so a
  :class:`~opensmi.ros2.server.bindings.KeyedBinding` makes them members of their own. The walk
  could also render it, as one array of a derived structure. A diagnostic id is a thing
  to subscribe to, and a waypoint is a value in a list.
* :data:`EMPTY`: it has no fields and renders nothing. ``std_msgs/msg/Empty`` is the request of
  every no-argument service, so refusing it would stop a Machine over a correct declaration.

The ``std_msgs`` multi-arrays and ``sensor_msgs/msg/PointCloud2`` are :data:`KEYED` even though
their ``data`` suits a blob, because their ``layout/dim`` and ``fields`` are lists somebody wants to
read one at a time. A deployment binds the bulk buffer with its own
:class:`~opensmi.ros2.server.recipes.Blob` and excludes ``fields``.

``rcl_interfaces`` is absent on purpose. Its one type used here, ``ParameterValue``, never reaches
OPC UA: a ROS2-parameter declaration names one typed field of the union and gets a scalar
(:mod:`opensmi.ros2.server.parameters`). Vendor packages are not listed either, since no shipped
Projection covers a vendor type. The first one that does should bring its package into this record.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Final

if TYPE_CHECKING:
    from collections.abc import Mapping

__all__ = ["CLASSIFIED", "CORE", "CURATED", "EMPTY", "KEYED", "SCOPE", "WALK"]

WALK: Final = "the generic field-walk renders it"
"""The verdict of a type the walk renders correctly."""

CURATED: Final = "a shipped Projection renders it"
"""The verdict of a type this package curates because the walk would get it wrong."""

KEYED: Final = "it repeats a message whose elements are nodes of their own"
"""The verdict of a type whose elements a client browses individually."""

EMPTY: Final = "it has no fields, so there is nothing to render"
"""The verdict of a message with no fields."""

SCOPE: Final[tuple[str, ...]] = (
    "builtin_interfaces",
    "std_msgs",
    "geometry_msgs",
    "sensor_msgs",
    "nav_msgs",
    "diagnostic_msgs",
    "actionlib_msgs",
    "shape_msgs",
    "stereo_msgs",
    "trajectory_msgs",
    "visualization_msgs",
)
"""The interface packages whose message types must all be classified.

These are ``ros2/common_interfaces`` plus ``builtin_interfaces``.
"""

CORE: Final[tuple[str, ...]] = ("std_msgs", "geometry_msgs", "sensor_msgs", "nav_msgs")
"""The packages the completeness check refuses to run without.

Without this, an image with none of them would pass by checking nothing.
"""

CLASSIFIED: Final[Mapping[str, str]] = {
    # -- builtin_interfaces ----------------------------------------------------------------------
    "builtin_interfaces/msg/Duration": WALK,
    "builtin_interfaces/msg/Time": WALK,
    # -- std_msgs --------------------------------------------------------------------------------
    "std_msgs/msg/Bool": WALK,
    "std_msgs/msg/Byte": WALK,
    "std_msgs/msg/ByteMultiArray": KEYED,
    "std_msgs/msg/Char": WALK,
    "std_msgs/msg/ColorRGBA": WALK,
    "std_msgs/msg/Empty": EMPTY,
    "std_msgs/msg/Float32": WALK,
    "std_msgs/msg/Float32MultiArray": KEYED,
    "std_msgs/msg/Float64": WALK,
    "std_msgs/msg/Float64MultiArray": KEYED,
    "std_msgs/msg/Header": WALK,
    "std_msgs/msg/Int16": WALK,
    "std_msgs/msg/Int16MultiArray": KEYED,
    "std_msgs/msg/Int32": WALK,
    "std_msgs/msg/Int32MultiArray": KEYED,
    "std_msgs/msg/Int64": WALK,
    "std_msgs/msg/Int64MultiArray": KEYED,
    "std_msgs/msg/Int8": WALK,
    "std_msgs/msg/Int8MultiArray": KEYED,
    "std_msgs/msg/MultiArrayDimension": WALK,
    "std_msgs/msg/MultiArrayLayout": KEYED,
    "std_msgs/msg/String": WALK,
    "std_msgs/msg/UInt16": WALK,
    "std_msgs/msg/UInt16MultiArray": KEYED,
    "std_msgs/msg/UInt32": WALK,
    "std_msgs/msg/UInt32MultiArray": KEYED,
    "std_msgs/msg/UInt64": WALK,
    "std_msgs/msg/UInt64MultiArray": KEYED,
    "std_msgs/msg/UInt8": WALK,
    "std_msgs/msg/UInt8MultiArray": KEYED,
    # -- geometry_msgs ---------------------------------------------------------------------------
    "geometry_msgs/msg/Accel": WALK,
    "geometry_msgs/msg/AccelStamped": WALK,
    "geometry_msgs/msg/AccelWithCovariance": WALK,
    "geometry_msgs/msg/AccelWithCovarianceStamped": WALK,
    "geometry_msgs/msg/Inertia": WALK,
    "geometry_msgs/msg/InertiaStamped": WALK,
    "geometry_msgs/msg/Point": WALK,
    "geometry_msgs/msg/Point32": WALK,
    "geometry_msgs/msg/PointStamped": WALK,
    "geometry_msgs/msg/Polygon": KEYED,
    "geometry_msgs/msg/PolygonInstance": KEYED,
    "geometry_msgs/msg/PolygonInstanceStamped": KEYED,
    "geometry_msgs/msg/PolygonStamped": KEYED,
    "geometry_msgs/msg/Pose": WALK,
    "geometry_msgs/msg/Pose2D": WALK,
    "geometry_msgs/msg/PoseArray": KEYED,
    "geometry_msgs/msg/PoseStamped": WALK,
    "geometry_msgs/msg/PoseWithCovariance": WALK,
    "geometry_msgs/msg/PoseWithCovarianceStamped": WALK,
    "geometry_msgs/msg/Quaternion": WALK,
    "geometry_msgs/msg/QuaternionStamped": WALK,
    "geometry_msgs/msg/Transform": WALK,
    "geometry_msgs/msg/TransformStamped": WALK,
    "geometry_msgs/msg/Twist": WALK,
    "geometry_msgs/msg/TwistStamped": WALK,
    "geometry_msgs/msg/TwistWithCovariance": WALK,
    "geometry_msgs/msg/TwistWithCovarianceStamped": WALK,
    "geometry_msgs/msg/Vector3": WALK,
    "geometry_msgs/msg/Vector3Stamped": WALK,
    "geometry_msgs/msg/VelocityStamped": WALK,
    "geometry_msgs/msg/VelocityWithCovarianceStamped": WALK,
    "geometry_msgs/msg/Wrench": WALK,
    "geometry_msgs/msg/WrenchStamped": WALK,
    # -- sensor_msgs -----------------------------------------------------------------------------
    "sensor_msgs/msg/BatteryState": WALK,
    "sensor_msgs/msg/CameraInfo": WALK,
    "sensor_msgs/msg/ChannelFloat32": WALK,
    "sensor_msgs/msg/CompressedImage": WALK,
    "sensor_msgs/msg/FluidPressure": WALK,
    "sensor_msgs/msg/Illuminance": WALK,
    "sensor_msgs/msg/Image": WALK,
    "sensor_msgs/msg/Imu": WALK,
    "sensor_msgs/msg/JointState": WALK,
    "sensor_msgs/msg/Joy": WALK,
    "sensor_msgs/msg/JoyFeedback": WALK,
    "sensor_msgs/msg/JoyFeedbackArray": KEYED,
    "sensor_msgs/msg/LaserEcho": WALK,
    "sensor_msgs/msg/LaserScan": WALK,
    "sensor_msgs/msg/MagneticField": WALK,
    "sensor_msgs/msg/MultiDOFJointState": KEYED,
    "sensor_msgs/msg/MultiEchoLaserScan": KEYED,
    "sensor_msgs/msg/NavSatFix": WALK,
    "sensor_msgs/msg/NavSatStatus": WALK,
    "sensor_msgs/msg/PointCloud": KEYED,
    "sensor_msgs/msg/PointCloud2": KEYED,
    "sensor_msgs/msg/PointField": WALK,
    "sensor_msgs/msg/Range": WALK,
    "sensor_msgs/msg/RegionOfInterest": WALK,
    "sensor_msgs/msg/RelativeHumidity": WALK,
    "sensor_msgs/msg/Temperature": WALK,
    "sensor_msgs/msg/TimeReference": WALK,
    # -- nav_msgs --------------------------------------------------------------------------------
    "nav_msgs/msg/Goals": KEYED,
    "nav_msgs/msg/GridCells": KEYED,
    "nav_msgs/msg/MapMetaData": WALK,
    "nav_msgs/msg/OccupancyGrid": CURATED,
    "nav_msgs/msg/Odometry": WALK,
    "nav_msgs/msg/Path": KEYED,
    "nav_msgs/msg/Trajectory": KEYED,
    "nav_msgs/msg/TrajectoryPoint": WALK,
    # -- diagnostic_msgs -------------------------------------------------------------------------
    "diagnostic_msgs/msg/DiagnosticArray": KEYED,
    # An array's statuses are members a client browses; a status's key-value pairs are an array.
    "diagnostic_msgs/msg/DiagnosticStatus": KEYED,
    "diagnostic_msgs/msg/KeyValue": WALK,
    # -- actionlib_msgs --------------------------------------------------------------------------
    "actionlib_msgs/msg/GoalID": WALK,
    "actionlib_msgs/msg/GoalStatus": WALK,
    "actionlib_msgs/msg/GoalStatusArray": KEYED,
    # -- shape_msgs ------------------------------------------------------------------------------
    "shape_msgs/msg/Mesh": KEYED,
    "shape_msgs/msg/MeshTriangle": WALK,
    "shape_msgs/msg/Plane": WALK,
    "shape_msgs/msg/SolidPrimitive": KEYED,
    # -- stereo_msgs -----------------------------------------------------------------------------
    "stereo_msgs/msg/DisparityImage": WALK,
    # -- trajectory_msgs -------------------------------------------------------------------------
    "trajectory_msgs/msg/JointTrajectory": KEYED,
    "trajectory_msgs/msg/JointTrajectoryPoint": WALK,
    "trajectory_msgs/msg/MultiDOFJointTrajectory": KEYED,
    "trajectory_msgs/msg/MultiDOFJointTrajectoryPoint": KEYED,
    # -- visualization_msgs ----------------------------------------------------------------------
    "visualization_msgs/msg/ImageMarker": KEYED,
    "visualization_msgs/msg/InteractiveMarker": KEYED,
    "visualization_msgs/msg/InteractiveMarkerControl": KEYED,
    "visualization_msgs/msg/InteractiveMarkerFeedback": WALK,
    "visualization_msgs/msg/InteractiveMarkerInit": KEYED,
    "visualization_msgs/msg/InteractiveMarkerPose": WALK,
    "visualization_msgs/msg/InteractiveMarkerUpdate": KEYED,
    "visualization_msgs/msg/Marker": KEYED,
    "visualization_msgs/msg/MarkerArray": KEYED,
    "visualization_msgs/msg/MenuEntry": WALK,
    "visualization_msgs/msg/MeshFile": WALK,
    "visualization_msgs/msg/UVCoordinate": WALK,
}
"""Every message type in :data:`SCOPE`, and what renders it.

Each type has its own line, so a type added upstream fails the check instead of joining silently.
"""
