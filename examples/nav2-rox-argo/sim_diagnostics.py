# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Publish ``/diagnostics`` for the simulated ROX Argo, which has no relayboard to publish them.

On the real robot the relayboard and the drivers publish diagnostics. The simulation has none, so
this node stands in with two entries derived from what the simulation does publish:

* ``rox_argo_sim: drive`` -- OK, with the message ``moving`` or ``standing`` and the commanded and
  measured velocities from ``/cmd_vel`` and ``/odom``; WARN while no odometry arrives;
* ``rox_argo_sim: laser scanners`` -- OK while both scanners publish, WARN with one, ERROR with none.

It runs in the simulation container, never in the Machine: the Machine only reads ``/diagnostics``.
"""

from __future__ import annotations

import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

HARDWARE_ID = "rox_argo_sim"
STALE_AFTER = 2.0
"""Seconds after which a topic that stopped counts as silent."""
MOVING = 0.02
"""The speed, in m/s or rad/s, above which the drive reports ``moving``."""


class SimDiagnostics(Node):
    """Watch odometry, the velocity command and both scanners, and publish a summary once a second."""

    def __init__(self) -> None:
        """Subscribe to what the simulation publishes and start the one-second timer."""
        super().__init__("rox_argo_sim_diagnostics")
        self.odometry: Odometry | None = None
        self.command = Twist()
        self.seen: dict[str, float] = {}
        self.create_subscription(Odometry, "/odom", self.on_odometry, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_command, 10)
        for scanner in ("lidar_1", "lidar_2"):
            self.create_subscription(
                LaserScan,
                f"/{scanner}/scan_filtered",
                lambda _message, scanner=scanner: self.seen.__setitem__(scanner, time.monotonic()),
                qos_profile_sensor_data,
            )
        self.publisher = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0, self.publish)

    def on_odometry(self, message: Odometry) -> None:
        """Keep the latest odometry."""
        self.odometry = message
        self.seen["odom"] = time.monotonic()

    def on_command(self, message: Twist) -> None:
        """Keep the latest velocity command."""
        self.command = message

    def fresh(self, name: str) -> bool:
        """Return whether a topic published within the last :data:`STALE_AFTER` seconds."""
        return time.monotonic() - self.seen.get(name, -STALE_AFTER) < STALE_AFTER

    def drive(self) -> DiagnosticStatus:
        """Return the drive's entry."""
        status = DiagnosticStatus(name=f"{HARDWARE_ID}: drive", hardware_id=HARDWARE_ID)
        if self.odometry is None or not self.fresh("odom"):
            status.level = DiagnosticStatus.WARN
            status.message = "no odometry"
            return status
        twist = self.odometry.twist.twist
        moving = max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.angular.z)) > MOVING
        status.level = DiagnosticStatus.OK
        status.message = "moving" if moving else "standing"
        status.values = [
            KeyValue(key="measured linear x", value=f"{twist.linear.x:.3f}"),
            KeyValue(key="measured linear y", value=f"{twist.linear.y:.3f}"),
            KeyValue(key="measured angular z", value=f"{twist.angular.z:.3f}"),
            KeyValue(key="commanded linear x", value=f"{self.command.linear.x:.3f}"),
            KeyValue(key="commanded linear y", value=f"{self.command.linear.y:.3f}"),
            KeyValue(key="commanded angular z", value=f"{self.command.angular.z:.3f}"),
        ]
        return status

    def scanners(self) -> DiagnosticStatus:
        """Return the laser scanners' entry."""
        reporting = [each for each in ("lidar_1", "lidar_2") if self.fresh(each)]
        level = (DiagnosticStatus.ERROR, DiagnosticStatus.WARN, DiagnosticStatus.OK)[len(reporting)]
        return DiagnosticStatus(
            level=level,
            name=f"{HARDWARE_ID}: laser scanners",
            hardware_id=HARDWARE_ID,
            message=f"{len(reporting)} of 2 scanners report",
            values=[
                KeyValue(key=each, value="reporting" if each in reporting else "silent")
                for each in ("lidar_1", "lidar_2")
            ],
        )

    def publish(self) -> None:
        """Publish both entries."""
        array = DiagnosticArray(status=[self.drive(), self.scanners()])
        array.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(array)


def main() -> None:
    """Spin the node until it is interrupted."""
    rclpy.init()
    node = SimDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
