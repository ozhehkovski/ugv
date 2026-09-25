#!/usr/bin/env python3
"""RPLIDAR A1 → sensor_msgs/LaserScan.

Uses the vendored `rplidar` library: on this A1 + CP2102 the Slamtec sllidar_ros2 driver times out
(DTR toggling resets the lidar during its handshake). Reads in a background thread and reconnects.
The 180° mounting is in the URDF (base_link → laser); the scan itself is not rotated.
The stamp is the estimated START of the scan (the library yields a scan only after it completes).
"""
from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .lidar_scan import bin_scan
from .rplidar_vendor import RPLidar, RPLidarException


class RPLidarNode(Node):
    def __init__(self) -> None:
        super().__init__("rplidar")
        self.declare_parameter("serial_port", "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("angle_bins", 360)
        self.declare_parameter("range_min", 0.15)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("invert", True)
        self.declare_parameter("angle_offset_deg", 0.0)
        gp = self.get_parameter
        self.port = str(gp("serial_port").value)
        self.baud = int(gp("baud").value)
        self.frame_id = str(gp("frame_id").value)
        self.bins = int(gp("angle_bins").value)
        self.range_min = float(gp("range_min").value)
        self.range_max = float(gp("range_max").value)
        self.invert = bool(gp("invert").value)
        self.angle_offset = float(gp("angle_offset_deg").value)

        self.scan_period = 0.18          # s, refined online from the inter-scan interval
        self._last_scan_t: float | None = None
        self.pub = self.create_publisher(LaserScan, "scan", 10)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="rplidar", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            lidar: RPLidar | None = None
            try:
                lidar = RPLidar(self.port, baudrate=self.baud)
                info = lidar.get_info()        # info before health: keeps this A1 in sync
                health = lidar.get_health()
                self.get_logger().info(f"RPLIDAR {self.port}: info={info} health={health}")
                for scan in lidar.iter_scans():
                    if self._stop.is_set() or not rclpy.ok():
                        break
                    self._publish(scan)
            except (RPLidarException, OSError) as exc:
                self.get_logger().warn(f"RPLIDAR: {exc} — reconnecting in 1.5 s")
                self._stop.wait(1.5)
            finally:
                if lidar is not None:
                    try:
                        lidar.stop()
                        lidar.stop_motor()
                        lidar.disconnect()
                    except (RPLidarException, OSError) as exc:
                        self.get_logger().debug(f"RPLIDAR close: {exc}")

    def _publish(self, scan: list[tuple[int, float, float]]) -> None:
        t = time.monotonic()
        if self._last_scan_t is not None:
            dt = t - self._last_scan_t
            if 0.05 < dt < 0.5:
                self.scan_period = 0.9 * self.scan_period + 0.1 * dt
        self._last_scan_t = t

        inc = 2.0 * math.pi / self.bins
        msg = LaserScan()
        start = self.get_clock().now() - Duration(seconds=self.scan_period)
        msg.header.stamp = start.to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi - inc
        msg.angle_increment = inc
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.scan_time = self.scan_period
        msg.time_increment = self.scan_period / self.bins
        msg.ranges = bin_scan(scan, self.bins, self.range_min, self.range_max, self.invert, self.angle_offset)
        self.pub.publish(msg)

    def destroy_node(self) -> bool:
        self._stop.set()
        self._thread.join(timeout=2.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RPLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
