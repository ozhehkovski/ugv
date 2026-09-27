#!/usr/bin/env python3
"""Publishes `carried` (std_msgs/Bool, latched) from the IMU: tilt or vertical acceleration while the
wheels are not commanded. The resting attitude and gravity are learnt during the first seconds."""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from .carry import CarryDetector

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _roll_pitch(q) -> tuple[float, float]:
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


class CarryNode(Node):
    def __init__(self) -> None:
        super().__init__("carry_detector")
        self.det = CarryDetector()
        self.calib: list[tuple[float, float, float]] = []
        self.ref: tuple[float, float, float] | None = None     # roll, pitch, |a| at rest
        self.pub = self.create_publisher(Bool, "carried", LATCHED)
        self.pub.publish(Bool(data=False))
        self.create_subscription(Imu, "imu/data", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel_safe", self._on_cmd, 10)

    def _on_cmd(self, msg: Twist) -> None:
        if abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self.det.command(time.monotonic())

    def _on_imu(self, msg: Imu) -> None:
        roll, pitch = _roll_pitch(msg.orientation)
        a = msg.linear_acceleration
        acc = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        if self.ref is None:
            self.calib.append((roll, pitch, acc))
            if len(self.calib) >= 100:                           # ~2 s at 50 Hz, robot at rest
                self.ref = tuple(float(v) for v in np.median(np.array(self.calib), axis=0))
                self.get_logger().info(f"carry detector: rest attitude roll {math.degrees(self.ref[0]):.1f}° "
                                       f"pitch {math.degrees(self.ref[1]):.1f}° |g| {self.ref[2]:.2f}")
            return
        tilt = math.hypot(roll - self.ref[0], pitch - self.ref[1])
        was = self.det.carried
        now = self.det.update(time.monotonic(), tilt, acc - self.ref[2])
        if now != was:
            self.get_logger().warn("CARRIED" if now else "put down")
            self.pub.publish(Bool(data=now))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CarryNode()
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
