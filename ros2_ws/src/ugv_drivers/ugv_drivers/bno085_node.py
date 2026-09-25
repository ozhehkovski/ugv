#!/usr/bin/env python3
"""BNO085 (I2C) → sensor_msgs/Imu.

GAME rotation vector (gyro + accel, no magnetometer → immune to motor magnetic noise), gyro and
accelerometer, published raw in `imu_link`; mounting is expressed by the URDF joint. Reconnects on
I2C errors. Dependencies: adafruit-blinka, adafruit-circuitpython-bno08x, adafruit-extended-bus.
"""
from __future__ import annotations

from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

_UNKNOWN = 1e6


class Bno085Node(Node):
    def __init__(self) -> None:
        super().__init__("bno085")
        self.declare_parameter("i2c_bus", 7)
        self.declare_parameter("address", 0x4A)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("yaw_variance", 0.02)
        self.declare_parameter("gyro_variance", 0.001)
        self.declare_parameter("accel_variance", 0.2)
        gp = self.get_parameter
        self.bus = int(gp("i2c_bus").value)
        self.addr = int(gp("address").value)
        self.frame_id = str(gp("frame_id").value)
        self.yaw_var = float(gp("yaw_variance").value)
        self.gyro_var = float(gp("gyro_variance").value)
        self.accel_var = float(gp("accel_variance").value)

        self._bno: Any = None
        self._warned = False
        self.pub = self.create_publisher(Imu, "imu/data", 20)
        self.create_timer(1.0 / float(gp("rate_hz").value), self._tick)
        self.get_logger().info(f"BNO085 /dev/i2c-{self.bus} 0x{self.addr:02X} → imu/data")

    def _connect(self) -> Any:
        from adafruit_bno08x import (  # type: ignore[import-untyped]
            BNO_REPORT_ACCELEROMETER,
            BNO_REPORT_GAME_ROTATION_VECTOR,
            BNO_REPORT_GYROSCOPE,
        )
        from adafruit_bno08x.i2c import BNO08X_I2C  # type: ignore[import-untyped]
        from adafruit_extended_bus import ExtendedI2C  # type: ignore[import-untyped]

        bno = BNO08X_I2C(ExtendedI2C(self.bus), address=self.addr)
        for report in (BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE, BNO_REPORT_ACCELEROMETER):
            bno.enable_feature(report)
        return bno

    def _tick(self) -> None:
        if self._bno is None:
            try:
                self._bno = self._connect()
            except (OSError, RuntimeError, ValueError) as exc:
                if not self._warned:
                    self.get_logger().warn(f"IMU unavailable: {exc} — retrying")
                    self._warned = True
                return
            self._warned = False
            self.get_logger().info("BNO085 connected")
        try:
            qi, qj, qk, qr = self._bno.game_quaternion
            gx, gy, gz = self._bno.gyro
            ax, ay, az = self._bno.acceleration
        except (OSError, RuntimeError, ValueError, KeyError) as exc:
            self.get_logger().warn(f"IMU read error: {exc} — reconnecting")
            self._bno = None
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = (
            float(qi), float(qj), float(qk), float(qr))
        msg.orientation_covariance = [_UNKNOWN, 0.0, 0.0, 0.0, _UNKNOWN, 0.0, 0.0, 0.0, self.yaw_var]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = float(gx), float(gy), float(gz)
        g = self.gyro_var
        msg.angular_velocity_covariance = [g, 0.0, 0.0, 0.0, g, 0.0, 0.0, 0.0, g]
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = float(ax), float(ay), float(az)
        a = self.accel_var
        msg.linear_acceleration_covariance = [a, 0.0, 0.0, 0.0, a, 0.0, 0.0, 0.0, a]
        self.pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Bno085Node()
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
