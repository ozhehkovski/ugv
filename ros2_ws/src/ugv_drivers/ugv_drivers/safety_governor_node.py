#!/usr/bin/env python3
"""Direction-aware collision governor between the velocity smoother and the driver.

sub cmd_vel_in (Twist), scan (LaserScan)  ·  pub cmd_vel_out (Twist), safety_status (String)
Fail-safe: no fresh scan → zero command.
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .robot_config import load_robot_config
from .safety_governor import Body, SafetyGovernor


ESCAPE_CANDIDATES = [(v, w) for v in (0.12, 0.0, -0.1) for w in (0.4, 0.0, -0.4) if (v, w) != (0.0, 0.0)]


class SafetyGovernorNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_governor")
        self.declare_parameter("config_path", "")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("stop_margin", 0.05)
        self.declare_parameter("horizon", 1.5)
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("escape_clearance", 0.15)   # well above the Nav2 paddings: Nav2 can plan and move again
        self.declare_parameter("escape_timeout", 8.0)
        gp = self.get_parameter
        cfg = load_robot_config(str(gp("config_path").value) or None)
        ch = cfg["chassis"]
        body = Body(front=float(ch["axle_from_front"]),
                    rear=-(float(ch["length"]) - float(ch["axle_from_front"])),
                    half_width=float(ch["width"]) / 2.0)
        self.gov = SafetyGovernor(
            body,
            stop_margin=float(gp("stop_margin").value),
            horizon=float(gp("horizon").value),
            track=float(cfg["wheels"]["track"]),
            max_wheel_speed=float(cfg["limits"]["max_wheel_speed"]),
            min_wheel_speed=float(cfg["drivetrain"].get("min_wheel_speed", 0.0)),
        )
        self.base_frame = str(gp("base_frame").value)
        self.scan_timeout = float(gp("scan_timeout").value)
        self.points = np.empty((0, 2))
        self.scan_t = -math.inf
        self._laser_tf: tuple[float, float, float] | None = None
        self._last_status = ""

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.pub = self.create_publisher(Twist, "cmd_vel_out", 10)
        self.status_pub = self.create_publisher(String, "safety_status", 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel_in", self._on_cmd, 10)
        # ~/escape runs for seconds: its own thread, so scans and commands keep flowing meanwhile
        self.escape_pub = self.create_publisher(Twist, "cmd_vel/escape", 10)
        self.escape_clearance = float(gp("escape_clearance").value)
        self.escape_timeout = float(gp("escape_timeout").value)
        self.create_service(Trigger, "~/escape", self._on_escape, callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f"safety_governor: body {body}, margin {self.gov.stop_margin} m, horizon {self.gov.horizon} s")

    def _on_scan(self, msg: LaserScan) -> None:
        if self._laser_tf is None:
            try:
                t = self.tf.lookup_transform(self.base_frame, msg.header.frame_id, Time())
            except TransformException as exc:
                self.get_logger().warn(f"no TF {self.base_frame}←{msg.header.frame_id}: {exc}", throttle_duration_sec=5.0)
                return
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self._laser_tf = (t.transform.translation.x, t.transform.translation.y, yaw)
        tx, ty, yaw = self._laser_tf
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        lx, ly = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        c, s = math.cos(yaw), math.sin(yaw)
        self.points = np.column_stack((tx + c * lx - s * ly, ty + s * lx + c * ly))
        self.scan_t = time.monotonic()

    def _on_escape(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        """Back out of a tight spot (e.g. after manual driving) until Nav2 can plan again."""
        t0 = time.monotonic()
        res.success, res.message = False, "timeout"
        moved = False
        try:
            while time.monotonic() - t0 < self.escape_timeout:
                if time.monotonic() - self.scan_t > self.scan_timeout:
                    res.message = "no fresh scan"
                    break
                pts = self.points
                clearance = self.gov.limit(0.0, 0.0, pts).clearance
                if clearance >= self.escape_clearance:
                    res.success = True
                    res.message = f"clearance {clearance:.2f} m" + (" after escape" if moved else " (no escape needed)")
                    break
                best = self.gov.escape_command(pts, ESCAPE_CANDIDATES)
                if best is None:
                    res.message = f"no motion gains clearance (clearance {clearance:.2f} m)"
                    break
                out = Twist()
                out.linear.x, out.angular.z = best[0], best[1]
                self.escape_pub.publish(out)
                moved = True
                time.sleep(0.1)
        finally:
            self.escape_pub.publish(Twist())
        log = self.get_logger().info if res.success else self.get_logger().warn
        log(f"escape: {res.message}")
        return res

    def _on_cmd(self, msg: Twist) -> None:
        out = Twist()
        if time.monotonic() - self.scan_t > self.scan_timeout:
            status = "no_scan"
        else:
            d = self.gov.limit(float(msg.linear.x), float(msg.angular.z), self.points)
            out.linear.x, out.angular.z = d.v, d.w
            status = f"{d.reason} clearance={d.clearance:.2f}"
        self.pub.publish(out)
        self.status_pub.publish(String(data=status))
        reason = status.split()[0]
        if reason != self._last_status and reason in ("blocked", "no_scan"):
            self.get_logger().warn(f"safety: {status} (cmd v={msg.linear.x:.2f} w={msg.angular.z:.2f})")
        self._last_status = reason


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafetyGovernorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
