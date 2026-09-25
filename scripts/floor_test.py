#!/usr/bin/env python3
"""Floor tests through the full safety chain (publishes cmd_vel/teleop).

  floor_test.py brake    <v> <run_m> [estop]   drive straight, then stop (release or estop); stopping distance
  floor_test.py approach <v> <max_m>           drive toward what is ahead; how the safety zones slow/stop
  floor_test.py turn     <w> <secs>            tank turn in place; how the sweep check limits rotation

Guards: aborts when the front gap < 0.10 m, stops after max distance/time, always stops on exit.
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

FRONT, REAR, HALF_W = 0.555, -0.065, 0.28
ABORT_GAP = 0.10


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Tester:
    def __init__(self) -> None:
        rclpy.init()
        self.n = rclpy.create_node("floor_test")
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self.n)
        self.pose = None       # (x, y, th) from wheel odom
        self.v = self.w = 0.0
        self.pts: list[tuple[float, float]] = []
        self.cmd_out = (0.0, 0.0)
        self.n.create_subscription(Odometry, "/odom", self._odom, 10)
        self.n.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.n.create_subscription(Twist, "/cmd_vel_safe", lambda m: setattr(self, "cmd_out", (m.linear.x, m.angular.z)), 10)
        self.pub = self.n.create_publisher(Twist, "/cmd_vel/teleop", 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.estop = self.n.create_publisher(Bool, "/estop", latched)
        deadline = time.monotonic() + 5.0     # DDS discovery can take a few seconds
        while (self.pose is None or not self.pts) and time.monotonic() < deadline:
            self.spin(0.1)

    def _odom(self, m: Odometry) -> None:
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.v, self.w = m.twist.twist.linear.x, m.twist.twist.angular.z

    def _scan(self, m: LaserScan) -> None:
        try:
            t = self.tf.lookup_transform("base_footprint", m.header.frame_id, Time())
        except TransformException:
            return
        tx, ty, yaw = t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        pts = []
        for i, r in enumerate(m.ranges):
            if math.isfinite(r) and m.range_min <= r <= m.range_max:
                a = m.angle_min + i * m.angle_increment
                lx, ly = r * math.cos(a), r * math.sin(a)
                pts.append((tx + c * lx - s * ly, ty + s * lx + c * ly))
        self.pts = pts

    def front_gap(self) -> float:
        g = sorted(x - FRONT for x, y in self.pts if x > FRONT - 0.02 and abs(y) < HALF_W + 0.03)
        return g[1] if len(g) > 1 else math.inf   # 2nd nearest: ignore a single noisy return

    def sweep_min(self) -> float:
        """Nearest lidar point to the axle center (compare with the 0.62 m sweep radius)."""
        return min((math.hypot(x, y) for x, y in self.pts), default=math.inf)

    def spin(self, secs: float) -> None:
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self.n, timeout_sec=0.01)

    def send(self, v: float, w: float) -> None:
        m = Twist()
        m.linear.x, m.angular.z = v, w
        self.pub.publish(m)

    def stop(self) -> None:
        for _ in range(5):
            self.send(0.0, 0.0)
            self.spin(0.05)

    def dist_from(self, p0) -> float:
        return math.hypot(self.pose[0] - p0[0], self.pose[1] - p0[1])

    def log(self, t: float, extra: str = "") -> None:
        print(f"t={t:5.2f} v={self.v:+.3f} w={self.w:+.3f} cmd_safe=({self.cmd_out[0]:+.2f},{self.cmd_out[1]:+.2f}) "
              f"front_gap={self.front_gap():.2f} nearest={self.sweep_min():.2f} {extra}", flush=True)

    # ---------------------------------------------------------------- tests
    def brake(self, v: float, run_m: float, use_estop: bool) -> None:
        p0, t0, nxt = self.pose, time.monotonic(), 0.0
        print(f"BRAKE v={v} run={run_m} m, start front_gap={self.front_gap():.2f}")
        while self.dist_from(p0) < run_m:
            t = time.monotonic() - t0
            if self.front_gap() < ABORT_GAP or t > 15:
                print("ABORT: guard"); break
            self.send(v, 0.0)
            self.spin(0.05)
            if t >= nxt:
                self.log(t); nxt += 0.25
        v_at, p_stop, t_stop = self.v, self.pose, time.monotonic()
        if use_estop:
            self.estop.publish(Bool(data=True))
        else:
            self.stop()
        while time.monotonic() - t_stop < 3.0:
            self.spin(0.05)
            if abs(self.v) < 0.005 and time.monotonic() - t_stop > 0.3:
                break
        dt = time.monotonic() - t_stop
        self.spin(0.3)
        print(f"RESULT {'estop' if use_estop else 'release'}: speed at stop cmd {v_at:.3f} m/s → "
              f"stopping distance {self.dist_from(p_stop) * 100:.1f} cm in {dt:.2f} s; "
              f"total run {self.dist_from(p0):.2f} m; front_gap now {self.front_gap():.2f} m")
        if use_estop:
            self.estop.publish(Bool(data=False))
            self.spin(0.5)

    def approach(self, v: float, max_m: float) -> None:
        p0, t0, nxt = self.pose, time.monotonic(), 0.0
        print(f"APPROACH v={v} max={max_m} m, start front_gap={self.front_gap():.2f}")
        still = 0.0
        while self.dist_from(p0) < max_m:
            t = time.monotonic() - t0
            if self.front_gap() < ABORT_GAP or t > 25:
                print("ABORT: guard"); break
            self.send(v, 0.0)
            self.spin(0.05)
            still = still + 0.05 if abs(self.v) < 0.005 and t > 2.0 else 0.0
            if t >= nxt:
                self.log(t); nxt += 0.25
            if still > 1.5:
                print("robot held by the safety chain"); break
        self.stop()
        self.spin(0.5)
        print(f"RESULT approach: travelled {self.dist_from(p0):.2f} m, final front_gap {self.front_gap():.2f} m")

    def turn(self, w: float, secs: float) -> None:
        th0, t0, nxt, total, prev = self.pose[2], time.monotonic(), 0.0, 0.0, self.pose[2]
        print(f"TURN w={w} for {secs}s, nearest point {self.sweep_min():.2f} m (sweep radius 0.62)")
        while (t := time.monotonic() - t0) < secs:
            self.send(0.0, w)
            self.spin(0.05)
            d = math.atan2(math.sin(self.pose[2] - prev), math.cos(self.pose[2] - prev))
            total += d; prev = self.pose[2]
            if t >= nxt:
                self.log(t, f"turned={math.degrees(total):+.0f}°"); nxt += 0.25
        self.stop()
        self.spin(0.5)
        print(f"RESULT turn: turned {math.degrees(total):+.1f}° in {secs}s, nearest point {self.sweep_min():.2f} m")


def main() -> None:
    mode = sys.argv[1]
    t = Tester()
    try:
        if t.pose is None:
            raise RuntimeError("no /odom — is the stack running?")
        if mode == "brake":
            t.brake(float(sys.argv[2]), float(sys.argv[3]), len(sys.argv) > 4 and sys.argv[4] == "estop")
        elif mode == "approach":
            t.approach(float(sys.argv[2]), float(sys.argv[3]))
        elif mode == "turn":
            t.turn(float(sys.argv[2]), float(sys.argv[3]))
        else:
            raise SystemExit(__doc__)
    finally:
        t.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
