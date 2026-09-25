#!/usr/bin/env python3
"""cmd_vel → VESC Duet (two hub motors, diff drive) + wheel odometry.

Last line of defense: whatever arrives on cmd_vel is clamped to the HARD limits from robot.yaml
and ramped (soft start). Silence on cmd_vel longer than cmd_timeout ramps to a stop; `estop`
(std_msgs/Bool, latched) brakes immediately. Speed control runs inside the VESC (SET_RPM).

Topics:  sub cmd_vel (Twist), estop (Bool)  ·  pub odom (Odometry), /diagnostics
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from . import vesc_protocol as vp
from .diff_drive import (
    DriveLimits,
    Odometry2D,
    VelocityLimiter,
    apply_min_wheel_speed,
    body_to_wheels,
    erpm_per_mps,
    meters_per_tach,
)
from .estop import EstopLatch
from .robot_config import load_robot_config

STOPPED_MPS = 0.03  # below this a wheel counts as stopped (release the brake)


@dataclass
class _Side:
    name: str
    vesc_id: int
    sign: float
    can_id: int | None = None       # None → local channel, else forward-CAN target
    found: bool = False
    values: vp.VescValues | None = None
    values_t: float = 0.0
    last_tach: int | None = None
    fresh_tach: int | None = None   # tach from a frame not yet consumed by odometry


class VescDriver(Node):
    def __init__(self) -> None:
        super().__init__("vesc_driver")
        self.declare_parameter("config_path", "")
        self.declare_parameter("port", "")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        path = str(self.get_parameter("config_path").value) or None
        cfg = load_robot_config(path)
        dt_cfg = cfg["drivetrain"]
        self.port = str(self.get_parameter("port").value) or str(dt_cfg["port"])
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        radius = float(cfg["wheels"]["radius"])
        self.track = float(cfg["wheels"]["track"])
        pole_pairs = int(dt_cfg["pole_pairs"])
        gear = float(dt_cfg.get("gear_ratio", 1.0))
        self.k_erpm = erpm_per_mps(radius, pole_pairs, gear)
        self.m_per_tach = meters_per_tach(radius, pole_pairs, gear)
        self.rate = float(dt_cfg["rate_hz"])
        self.cmd_timeout = float(dt_cfg["cmd_timeout"])
        self.telemetry_timeout = float(dt_cfg["telemetry_timeout"])
        self.brake_current = float(dt_cfg["brake_current"])
        self.min_wheel_speed = float(dt_cfg.get("min_wheel_speed", 0.0))
        self.limits = DriveLimits.from_config(cfg["limits"])
        self.limiter = VelocityLimiter(self.limits)
        self.odom = Odometry2D(self.track)

        self.left = _Side("left", int(dt_cfg["left"]["vesc_id"]), -1.0 if dt_cfg["left"]["invert"] else 1.0)
        self.right = _Side("right", int(dt_cfg["right"]["vesc_id"]), -1.0 if dt_cfg["right"]["invert"] else 1.0)

        self._lock = threading.Lock()
        self._cmd_v = self._cmd_w = 0.0
        self._cmd_t = -math.inf
        self._estop = EstopLatch()
        self._ser: serial.Serial | None = None
        self._decoder = vp.FrameDecoder()
        self._state = "connecting"
        self._meas_v = self._meas_w = 0.0

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "estop", self._on_estop, latched)
        self.create_timer(1.0, self._publish_diagnostics)

        self._alive = True
        self._io = threading.Thread(target=self._io_loop, name="vesc-io", daemon=True)
        self._io.start()
        self.get_logger().info(
            f"vesc_driver: port={self.port} L=id{self.left.vesc_id} R=id{self.right.vesc_id} "
            f"track={self.track:.3f} max v={self.limits.max_linear} w={self.limits.max_angular} "
            f"accel={self.limits.linear_accel}")

    # ---------------------------------------------------------------- callbacks
    def _on_cmd(self, msg: Twist) -> None:
        with self._lock:
            self._cmd_v, self._cmd_w = float(msg.linear.x), float(msg.angular.z)
            self._cmd_t = time.monotonic()

    def _on_estop(self, msg: Bool) -> None:
        with self._lock:
            changed = self._estop.engaged != bool(msg.data)
            self._estop.set(bool(msg.data))
        if changed:
            self.get_logger().warn(f"ESTOP {'ENGAGED' if msg.data else 'released'}")

    # ---------------------------------------------------------------- serial
    def _send(self, payload: bytes) -> None:
        if self._ser is None:
            raise serial.SerialException("port closed")
        self._ser.write(vp.encode_frame(payload))

    def _read_frames(self) -> None:
        if self._ser is None:
            return
        n = self._ser.in_waiting
        if not n:
            return
        now = time.monotonic()
        for payload in self._decoder.feed(self._ser.read(n)):
            values = vp.parse_values(payload)
            if values is None:
                continue
            for side in (self.left, self.right):
                if values.controller_id == side.vesc_id:
                    side.values, side.values_t = values, now
                    side.fresh_tach = values.tachometer

    def _await_values(self, side: _Side, can_id: int | None, timeout: float = 0.4) -> bool:
        side.values = None
        self._send(vp.cmd_get_values(can_id))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._read_frames()
            if side.values is not None:
                return True
            time.sleep(0.01)
        return False

    def _connect(self) -> bool:
        try:
            self._ser = serial.Serial(self.port, 115200, timeout=0.0)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warn(f"{self.port}: {exc}", throttle_duration_sec=5.0)
            self._ser = None
            return False
        self._decoder = vp.FrameDecoder()
        for side in (self.left, self.right):
            side.found = False
            side.last_tach = side.fresh_tach = None
            for can_id in (None, side.vesc_id):     # local first, then forward-CAN
                if self._await_values(side, can_id) and side.values and side.values.controller_id == side.vesc_id:
                    side.can_id, side.found = can_id, True
                    break
            how = "missing" if not side.found else ("local" if side.can_id is None else "forward-CAN")
            log = self.get_logger().info if side.found else self.get_logger().error
            log(f"{side.name} VESC id={side.vesc_id}: {how}")
        if not (self.left.found and self.right.found):
            self._close()
            return False
        return True

    def _close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except (serial.SerialException, OSError):
                pass
        self._ser = None

    # ---------------------------------------------------------------- control loop
    def _io_loop(self) -> None:
        period = 1.0 / self.rate
        prev = time.monotonic()
        while self._alive:
            if self._ser is None:
                self._state = "connecting"
                self.limiter.reset()
                if not self._connect():
                    time.sleep(1.0)
                    prev = time.monotonic()
                    continue
                self._state = "ok"
            t0 = time.monotonic()
            dt, prev = t0 - prev, t0
            try:
                self._read_frames()
                self._update_odometry()
                self._command(t0, dt)
                for side in (self.left, self.right):
                    self._send(vp.cmd_get_values(side.can_id))
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f"VESC serial: {exc} — reconnecting")
                self._close()
                continue
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def _command(self, now: float, dt: float) -> None:
        with self._lock:
            stale = now - self._cmd_t > self.cmd_timeout
            v_t, w_t = (0.0, 0.0) if stale else (self._cmd_v, self._cmd_w)
            allowed = self._estop.allow(v_t, w_t, stale)
            latch_state = self._estop.state
        telemetry_ok = all(now - s.values_t < self.telemetry_timeout for s in (self.left, self.right))
        if not allowed or not telemetry_ok:
            self._state = latch_state if not allowed else "telemetry_lost"
            self.limiter.reset()
            for side in (self.left, self.right):
                self._brake(side)
            return
        self._state = "ok"
        v, w = self.limiter.step(v_t, w_t, dt)
        wl, wr = body_to_wheels(v, w, self.track, self.limits.max_wheel_speed)
        if abs(v_t) > 1e-3 or abs(w_t) > 1e-3:      # only while a motion is requested, never when stopping
            wl, wr = apply_min_wheel_speed(wl, wr, self.min_wheel_speed)
        for side, speed in ((self.left, wl), (self.right, wr)):
            if abs(speed) < 1e-3:
                self._brake(side)
            else:
                self._send(vp.cmd_set_rpm(speed * self.k_erpm * side.sign, side.can_id))

    def _brake(self, side: _Side) -> None:
        moving = side.values is not None and abs(side.values.erpm / self.k_erpm) > STOPPED_MPS
        if moving:
            self._send(vp.cmd_set_current_brake(self.brake_current, side.can_id))
        else:
            self._send(vp.cmd_set_current(0.0, side.can_id))

    def _update_odometry(self) -> None:
        l, r = self.left, self.right
        if l.fresh_tach is None or r.fresh_tach is None or l.values is None or r.values is None:
            return
        if l.last_tach is not None and r.last_tach is not None:
            dl = (l.fresh_tach - l.last_tach) * self.m_per_tach * l.sign
            dr = (r.fresh_tach - r.last_tach) * self.m_per_tach * r.sign
            self.odom.update(dl, dr)
        l.last_tach, r.last_tach = l.fresh_tach, r.fresh_tach
        l.fresh_tach = r.fresh_tach = None
        vl = l.values.erpm / self.k_erpm * l.sign
        vr = r.values.erpm / self.k_erpm * r.sign
        self._meas_v, self._meas_w = (vl + vr) / 2.0, (vr - vl) / self.track
        self._publish_odom()

    def _publish_odom(self) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.odom.x
        msg.pose.pose.position.y = self.odom.y
        msg.pose.pose.orientation.z = math.sin(self.odom.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.odom.theta / 2.0)
        msg.twist.twist.linear.x = self._meas_v
        msg.twist.twist.angular.z = self._meas_w
        for cov in (msg.pose.covariance, msg.twist.covariance):
            cov[0] = cov[7] = 0.002
            cov[35] = 0.02   # skid on turns: trust IMU yaw more
            cov[14] = cov[21] = cov[28] = 1e6
        self.odom_pub.publish(msg)

    # ---------------------------------------------------------------- diagnostics
    def _publish_diagnostics(self) -> None:
        status = DiagnosticStatus(name="ugv/vesc_driver", hardware_id=self.port)
        state = self._state
        faults = [s for s in (self.left, self.right) if s.values is not None and s.values.fault != 0]
        if state == "ok" and not faults:
            status.level, status.message = DiagnosticStatus.OK, "ok"
        elif state == "estop":
            status.level, status.message = DiagnosticStatus.WARN, "estop engaged"
        elif state == "rearm":
            status.level, status.message = DiagnosticStatus.WARN, "estop released: waiting for a zero command"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = state if not faults else f"VESC fault on {', '.join(s.name for s in faults)}"
        for side in (self.left, self.right):
            val = side.values
            if val is None:
                status.values.append(KeyValue(key=f"{side.name}.state", value="no data"))
                continue
            status.values.extend([
                KeyValue(key=f"{side.name}.v_in", value=f"{val.v_in:.1f}"),
                KeyValue(key=f"{side.name}.erpm", value=f"{val.erpm:.0f}"),
                KeyValue(key=f"{side.name}.current_motor", value=f"{val.current_motor:.2f}"),
                KeyValue(key=f"{side.name}.temp_fet", value=f"{val.temp_fet:.1f}"),
                KeyValue(key=f"{side.name}.fault", value=str(val.fault)),
            ])
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(status)
        self.diag_pub.publish(arr)

    def destroy_node(self) -> bool:
        self._alive = False
        self._io.join(timeout=1.0)
        if self._ser is not None:
            try:
                for side in (self.left, self.right):
                    self._send(vp.cmd_set_current(0.0, side.can_id))
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warn(f"could not release motors on shutdown: {exc}")
        self._close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VescDriver()
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
