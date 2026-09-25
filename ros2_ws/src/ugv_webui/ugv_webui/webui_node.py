#!/usr/bin/env python3
"""Operator web panel: map + lidar + robot, camera, status, joystick and a big STOP.

stdlib HTTP server (nothing extra to install on the Jetson):
  GET  /               index.html
  GET  /api/state      JSON snapshot (pose, scan, status); the browser polls it
  GET  /api/map.png    current /map rendered to PNG
  GET  /camera.mjpg    MJPEG stream of camera/image/compressed
  POST /api/estop      {"engage": true|false}  → latched /estop
  POST /api/teleop     {"v": m/s, "w": rad/s}  → cmd_vel/teleop while updates keep coming
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PolygonStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .map_render import occupancy_to_image
from .teleop_gate import TeleopGate

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
MAX_BODY = 4096


def _yaw(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WebUi(Node):
    def __init__(self) -> None:
        super().__init__("webui")
        self.declare_parameter("port", 8090)
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("teleop_max_linear", 0.5)
        self.declare_parameter("teleop_max_angular", 0.8)
        self.declare_parameter("camera_fps", 12.0)
        gp = self.get_parameter
        self.port = int(gp("port").value)
        self.base_frame = str(gp("base_frame").value)
        self.camera_period = 1.0 / float(gp("camera_fps").value)
        self.gate = TeleopGate(0.3, float(gp("teleop_max_linear").value), float(gp("teleop_max_angular").value))
        self.static_dir = os.path.join(get_package_share_directory("ugv_webui"), "static")

        self.lock = threading.Lock()
        self.jpeg: bytes | None = None
        self.jpeg_t = 0.0
        self.map_png: bytes | None = None
        self.map_meta: dict[str, Any] | None = None
        self.scan_base: list[tuple[float, float]] = []
        self.scan_t = 0.0
        self.footprint: list[tuple[float, float]] = []
        self.pose: dict[str, float] | None = None
        self.pose_frame: str | None = None
        self.odom_twist = (0.0, 0.0)
        self.cmd_in = (0.0, 0.0)
        self.cmd_out = (0.0, 0.0)
        self.cmd_out_t = 0.0
        self.source = "none"
        self.estop = False
        self.drive: dict[str, Any] = {"level": None, "message": "нет данных", "values": {}}
        self.drive_t = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.teleop_pub = self.create_publisher(Twist, "cmd_vel/teleop", 10)
        self.estop_pub = self.create_publisher(Bool, "estop", LATCHED)
        self.create_subscription(CompressedImage, "camera/image/compressed", self._on_image, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "map", self._on_map, LATCHED)
        self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PolygonStamped, "footprint", self._on_footprint, 1)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(Twist, "cmd_vel_mux", self._on_cmd_in, 10)
        self.create_subscription(Twist, "cmd_vel_safe", self._on_cmd_out, 10)
        self.create_subscription(String, "cmd_vel_source", self._on_source, 10)
        self.create_subscription(Bool, "estop", self._on_estop, LATCHED)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 10)
        self.create_timer(0.05, self._teleop_tick)
        self.create_timer(0.1, self._pose_tick)

        self.httpd = ThreadingHTTPServer(("0.0.0.0", self.port), self._handler_class())
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, name="http", daemon=True).start()
        self.get_logger().info(f"web panel on http://0.0.0.0:{self.port}")

    # ------------------------------------------------------------------ ROS
    def _on_image(self, msg: CompressedImage) -> None:
        with self.lock:
            self.jpeg, self.jpeg_t = bytes(msg.data), time.monotonic()

    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        img = occupancy_to_image(msg.data, info.width, info.height)
        ok, buf = cv2.imencode(".png", img)
        if not ok:
            self.get_logger().error("map PNG encoding failed")
            return
        with self.lock:
            version = (self.map_meta or {}).get("version", 0) + 1
            self.map_png = buf.tobytes()
            self.map_meta = {
                "version": version, "frame": msg.header.frame_id,
                "resolution": info.resolution, "width": info.width, "height": info.height,
                "origin": [info.origin.position.x, info.origin.position.y, _yaw(info.origin.orientation)],
            }

    def _on_scan(self, msg: LaserScan) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
        except TransformException:
            return
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        yaw = _yaw(tf.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        pts: list[tuple[float, float]] = []
        for i in range(0, len(msg.ranges), 2):
            r = msg.ranges[i]
            if not (math.isfinite(r) and msg.range_min <= r <= msg.range_max):
                continue
            a = msg.angle_min + i * msg.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            pts.append((round(tx + c * lx - s * ly, 3), round(ty + s * lx + c * ly, 3)))
        with self.lock:
            self.scan_base, self.scan_t = pts, time.monotonic()

    def _on_footprint(self, msg: PolygonStamped) -> None:
        with self.lock:
            self.footprint = [(p.x, p.y) for p in msg.polygon.points]

    def _on_odom(self, msg: Odometry) -> None:
        with self.lock:
            self.odom_twist = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _on_cmd_in(self, msg: Twist) -> None:
        with self.lock:
            self.cmd_in = (msg.linear.x, msg.angular.z)

    def _on_cmd_out(self, msg: Twist) -> None:
        with self.lock:
            self.cmd_out, self.cmd_out_t = (msg.linear.x, msg.angular.z), time.monotonic()

    def _on_source(self, msg: String) -> None:
        with self.lock:
            self.source = msg.data

    def _on_estop(self, msg: Bool) -> None:
        with self.lock:
            self.estop = bool(msg.data)

    def _on_diag(self, msg: DiagnosticArray) -> None:
        for st in msg.status:
            if st.name == "ugv/vesc_driver":
                with self.lock:
                    self.drive = {"level": int.from_bytes(st.level, "little") if isinstance(st.level, bytes) else int(st.level),
                                  "message": st.message, "values": {kv.key: kv.value for kv in st.values}}
                    self.drive_t = time.monotonic()

    def _pose_tick(self) -> None:
        for frame in ("map", "odom"):
            try:
                tf = self.tf_buffer.lookup_transform(frame, self.base_frame, Time())
            except TransformException:
                continue
            with self.lock:
                self.pose = {"x": tf.transform.translation.x, "y": tf.transform.translation.y,
                             "th": _yaw(tf.transform.rotation)}
                self.pose_frame = frame
            return
        with self.lock:
            self.pose, self.pose_frame = None, None

    def _teleop_tick(self) -> None:
        cmd = self.gate.current(time.monotonic())
        if cmd is None:
            return
        msg = Twist()
        msg.linear.x, msg.angular.z = cmd
        self.teleop_pub.publish(msg)

    # ------------------------------------------------------------------ API
    def set_estop(self, engage: bool) -> None:
        if engage:
            self.gate.cancel()
        self.estop_pub.publish(Bool(data=engage))
        with self.lock:
            self.estop = engage
        self.get_logger().warn(f"web: estop {'ENGAGED' if engage else 'released'}")

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            return {
                "estop": self.estop,
                "source": self.source,
                "drive": self.drive,
                "drive_age": round(now - self.drive_t, 2) if self.drive_t else None,
                "pose": self.pose,
                "pose_frame": self.pose_frame,
                "odom": {"v": round(self.odom_twist[0], 3), "w": round(self.odom_twist[1], 3)},
                "cmd_in": {"v": round(self.cmd_in[0], 3), "w": round(self.cmd_in[1], 3)},
                "cmd_out": {"v": round(self.cmd_out[0], 3), "w": round(self.cmd_out[1], 3)},
                "cmd_out_age": round(now - self.cmd_out_t, 2) if self.cmd_out_t else None,
                "scan": self.scan_base,
                "scan_age": round(now - self.scan_t, 2) if self.scan_t else None,
                "camera_age": round(now - self.jpeg_t, 2) if self.jpeg_t else None,
                "footprint": self.footprint,
                "map": self.map_meta,
            }

    def _handler_class(self) -> type[BaseHTTPRequestHandler]:
        ui = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, fmt: str, *args: Any) -> None:  # silence per-request logs
                return

            def _send(self, code: int, body: bytes, ctype: str) -> None:
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _json(self, obj: Any, code: int = 200) -> None:
                self._send(code, json.dumps(obj, separators=(",", ":")).encode(), "application/json")

            def do_GET(self) -> None:  # noqa: N802
                path = self.path.split("?", 1)[0]
                if path in ("/", "/index.html"):
                    with open(os.path.join(ui.static_dir, "index.html"), "rb") as f:
                        self._send(200, f.read(), "text/html; charset=utf-8")
                elif path == "/api/state":
                    self._json(ui.snapshot())
                elif path == "/api/map.png":
                    with ui.lock:
                        png = ui.map_png
                    if png is None:
                        self._json({"error": "no map yet"}, 404)
                    else:
                        self._send(200, png, "image/png")
                elif path == "/camera.mjpg":
                    self._stream_camera()
                else:
                    self._json({"error": "not found"}, 404)

            def do_POST(self) -> None:  # noqa: N802
                try:
                    n = int(self.headers.get("Content-Length", "0"))
                    if n > MAX_BODY:
                        raise ValueError("body too large")
                    body = json.loads(self.rfile.read(n) or b"{}")
                    if self.path == "/api/estop":
                        ui.set_estop(bool(body["engage"]))
                    elif self.path == "/api/teleop":
                        if ui.estop:
                            self._json({"error": "estop engaged"}, 409)
                            return
                        ui.gate.set(float(body["v"]), float(body["w"]), time.monotonic())
                    else:
                        self._json({"error": "not found"}, 404)
                        return
                except (ValueError, KeyError, TypeError) as exc:
                    self._json({"error": str(exc)}, 400)
                    return
                self._json({"ok": True})

            def _stream_camera(self) -> None:
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "close")
                self.end_headers()
                last = None
                try:
                    while rclpy.ok():
                        with ui.lock:
                            jpeg = ui.jpeg
                        if jpeg is not None and jpeg is not last:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                             + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                            last = jpeg
                        time.sleep(ui.camera_period)
                except (BrokenPipeError, ConnectionResetError):
                    return  # browser closed the stream

        return Handler

    def destroy_node(self) -> bool:
        self.httpd.shutdown()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WebUi()
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
