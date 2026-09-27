#!/usr/bin/env python3
"""Follow-me: detect people (YOLO on TensorRT), range them with the lidar, track one person, keep 40 cm.

~/enable (std_srvs/SetBool) · ~/status (String, JSON for the web panel) · cmd_vel/follow (Twist)
States: off → search (waiting for a person in front ≤ 3 m) → follow ⇄ lost (stop, look toward the last
bearing, re-acquire by position + clothes). Commands go through the mux / smoother / safety governor /
driver chain like everything else; STOP and manual driving disable the mode (web panel).
"""
from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from .controller import FollowConfig, follow_command
from .detection import appearance, bbox_bearings, floor_range, leg_candidates, lidar_range_in_sector
from .tracker import Candidate, Tracker


def _yaw(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FollowNode(Node):
    def __init__(self) -> None:
        super().__init__("follow")
        self.declare_parameter("engine_path", "/home/luki/ugv_models/yolo11n.engine")
        self.declare_parameter("conf", 0.45)
        self.declare_parameter("detect_hz", 10.0)
        self.declare_parameter("idle_detect_hz", 2.0)
        self.declare_parameter("gap", 0.40)
        self.declare_parameter("search_turn_time", 6.0)
        gp = self.get_parameter
        self.detect_hz = float(gp("detect_hz").value)
        self.idle_hz = float(gp("idle_detect_hz").value)
        self.search_turn_time = float(gp("search_turn_time").value)
        self.ctl = FollowConfig(gap=float(gp("gap").value))

        from .yolo_trt import YoloTrt       # heavy import after the parameters (clear error if it fails)
        self.yolo = YoloTrt(str(gp("engine_path").value), conf=float(gp("conf").value))

        self.lock = threading.Lock()
        self.jpeg: bytes | None = None
        self.intr: tuple[float, float, float, float] | None = None
        self.dets: list[dict] = []           # latest detections: bbox, score, hist
        self.dets_t = 0.0
        self.img_size = (0, 0)
        self.det_fps = 0.0
        self.scan_pts = np.empty((0, 2))
        self.laser_tf: tuple[float, float, float] | None = None

        self.enabled = False
        self.state = "off"
        self.tracker = Tracker()
        self.lost_since = 0.0
        self.last_bearing = 0.0
        self.target_det: int | None = None
        self.gap_now: float | None = None

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel/follow", 10)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.create_subscription(CompressedImage, "camera/image/compressed", self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "camera/camera_info", self._on_info, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)
        self.create_service(SetBool, "~/enable", self._on_enable)
        self.create_timer(0.1, self._control)
        self.create_timer(0.2, self._publish_status)
        self._alive = True
        threading.Thread(target=self._detect_loop, name="yolo", daemon=True).start()
        self.get_logger().info(f"follow: YOLO {self.yolo.imgsz}px ready, gap {self.ctl.gap:.2f} m")

    # ---------------------------------------------------------------- inputs
    def _on_image(self, msg: CompressedImage) -> None:
        with self.lock:
            self.jpeg = bytes(msg.data)

    def _on_info(self, msg: CameraInfo) -> None:
        self.intr = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _on_scan(self, msg: LaserScan) -> None:
        if self.laser_tf is None:
            try:
                t = self.tf.lookup_transform("base_footprint", msg.header.frame_id, Time())
            except TransformException:
                return
            self.laser_tf = (t.transform.translation.x, t.transform.translation.y, _yaw(t.transform.rotation))
        tx, ty, yaw = self.laser_tf
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        lx, ly = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        c, s = math.cos(yaw), math.sin(yaw)
        pts = np.column_stack((tx + c * lx - s * ly, ty + s * lx + c * ly))
        with self.lock:
            self.scan_pts = pts

    def _on_enable(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        if req.data:
            self.tracker.reset()
            self.enabled, self.state = True, "search"
            self.get_logger().info("follow: enabled — waiting for a person in front")
        elif self.enabled:
            self.enabled, self.state = False, "off"
            self.tracker.reset()
            self.cmd_pub.publish(Twist())
            self.get_logger().info("follow: disabled")
        res.success, res.message = True, self.state
        return res

    # ---------------------------------------------------------------- detection thread
    def _detect_loop(self) -> None:
        last = 0.0
        while self._alive and rclpy.ok():
            period = 1.0 / (self.detect_hz if self.enabled else self.idle_hz)
            wait = period - (time.monotonic() - last)
            if wait > 0:
                time.sleep(min(wait, 0.05))
                continue
            with self.lock:
                jpeg = self.jpeg
            if jpeg is None:
                time.sleep(0.1)
                continue
            last = time.monotonic()
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            try:
                found = self.yolo(img)
            except RuntimeError as exc:
                self.get_logger().error(f"YOLO: {exc}", throttle_duration_sec=5.0)
                continue
            dets = [{"bbox": (d.x1, d.y1, d.x2, d.y2), "score": d.score,
                     "hist": appearance(img, (d.x1, d.y1, d.x2, d.y2))} for d in found]
            dt = time.monotonic() - last
            with self.lock:
                self.dets, self.dets_t = dets, time.monotonic()
                self.img_size = (img.shape[1], img.shape[0])
                self.det_fps = 0.8 * self.det_fps + 0.2 / max(dt, period)

    # ---------------------------------------------------------------- control
    def _robot_pose(self) -> tuple[float, float, float] | None:
        try:
            t = self.tf.lookup_transform("odom", "base_footprint", Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, _yaw(t.transform.rotation)

    def _camera_origin(self) -> tuple[float, float, float] | None:
        try:
            t = self.tf.lookup_transform("base_footprint", "camera_link", Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, t.transform.translation.z

    def _candidates(self, robot: tuple[float, float, float]) -> list[Candidate]:
        with self.lock:
            dets = list(self.dets) if time.monotonic() - self.dets_t < 0.5 else []
            pts = self.scan_pts
        cam = self._camera_origin()
        rx, ry, ryaw = robot
        c, s = math.cos(ryaw), math.sin(ryaw)

        def to_odom(bx: float, by: float) -> tuple[float, float]:
            return rx + c * bx - s * by, ry + s * bx + c * by

        out: list[Candidate] = []
        if self.intr is not None and cam is not None:
            fx, fy, cx, cy = self.intr
            for i, d in enumerate(dets):
                x1, y1, x2, y2 = d["bbox"]
                centre, left, right = bbox_bearings(x1, x2, fx, cx)
                hit = lidar_range_in_sector(pts, (cam[0], cam[1]), left, right)
                if hit is not None:
                    bx, by = hit[0], hit[1]
                else:
                    r = floor_range(y2, fy, cy, cam[2])
                    if r is None:
                        continue
                    bx, by = cam[0] + r * math.cos(centre), cam[1] + r * math.sin(centre)
                ox, oy = to_odom(bx, by)
                cand = Candidate(ox, oy, True, d["hist"])
                cand.det_index = i            # type: ignore[attr-defined]  (for the panel overlay)
                out.append(cand)
        for bx, by in leg_candidates(pts):
            ox, oy = to_odom(bx, by)
            out.append(Candidate(ox, oy, False))
        return out

    def _control(self) -> None:
        if not self.enabled:
            return
        now = time.monotonic()
        robot = self._robot_pose()
        if robot is None:
            self.cmd_pub.publish(Twist())
            return
        cands = self._candidates(robot)
        cmd = Twist()
        self.target_det = None
        if not self.tracker.locked:
            self.state = "search"
            if self.tracker.try_lock(cands, robot, now):
                self.state = "follow"
                self.get_logger().info("follow: target locked")
        else:
            used = self.tracker.update(cands, now)
            if used is not None:
                self.target_det = getattr(used, "det_index", None)
            tg = self.tracker.target
            rx, ry, ryaw = robot
            dx, dy = tg.x - rx, tg.y - ry
            bx = math.cos(ryaw) * dx + math.sin(ryaw) * dy
            by = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
            self.gap_now = math.hypot(bx, by) - self.ctl.nose
            if not self.tracker.lost(now):
                if self.state != "follow":
                    self.get_logger().info("follow: target re-acquired")
                self.state = "follow"
                self.last_bearing = math.atan2(by, bx)
                cmd.linear.x, cmd.angular.z = follow_command(self.ctl, bx, by)
            else:
                if self.state != "lost":
                    self.lost_since = now
                    self.get_logger().warn("follow: target lost — stopping, looking around")
                self.state = "lost"
                # look toward where the person went (tank turn), but only for a while
                if now - self.lost_since < self.search_turn_time and abs(self.last_bearing) > 0.35:
                    cmd.angular.z = math.copysign(0.4, self.last_bearing)
        self.cmd_pub.publish(cmd)

    # ---------------------------------------------------------------- status for the panel
    def _publish_status(self) -> None:
        with self.lock:
            dets = [[round(v, 1) for v in d["bbox"]] + [round(d["score"], 2)]
                    for d in self.dets] if time.monotonic() - self.dets_t < 1.0 else []
            size, fps = self.img_size, self.det_fps
        tg = self.tracker.target
        status = {"state": self.state, "enabled": self.enabled, "detections": dets, "target_det": self.target_det,
                  "img": list(size), "fps": round(fps, 1),
                  "gap": round(self.gap_now, 2) if (tg is not None and self.gap_now is not None) else None,
                  "target_odom": [round(tg.x, 2), round(tg.y, 2)] if tg is not None else None}
        self.status_pub.publish(String(data=json.dumps(status)))

    def destroy_node(self) -> bool:
        self._alive = False
        self.cmd_pub.publish(Twist())
        self.yolo.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FollowNode()
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
