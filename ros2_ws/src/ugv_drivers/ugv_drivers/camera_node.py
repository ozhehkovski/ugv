#!/usr/bin/env python3
"""The single owner of the USB camera: publishes JPEG frames + CameraInfo (intrinsics from robot.yaml).

Low latency: MJPG, 1-frame buffer, a grab loop that always keeps the newest frame.
"""
from __future__ import annotations

import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage

from .robot_config import RobotConfig, camera_intrinsics, load_robot_config


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("camera")
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("capture_width", 1280)
        self.declare_parameter("capture_height", 720)
        self.declare_parameter("pub_width", 640)
        self.declare_parameter("fps", 20.0)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("config_path", "")
        gp = self.get_parameter
        self.device = str(gp("device").value)
        self.cap_w, self.cap_h = int(gp("capture_width").value), int(gp("capture_height").value)
        self.pub_w = int(gp("pub_width").value)
        self.fps = float(gp("fps").value)
        self.quality = int(gp("jpeg_quality").value)
        self.frame_id = str(gp("frame_id").value)
        self.cfg: RobotConfig = load_robot_config(str(gp("config_path").value) or None)

        self.pub = self.create_publisher(CompressedImage, "camera/image/compressed", 5)
        self.info_pub = self.create_publisher(CameraInfo, "camera/camera_info", 5)
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._info: CameraInfo | None = None
        self._alive = True
        self._thread = threading.Thread(target=self._grab_loop, name="camera", daemon=True)
        self._thread.start()
        self.create_timer(1.0 / self.fps, self._publish)

    def _open(self) -> cv2.VideoCapture | None:
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cap_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cap_h)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _grab_loop(self) -> None:
        cap: cv2.VideoCapture | None = None
        while self._alive and rclpy.ok():
            if cap is None:
                cap = self._open()
                if cap is None:
                    self.get_logger().warn(f"camera {self.device} not available — retrying", throttle_duration_sec=10.0)
                    time.sleep(2.0)
                    continue
                self.get_logger().info(f"camera {self.device} opened")
            ok, frame = cap.read()
            if not ok:
                self.get_logger().warn("camera read failed — reopening")
                cap.release()
                cap = None
                continue
            h, w = frame.shape[:2]
            if w != self.pub_w:
                nh = max(1, int(self.pub_w * h / w))
                frame = cv2.resize(frame, (self.pub_w, nh), interpolation=cv2.INTER_AREA)
                h, w = nh, self.pub_w
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if not ok:
                continue
            with self._lock:
                self._jpeg = buf.tobytes()
                if self._info is None or self._info.width != w:
                    self._info = self._make_info(w, h)
        if cap is not None:
            cap.release()

    def _make_info(self, w: int, h: int) -> CameraInfo:
        fx, fy, cx, cy, _, _ = camera_intrinsics(self.cfg, w, h)
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width, info.height = w, h
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _publish(self) -> None:
        with self._lock:
            jpeg, info = self._jpeg, self._info
        if jpeg is None or info is None:
            return
        stamp = self.get_clock().now().to_msg()
        msg = CompressedImage(format="jpeg", data=jpeg)
        msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
        self.pub.publish(msg)
        info.header.stamp = stamp
        self.info_pub.publish(info)

    def destroy_node(self) -> bool:
        self._alive = False
        self._thread.join(timeout=2.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
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
