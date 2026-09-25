"""Access to ugv_description/config/robot.yaml plus derived geometry. No ROS runtime imports."""
from __future__ import annotations

import math
import os
from typing import Any

import yaml

RobotConfig = dict[str, Any]


def load_robot_config(path: str | None = None) -> RobotConfig:
    if path is None:
        from ament_index_python.packages import get_package_share_directory

        path = os.path.join(get_package_share_directory("ugv_description"), "config", "robot.yaml")
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise ValueError(f"{path}: expected a mapping")
    return cfg


def footprint_polygon(cfg: RobotConfig, padding: float | None = None) -> list[tuple[float, float]]:
    """Body rectangle in base_footprint (origin = drive-axle center), CCW from front-left."""
    ch = cfg["chassis"]
    pad = float(ch.get("footprint_padding", 0.0)) if padding is None else padding
    front = float(ch["axle_from_front"]) + pad
    rear = -(float(ch["length"]) - float(ch["axle_from_front"])) - pad
    half_w = float(ch["width"]) / 2.0 + pad
    return [(front, half_w), (rear, half_w), (rear, -half_w), (front, -half_w)]


def turn_sweep_radius(cfg: RobotConfig, padding: float | None = None) -> float:
    """Radius of the circle swept by the farthest body corner in an in-place (tank) turn."""
    return max(math.hypot(x, y) for x, y in footprint_polygon(cfg, padding))


def camera_intrinsics(cfg: RobotConfig, width: int | None = None, height: int | None = None) -> tuple[float, float, float, float, int, int]:
    """(fx, fy, cx, cy, w, h) from the configured HFOV; scales with the published resolution."""
    cam = cfg["sensors"]["camera"]
    w = int(width) if width else int(cam["width"])
    h = int(height) if height else int(cam["height"])
    fx = (w / 2.0) / math.tan(math.radians(float(cam["hfov_deg"])) / 2.0)
    return fx, fx, w / 2.0, h / 2.0, w, h
