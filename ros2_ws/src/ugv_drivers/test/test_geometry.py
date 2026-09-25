import math
import os

import pytest

from ugv_drivers.lidar_scan import bin_scan
from ugv_drivers.robot_config import camera_intrinsics, footprint_polygon, load_robot_config, turn_sweep_radius

CONFIG = os.path.join(os.path.dirname(__file__), "..", "..", "ugv_description", "config", "robot.yaml")


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_robot_config(CONFIG)


def test_footprint_matches_measured_chassis(cfg: dict) -> None:
    poly = footprint_polygon(cfg, padding=0.0)
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    assert max(xs) == pytest.approx(0.555)          # nose, axle 55.5 cm behind it
    assert min(xs) == pytest.approx(-0.065)         # tail, 6.5 cm behind the axle
    assert max(xs) - min(xs) == pytest.approx(0.62)
    assert max(ys) - min(ys) == pytest.approx(0.56)


def test_turn_sweep_radius(cfg: dict) -> None:
    assert turn_sweep_radius(cfg, padding=0.0) == pytest.approx(math.hypot(0.555, 0.28))


def test_lidar_is_12cm_behind_nose(cfg: dict) -> None:
    assert cfg["chassis"]["axle_from_front"] - cfg["sensors"]["lidar"]["xyz"][0] == pytest.approx(0.12)


def test_hard_speed_limit_is_2kmh(cfg: dict) -> None:
    assert cfg["limits"]["max_linear"] <= 2.0 / 3.6
    assert cfg["limits"]["max_wheel_speed"] <= 2.0 / 3.6


def test_camera_intrinsics_scale(cfg: dict) -> None:
    fx, fy, cx, cy, w, h = camera_intrinsics(cfg, 640, 360)
    assert (cx, cy, w, h) == (320.0, 180.0, 640, 360)
    assert fx == fy == pytest.approx(320 / math.tan(math.radians(35)))


def test_bin_scan_invert_filter_and_nearest() -> None:
    meas = [(15, 10.0, 1000.0), (15, 10.2, 800.0), (0, 20.0, 500.0), (15, 30.0, 50.0), (15, 40.0, 20000.0)]
    r = bin_scan(meas, bins=360, range_min=0.15, range_max=12.0, invert=False)
    assert r[10] == pytest.approx(0.8)                 # nearer point wins
    assert math.isinf(r[20]) and math.isinf(r[30]) and math.isinf(r[40])
    r_inv = bin_scan([(15, 10.0, 1000.0)], bins=360, range_min=0.15, range_max=12.0, invert=True)
    assert r_inv[350] == pytest.approx(1.0)
