import math

import cv2
import numpy as np
import pytest

from ugv_mapping.floor_vision import Camera, FloorModel, floor_mask, ground_point, obstacle_rays, reference_patch

CAM = Camera(fx=457.0, fy=457.0, cx=320.0, cy=180.0, height=0.11)   # 640×360, hfov 70°


def wood_floor(rng: np.random.Generator) -> np.ndarray:
    """Brown floor with grain noise and a bright sun stripe, grey wall above the horizon."""
    img = np.zeros((360, 640, 3), np.uint8)
    img[:180] = (200, 200, 205)
    floor = np.clip(np.array([70, 110, 160]) + rng.normal(0, 8, (180, 640, 3)), 0, 255)
    img[180:] = floor.astype(np.uint8)
    stripe = img[250:270, 100:500].astype(np.int16) + 60            # sunlight: brighter, same colour
    img[250:270, 100:500] = np.clip(stripe, 0, 255).astype(np.uint8)
    return img


def model_for(img: np.ndarray) -> FloorModel:
    return FloorModel.fit(cv2.cvtColor(reference_patch(img), cv2.COLOR_BGR2LAB))


def test_ground_projection() -> None:
    fwd, left = ground_point(320, 180 + 457 * 0.11 / 1.0, CAM)
    assert fwd == pytest.approx(1.0) and left == pytest.approx(0.0)
    fwd, left = ground_point(320 + 45.7, 180 + 45.7 * 0.11 / 0.11, CAM)
    assert left == pytest.approx(-fwd * 0.1)
    assert ground_point(320, 175, CAM) is None


def test_clear_floor_with_sunlight_has_no_obstacles() -> None:
    img = wood_floor(np.random.default_rng(1))
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    rays = obstacle_rays(floor_mask(lab, model_for(img), 5.0), CAM, bins=40, max_range=2.0)
    assert all(r is None for _, r in rays)


def test_black_shoe_on_the_floor_is_found_at_the_right_place() -> None:
    img = wood_floor(np.random.default_rng(2))
    foot_row = int(180 + 457 * 0.11 / 0.8)                 # a shoe whose foot is 0.8 m ahead
    img[foot_row - 25:foot_row + 1, 380:440] = (25, 25, 30)    # to the right of the centre
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    rays = obstacle_rays(floor_mask(lab, model_for(img), 5.0), CAM, bins=40, max_range=2.0)
    hits = [(b, r) for b, r in rays if r is not None]
    assert hits, "shoe not detected"
    for b, r in hits:
        assert r == pytest.approx(0.8 / math.cos(b), abs=0.08)
        assert b < 0                                       # right of the camera axis
    assert len(hits) <= 6                                  # only the shoe's columns


def test_model_blend() -> None:
    a = FloorModel(np.zeros(3), np.eye(3))
    b = FloorModel(np.ones(3), 3 * np.eye(3))
    c = a.blend(b, 0.5)
    assert np.allclose(c.mean, 0.5) and np.allclose(c.inv_cov, 2 * np.eye(3))
