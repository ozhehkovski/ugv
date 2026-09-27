import math

import numpy as np
import pytest

from ugv_follow.controller import FollowConfig, follow_command
from ugv_follow.detection import (
    bbox_bearings,
    cluster_points,
    floor_range,
    leg_candidates,
    letterbox,
    lidar_range_in_sector,
    postprocess_yolo,
)
from ugv_follow.tracker import Candidate, Tracker

CFG = FollowConfig()


# ---------------------------------------------------------------- controller
def test_keeps_40cm_gap_and_never_reverses() -> None:
    at_gap = CFG.nose + CFG.gap
    assert follow_command(CFG, at_gap, 0.0) == (0.0, 0.0)
    assert follow_command(CFG, at_gap - 0.2, 0.0)[0] == 0.0          # too close: stop, no reverse
    v, w = follow_command(CFG, at_gap + 1.0, 0.0)
    assert v == pytest.approx(CFG.v_max) and w == 0.0
    v, _ = follow_command(CFG, at_gap + 0.2, 0.0)
    assert 0.0 < v < CFG.v_max


def test_person_far_to_the_side_turns_in_place() -> None:
    v, w = follow_command(CFG, 0.5, 1.5)
    assert v == 0.0 and w > 0.0
    v, w = follow_command(CFG, 0.5, -1.5)
    assert v == 0.0 and w < 0.0


def test_speed_caps() -> None:
    v, w = follow_command(CFG, 5.0, 2.0)
    assert v <= CFG.v_max and abs(w) <= CFG.w_max


# ---------------------------------------------------------------- detection helpers
def test_letterbox_and_postprocess_roundtrip() -> None:
    img = np.zeros((360, 640, 3), dtype=np.uint8)
    lb, scale, pad = letterbox(img, 640)
    assert lb.shape == (640, 640, 3) and scale == 1.0 and pad == (0, 140)
    pred = np.zeros((84, 3), dtype=np.float32)
    pred[:4, 0] = [320, 140 + 180, 100, 200]       # a person box centred in the image
    pred[4, 0] = 0.9
    pred[:4, 1] = [100, 300, 50, 50]; pred[4 + 2, 1] = 0.95    # a car: filtered by class
    pred[:4, 2] = [322, 322, 100, 200]; pred[4, 2] = 0.5       # duplicate person: suppressed by NMS
    out = postprocess_yolo(pred, scale, pad, (360, 640), conf=0.4, iou=0.5, classes=(0,))
    assert len(out) == 1
    x1, y1, x2, y2, s, c = out[0]
    assert (x1, y1, x2, y2) == pytest.approx((270, 80, 370, 280)) and c == 0 and s == pytest.approx(0.9)


def test_bearings_and_lidar_sector() -> None:
    c, left, right = bbox_bearings(300, 340, fx=450.0, cx=320.0)
    assert c == pytest.approx(0.0) and left > 0 > right
    pts = np.array([[1.5, 0.02], [1.52, -0.03], [3.0, 0.0], [1.0, 1.0]])
    x, y, r = lidar_range_in_sector(pts, (0.0, 0.0), left=0.1, right=-0.1)
    assert x == pytest.approx(1.51, abs=0.02) and r < 2.0
    assert lidar_range_in_sector(pts, (0.0, 0.0), left=-0.5, right=-0.6) is None


def test_leg_clusters_pair_into_one_person_and_walls_are_ignored() -> None:
    leg1 = np.column_stack((np.full(4, 1.5), np.linspace(0.05, 0.12, 4)))
    leg2 = np.column_stack((np.full(4, 1.5), np.linspace(-0.12, -0.05, 4)))
    wall = np.column_stack((np.full(40, 3.0), np.linspace(-1.0, 1.0, 40)))
    assert len(cluster_points(np.vstack((leg1, wall)))) == 2
    people = leg_candidates(np.vstack((leg1, leg2, wall)))
    assert len(people) == 1 and people[0] == pytest.approx((1.5, 0.0), abs=0.05)


def test_floor_range() -> None:
    assert floor_range(v_bottom=180 + 45, fy=450, cy=180, cam_height=0.11) == pytest.approx(1.1)
    assert floor_range(v_bottom=170, fy=450, cy=180, cam_height=0.11) is None


# ---------------------------------------------------------------- tracker
def cam(x: float, y: float, hist=None) -> Candidate:
    return Candidate(x, y, True, hist)


def test_lock_picks_nearest_person_in_front() -> None:
    tr = Tracker()
    assert not tr.try_lock([cam(-1.0, 0.0), cam(0.5, 2.0)], (0.0, 0.0, 0.0), 0.0)   # behind / to the side
    assert tr.try_lock([cam(2.5, 0.2), cam(1.2, -0.3), Candidate(0.8, 0.0, False)], (0.0, 0.0, 0.0), 0.0)
    assert (tr.target.x, tr.target.y) == (1.2, -0.3)


def test_follows_a_walking_person_and_ignores_a_bystander() -> None:
    tr = Tracker()
    tr.try_lock([cam(1.0, 0.0)], (0.0, 0.0, 0.0), 0.0)
    t = 0.0
    for k in range(1, 30):                          # walks along +x at 0.8 m/s; a bystander stands at (2.5, 0.9)
        t = k * 0.1
        used = tr.update([cam(1.0 + 0.8 * t, 0.0), cam(2.5, 0.9)], t)
        assert used is not None and used.y == 0.0
    assert tr.target.x == pytest.approx(1.0 + 0.8 * t, abs=0.15)
    assert tr.target.vx == pytest.approx(0.8, abs=0.25)


def test_lidar_legs_keep_the_track_when_the_camera_drops() -> None:
    tr = Tracker()
    tr.try_lock([cam(1.0, 0.0)], (0.0, 0.0, 0.0), 0.0)
    for k in range(1, 20):
        assert tr.update([Candidate(1.0, 0.02 * k / 10, False)], k * 0.1) is not None
    assert not tr.lost(2.0)


def test_lost_then_reacquired_only_by_similar_clothes_nearby() -> None:
    red, blue = np.array([1.0, 0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])
    tr = Tracker()
    tr.try_lock([cam(1.0, 0.0, red)], (0.0, 0.0, 0.0), 0.0)
    assert tr.update([], 1.5) is None and tr.lost(1.5)
    assert tr.update([cam(1.3, 0.2, blue)], 1.6) is None          # different clothes: not our person
    assert tr.update([cam(4.0, 0.0, red)], 1.7) is None           # too far from where we lost them
    assert tr.update([cam(1.4, 0.1, red)], 1.8) is not None and not tr.lost(1.8)
