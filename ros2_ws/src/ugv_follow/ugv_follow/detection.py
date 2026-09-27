"""Pure pre/post-processing for YOLO detections and camera+lidar fusion. numpy + OpenCV, no ROS."""
from __future__ import annotations

import math

import cv2
import numpy as np


def letterbox(img: np.ndarray, size: int) -> tuple[np.ndarray, float, tuple[int, int]]:
    """Resize keeping aspect ratio and pad to size×size (gray 114). Returns (image, scale, (pad_x, pad_y))."""
    h, w = img.shape[:2]
    scale = min(size / h, size / w)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    px, py = (size - nw) // 2, (size - nh) // 2
    out = np.full((size, size, 3), 114, dtype=np.uint8)
    out[py:py + nh, px:px + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    return out, scale, (px, py)


def postprocess_yolo(pred: np.ndarray, scale: float, pad: tuple[int, int], shape: tuple[int, int],
                     conf: float, iou: float, classes: tuple[int, ...]) -> list[tuple[float, float, float, float, float, int]]:
    """YOLOv8/11 raw head (4 + n_classes, n_anchors) → [(x1, y1, x2, y2, score, cls)] in original pixels."""
    scores_all = pred[4:]
    cls = np.argmax(scores_all, axis=0)
    score = scores_all[cls, np.arange(pred.shape[1])]
    keep = (score >= conf) & np.isin(cls, classes)
    if not keep.any():
        return []
    cx, cy, w, h = pred[0, keep], pred[1, keep], pred[2, keep], pred[3, keep]
    score, cls = score[keep], cls[keep]
    px, py = pad
    x1 = ((cx - w / 2) - px) / scale
    y1 = ((cy - h / 2) - py) / scale
    x2 = ((cx + w / 2) - px) / scale
    y2 = ((cy + h / 2) - py) / scale
    H, W = shape
    x1, x2 = np.clip(x1, 0, W - 1), np.clip(x2, 0, W - 1)
    y1, y2 = np.clip(y1, 0, H - 1), np.clip(y2, 0, H - 1)
    idx = nms(np.column_stack((x1, y1, x2, y2)), score, iou)
    return [(float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i]), float(score[i]), int(cls[i])) for i in idx]


def nms(boxes: np.ndarray, scores: np.ndarray, iou: float) -> list[int]:
    """Greedy non-maximum suppression (boxes N×4 as x1, y1, x2, y2)."""
    order = list(np.argsort(-scores))
    area = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    keep: list[int] = []
    while order:
        i = order.pop(0)
        keep.append(int(i))
        rest = np.asarray(order, dtype=int)
        if len(rest) == 0:
            break
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        overlap = inter / np.maximum(area[i] + area[rest] - inter, 1e-9)
        order = [int(j) for j, o in zip(rest, overlap) if o <= iou]
    return keep


def bbox_bearings(x1: float, x2: float, fx: float, cx: float) -> tuple[float, float, float]:
    """(centre, left edge, right edge) bearings in rad, + = left (REP-103), for a pinhole camera."""
    return (math.atan2(cx - (x1 + x2) / 2.0, fx), math.atan2(cx - x1, fx), math.atan2(cx - x2, fx))


def lidar_range_in_sector(points: np.ndarray, origin: tuple[float, float], left: float, right: float,
                          max_range: float = 6.0, min_points: int = 2) -> tuple[float, float, float] | None:
    """Nearest cluster of lidar points (base frame, N×2) inside the bearing sector [right, left] seen from
    `origin` (the camera). Returns (x, y, range) of the cluster's median point, or None.

    Legs are the closest thing inside a person's bbox sector; a cluster is the points within 0.3 m of the
    nearest point, so a single noisy return does not decide the range.
    """
    if len(points) == 0:
        return None
    rel = points - np.asarray(origin)
    ang = np.arctan2(rel[:, 1], rel[:, 0])
    rng = np.hypot(rel[:, 0], rel[:, 1])
    sel = (ang <= left) & (ang >= right) & (rel[:, 0] > 0.05) & (rng < max_range)
    if sel.sum() < min_points:
        return None
    p, r = points[sel], rng[sel]
    near = r <= r.min() + 0.3
    if near.sum() < min_points:
        return None
    x, y = float(np.median(p[near, 0])), float(np.median(p[near, 1]))
    return x, y, float(np.median(r[near]))


def cluster_points(points: np.ndarray, gap: float = 0.1) -> list[np.ndarray]:
    """Euclidean clusters: points chained by neighbours closer than `gap`."""
    n = len(points)
    if n == 0:
        return []
    near = np.hypot(points[:, None, 0] - points[None, :, 0], points[:, None, 1] - points[None, :, 1]) <= gap
    label = np.full(n, -1)
    k = 0
    for i in range(n):                      # flood fill over the neighbour graph (≤ 360 points)
        if label[i] >= 0:
            continue
        stack, label[i] = [i], k
        while stack:
            j = stack.pop()
            for m in np.nonzero(near[j] & (label < 0))[0]:
                label[m] = k
                stack.append(m)
        k += 1
    return [points[label == c] for c in range(k)]


def leg_candidates(points: np.ndarray, max_range: float = 3.5, leg_max_width: float = 0.25,
                   pair_dist: float = 0.45) -> list[tuple[float, float]]:
    """Person-like positions from the scan: small clusters (legs), pairs within `pair_dist` merged."""
    legs = []
    for c in cluster_points(points):
        if len(c) < 2:
            continue
        width = np.hypot(*(c.max(axis=0) - c.min(axis=0)))
        centre = c.mean(axis=0)
        if width <= leg_max_width and np.hypot(*centre) <= max_range:
            legs.append(centre)
    people: list[tuple[float, float]] = []
    used = [False] * len(legs)
    for i, a in enumerate(legs):
        if used[i]:
            continue
        group = [a]
        for j in range(i + 1, len(legs)):
            if not used[j] and np.hypot(*(legs[j] - a)) <= pair_dist:
                group.append(legs[j])
                used[j] = True
        people.append(tuple(float(v) for v in np.mean(group, axis=0)))
    return people


def floor_range(v_bottom: float, fy: float, cy: float, cam_height: float) -> float | None:
    """Range to a person's feet from the bbox bottom row, camera level and cam_height above the floor."""
    dv = v_bottom - cy
    if dv < 3.0:          # feet at/above the horizon: not usable
        return None
    return cam_height * fy / dv


def appearance(bgr: np.ndarray, bbox: tuple[float, float, float, float]) -> np.ndarray | None:
    """Normalized HSV colour histogram of the middle of the bbox (clothes), for re-identification."""
    x1, y1, x2, y2 = (int(v) for v in bbox)
    h = y2 - y1
    roi = bgr[max(0, y1 + h // 5):max(0, y1 + 3 * h // 5), max(0, x1):max(0, x2)]
    if roi.size == 0 or roi.shape[0] < 4 or roi.shape[1] < 4:
        return None
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256])
    return cv2.normalize(hist, hist).flatten()


def similarity(a: np.ndarray | None, b: np.ndarray | None) -> float:
    if a is None or b is None:
        return 0.0
    return float(cv2.compareHist(a.astype(np.float32), b.astype(np.float32), cv2.HISTCMP_CORREL))
