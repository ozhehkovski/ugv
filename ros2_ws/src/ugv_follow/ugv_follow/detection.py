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
    rects = [[float(a), float(b), float(c - a), float(d - b)] for a, b, c, d in zip(x1, y1, x2, y2)]
    idx = cv2.dnn.NMSBoxes(rects, score.astype(float).tolist(), conf, iou)
    idx = np.asarray(idx).reshape(-1)
    return [(float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i]), float(score[i]), int(cls[i])) for i in idx]


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
