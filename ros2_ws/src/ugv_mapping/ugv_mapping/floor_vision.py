"""Low obstacles from the front camera: where does the floor end? Pure numpy/OpenCV, no ROS.

The lidar sits 19.5 cm high and misses shoes, bags, cables. The camera (11 cm high, level) sees the
floor below the horizon. A colour model of the floor (Lab; brightness weighted down so shadows and sun
stripes stay "floor") is learnt from a reference patch right in front of the nose. Scanning each image
column upward from the bottom, the first run of non-floor pixels is the foot of an obstacle; projecting
that pixel onto the floor plane gives its position.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

L_WEIGHT = 0.2           # brightness counts 20 %: shadows / sunlight change L, not the colour of the floor


@dataclass
class FloorModel:
    mean: np.ndarray       # (3,) Lab
    inv_cov: np.ndarray    # (3, 3)

    @classmethod
    def fit(cls, lab_pixels: np.ndarray, min_std: float = 4.0) -> FloorModel:
        px = lab_pixels.reshape(-1, 3).astype(np.float64) * np.array([L_WEIGHT, 1.0, 1.0])
        mean = px.mean(axis=0)
        cov = np.cov(px, rowvar=False) + np.eye(3) * min_std ** 2
        return cls(mean, np.linalg.inv(cov))

    def distance(self, lab: np.ndarray) -> np.ndarray:
        d = lab.reshape(-1, 3).astype(np.float64) * np.array([L_WEIGHT, 1.0, 1.0]) - self.mean
        return np.sqrt(np.einsum("ij,jk,ik->i", d, self.inv_cov, d)).reshape(lab.shape[:2])

    def blend(self, other: FloorModel, rate: float) -> FloorModel:
        return FloorModel((1 - rate) * self.mean + rate * other.mean, (1 - rate) * self.inv_cov + rate * other.inv_cov)


@dataclass(frozen=True)
class Camera:
    fx: float
    fy: float
    cx: float
    cy: float
    height: float          # above the floor, m


def reference_patch(img: np.ndarray) -> np.ndarray:
    """Bottom-centre patch: the floor just in front of the nose (~0.3 m)."""
    h, w = img.shape[:2]
    return img[int(0.86 * h):h, int(0.38 * w):int(0.62 * w)]


def floor_mask(lab: np.ndarray, model: FloorModel, threshold: float) -> np.ndarray:
    mask = (model.distance(lab) < threshold).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))     # speckles of texture
    return mask.astype(bool)


def floor_boundary(mask: np.ndarray, horizon: int, bins: int, min_run: int = 4, min_fill: float = 0.5) -> list[int | None]:
    """For each column bin: the image row where the floor ends (foot of an obstacle), None if the floor
    reaches the horizon. A bin row counts as non-floor when < min_fill of its pixels are floor; the obstacle
    must be at least min_run rows tall, so a thin line on the floor is not an obstacle."""
    h, w = mask.shape
    edges = np.linspace(0, w, bins + 1).astype(int)
    out: list[int | None] = []
    for b in range(bins):
        col = mask[:, edges[b]:edges[b + 1]].mean(axis=1) >= min_fill
        run, foot = 0, None
        for r in range(h - 1, horizon, -1):
            if col[r]:
                run = 0
                continue
            run += 1
            if run >= min_run:
                foot = r + run - 1          # the lowest row of the obstacle run
                break
        out.append(foot)
    return out


def ground_point(u: float, v: float, cam: Camera) -> tuple[float, float] | None:
    """Pixel on the floor → (forward, left) in metres from the camera, level camera."""
    dv = v - cam.cy
    if dv <= 1.0:
        return None
    fwd = cam.height * cam.fy / dv
    return fwd, -(u - cam.cx) * fwd / cam.fx


def obstacle_rays(mask: np.ndarray, cam: Camera, bins: int, max_range: float) -> list[tuple[float, float | None]]:
    """Per column bin: (bearing, range to the obstacle foot or None when free up to max_range)."""
    h, w = mask.shape
    horizon = int(cam.cy + cam.height * cam.fy / max_range)       # row of the floor at max_range
    feet = floor_boundary(mask, min(horizon, h - 2), bins)
    edges = np.linspace(0, w, bins + 1)
    rays: list[tuple[float, float | None]] = []
    for b, foot in enumerate(feet):
        u = (edges[b] + edges[b + 1]) / 2.0
        bearing = math.atan2(cam.cx - u, cam.fx)
        rng = None
        if foot is not None:
            p = ground_point(u, foot + 0.5, cam)
            if p is not None and math.hypot(*p) <= max_range:
                rng = math.hypot(*p)
        rays.append((bearing, rng))
    return rays
