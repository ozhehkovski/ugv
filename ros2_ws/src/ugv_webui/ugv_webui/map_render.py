"""OccupancyGrid → grayscale image. Pure numpy, no ROS."""
from __future__ import annotations

from collections.abc import Sequence

import numpy as np

UNKNOWN = 205
FREE = 254
OCCUPIED = 0


def occupancy_to_image(data: Sequence[int], width: int, height: int) -> np.ndarray:
    """Row 0 of the result is the TOP of the map (max y), like any image.

    Cell values: -1 unknown, 0..100 occupancy probability (%).
    """
    if len(data) != width * height:
        raise ValueError(f"grid size mismatch: {len(data)} cells for {width}x{height}")
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    img = np.full((height, width), UNKNOWN, dtype=np.uint8)
    known = grid >= 0
    # linear shade between free (0 %) and occupied (100 %)
    img[known] = (FREE - np.clip(grid[known], 0, 100) * (FREE - OCCUPIED) / 100.0).astype(np.uint8)
    img[grid >= 65] = OCCUPIED
    img[(grid >= 0) & (grid <= 25)] = FREE
    return np.flipud(img)
