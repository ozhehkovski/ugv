import math

import numpy as np

from ugv_mapping.accessibility import BLOCKED, PASSABLE, TURNABLE, UNKNOWN, GridInfo
from ugv_mapping.frontiers import Frontier, choose_frontier, find_frontiers

RES = 0.05
INFO = GridInfo(width=100, height=60, resolution=RES, origin_x=0.0, origin_y=0.0)   # 5 m × 3 m


def corridor_layer() -> np.ndarray:
    """Explored left part (x < 3 m) of a corridor, unexplored right part."""
    L = np.full((INFO.height, INFO.width), UNKNOWN, dtype=np.int8)
    L[:, :60] = BLOCKED
    L[10:50, 5:60] = PASSABLE
    L[20:40, 15:50] = TURNABLE
    return L


def test_frontier_at_the_explored_edge_looks_into_unknown() -> None:
    fs = find_frontiers(corridor_layer(), INFO, robot_xy=(1.0, 1.5))
    assert len(fs) == 1
    f = fs[0]
    assert 2.2 <= f.x <= 3.0 and 0.5 <= f.y <= 2.5        # near the boundary x = 3 m, in turn-room
    assert abs(f.yaw) < math.radians(30)                  # facing +x, into the unknown
    assert f.distance > 1.2


def test_fully_explored_room_has_no_frontiers() -> None:
    L = np.full((INFO.height, INFO.width), BLOCKED, dtype=np.int8)
    L[10:50, 10:90] = TURNABLE
    assert find_frontiers(L, INFO, robot_xy=(2.5, 1.5)) == []


def test_tiny_frontiers_are_ignored() -> None:
    L = np.full((INFO.height, INFO.width), BLOCKED, dtype=np.int8)
    L[10:50, 10:90] = TURNABLE
    L[30, 91] = UNKNOWN                                   # a single speckle next to the room
    assert find_frontiers(L, INFO, robot_xy=(2.5, 1.5), min_size_m=0.3) == []


def test_choose_prefers_close_and_skips_blacklist_and_own_spot() -> None:
    near = Frontier(1.0, 0.0, 0.0, size=20, distance=1.0)
    far = Frontier(4.0, 0.0, 0.0, size=20, distance=4.0)
    here = Frontier(0.1, 0.0, 0.0, size=50, distance=0.1)
    assert choose_frontier([far, near, here], blacklist=[]) is near
    assert choose_frontier([far, near], blacklist=[(1.1, 0.1)]) is far
    assert choose_frontier([here], blacklist=[]) is None


def test_dead_end_corridor_without_turn_room_is_skipped() -> None:
    L = np.full((INFO.height, INFO.width), UNKNOWN, dtype=np.int8)
    L[:, :80] = BLOCKED
    L[20:40, 5:30] = TURNABLE                  # a room
    L[27:33, 30:80] = PASSABLE                 # a long narrow corridor leading into the unknown
    fs = find_frontiers(L, INFO, robot_xy=(0.8, 1.5))
    assert all(f.x < 2.2 for f in fs)          # no goal deep in the corridor
    assert not any(f.x > 3.0 for f in fs)


def test_goal_cell_is_turnable() -> None:
    L = corridor_layer()
    f = find_frontiers(L, INFO, robot_xy=(1.0, 1.5))[0]
    assert L[INFO.cell(f.x, f.y)] == TURNABLE
