"""On-disk map storage. Pure Python, no ROS.

<maps_dir>/
  active.json                {"map": "<name>", "pose": [x, y, theta], "updated": "..."}
  <name>/map.posegraph       slam_toolbox serialized pose graph   (map_file_name = <name>/map)
  <name>/map.data
  <name>/map.pgm, map.yaml   occupancy image for Nav2 / viewing
"""
from __future__ import annotations

import json
import os
import re
import shutil
import tempfile
from dataclasses import dataclass, field
from datetime import datetime

MODES = ("mapping", "localization")
NAME_RE = re.compile(r"^[A-Za-z0-9._-]{1,64}$")


def valid_name(name: str) -> bool:
    return bool(NAME_RE.match(name)) and name not in (".", "..")


def auto_name(now: datetime | None = None) -> str:
    return (now or datetime.now()).strftime("map-%Y%m%d-%H%M%S")


@dataclass
class ActiveState:
    map: str | None = None
    pose: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    mode: str = "mapping"     # "mapping": SLAM extends the map · "localization": the map is fixed


class MapStore:
    def __init__(self, root: str) -> None:
        self.root = os.path.expanduser(root)
        os.makedirs(self.root, exist_ok=True)

    def dir(self, name: str) -> str:
        if not valid_name(name):
            raise ValueError(f"invalid map name {name!r} (letters, digits, . _ -)")
        return os.path.join(self.root, name)

    def base(self, name: str) -> str:
        """slam_toolbox filename without extension."""
        return os.path.join(self.dir(name), "map")

    def exists(self, name: str) -> bool:
        return os.path.isfile(self.base(name) + ".posegraph")

    def list(self) -> list[str]:
        names = [n for n in os.listdir(self.root) if valid_name(n) and os.path.isdir(os.path.join(self.root, n))]
        return sorted(n for n in names if self.exists(n))

    def delete(self, name: str) -> None:
        shutil.rmtree(self.dir(name))

    def copy(self, src: str, dst: str) -> None:
        if os.path.exists(self.dir(dst)):
            raise ValueError(f"map {dst!r} already exists")
        shutil.copytree(self.dir(src), self.dir(dst))

    def write_meta(self, name: str, pose: list[float]) -> None:
        """Last robot pose stored next to the map (used when the map is loaded later)."""
        path = os.path.join(self.dir(name), "meta.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"pose": [round(v, 4) for v in pose], "saved": datetime.now().isoformat(timespec="seconds")}, f)

    def read_meta_pose(self, name: str) -> list[float]:
        try:
            with open(os.path.join(self.dir(name), "meta.json"), encoding="utf-8") as f:
                pose = json.load(f).get("pose")
        except (FileNotFoundError, json.JSONDecodeError):
            return [0.0, 0.0, 0.0]
        return [float(v) for v in pose] if pose and len(pose) == 3 else [0.0, 0.0, 0.0]

    # ---------------------------------------------------------------- active state
    @property
    def _active_path(self) -> str:
        return os.path.join(self.root, "active.json")

    def read_active(self) -> ActiveState:
        try:
            with open(self._active_path, encoding="utf-8") as f:
                data = json.load(f)
        except FileNotFoundError:
            return ActiveState()
        except (json.JSONDecodeError, OSError) as exc:
            raise ValueError(f"corrupt {self._active_path}: {exc}") from exc
        name = data.get("map")
        pose = data.get("pose") or [0.0, 0.0, 0.0]
        if name is not None and not valid_name(str(name)):
            raise ValueError(f"invalid active map name {name!r}")
        if len(pose) != 3:
            raise ValueError(f"invalid pose {pose!r}")
        mode = data.get("mode", "mapping")
        if mode not in MODES:
            raise ValueError(f"invalid mode {mode!r}")
        return ActiveState(map=name, pose=[float(v) for v in pose], mode=mode)

    def write_active(self, state: ActiveState) -> None:
        """Atomic write: a power cut never leaves a half-written file."""
        payload = {"map": state.map, "pose": [round(v, 4) for v in state.pose], "mode": state.mode,
                   "updated": datetime.now().isoformat(timespec="seconds")}
        fd, tmp = tempfile.mkstemp(dir=self.root, prefix=".active-", suffix=".json")
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(payload, f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self._active_path)
