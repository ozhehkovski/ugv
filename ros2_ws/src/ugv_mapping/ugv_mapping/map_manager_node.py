#!/usr/bin/env python3
"""Map manager: owns the slam_toolbox process and makes the map survive restarts.

* at start: continue the active map from ~/ugv_maps (slam_toolbox map_file_name + map_start_pose =
  the last robot pose), or start a new auto-named map;
* the robot pose is stored every few seconds, the pose graph is autosaved periodically and on stop;
* ~/command (ugv_interfaces/MapCommand): list | save | save_as | load | new | set_pose | delete.

slam_toolbox 2.6 has no reset service and deserializing into a running session is fragile, so every
map switch restarts the slam_toolbox process with the right parameters (≈3 s without map→odom).
The child runs in its own session: on SIGINT/SIGTERM this node saves first, then stops it.
"""
from __future__ import annotations

import math
import os
import signal
import subprocess
import threading
import time
from typing import Any

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from ugv_interfaces.srv import MapCommand

from .map_store import ActiveState, MapStore, auto_name

CALL_TIMEOUT = 20.0


def _yaw(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MapManager(Node):
    def __init__(self) -> None:
        super().__init__("map_manager")
        self.declare_parameter("maps_dir", "~/ugv_maps")
        self.declare_parameter("slam_params", "")      # slam_toolbox yaml, passed by robot.launch.py
        self.declare_parameter("autosave_period", 60.0)
        self.declare_parameter("pose_period", 2.0)
        self.declare_parameter("base_frame", "base_footprint")
        gp = self.get_parameter
        self.store = MapStore(str(gp("maps_dir").value))
        self.slam_params = str(gp("slam_params").value)
        if not os.path.isfile(self.slam_params):
            raise RuntimeError(f"slam_params file not found: {self.slam_params!r}")
        self.base_frame = str(gp("base_frame").value)

        self.lock = threading.RLock()          # serializes map operations
        self.proc: subprocess.Popen | None = None
        self.slam_started = 0.0
        try:
            self.state = self.store.read_active()
        except ValueError as exc:
            self.get_logger().error(f"{exc} — starting a new map")
            self.state = ActiveState()

        cb = ReentrantCallbackGroup()
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.serialize_cli = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map", callback_group=cb)
        self.save_img_cli = self.create_client(SaveMap, "/slam_toolbox/save_map", callback_group=cb)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "~/status", latched)
        self.create_timer(5.0, self._publish_status, callback_group=cb)
        self.create_service(MapCommand, "~/command", self._on_command, callback_group=cb)
        self.create_timer(float(gp("pose_period").value), self._pose_tick, callback_group=cb)
        self.create_timer(float(gp("autosave_period").value), self._autosave_tick, callback_group=cb)

        if self.state.map and self.store.exists(self.state.map):
            self._start_slam(self.state.map, self.state.pose)
        else:
            if self.state.map:
                self.get_logger().warn(f"active map {self.state.map!r} has no saved pose graph yet — new map")
            self._start_new()

    # ---------------------------------------------------------------- slam process
    def _start_slam(self, load: str | None, pose: list[float] | None) -> None:
        cmd = ["ros2", "run", "slam_toolbox", "async_slam_toolbox_node", "--ros-args",
               "-r", "__node:=slam_toolbox", "--params-file", self.slam_params]
        if load:
            x, y, th = (float(v) for v in (pose or [0.0, 0.0, 0.0]))
            cmd += ["-p", f"map_file_name:={self.store.base(load)}",
                    "-p", f"map_start_pose:=[{x:.4f}, {y:.4f}, {th:.4f}]"]
            self.get_logger().info(f"SLAM: continuing map {load!r} at ({x:.2f}, {y:.2f}, {math.degrees(th):.0f}°)")
        else:
            self.get_logger().info(f"SLAM: new map {self.state.map!r}")
        self.proc = subprocess.Popen(cmd, start_new_session=True)
        self.slam_started = time.monotonic()
        self._publish_status()

    def _stop_slam(self) -> None:
        proc, self.proc = self.proc, None
        if proc is None or proc.poll() is not None:
            return
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warn("slam_toolbox did not stop on SIGINT — killing")
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5.0)

    def _start_new(self) -> None:
        self.state = ActiveState(map=auto_name(), pose=[0.0, 0.0, 0.0])
        self.store.write_active(self.state)
        self._start_slam(None, None)

    # ---------------------------------------------------------------- saving
    def _call(self, client: Any, request: Any) -> Any:
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"{client.srv_name} not available")
        fut = client.call_async(request)
        deadline = time.monotonic() + CALL_TIMEOUT
        while not fut.done():
            if time.monotonic() > deadline:
                raise RuntimeError(f"{client.srv_name} timed out")
            time.sleep(0.02)
        return fut.result()

    def _save(self, name: str) -> None:
        """Serialize the running pose graph (+ PGM image) into <maps>/<name>/map.*"""
        if self.proc is None or self.proc.poll() is not None:
            raise RuntimeError("slam_toolbox is not running")
        os.makedirs(self.store.dir(name), exist_ok=True)
        base = self.store.base(name)
        res = self._call(self.serialize_cli, SerializePoseGraph.Request(filename=base))
        if res.result != 0:
            raise RuntimeError(f"serialize_map failed (code {res.result}) — no scans yet?")
        img = self._call(self.save_img_cli, SaveMap.Request(name=String(data=base)))
        if img.result != 0:
            self.get_logger().warn(f"save_map image failed (code {img.result})")
        self.store.write_meta(name, self.state.pose)
        self.get_logger().info(f"map {name!r} saved")

    def _autosave_tick(self) -> None:
        if time.monotonic() - self.slam_started < 20.0 or not self.state.map:
            return
        with self.lock:
            try:
                self._save(self.state.map)
            except RuntimeError as exc:
                self.get_logger().warn(f"autosave: {exc}", throttle_duration_sec=60.0)

    def _pose_tick(self) -> None:
        # after a SLAM restart the TF buffer still holds the old map→odom: wait for the new one
        if time.monotonic() - self.slam_started < 8.0:
            return
        if not self.lock.acquire(blocking=False):      # a map operation is running
            return
        try:
            try:
                t = self.tf.lookup_transform("map", self.base_frame, Time())
            except TransformException:
                return
            pose = [t.transform.translation.x, t.transform.translation.y, _yaw(t.transform.rotation)]
            old = self.state.pose
            if math.hypot(pose[0] - old[0], pose[1] - old[1]) > 0.01 or abs(pose[2] - old[2]) > 0.01:
                self.state.pose = pose
                self.store.write_active(self.state)
        finally:
            self.lock.release()

    # ---------------------------------------------------------------- commands
    def _on_command(self, req: MapCommand.Request, res: MapCommand.Response) -> MapCommand.Response:
        cmd, name = req.command.strip(), req.name.strip()
        try:
            with self.lock:
                res.message = self._dispatch(cmd, name, list(req.pose))
            res.success = True
        except (RuntimeError, ValueError, OSError) as exc:
            res.success, res.message = False, str(exc)
            self.get_logger().warn(f"command {cmd!r} failed: {exc}")
        res.maps = self.store.list()
        res.active = self.state.map or ""
        self._publish_status()
        return res

    def _dispatch(self, cmd: str, name: str, pose: list[float]) -> str:
        if pose and len(pose) != 3:
            raise ValueError("pose must be [x, y, theta]")
        if cmd == "list":
            return "ok"
        if cmd == "save":
            self._save(self.state.map)
            return f"saved {self.state.map}"
        if cmd == "save_as":
            if name in self.store.list():
                raise ValueError(f"map {name!r} already exists")
            self._save(name)
            self.state.map = name
            self.store.write_active(self.state)
            return f"saved as {name}"
        if cmd == "load":
            if not self.store.exists(name):
                raise ValueError(f"no map {name!r}")
            self._save_current_quietly()
            start = pose or self.store.read_meta_pose(name)
            self._stop_slam()
            self.state = ActiveState(map=name, pose=start)
            self.store.write_active(self.state)
            self._start_slam(name, start)
            return f"loaded {name}"
        if cmd == "new":
            self._save_current_quietly()
            self._stop_slam()
            self._start_new()
            return f"new map {self.state.map}"
        if cmd == "set_pose":
            if not pose:
                raise ValueError("set_pose needs [x, y, theta]")
            self._save(self.state.map)          # must exist to be reloaded at the new pose
            self._stop_slam()
            self.state.pose = pose
            self.store.write_active(self.state)
            self._start_slam(self.state.map, pose)
            return "pose set"
        if cmd == "delete":
            if name == self.state.map:
                raise ValueError("cannot delete the active map")
            if not self.store.exists(name):
                raise ValueError(f"no map {name!r}")
            self.store.delete(name)
            return f"deleted {name}"
        raise ValueError(f"unknown command {cmd!r}")

    def _save_current_quietly(self) -> None:
        if not self.state.map:
            return
        try:
            self._save(self.state.map)
        except RuntimeError as exc:
            self.get_logger().warn(f"could not save {self.state.map!r} before switching: {exc}")

    def _publish_status(self) -> None:
        running = self.proc is not None and self.proc.poll() is None
        self.status_pub.publish(String(data=f"{self.state.map or ''}|{'running' if running else 'stopped'}"))

    # ---------------------------------------------------------------- shutdown
    def shutdown(self) -> None:
        with self.lock:
            self._save_current_quietly()
            self.store.write_active(self.state)
            self._stop_slam()


def main() -> None:
    # own signal handling: the final save needs a live ROS context
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    node = MapManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    stop.wait()
    node.get_logger().info("stopping: saving the map")
    node.shutdown()
    executor.shutdown(timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
