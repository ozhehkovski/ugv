#!/usr/bin/env python3
"""Autonomous frontier exploration on the accessible-terrain layer, driving through Nav2.

~/enable (std_srvs/SetBool): start / stop.   ~/status (String): what it is doing.
Loop: pick the best frontier (large, close, not blacklisted) → NavigateToPose → repeat.
Failed or timed-out goals are blacklisted. No frontiers left → the map is saved and the robot
returns to where exploration started. A goal canceled by anyone else (STOP button, manual
driving) stops exploration: it never resumes on its own.
"""
from __future__ import annotations

import math
import time
from typing import Any

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from ugv_interfaces.srv import MapCommand

from .accessibility import GridInfo
from .frontiers import choose_frontier, find_frontiers

STUCK_DIST = 0.10     # m: moving less than this ...
STUCK_TIME = 30.0     # s: ... for this long means the robot is stuck on this goal
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _yaw(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Explorer(Node):
    def __init__(self) -> None:
        super().__init__("explorer")
        self.declare_parameter("goal_timeout", 90.0)
        self.declare_parameter("return_home", True)
        self.declare_parameter("max_failures", 5)
        self.goal_timeout = float(self.get_parameter("goal_timeout").value)
        self.return_home = bool(self.get_parameter("return_home").value)
        self.max_failures = int(self.get_parameter("max_failures").value)

        self.layer: OccupancyGrid | None = None
        self.enabled = False
        self.phase = "idle"                    # idle | explore | home
        self.goal_handle: Any = None
        self.goal_xy: tuple[float, float] | None = None
        self.goal_t = 0.0
        self.pending = False                   # goal sent, acceptance not yet known
        self.self_cancel = False
        self.blacklist: list[tuple[float, float]] = []
        self.failures = 0
        self.home: tuple[float, float, float] | None = None
        self.progress_xy: tuple[float, float] | None = None   # stuck detection
        self.progress_t = 0.0
        self.visited = 0

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.map_cli = self.create_client(MapCommand, "map_manager/command")
        self.status_pub = self.create_publisher(String, "~/status", LATCHED)
        self.create_subscription(OccupancyGrid, "map_accessible", self._on_layer, LATCHED)
        self.create_service(SetBool, "~/enable", self._on_enable)
        self.create_timer(1.0, self._tick)
        self._status("idle")

    # ---------------------------------------------------------------- inputs
    def _on_layer(self, msg: OccupancyGrid) -> None:
        self.layer = msg

    def _pose(self) -> tuple[float, float, float] | None:
        try:
            t = self.tf.lookup_transform("map", "base_footprint", Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, _yaw(t.transform.rotation)

    def _on_enable(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        if req.data and not self.enabled:
            self.home = self._pose()
            if self.home is None:
                res.success, res.message = False, "no robot pose on the map yet"
                return res
            self.enabled, self.phase = True, "explore"
            self.blacklist, self.failures, self.visited = [], 0, 0
            self._status("exploring: looking for frontiers")
            self.get_logger().info(f"exploration started at {tuple(round(v, 2) for v in self.home)}")
        elif not req.data and self.enabled:
            self._stop("stopped by operator")
        res.success, res.message = True, self.phase
        return res

    # ---------------------------------------------------------------- loop
    def _tick(self) -> None:
        if not self.enabled or self.pending:
            return
        if self.goal_handle is not None:
            now = time.monotonic()
            pose = self._pose()
            if pose is not None and (self.progress_xy is None or math.dist(pose[:2], self.progress_xy) > STUCK_DIST):
                self.progress_xy, self.progress_t = pose[:2], now
            stuck = now - self.progress_t > STUCK_TIME
            if stuck or now - self.goal_t > self.goal_timeout:
                self.get_logger().warn(f"goal {self.goal_xy} {'stuck' if stuck else 'timed out'} — blacklisted")
                self._blacklist_goal()
                self._cancel_own()
            return
        if self.phase == "home":
            return
        pose = self._pose()
        if pose is None or self.layer is None:
            return
        m = self.layer
        info = GridInfo(m.info.width, m.info.height, m.info.resolution,
                        m.info.origin.position.x, m.info.origin.position.y)
        layer = np.asarray(m.data, dtype=np.int8).reshape(info.height, info.width)
        frontiers = find_frontiers(layer, info, (pose[0], pose[1]))
        target = choose_frontier(frontiers, self.blacklist)
        if target is None:
            self._finish(len(frontiers))
            return
        self._send(target.x, target.y, target.yaw)
        self._status(f"exploring: goal {self.visited + 1} at ({target.x:.1f}, {target.y:.1f}), "
                     f"{len(frontiers)} frontier(s)")

    def _send(self, x: float, y: float, yaw: float) -> None:
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self._stop("Nav2 not available")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.goal_xy, self.goal_t, self.pending, self.self_cancel = (x, y), time.monotonic(), True, False
        self.progress_xy, self.progress_t = None, time.monotonic()
        self.nav.send_goal_async(goal).add_done_callback(self._on_accepted)

    def _on_accepted(self, fut: Any) -> None:
        self.pending = False
        handle = fut.result()
        if not handle.accepted:
            self._blacklist_goal()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut: Any) -> None:
        status = fut.result().status
        self.goal_handle = None
        if status == GoalStatus.STATUS_CANCELED and not self.self_cancel:
            self._stop("stopped: goal canceled (STOP or manual driving)")
            return
        if self.phase == "home":
            ok = status == GoalStatus.STATUS_SUCCEEDED
            self._stop("done: map explored, back at the start" if ok else "done: map explored (could not return to the start)")
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.failures = 0
            self.visited += 1
        elif status != GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f"goal {self.goal_xy} failed (status {status}) — blacklisted")
            self._blacklist_goal()

    # ---------------------------------------------------------------- helpers
    def _blacklist_goal(self) -> None:
        if self.goal_xy is not None:
            self.blacklist.append(self.goal_xy)
        self.failures += 1
        if self.failures >= self.max_failures:
            self._stop(f"stopped: {self.failures} goals in a row failed")

    def _cancel_own(self) -> None:
        if self.goal_handle is not None:
            self.self_cancel = True
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def _finish(self, n_frontiers: int) -> None:
        self.get_logger().info(f"no reachable frontiers left ({n_frontiers} blacklisted/too close) — saving the map")
        if self.map_cli.wait_for_service(timeout_sec=1.0):
            self.map_cli.call_async(MapCommand.Request(command="save"))
        if self.return_home and self.home is not None:
            self.phase = "home"
            self._send(*self.home)
            self._status("exploration complete: returning to the start")
        else:
            self._stop("done: map explored")

    def _stop(self, reason: str) -> None:
        self._cancel_own()
        self.enabled, self.phase = False, "idle"
        self._status(reason)
        self.get_logger().info(f"exploration: {reason}")

    def _status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
