#!/usr/bin/env python3
"""Virtual walls for lidar-invisible obstacles (mirrors, glass), stored with the active map.

* ~/set (ugv_interfaces/SetWalls): replace or append walls (web panel)
* bump (geometry_msgs/PointStamped, base frame) from the drive: a wall across the path at the contact
* publishes ~/segments (Float64MultiArray, latched, map frame) for the safety governor and the
  accessibility layer, and ~/cloud (PointCloud2, map frame, 2 Hz) for the Nav2 costmaps
"""
from __future__ import annotations

import math
import os
import struct

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener
from ugv_interfaces.srv import SetWalls

from . import walls
from .map_store import MapStore

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
BUMP_WALL_HALF = 0.35     # m: a bump creates a 0.7 m wall across the path
BUMP_WALL_AHEAD = 0.03    # m: just beyond the contact point


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class VirtualWalls(Node):
    def __init__(self) -> None:
        super().__init__("virtual_walls")
        self.declare_parameter("maps_dir", "~/ugv_maps")
        self.store = MapStore(str(self.get_parameter("maps_dir").value))
        self.map_name: str | None = None
        self.segs: list[walls.Segment] = []
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.seg_pub = self.create_publisher(Float64MultiArray, "~/segments", LATCHED)
        self.cloud_pub = self.create_publisher(PointCloud2, "~/cloud", 10)
        self.event_pub = self.create_publisher(String, "~/events", 10)
        self.create_subscription(String, "map_manager/status", self._on_map_status, LATCHED)
        self.create_subscription(PointStamped, "bump", self._on_bump, 10)
        self.create_service(SetWalls, "~/set", self._on_set)
        self.create_timer(0.5, self._publish_cloud)

    # ---------------------------------------------------------------- storage
    def _path(self) -> str | None:
        return os.path.join(self.store.dir(self.map_name), "walls.json") if self.map_name else None

    def _on_map_status(self, msg: String) -> None:
        name = msg.data.split("|")[0] or None
        if name == self.map_name:
            return
        self.map_name = name
        try:
            self.segs = walls.load(self._path()) if name else []
        except ValueError as exc:
            self.get_logger().error(f"walls of {name!r}: {exc} — ignored")
            self.segs = []
        self.get_logger().info(f"map {name!r}: {len(self.segs)} virtual wall(s)")
        self._publish_segments()

    def _commit(self, segs: list[walls.Segment]) -> None:
        path = self._path()
        if path is None:
            raise ValueError("no active map yet")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        walls.save(path, segs)
        self.segs = segs
        self._publish_segments()
        self._publish_cloud()

    def _on_set(self, req: SetWalls.Request, res: SetWalls.Response) -> SetWalls.Response:
        try:
            new = walls.parse_flat(list(req.segments))
            if req.mode == "set":
                segs = new
            elif req.mode == "add":
                segs = walls.validate(self.segs + new)
            else:
                raise ValueError(f"unknown mode {req.mode!r}")
            self._commit(segs)
            res.success, res.message = True, f"{len(segs)} wall(s)"
        except (ValueError, OSError) as exc:
            res.success, res.message = False, str(exc)
        res.segments = walls.flatten(self.segs)
        return res

    # ---------------------------------------------------------------- bumps
    def _on_bump(self, msg: PointStamped) -> None:
        try:
            t = self.tf.lookup_transform("map", msg.header.frame_id or "base_footprint", Time())
        except TransformException as exc:
            self.get_logger().warn(f"bump: no map pose ({exc}) — no wall added")
            return
        x, y, yaw = t.transform.translation.x, t.transform.translation.y, _yaw(t.transform.rotation)
        cx = msg.point.x + math.copysign(BUMP_WALL_AHEAD, msg.point.x)
        seg = walls.wall_across(x, y, yaw, cx, msg.point.y, BUMP_WALL_HALF)
        try:
            self._commit(walls.validate(self.segs + [seg]))
        except (ValueError, OSError) as exc:
            self.get_logger().error(f"bump wall not saved: {exc}")
            return
        text = f"bump at ({(seg[0] + seg[2]) / 2:.2f}, {(seg[1] + seg[3]) / 2:.2f}): virtual wall added"
        self.get_logger().warn(text)
        self.event_pub.publish(String(data=text))

    # ---------------------------------------------------------------- output
    def _publish_segments(self) -> None:
        self.seg_pub.publish(Float64MultiArray(data=walls.flatten(self.segs)))

    def _publish_cloud(self) -> None:
        pts = walls.sample(self.segs)
        msg = PointCloud2()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = 1, len(pts)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(("x", "y", "z"))]
        msg.is_bigendian, msg.point_step, msg.is_dense = False, 12, True
        msg.row_step = 12 * len(pts)
        msg.data = b"".join(struct.pack("<fff", float(px), float(py), 0.15) for px, py in pts)
        self.cloud_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VirtualWalls()
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
