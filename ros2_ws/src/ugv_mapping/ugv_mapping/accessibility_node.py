#!/usr/bin/env python3
"""Publishes the accessible-terrain layer (map_accessible, OccupancyGrid) for the real footprint.

Cell values: -1 unknown · 0 known but not reachable/too tight · 60 passable · 100 room for a tank turn.
Recomputed when the map changes or the robot moves more than 0.2 m (at most once a second).
"""
from __future__ import annotations

import math

import rclpy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from ugv_drivers.robot_config import load_robot_config, turn_sweep_radius

from . import walls
from .accessibility import GridInfo, accessible_layer

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class AccessibilityNode(Node):
    def __init__(self) -> None:
        super().__init__("accessibility")
        self.declare_parameter("margin", 0.05)
        self.declare_parameter("base_frame", "base_footprint")
        cfg = load_robot_config()
        self.half_width = float(cfg["chassis"]["width"]) / 2.0
        self.turn_radius = turn_sweep_radius(cfg, padding=0.0)
        self.margin = float(self.get_parameter("margin").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.map: OccupancyGrid | None = None
        self.dirty = False
        self.last_xy: tuple[float, float] | None = None
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.pub = self.create_publisher(OccupancyGrid, "map_accessible", LATCHED)
        self.create_subscription(OccupancyGrid, "map", self._on_map, LATCHED)
        self.walls: list[walls.Segment] = []
        self.create_subscription(Float64MultiArray, "virtual_walls/segments", self._on_walls, LATCHED)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f"accessibility: passable ≥ {self.half_width + self.margin:.2f} m, "
            f"turnable ≥ {self.turn_radius + self.margin:.2f} m from obstacles")

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map, self.dirty = msg, True

    def _on_walls(self, msg: Float64MultiArray) -> None:
        try:
            self.walls = walls.parse_flat(list(msg.data))
        except ValueError as exc:
            self.get_logger().error(f"virtual walls ignored: {exc}")
            return
        self.dirty = True

    def _tick(self) -> None:
        if self.map is None:
            return
        try:
            t = self.tf.lookup_transform(self.map.header.frame_id, self.base_frame, Time())
            xy = (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            xy = None
        moved = xy is not None and (self.last_xy is None or math.dist(xy, self.last_xy) > 0.2)
        if not (self.dirty or moved):
            return
        self.dirty, self.last_xy = False, xy
        m = self.map
        info = GridInfo(m.info.width, m.info.height, m.info.resolution,
                        m.info.origin.position.x, m.info.origin.position.y)
        grid = np.asarray(m.data, dtype=np.int16).reshape(info.height, info.width)
        if self.walls:     # mirrors / glass the lidar cannot see
            grid = walls.rasterize(grid, self.walls, (info.origin_x, info.origin_y), info.resolution)
        layer = accessible_layer(grid.ravel(), info, xy, self.half_width, self.turn_radius, self.margin)
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = m.header.frame_id
        out.info = m.info
        out.data = layer.ravel().tolist()
        self.pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AccessibilityNode()
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
