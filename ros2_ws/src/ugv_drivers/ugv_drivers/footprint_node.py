#!/usr/bin/env python3
"""Publishes the robot body polygon (from robot.yaml) for collision_monitor and RViz."""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.node import Node

from .robot_config import footprint_polygon, load_robot_config, turn_sweep_radius


class FootprintNode(Node):
    def __init__(self) -> None:
        super().__init__("footprint_publisher")
        self.declare_parameter("config_path", "")
        self.declare_parameter("frame_id", "base_footprint")
        cfg = load_robot_config(str(self.get_parameter("config_path").value) or None)
        self.msg = PolygonStamped()
        self.msg.header.frame_id = str(self.get_parameter("frame_id").value)
        self.msg.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in footprint_polygon(cfg)]
        self.pub = self.create_publisher(PolygonStamped, "footprint", 1)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f"footprint {footprint_polygon(cfg)}; in-place turn sweep radius {turn_sweep_radius(cfg):.3f} m")

    def _tick(self) -> None:
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FootprintNode()
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
