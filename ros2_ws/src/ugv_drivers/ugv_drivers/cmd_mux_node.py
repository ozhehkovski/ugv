#!/usr/bin/env python3
"""Velocity command mux: teleop > follow > nav (priorities/timeouts from parameters).

Publishes the winner on `cmd_vel_out` at a fixed rate (zero when nobody is active) and the
active source name on `cmd_vel_source`.
"""
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .cmd_mux import CmdMux, MuxInput


class CmdMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_mux")
        self.declare_parameter("inputs", ["teleop", "follow", "nav"])
        self.declare_parameter("rate_hz", 20.0)
        names = [str(n) for n in self.get_parameter("inputs").value]
        specs: list[MuxInput] = []
        for name in names:
            self.declare_parameter(f"{name}.topic", f"cmd_vel/{name}")
            self.declare_parameter(f"{name}.priority", 0)
            self.declare_parameter(f"{name}.timeout", 0.5)
            spec = MuxInput(
                name=name,
                priority=int(self.get_parameter(f"{name}.priority").value),
                timeout=float(self.get_parameter(f"{name}.timeout").value),
            )
            specs.append(spec)
            topic = str(self.get_parameter(f"{name}.topic").value)
            self.create_subscription(Twist, topic, lambda msg, n=name: self._on_cmd(n, msg), 10)
            self.get_logger().info(f"input {name}: {topic} priority={spec.priority} timeout={spec.timeout}s")
        self.mux = CmdMux(specs)
        self.pub = self.create_publisher(Twist, "cmd_vel_out", 10)
        self.src_pub = self.create_publisher(String, "cmd_vel_source", 10)
        self._active: str | None = None
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)

    def _on_cmd(self, name: str, msg: Twist) -> None:
        self.mux.update(name, float(msg.linear.x), float(msg.angular.z), time.monotonic())

    def _tick(self) -> None:
        name, v, w = self.mux.select(time.monotonic())
        out = Twist()
        out.linear.x, out.angular.z = v, w
        self.pub.publish(out)
        if name != self._active:
            self.get_logger().info(f"active source: {self._active} → {name}")
            self._active = name
        self.src_pub.publish(String(data=name or "none"))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CmdMuxNode()
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
