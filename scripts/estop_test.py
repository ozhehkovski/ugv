#!/usr/bin/env python3
"""Drive 0.15 m/s, engage estop at t=2.5 s (teleop keeps publishing), release at t=5 s."""
import time
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

rclpy.init(); n = rclpy.create_node("estop_test")
st = {"v": None, "diag": None}
n.create_subscription(Odometry, "/odom", lambda m: st.__setitem__("v", round(m.twist.twist.linear.x, 3)), 10)
def dg(m):
    for s in m.status:
        if s.name == "ugv/vesc_driver": st["diag"] = s.message
n.create_subscription(DiagnosticArray, "/diagnostics", dg, 10)
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
estop = n.create_publisher(Bool, "/estop", qos)
cmd = n.create_publisher(Twist, "/cmd_vel/teleop", 10)
time.sleep(1.0)
t0 = time.monotonic(); nxt = 0.0; state = None
while (t := time.monotonic() - t0) < 7.5:
    if t < 6.0:
        m = Twist(); m.linear.x = 0.15; cmd.publish(m)
    want = 2.5 <= t < 5.0
    if want != state:
        estop.publish(Bool(data=want)); state = want
    rclpy.spin_once(n, timeout_sec=0.02)
    if t >= nxt:
        print(f"t={t:3.1f} estop={state!s:5} odom_v={st['v']} diag={st['diag']}"); nxt += 0.5
estop.publish(Bool(data=False)); time.sleep(0.5)
