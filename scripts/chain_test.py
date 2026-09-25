#!/usr/bin/env python3
"""Publish teleop cmd for N s, record the velocity chain + odom, print a timeline."""
import sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

v_cmd, w_cmd, dur = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
rclpy.init(); n = rclpy.create_node("chain_test")
last = {"mux": None, "smoothed": None, "safe": None, "odom_v": None, "odom_w": None, "src": None}
def tw(key):
    def cb(m): last[key] = (round(m.linear.x, 3), round(m.angular.z, 3))
    return cb
n.create_subscription(Twist, "/cmd_vel_mux", tw("mux"), 10)
n.create_subscription(Twist, "/cmd_vel_smoothed", tw("smoothed"), 10)
n.create_subscription(Twist, "/cmd_vel_safe", tw("safe"), 10)
def od(m): last["odom_v"], last["odom_w"] = round(m.twist.twist.linear.x, 3), round(m.twist.twist.angular.z, 3)
n.create_subscription(Odometry, "/odom", od, 10)
n.create_subscription(String, "/cmd_vel_source", lambda m: last.__setitem__("src", m.data), 10)
pub = n.create_publisher(Twist, "/cmd_vel/teleop", 10)
time.sleep(1.0)
t0 = time.monotonic(); nxt = 0.0
while time.monotonic() - t0 < dur + 2.5:
    t = time.monotonic() - t0
    if t < dur:
        m = Twist(); m.linear.x, m.angular.z = v_cmd, w_cmd; pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.02)
    if t >= nxt:
        print(f"t={t:4.1f} src={last['src']} mux={last['mux']} smooth={last['smoothed']} safe={last['safe']} odom v={last['odom_v']} w={last['odom_w']}")
        nxt += 0.5
