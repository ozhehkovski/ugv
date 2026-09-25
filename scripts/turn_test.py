#!/usr/bin/env python3
"""Bench only: send w directly to the driver input (bypasses the safety chain)."""
import sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
w_cmd = float(sys.argv[1])
rclpy.init(); n = rclpy.create_node("turn_test"); st = {}
def od(m): st.update(v=round(m.twist.twist.linear.x, 3), w=round(m.twist.twist.angular.z, 3), th=m.pose.pose.orientation)
n.create_subscription(Odometry, "/odom", od, 10)
pub = n.create_publisher(Twist, "/cmd_vel_safe", 10)
time.sleep(1.0); t0 = time.monotonic(); nxt = 0
while (t := time.monotonic() - t0) < 5.0:
    if t < 3.5:
        m = Twist(); m.angular.z = w_cmd; pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.03)
    if t >= nxt: print(f"t={t:3.1f} v={st.get('v')} w={st.get('w')}"); nxt += 0.5
