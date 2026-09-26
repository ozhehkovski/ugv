#!/usr/bin/env python3
"""Odometry calibration on the floor, through the full safety chain.

  calibrate.py distance <v> <m>    straight run facing a flat wall; the lidar centre beam gives the true
                                   travel → wheel-radius scale  (v < 0 = back away from the wall)
  calibrate.py spin <deg> [w]      turn in place; IMU yaw gives the true angle → effective track width

Prints the value to put into ugv_description/config/robot.yaml.
"""
from __future__ import annotations

import math
import statistics
import sys
import time

from floor_test import FRONT, Tester

CENTRE_HALF_WIDTH = 0.15   # m: lidar points with |y| below this count as "straight ahead" (1° ≈ 6 cm at 3.5 m)


def wall_x(t: Tester) -> float | None:
    xs = [x for x, y in t.pts if x > FRONT and abs(y) < CENTRE_HALF_WIDTH]
    return statistics.median(xs) if len(xs) >= 3 else None


def settle(t: Tester, secs: float = 1.0) -> None:
    t.stop()
    t.spin(secs)


def distance(t: Tester, v: float, run_m: float, radius: float) -> None:
    samples = []
    for _ in range(10):
        t.spin(0.15)
        if (x := wall_x(t)) is not None:
            samples.append(x)
    if len(samples) < 5:
        raise SystemExit("no flat target straight ahead (need ≥3 lidar points within ±15 cm of the centre line)")
    x0, p0 = statistics.median(samples), t.pose
    print(f"wall ahead at {x0:.3f} m (axle) — driving {v:+.2f} m/s for {run_m} m")
    t0 = time.monotonic()
    while t.dist_from(p0) < run_m and time.monotonic() - t0 < 30:
        if v > 0 and t.front_gap() < 0.15:
            print("stop: close to the wall"); break
        t.send(v, 0.0)
        t.spin(0.05)
    settle(t, 1.5)
    samples = []
    for _ in range(10):
        t.spin(0.15)
        if (x := wall_x(t)) is not None:
            samples.append(x)
    x1 = statistics.median(samples)
    true_d, odom_d = abs(x1 - x0), t.dist_from(p0)
    scale = true_d / odom_d
    print(f"RESULT distance: lidar {true_d:.3f} m, wheel odom {odom_d:.3f} m → scale {scale:.4f}")
    print(f"       wheels.radius: {radius:.4f} → {radius * scale:.4f}  (Ø {radius * scale * 2000:.0f} mm)")


def spin(t: Tester, deg: float, w: float, track: float) -> None:
    target, sign = math.radians(abs(deg)), (1.0 if deg > 0 else -1.0)
    map0 = t.map_pose()
    imu_prev, odo_prev = t.imu_yaw, t.pose[2]
    imu_tot = odo_tot = 0.0
    t0 = time.monotonic()
    while abs(imu_tot) < target and time.monotonic() - t0 < 60:
        t.send(0.0, sign * w)
        t.spin(0.05)
        imu_tot += math.atan2(math.sin(t.imu_yaw - imu_prev), math.cos(t.imu_yaw - imu_prev))
        odo_tot += math.atan2(math.sin(t.pose[2] - odo_prev), math.cos(t.pose[2] - odo_prev))
        imu_prev, odo_prev = t.imu_yaw, t.pose[2]
    settle(t, 1.5)
    imu_tot += math.atan2(math.sin(t.imu_yaw - imu_prev), math.cos(t.imu_yaw - imu_prev))
    odo_tot += math.atan2(math.sin(t.pose[2] - odo_prev), math.cos(t.pose[2] - odo_prev))
    k = odo_tot / imu_tot
    t.spin(1.5)
    map1 = t.map_pose()
    if map0 and map1 and abs(deg) < 300:     # SLAM yaw only unambiguous below a full turn
        d = math.atan2(math.sin(map1[2] - map0[2]), math.cos(map1[2] - map0[2]))
        if d * imu_tot < 0:                   # wrapped past ±180°
            d += math.copysign(2 * math.pi, imu_tot)
        print(f"SLAM check: map yaw {math.degrees(d):+.1f}° vs IMU {math.degrees(imu_tot):+.1f}° "
              f"(IMU/SLAM {imu_tot / d:.4f})")
    print(f"RESULT spin: IMU {math.degrees(imu_tot):+.1f}°, wheel odom {math.degrees(odo_tot):+.1f}° → ratio {k:.4f}")
    print(f"       wheels.track (effective): {track:.4f} → {track * k:.4f}")


def main() -> None:
    from ugv_drivers.robot_config import load_robot_config

    cfg = load_robot_config()
    t = Tester()
    try:
        if t.pose is None or math.isnan(t.imu_yaw):
            raise SystemExit("no /odom or /imu/data — is the stack running?")
        if sys.argv[1] == "distance":
            distance(t, float(sys.argv[2]), float(sys.argv[3]), float(cfg["wheels"]["radius"]))
        elif sys.argv[1] == "spin":
            spin(t, float(sys.argv[2]), float(sys.argv[3]) if len(sys.argv) > 3 else 0.5, float(cfg["wheels"]["track"]))
        else:
            raise SystemExit(__doc__)
    finally:
        t.stop()


if __name__ == "__main__":
    main()
