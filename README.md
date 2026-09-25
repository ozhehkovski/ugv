# UGV follow-me

ROS 2 Humble stack for a follow-me cart. It runs on a Jetson Orin Nano (JetPack 6.2).

| Part | Hardware |
|---|---|
| Chassis | 62 × 56 cm plywood deck. Two hub motors on the rear axle (Ø165 mm), two swivel casters in front |
| Drive | VESC Duet XS100 (FW 7.00): id65 = right (local), id66 = left (forward-CAN). One USB |
| Lidar | RPLIDAR A1M8, centered, 12 cm behind the nose, mounted rotated 180° |
| IMU | BNO085, I2C bus 7, address 0x4A |
| Camera | USB 1280×720 MJPG |

All geometry lives in [`robot.yaml`](ros2_ws/src/ugv_description/config/robot.yaml). The URDF, footprint, camera intrinsics and drive all read it.
`base_footprint` is the floor point under the **drive axle center**, which is the rotation center of an in-place turn. It is 55.5 cm behind the nose. The nose corners sweep a circle of about 0.62 m radius during a tank turn.

## Packages

- `ugv_description`: `robot.yaml` and the URDF.
- `ugv_drivers`: VESC diff-drive driver, RPLIDAR, BNO085, camera, command mux, footprint publisher. The pure-Python core has unit tests.
- `ugv_bringup`: launch files and configs (EKF, safety chain, SLAM baseline).

## Velocity safety chain

```
cmd_vel/teleop (prio 100) ┐
cmd_vel/follow (prio 50)  ├─ cmd_mux → velocity_smoother → collision_monitor → cmd_vel_safe → vesc_driver
cmd_vel/nav    (prio 10)  ┘   (timeouts)   (soft accel)      (body polygon, turn sweep)    (hard caps, ramp, watchdog, estop)
```

The driver enforces the hard limits on its own: 0.55 m/s (2 km/h), 0.9 rad/s and 0.3 m/s² acceleration. If `cmd_vel` is silent for more than 0.3 s, or VESC telemetry is lost, it stops the wheels.

## Run

```bash
./scripts/deploy.sh                                   # rsync + colcon build on the robot (UGV_HOST=luki@192.168.1.58)
ssh luki@192.168.1.58 '~/ugv_ws/scripts/robot_up.sh start'   # background; `stop` to stop, log in ~/ugv_base.log
# or in the foreground on the robot:
ros2 launch ugv_bringup base.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel/teleop
```

Emergency stop is a latched `std_msgs/Bool` on `/estop`. Publish from a process that stays alive, as `scripts/estop_test.py` does. `ros2 topic pub --once` exits before the message is delivered.

## Tests

```bash
cd ros2_ws/src/ugv_drivers && python3 -m pytest -q test
```

Bench scripts for use with the robot on a stand are in `scripts/`: `vesc_bench.py`, `chain_test.py`, `estop_test.py` and `turn_test.py`.

## Plan

See [docs/PLAN.md](docs/PLAN.md).
