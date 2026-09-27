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
- `ugv_mapping`: `map_manager`, which keeps the SLAM map across restarts (maps in `~/ugv_maps`, autosave, load/new/save-as, set pose), and the accessible-terrain layer.
- `ugv_interfaces`: services (`MapCommand`, `SetWalls`).
- `ugv_follow`: follow-me. YOLO on TensorRT plus lidar ranging, a single-target tracker, and a 40 cm gap controller.
- `ugv_webui`: operator web panel on port 8090. It shows the live SLAM map with lidar and footprint, the camera and the status. It has a joystick/WASD teleop with a dead-man, and a big STOP (Space).

## Velocity safety chain

```
cmd_vel/teleop (prio 100) ┐
cmd_vel/follow (prio 50)  ├─ cmd_mux → velocity_smoother → safety_governor → cmd_vel_safe → vesc_driver
cmd_vel/nav    (prio 10)  ┘   (timeouts)   (soft accel)      (body along the arc, 5 cm)  (hard caps, ramp, watchdog, estop)
```

`safety_governor` simulates the body rectangle along the commanded arc for 1.5 s. It scales the command by the time left until the clearance would drop below 5 cm. It only limits motion that closes in, so backing away or turning away from a wall always works. With no fresh scan it outputs zero.

The driver enforces the hard limits on its own: 0.55 m/s (2 km/h), 0.9 rad/s and 0.3 m/s² acceleration. If `cmd_vel` is silent for more than 0.3 s, or VESC telemetry is lost, it stops the wheels.

## Run

```bash
./scripts/deploy.sh                                   # rsync + colcon build on the robot (UGV_HOST=luki@192.168.1.58)
ssh luki@192.168.1.58 '~/ugv_ws/scripts/robot_up.sh install'  # once: systemd user service `ugv`, autostarts at boot
ssh luki@192.168.1.58 '~/ugv_ws/scripts/robot_up.sh restart'  # after a deploy; also start|stop|status|log (log: ~/ugv_base.log)
# open http://192.168.1.58:8090 from a phone/laptop on the same Wi-Fi
# or in the foreground on the robot:
ros2 launch ugv_bringup robot.launch.py        # base.launch.py = without SLAM and web panel
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel/teleop
```

Emergency stop is a latched `std_msgs/Bool` on `/estop`. The web panel owns a live publisher for it. After release, the driver will not move until it sees a zero command, so a held joystick cannot make the robot jump. Publish from a process that stays alive, as `scripts/estop_test.py` does. `ros2 topic pub --once` exits before the message is delivered.

## Fleet console (laptop)

`console/` is a Next.js app that lets one operator run many robots:

- Add a real robot by the address of its web panel, for example `http://192.168.1.58:8090`.
- Set goals on the map, or select several robots and give them one goal.
- Robots that need a human are listed in the "Нужен оператор" queue. For each one you can take manual control (joystick or WASD), then return it to auto mode.
- The demo mode is a 2D simulation of 10 robots on rough terrain: forest, rocks, mud, a river with fords, a ravine and hills. The robots share positions and obstacles over a simulated mesh network, and some of them stop to act as relays that keep the chain back to the base. "Подкинуть проблему" makes one robot get stuck so you can rescue it.

```bash
pnpm --dir console install
pnpm --dir console dev          # http://localhost:3000
pnpm --dir console test         # swarm simulation tests
```

## Tests

```bash
cd ros2_ws/src/ugv_drivers && python3 -m pytest -q test
cd ros2_ws/src/ugv_webui && python3 -m pytest -q test
```

Bench scripts for use with the robot on a stand are in `scripts/`: `vesc_bench.py`, `chain_test.py`, `estop_test.py` and `turn_test.py`.

## Plan

See [docs/PLAN.md](docs/PLAN.md).
