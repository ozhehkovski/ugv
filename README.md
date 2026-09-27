# UGV Follow-Me Robot: ROS 2 Humble on Jetson Orin Nano

Open-source **follow-me robot cart** (a human-following UGV) built on **ROS 2 Humble** and an **NVIDIA Jetson Orin Nano** (JetPack 6.2). The robot finds a person with **YOLO on TensorRT**, measures the distance with an **RPLIDAR A1** lidar, and follows at a 40 cm gap. It builds a map with **SLAM Toolbox**, keeps it across restarts, and runs every velocity command through a safety chain with collision prediction and an emergency stop. A Next.js **robot fleet management** console controls many robots at once and includes a swarm simulation over a mesh network.

**Contents:** [Features](#features) · [Hardware](#hardware) · [ROS 2 packages](#ros-2-packages) · [Safety chain](#velocity-safety-chain) · [Quick start](#quick-start-on-the-jetson-orin-nano) · [Fleet console](#robot-fleet-management-console) · [Tests](#tests) · [Roadmap](#roadmap)

## Features

- **Person following (follow-me mode):** YOLO person detection on TensorRT, lidar ranging, a single-target tracker and a gap controller that holds 40 cm.
- **ROS 2 SLAM with a persistent map:** SLAM Toolbox baseline, map autosave in `~/ugv_maps`, load/new/save-as, set pose, a fixed-map work mode and carry detection that protects the map.
- **Obstacle avoidance and safety:** a governor that predicts the robot body along the commanded arc, hard speed and acceleration caps in the driver, a watchdog and a latched e-stop.
- **Virtual walls and bump detection** for obstacles the lidar cannot see.
- **VESC diff-drive driver** for two hub motors over one USB and CAN.
- **ROS 2 web interface:** a browser panel with the live map, lidar, camera, joystick/WASD teleop and a big STOP button.
- **Fleet console:** multi-robot control, an operator queue, and a 10-robot swarm demo on rough terrain.

## Hardware

| Part | Hardware |
|---|---|
| Computer | NVIDIA Jetson Orin Nano, JetPack 6.2 |
| Chassis | 62 × 56 cm plywood deck. Two hub motors on the rear axle (Ø165 mm), two swivel casters in front |
| Drive | VESC Duet XS100 (FW 7.00): id65 = right (local), id66 = left (forward-CAN). One USB |
| Lidar | RPLIDAR A1M8, centered, 12 cm behind the nose, mounted rotated 180° |
| IMU | BNO085, I2C bus 7, address 0x4A |
| Camera | USB 1280×720 MJPG |

All geometry lives in [`robot.yaml`](ros2_ws/src/ugv_description/config/robot.yaml). The URDF, footprint, camera intrinsics and drive all read it.
`base_footprint` is the floor point under the **drive axle center**, which is the rotation center of an in-place turn. It is 55.5 cm behind the nose. The nose corners sweep a circle of about 0.62 m radius during a tank turn.

## ROS 2 packages

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

## Quick start on the Jetson Orin Nano

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

## Robot fleet management console

`console/` is a Next.js app that lets one operator run many robots from a laptop:

- Add a real robot by the address of its web panel, for example `http://192.168.1.58:8090`.
- Set goals on the map, or select several robots and give them one goal.
- Robots that need a human are listed in the "Нужен оператор" (operator needed) queue. For each one you can take manual control (joystick or WASD), then return it to auto mode.
- The demo mode is a 2D swarm robotics simulation of 10 robots on rough terrain: forest, rocks, mud, a river with fords, a ravine and hills. The robots share positions and obstacles over a simulated mesh network, and some of them stop to act as relays that keep the chain back to the base. "Подкинуть проблему" (inject a problem) makes one robot get stuck so you can rescue it.

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

## Roadmap

See [docs/PLAN.md](docs/PLAN.md).
