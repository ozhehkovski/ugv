# Work plan

## Requirements
- SLAM: build a map, save it, reload it and keep updating it.
- Map of accessible terrain. Navigation over areas already explored.
- Follow-me mode that keeps about 40 cm from the person, measured from the nose.
- Soft start. Never faster than 2 km/h (0.55 m/s).
- Footprint-aware: no body edge touches anything during maneuvers. Turns are tank turns (in place).
- Everything on ROS 2. Later: autonomous go-to-point and robots working together.

## Stages

### 0. Foundation ✅ 2026-09-25
- [x] Repo. From the old code only the drivers and configs were kept.
- [x] `robot.yaml` holds the real geometry (62×56, axle 55.5 cm from the nose, lidar 12 cm from the nose).
- [x] URDF with a frame prefix (ready for multiple robots).
- [x] `deploy.sh`: rsync + colcon build on the Jetson.
- [ ] systemd unit for autostart, rosbag recording, foxglove_bridge.

### 1. Drive and safety (in progress)
- [x] Bench test of the VESC: id65 = right, id66 = left, + = forward on both.
- [x] Driver with hard caps, soft-start ramp, cmd watchdog, telemetry watchdog, estop and diagnostics.
- [x] Chain: cmd_mux → velocity_smoother → safety_governor (own, direction-aware).
  Nav2 Humble's collision_monitor was replaced: its stop polygon zeroed every command while an obstacle was inside it, so the robot got trapped next to a sofa and could not back away.
- [x] Bench checks: straight run, tank turn, estop, stop on timeout.
- [ ] Tune the id66 speed PID in VESC Tool. It oscillates: 39–489 ERPM measured for 300 commanded. id65 is stable.
- [x] Low-speed stall: on the floor the wheels stall below ~0.04 m/s. The driver lifts slower non-zero commands to `min_wheel_speed` 0.06 m/s, keeping the curvature.
- [x] Slow zone only ahead of the nose. Side obstacles are handled by the approach check, which knows the turn direction.
- [x] After estop release, require a zero command before moving again.
- [x] Web panel (map + lidar + footprint, camera, status, joystick with dead-man, big STOP).
- [x] Heading hold: gyro yaw-rate PI in the driver. The front casters swing and push the robot off course. Gyro sign verified (+z = CCW).
- [ ] Verify heading hold on a 1.5 m straight run: drift < 2°.
- [ ] Test on the floor: stop distance, slow zones, the tank-turn circle near a wall.
  - 2026-09-25: stop from 0.085 m/s = 5.2 cm / 0.6 s. The front slow zone cut 0.2 → 0.08 m/s.
  - Turn next to the sofa: turning toward it was limited by approach (0.4 → 0.13 rad/s, matches the 11°-to-contact prediction).
    After the fixes it turned 21° and stopped 4–5 cm from an obstacle (a leg). No contact.
  - Braking on the floor (safety governor chain, 2026-09-25):
    | speed | stop | distance | time |
    |---|---|---|---|
    | 0.315 m/s | release (smoother 0.5 m/s²) | 15.6 cm | 0.91 s |
    | 0.316 m/s | estop (VESC brake current) | 7.6 cm | 0.44 s |
    | 0.541 m/s | estop | 14.0 cm | 0.61 s |
    | 0.20 m/s reverse | release | 8.6 cm | 0.6 s |
    Soft start: 0 → 0.3 m/s in about 1 s. Measured top speed 0.54 m/s for a 0.5 command (check the wheel radius).
  - TODO: approach toward a wall/box (≥25 cm tall: the lidar is at 19.5 cm); turn with a wall 10–20 cm off the side.
  - TODO: people need a larger margin than furniture. The stop zone ahead is 10 cm, which is fine for walls but not for feet.
- [ ] Measure the real track width (center to center) and check it with a 360° spin.

### 2. Localization
- [ ] Calibrate the EKF (wheel odometry + BNO085). With casters the wheel yaw is usable, so consider fusing vyaw.
- [ ] Check lidar orientation (object in front → bearing 0°) and IMU yaw sign.
- [ ] Drift test: 5 m straight, 360° spin.

### 3. Maps
- [ ] slam_toolbox, lifelong mode: save the pose graph, load it at start, keep mapping.
- [ ] Map manager service: save / load / list maps.
- [ ] Accessible-terrain layer: cells reachable with the real footprint.
- [ ] Frontier exploration (explore_lite).

### 4. Navigation
- [ ] Nav2 with the rectangular footprint. Controller: MPPI (DiffDrive), with the footprint critic.
- [ ] In-place turn only when the 0.62 m sweep circle is free. Otherwise back up or drive forward first.
- [ ] Output to `cmd_vel/nav`.

### 5. Follow-me
- [ ] YOLO11n (TensorRT) + ByteTrack + re-ID so the robot locks onto one person.
- [ ] Range to the person from the lidar (legs), bearing from the camera.
- [ ] Controller keeps 0.4 m from the nose, never reverses. Output to `cmd_vel/follow`.
- [ ] Person lost: Nav2 to the last seen point, then search by turning in place.
- [ ] Mode state machine: manual / explore / navigate / follow.

### 6. Multi-robot
- [ ] Namespaces + TF prefix, DDS over Wi-Fi (CycloneDDS or Zenoh).
- [ ] Shared map, pose exchange, coordination (Open-RMF or a simple dispatcher).
