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
- [x] Chain: cmd_mux → velocity_smoother → collision_monitor (approach on the body polygon + stop/slow zones).
- [x] Bench checks: straight run, tank turn, estop, stop on timeout.
- [ ] Tune the id66 speed PID in VESC Tool. It oscillates: 39–489 ERPM measured for 300 commanded. id65 is stable.
- [ ] Low-speed deadband: below about 60 ERPM (0.026 m/s per wheel) the wheels do not move.
- [x] After estop release, require a zero command before moving again.
- [x] Web panel (map + lidar + footprint, camera, status, joystick with dead-man, big STOP).
- [ ] Test on the floor: stop distance, slow zones, the tank-turn circle near a wall.
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
