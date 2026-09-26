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
- [x] systemd user service `ugv` with autostart at boot (linger, no root), `robot_up.sh install|start|stop|restart|status|log`.
- [ ] rosbag recording, foxglove_bridge.

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
- [x] Heading hold on the floor: reverse 0.9 m at 0.2 m/s → −0.5°, 0.6 cm lateral; forward 0.75 m at 0.3 m/s → 0.0°, −1.2 cm (wheel odometry).
  Turn by 75° → 74.5°.
  3 m run along a tape line at 0.3 m/s (2026-09-26): odometry +1.9°, 2.9 cm left. SLAM +0.6°, 20 cm left.
  The SLAM figure is inconsistent: a long open corridor is degenerate for scan matching.
  Tape runs 1–2 read ~15 cm left, but the IMU showed +0.03° → the tape was crooked.
  Run 3 along the parquet: **5 cm left over 3.16 m (≈0.9°)**. IMU −1.07°, most of it during braking,
  when the hold was off → heading hold now stays active while rolling faster than 0.02 m/s.
- [x] Test on the floor: stop distance, slow zones, the tank-turn circle near a wall.
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
  - Approach toward a bookshelf at 0.3 m/s: full speed until 0.6 m, smooth slowdown, stop 6 cm from it, held.
  - Turn beside a wall (2 cm off the front-right corner): toward the wall 0°, the robot is held; away from the wall +50° freely.
  - TODO: people need a larger margin than furniture. The stop zone ahead is 10 cm, which is fine for walls but not for feet.
- [ ] Measure the real track width (center to center) and check it with a 360° spin.

### 2. Localization ✅ 2026-09-26
- [x] Wheel radius, lidar-calibrated on straight runs toward a bookshelf (`scripts/calibrate.py distance`):
  scale 1.0549 (2.5 m reverse) and 1.0559 (2.3 m forward) → **r = 0.087 m** (was 0.0825).
  Before this fix the hard cap of 0.55 m/s was really ≈0.58 m/s (2.1 km/h) — now correct.
- [x] Effective track, IMU-calibrated spins (`calibrate.py spin`): ±360° and ±180° → 0.4832–0.4856 → **0.484 m** (was 0.515).
  IMU yaw independently confirmed by SLAM: 186.9° vs 187.0°. After the fix: wheels 186.6° / IMU 186.4° / SLAM 186.5°.
- [x] IMU: gyro bias at rest 0.0000 rad/s (σ 0.0003), yaw drift 0 °/min over 15 s; yaw sign (+z = CCW) confirmed.
- [x] Lidar orientation: obstacles ahead are seen ahead (approach test stopped at the shelf in front).
- [x] Straight 3 m: 5 cm (≈0.9°) along the parquet; IMU heading change +0.20° after the braking fix.
- [ ] EKF: fuses wheel vx + IMU yaw/vyaw. Revisit if SLAM shows drift on long loops.

### 3. Maps (persistence ✅, accessibility ✅, exploration → after stage 4)
- [x] `map_manager` owns slam_toolbox. It continues the active map at start (map_file_name + last pose), stores the pose every 2 s,
  autosaves every 60 s and on stop (systemd KillMode=mixed, so the save happens before slam_toolbox stops).
  Verified: drove 1.31 m, restarted the service → pose (1.342, 0.051) → (1.328, 0.062), same map continued.
- [x] Map commands (ugv_interfaces/MapCommand): list / save / save_as / load / new / set_pose / delete. All verified through the web panel API.
- [x] Accessible-terrain layer `map_accessible`: passable ≥ 0.33 m from obstacles, tank-turn room ≥ 0.67 m,
  only cells reachable from the robot. Unknown speckles between lidar rays are closed (≤10 cm); big unexplored areas are excluded.
- [x] Web panel: accessibility overlay, «Карты» panel, «Я здесь» (click + drag on the map sets the robot pose).
- [ ] Frontier exploration (needs Nav2, stage 4).
- [ ] Teleop mux timeout 0.5 → 0.3 s: a lost release message adds up to 0.5 s × speed of travel.

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
