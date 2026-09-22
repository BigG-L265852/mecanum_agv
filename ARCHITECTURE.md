# Architecture

This document explains how `mecanum_agv` is put together and *why* each piece was
built the way it was. See `README.md` for setup/run instructions.

## 1. Two-machine split

| Machine | Runs | Why |
|---|---|---|
| Raspberry Pi (on the robot) | `bringup.launch.py`: `robot_state_publisher`, `mecanum_drive_node`, RPLIDAR driver | Only needs to collect and publish sensor/odometry data — stays lightweight |
| Dev laptop | `slam.launch.py` (mapping) or `localization.launch.py` (once a map exists): `slam_toolbox`/AMCL + RViz | SLAM is compute-heavy; running it off-robot avoids needing a screen/GPU on the Pi |

Both machines must share the same `ROS_DOMAIN_ID` — ROS 2's DDS middleware
publishes transparently over the LAN, so `/scan`, `/odom` and `/tf` are visible
across machines without any extra bridging code.

## 2. Realtime motor control: a separate Arduino Mega

Motor PWM output, encoder tick counting, and per-wheel PID velocity control live
entirely in `arduino_firmware/mecanum_agv_firmware/mecanum_agv_firmware.ino` —
not on the Pi.

- **Why an MCU at all, not just the Pi:** PWM timing and encoder interrupts need
  hard realtime guarantees. A general-purpose Linux scheduler on the Pi can't
  reliably promise that, so low-level control is split out to a microcontroller
  running a fixed 20 Hz loop (`CONTROL_INTERVAL_MS = 50`) via `millis()`.
- **Why a Mega 2560, not an Uno:** 4 wheel encoders need 4 interrupt-capable
  pins (`18, 19, 20, 21` on the Mega). An Uno only has 2 — not enough for a
  4-wheel mecanum robot.
- **Known simplification:** encoders only count magnitude (`RISING` edge), not
  true quadrature A/B decoding. Direction is inferred from the last *commanded*
  sign (`g_last_sign`), not measured. This means a wheel being back-driven
  against its commanded direction (e.g. pushed by a collision) isn't detected —
  no stall detection. Acceptable for a time-boxed project; a known limitation
  if stall/collision detection is ever needed.

## 3. Pi ↔ Arduino protocol: a small custom text protocol

```
Pi  → Arduino:  "V <w_fl> <w_fr> <w_rl> <w_rr>\n"   target wheel angular velocities, rad/s
Arduino → Pi:   "E <w_fl> <w_fr> <w_rl> <w_rr>\n"   measured wheel angular velocities, rad/s
```
Serial, 115200 baud.

**Why not micro-ROS:** a plain ASCII protocol is easy to inspect with a serial
monitor and needs no extra library on the MCU. The cost is that parsing/error
handling is hand-rolled — `parseSerialLine` has no checksum, and malformed
lines are silently dropped. A reasonable trade-off for a time-boxed project:
fast to get working, less robust against line noise/corruption than a proper
framed protocol would be.

## 4. Kinematics (`mecanum_agv/mecanum_drive_node.py`)

Inverse kinematics (body twist → 4 wheel angular velocities):
```
w_fl = (vx - vy - l·wz) / r        w_fr = (vx + vy + l·wz) / r
w_rl = (vx + vy - l·wz) / r        w_rr = (vx - vy + l·wz) / r
```
where `l = wheel_separation_x + wheel_separation_y`. Forward kinematics
(encoder feedback → body twist) is the matrix inverse of this.

**Why these specific signs:** they encode the **X-pattern roller layout**
(front-left/rear-right roller axes parallel, front-right/rear-left parallel).
This is the single most load-bearing assumption in the whole system — wire the
wheels backwards and the robot spins in place instead of strafing sideways.
Flagged explicitly in code comments and `README.md` for this reason.

**Why the kinematics live on the Pi, not the Arduino:** it keeps the Arduino
"dumb" (just: here are 4 target velocities, here are 4 measured velocities)
and keeps robot geometry (`wheel_radius`, `wheel_separation_x/y`) centralized
in ROS parameters (`config/robot_params.yaml`) instead of hardcoded in
firmware.

## 5. Odometry: encoder-only, with frame rotation

`_odom_timer_callback` integrates `vx, vy, wz` at 30 Hz and explicitly rotates
the per-step delta from the robot frame into the odom frame before
accumulating:
```python
delta_x = (vx*cos(theta) - vy*sin(theta)) * dt
delta_y = (vx*sin(theta) + vy*cos(theta)) * dt
```

**Why this matters specifically for mecanum:** a differential-drive robot
always has `vy = 0`, so this rotation step is often skipped in tutorials. A
holonomic mecanum robot has real sideways velocity, so without this rotation
the map would skew every time the robot turned while moving.

**Known limitation:** there's no IMU fusion (no `robot_localization`/EKF) —
odometry is pure wheel-encoder integration, which is sensitive to the wheel
slip that's inherent to mecanum rollers. Acceptable for Phase 1 mapping in a
controlled space; drift would become more visible on longer/larger runs.

**Also built in:** a watchdog (`_cmd_vel_watchdog_callback`) that zeroes all
wheel targets if `/cmd_vel` goes silent for >0.5s — stops the robot from
coasting if the teleop/controller node dies or the link drops.

## 6. URDF and TF tree

`urdf/mecanum_agv.urdf.xacro` defines only static geometry:
`base_footprint → base_link → laser_frame`, plus 4 **visual-only** wheel links
(fixed joints — actual odometry comes from encoder integration in
`mecanum_drive_node`, not from joint states).

Full TF tree:
```
map → odom → base_footprint → base_link → laser_frame
```
- `map → odom`: published by `slam_toolbox` (mapping) or AMCL (localization)
- `odom → base_footprint`: published by `mecanum_drive_node` (§5)
- everything else: `robot_state_publisher` + this URDF

This is the standard ROS 2 REP-105 layered pattern — each layer has exactly
one owning node, which is what `slam_toolbox`/AMCL/Nav2 expect out of the box.

## 7. SLAM: two bugs fixed along the way

- `robot_description` (the xacro command output) had to be explicitly wrapped
  in `ParameterValue(..., value_type=str)` in `bringup.launch.py` — otherwise
  `robot_state_publisher` tried to parse the xacro output as YAML and crashed.
- `async_slam_toolbox_node` is a **lifecycle node**. Without explicitly driving
  it through `configure → activate` (the `EmitEvent`/`RegisterEventHandler`
  block in `slam.launch.py`), it silently stays `unconfigured` forever — never
  subscribes to `/scan`, never publishes `/map`, no error at all. Easy to
  reintroduce if the launch file is ever "simplified" back to a plain `Node`.

Both are silent failure modes (nothing crashes, nothing happens) — the kind of
bug that costs the most time to track down, which is why they're called out
here explicitly.

## 8. Localization phase (`localization.launch.py`)

Once a map is saved (`nav2_map_server`'s `map_saver_cli`, see `maps/README.md`),
`localization.launch.py` runs `nav2_map_server` + `nav2_amcl` + a
`nav2_lifecycle_manager` (which handles the configure/activate lifecycle
transitions for both, the same problem §7 solves by hand for slam_toolbox) to
localize the robot against that fixed map instead of building a new one.

## 9. Known upstream bug: RViz Map display

RViz2's native `Map` display fails to link its `indexed_8bit_image` palette
shader on a wide range of GPUs — confirmed as a long-standing, unresolved
upstream bug ([ros2/rviz#1279](https://github.com/ros2/rviz/issues/1279)), not
fixable via RViz config.

**Workaround:** `mecanum_agv/map_to_pointcloud.py` republishes `/map` as a
colored `sensor_msgs/PointCloud2` on `/map_points` (gray = free, red =
occupied), reusing the same shader path that `LaserScan` already uses
successfully. Both `slam.launch.py` and `localization.launch.py` start it
automatically. There's also `map_to_png.py` for viewing `/map` outside RViz
entirely.

**Rule going forward:** always view the map via `/map_points`, never expect
the native `Map` display to work. A real fix would mean rebuilding RViz's
Ogre-linked libraries from source — out of scope here.

## 10. Open items / PLACEHOLDERs

Everything literally marked `PLACEHOLDER` in the code still needs to be
measured on the physical robot:

- Motor/encoder pins and `TICKS_PER_REV` in the Arduino firmware (depends on
  the actual motor/gearbox stack)
- Wheel radius and wheel separation — must match between
  `config/robot_params.yaml` **and** `urdf/mecanum_agv.urdf.xacro`
- PID gains (`KP=2.0, KI=5.0, KD=0.0` are untuned defaults)
- Chassis dimensions in the URDF (currently a `0.24 × 0.20 × 0.08 m` placeholder)

Also still pending:
- Raspberry Pi connection details (IP/SSH) — with teammates as of last check
- No IMU fusion yet for more robust odometry against wheel slip
