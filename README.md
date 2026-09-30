# mecanum_agv

ROS 2 Jazzy package for a 4-mecanum-wheel AGV — BIC mapping challenge: SLAM, AMCL
localization and autonomous navigation with Nav2.

Two machines:
- **Raspberry Pi** (on the robot): runs `bringup.launch.py` — robot_state_publisher, the
  mecanum drive/odom node (talks to the Arduino over serial), and the RPLIDAR.
- **Dev laptop**: runs `slam.launch.py` (slam_toolbox + RViz) or `navigation.launch.py`
  (AMCL/SLAM + the full Nav2 stack + RViz). Must share the same
  `ROS_DOMAIN_ID` as the Pi so `/scan`, `/odom` and `/tf` are visible over the network.

## Setup (once per machine)

Dev laptop only (Nav2 runs on the laptop, not on the Pi):
```
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

Both machines: clone this repo into `~/ros2_ws/src/`, then build:
```
cd ~/ros2_ws
colcon build --packages-select mecanum_agv
```
Rebuild after every change to a launch/config/map file: they're *copied* into
`install/`. (Don't switch to `--symlink-install` in an existing workspace — colcon then
fails with `File exists`; delete `build/mecanum_agv` and `install/mecanum_agv` first.)

Add these lines to the end of `~/.bashrc` on **both** the laptop and the Pi (once),
so every new terminal has ROS, the same `ROS_DOMAIN_ID` (otherwise the laptop never sees
the Pi's `/scan`, `/odom` and `/tf`) and this workspace loaded:
```
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=1
source ~/ros2_ws/install/setup.bash
```
Each `source` line only once — check with `grep -n setup.bash ~/.bashrc`. Run
`source ~/.bashrc` (or open a new terminal) afterwards.

Arduino: open `arduino_firmware/mecanum_agv_firmware/mecanum_agv_firmware.ino` in the
Arduino IDE, board *Arduino Mega 2560*, and upload (after checking the PLACEHOLDER pins).

## Every new terminal

Nothing to do if you did the `~/.bashrc` step above. If `ros2 launch` says
`Package 'mecanum_agv' not found`, the workspace isn't sourced:
```
source ~/ros2_ws/install/setup.bash
```

## Before it will actually work

Everything marked `PLACEHOLDER` in the code must be measured/verified on the real robot:
- Motor + encoder pins and ticks-per-revolution (`arduino_firmware/.../mecanum_agv_firmware.ino`)
- Wheel radius and wheel separation (`config/robot_params.yaml`, must match `urdf/mecanum_agv.urdf.xacro`)
- Robot footprint for Nav2 (`config/nav2_params.yaml`, `footprint`, twice)
- Mecanum roller orientation: the kinematics assume an X-pattern (front-left/rear-right
  rollers parallel, front-right/rear-left parallel). Wired backwards, the robot spins on
  the spot instead of strafing.
- LiDAR serial port and baud rate (115200 for RPLIDAR A1/A2, 256000 for A3/S1)

## Running the robot

### 1. Pi: start the hardware (always)
```
ros2 launch mecanum_agv bringup.launch.py serial_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0
```
Arduino Mega = `/dev/ttyACM0`, RPLIDAR = `/dev/ttyUSB0` — but the number changes after
unplugging/replugging (e.g. `/dev/ttyUSB1`), so check first with `ls /dev/ttyACM* /dev/ttyUSB*`.

### 2. Laptop: build the map (once per room)
```
ros2 launch mecanum_agv slam.launch.py
```
Drive around slowly with the keyboard, in a second laptop terminal:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
When the map looks complete in RViz, save it **exactly here** (navigation loads this file):
```
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/mecanum_agv/maps/map
cd ~/ros2_ws && colcon build --packages-select mecanum_agv
```
Then stop `slam.launch.py` (Ctrl+C).

### 3. Laptop: drive autonomously
```
ros2 launch mecanum_agv navigation.launch.py
```
1. AMCL assumes the robot starts at the map origin (where mapping started). If it's
   somewhere else, click **2D Pose Estimate** in RViz and drag where it actually is/faces
   (the yellow particle cloud should converge while driving).
2. Click **2D Goal Pose** anywhere on the map — the robot plans a path (blue) and drives there.
   A new click replaces the current goal.

Or drive a fixed route without clicking (edit `config/waypoints.yaml` first, then rebuild),
in a second laptop terminal while `navigation.launch.py` runs:
```
ros2 run mecanum_agv waypoint_mission                      # once
ros2 run mecanum_agv waypoint_mission --ros-args -p loop:=true
```

Navigate while mapping an unknown room instead (no saved map needed; replaces step 2):
```
ros2 launch mecanum_agv navigation.launch.py slam:=true
```

Tuning lives in `config/nav2_params.yaml` — start with the footprint (PLACEHOLDER) and the
speed limits (`vx_max`/`vy_max`/`wz_max` in `FollowPath` **and** `velocity_smoother`).
See ARCHITECTURE.md §9 for the design.

### Stopping
Ctrl+C in each terminal. To stop every launch file at once (e.g. from another terminal):
```
pkill -INT -f "ros2 launch mecanum_agv"
```
Use `-INT` (= Ctrl+C): a plain `pkill -f` sends SIGTERM, which only kills the
`ros2 launch` process itself and leaves all its nodes (Nav2, AMCL, LiDAR driver, ...)
running as orphans. Check with `ros2 node list` that nothing is left.

## Testing with only the LiDAR (no robot)

LiDAR on the laptop's USB, everything on the laptop. Step by step in
`LOCALIZATION_TESTING.md`; the commands in short:
```
ls /dev/ttyUSB*                                             # which port the LiDAR got
ros2 launch mecanum_agv lidar_test_bringup.launch.py lidar_port:=/dev/ttyUSB0   # terminal 1, keep running

# terminal 2 — map (walk slowly with the laptop), then save as in step 2 above:
ros2 launch mecanum_agv slam.launch.py params_file:=$(ros2 pkg prefix mecanum_agv)/share/mecanum_agv/config/mapper_params_no_odom_test.yaml

# terminal 2 — or Nav2 on the saved map (plans and sends /cmd_vel, but nothing moves):
ros2 launch mecanum_agv navigation.launch.py

# terminal 2 — or Nav2 while mapping:
ros2 launch mecanum_agv navigation.launch.py slam:=true slam_params_file:=$(ros2 pkg prefix mecanum_agv)/share/mecanum_agv/config/mapper_params_no_odom_test.yaml
```

## Checking that things work

```
ros2 topic hz /scan                  # LiDAR: ~7 Hz for an RPLIDAR A1
ros2 topic echo /odom --once         # wheel odometry from mecanum_drive_node (robot only)
ros2 topic echo /cmd_vel             # what Nav2 / teleop tells the wheels to do
ros2 run tf2_ros tf2_echo map base_footprint   # where the robot thinks it is
ros2 run mecanum_agv map_to_png ~/map.png      # current /map as a PNG, outside RViz
```

**RViz empty / `Global Status: Error` / `ros2 topic hz /scan` shows nothing?** Look at the
LiDAR terminal. If it shows
```
[rplidar_composition]: Start
[ERROR] [rplidar_composition]: Cannot start scan: '80008000'
[ERROR] [rplidar_composition]: Failed to set scan mode
```
or stops after `Start` without errors while `/scan` stays silent, the LiDAR itself is stuck
(it still reports health `0`). Not a code or port problem: Ctrl+C the launch, unplug the
LiDAR's USB, wait ~10 s, plug it back in (same port is fine) and start again.

## Required TF tree

```
map -> odom -> base_footprint -> base_link -> laser_frame
```

`map -> odom` comes from slam_toolbox (mapping) or AMCL (localization/navigation), `odom -> base_footprint` from `mecanum_drive_node`
(encoder odometry), the rest from `robot_state_publisher` + the URDF.
