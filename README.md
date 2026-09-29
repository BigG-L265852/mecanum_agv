# mecanum_agv

ROS 2 Jazzy package for a 4-mecanum-wheel AGV — BIC mapping challenge: SLAM, AMCL
localization and autonomous navigation with Nav2.

Two machines:
- **Raspberry Pi** (on the robot): runs `bringup.launch.py` — robot_state_publisher, the
  mecanum drive/odom node (talks to the Arduino over serial), and the RPLIDAR.
- **Dev laptop**: runs `slam.launch.py` (slam_toolbox + RViz) or `navigation.launch.py`
  (AMCL/SLAM + the full Nav2 stack + RViz). Must share the same
  `ROS_DOMAIN_ID` as the Pi so `/scan`, `/odom` and `/tf` are visible over the network.

## Setup

The dev laptop needs the full Nav2 stack (the Pi doesn't — Nav2 runs on the laptop):
```
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

Clone this repo into `~/ros2_ws/src/` on both machines, then on each:

```
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Flash `arduino_firmware/mecanum_agv_firmware/mecanum_agv_firmware.ino` to the Arduino
(Mega 2560 — an Uno doesn't have enough interrupt pins for 4 encoders) after updating
the pin numbers and `TICKS_PER_REV` for your actual wiring/motors.

## Before it will actually work

Everything marked `PLACEHOLDER` in the code must be measured/verified on the real robot:
- Motor + encoder pins and ticks-per-revolution (`arduino_firmware/.../mecanum_agv_firmware.ino`)
- Wheel radius and wheel separation (`config/robot_params.yaml`, must match `urdf/mecanum_agv.urdf.xacro`)
- Mecanum roller orientation: the kinematics assume an X-pattern (front-left/rear-right
  rollers parallel, front-right/rear-left parallel). Wired backwards, the robot spins on
  the spot instead of strafing.
- LiDAR serial port and baud rate (115200 for RPLIDAR A1/A2, 256000 for A3/S1)

## Running

On the Pi:
```
ros2 launch mecanum_agv bringup.launch.py serial_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0
```

On the dev laptop:
```
ros2 launch mecanum_agv slam.launch.py
```

Drive it around with teleop to build the map:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Save the finished map (from the dev laptop, once the area is fully mapped):
```
ros2 run nav2_map_server map_saver_cli -f ~/maps/bic_workspace
```

## Autonomous navigation (Nav2)

Needs a saved map in `maps/map.yaml` (see `maps/README.md`) and the Pi running `bringup.launch.py`.
On the dev laptop:
```
ros2 launch mecanum_agv navigation.launch.py
```
1. AMCL assumes the robot starts at the map origin (where mapping started). If it's
   somewhere else, click **2D Pose Estimate** in RViz and drag where it actually is/faces
   (the yellow particle cloud should converge while driving).
2. Click **2D Goal Pose** anywhere on the map — the robot plans a path (blue) and drives there.

Or drive a fixed route without clicking (edit `config/waypoints.yaml` first):
```
ros2 run mecanum_agv waypoint_mission                      # once
ros2 run mecanum_agv waypoint_mission --ros-args -p loop:=true
```

Navigate while mapping an unknown room (no saved map needed):
```
ros2 launch mecanum_agv navigation.launch.py slam:=true
```

Tuning lives in `config/nav2_params.yaml` — start with the footprint (PLACEHOLDER) and the
speed limits (`vx_max`/`vy_max`/`wz_max` in `FollowPath` **and** `velocity_smoother`).
See ARCHITECTURE.md §9 for the design.

## Required TF tree

```
map -> odom -> base_footprint -> base_link -> laser_frame
```

`map -> odom` comes from slam_toolbox (mapping) or AMCL (localization/navigation), `odom -> base_footprint` from `mecanum_drive_node`
(encoder odometry), the rest from `robot_state_publisher` + the URDF.
