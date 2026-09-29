#!/usr/bin/env python3
"""Drives a fixed route autonomously: sends the waypoints from a yaml file to Nav2.

Requires navigation.launch.py to be running and (with AMCL) the robot to be
localized — set "2D Pose Estimate" in RViz first, or pass initial_pose:=true to
publish the pose from the waypoints file.

    ros2 run mecanum_agv waypoint_mission
    ros2 run mecanum_agv waypoint_mission --ros-args -p loop:=true \\
        -p waypoints_file:=/path/to/waypoints.yaml
"""
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def _pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator(node_name='waypoint_mission')

    default_file = os.path.join(
        get_package_share_directory('mecanum_agv'), 'config', 'waypoints.yaml')
    navigator.declare_parameter('waypoints_file', default_file)
    navigator.declare_parameter('loop', False)
    navigator.declare_parameter('initial_pose', False)
    waypoints_file = navigator.get_parameter('waypoints_file').value
    loop = navigator.get_parameter('loop').value
    set_initial_pose = navigator.get_parameter('initial_pose').value

    with open(waypoints_file) as f:
        config = yaml.safe_load(f)
    waypoints = config['waypoints']
    navigator.info(f'Loaded {len(waypoints)} waypoints from {waypoints_file}')

    if set_initial_pose:
        start = config['initial_pose']
        navigator.setInitialPose(_pose(navigator, start['x'], start['y'], start.get('yaw', 0.0)))
        # Keeps re-publishing that pose until AMCL confirms it.
        navigator.waitUntilNav2Active(localizer='amcl')
    else:
        # Wait on planner_server rather than amcl: BasicNavigator's amcl check keeps
        # publishing its own default (0, 0) initial pose until AMCL answers, which would
        # overwrite a pose already set in RViz. Also works with slam:=true (no AMCL).
        navigator.waitUntilNav2Active(localizer='planner_server')

    try:
        while True:
            poses = [_pose(navigator, w['x'], w['y'], w.get('yaw', 0.0)) for w in waypoints]
            navigator.followWaypoints(poses)
            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    navigator.get_logger().info(
                        f'Driving to waypoint {feedback.current_waypoint + 1}/{len(poses)}',
                        throttle_duration_sec=2.0)
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                navigator.info('Route finished')
            elif result == TaskResult.CANCELED:
                navigator.warn('Route was canceled')
                break
            else:
                navigator.error('Route failed — check Nav2 output (localized? goal reachable?)')
                break
            if not loop:
                break
    except KeyboardInterrupt:
        navigator.cancelTask()
    finally:
        navigator.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
