"""Runs slam_toolbox (online async) + RViz. Run this ON THE DEV LAPTOP, with
ROS_DOMAIN_ID matching the Pi so /scan, /odom and /tf are visible over the network."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_agv')
    params_path = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

    open_rviz_arg = DeclareLaunchArgument(
        'open_rviz', default_value='true', description='Launch RViz alongside SLAM')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_path],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('open_rviz')),
    )

    return LaunchDescription([open_rviz_arg, slam_toolbox_node, rviz_node])
