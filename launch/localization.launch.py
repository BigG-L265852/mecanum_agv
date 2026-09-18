"""Runs map_server + AMCL (global localization against a saved map) + RViz.
Run this ON THE DEV LAPTOP, with ROS_DOMAIN_ID matching the Pi so /scan,
/odom and /tf are visible over the network.

Requires a map already saved to maps/map.yaml (see maps/README.md) — produced
once with slam.launch.py + map_saver_cli."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_agv')
    default_map_path = os.path.join(pkg_share, 'maps', 'map.yaml')
    default_params_path = os.path.join(pkg_share, 'config', 'amcl_params.yaml')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map_path,
        description='Full path to the saved map yaml')
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params_path,
        description='Full path to the AMCL parameters yaml')
    open_rviz_arg = DeclareLaunchArgument(
        'open_rviz', default_value='true', description='Launch RViz alongside AMCL')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map')}],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('open_rviz')),
    )

    # Same RViz Map-display shader workaround used in slam.launch.py (ros2/rviz#1279).
    map_to_pointcloud_node = Node(
        package='mecanum_agv',
        executable='map_to_pointcloud',
        name='map_to_pointcloud',
        output='screen',
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        open_rviz_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        rviz_node,
        map_to_pointcloud_node,
    ])
