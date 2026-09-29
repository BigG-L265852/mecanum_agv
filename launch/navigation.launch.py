"""Full Nav2 stack: autonomous navigation to goals / along waypoints.
Run this ON THE DEV LAPTOP, with ROS_DOMAIN_ID matching the Pi so /scan, /odom,
/tf and /cmd_vel go over the network (bringup.launch.py runs on the Pi).

Two modes:
  slam:=false (default)  localize with AMCL on a saved map (includes localization.launch.py)
  slam:=true             build the map while driving (includes slam.launch.py)

Then give a goal with RViz's "2D Goal Pose" tool, or run a waypoint route with
  ros2 run mecanum_agv waypoint_mission

cmd_vel chain (same as nav2_bringup's navigation_launch.py):
  controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed
  -> collision_monitor -> /cmd_vel -> mecanum_drive_node
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_agv')
    default_map_path = os.path.join(pkg_share, 'maps', 'map.yaml')
    default_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    amcl_params_path = os.path.join(pkg_share, 'config', 'amcl_params.yaml')
    slam_params_path = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'navigation.rviz')

    slam = LaunchConfiguration('slam')
    params_file = LaunchConfiguration('params_file')

    slam_arg = DeclareLaunchArgument(
        'slam', default_value='false',
        description='true: map with slam_toolbox while navigating; false: AMCL on a saved map')
    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map_path,
        description='Full path to the saved map yaml (ignored with slam:=true)')
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params_path,
        description='Full path to the Nav2 parameters yaml')
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file', default_value=slam_params_path,
        description='slam_toolbox parameters for slam:=true '
                    '(mapper_params_no_odom_test.yaml for a LiDAR-only test)')
    open_rviz_arg = DeclareLaunchArgument(
        'open_rviz', default_value='true', description='Launch RViz with the navigation config')

    # Both includes use a launch argument called params_file, like this file does.
    # Launch configurations are shared between parent and included files, in BOTH
    # directions: without an explicit params_file AMCL inherits nav2_params.yaml
    # (no amcl section -> defaults, incl. the diff-drive motion model), and without
    # the scoped GroupAction the include's value leaks back and every Nav2 server
    # below gets amcl_params.yaml. Both fail silently at first.
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': amcl_params_path,
            'open_rviz': 'false',
        }.items(),
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'slam.launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('slam_params_file'),
            'open_rviz': 'false',
        }.items(),
    )
    localization_group = GroupAction(
        [localization_launch], scoped=True, condition=UnlessCondition(slam))
    slam_group = GroupAction([slam_launch], scoped=True, condition=IfCondition(slam))

    def nav2_node(package, executable, remappings=()):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            parameters=[params_file],
            remappings=list(remappings),
        )

    # Order matters only for the lifecycle manager's node_names below: it configures
    # and activates them in that order (same order as nav2_bringup).
    nav2_specs = [
        ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_smoother', 'smoother_server', []),
        ('nav2_planner', 'planner_server', []),
        ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_collision_monitor', 'collision_monitor', []),
        ('nav2_bt_navigator', 'bt_navigator', []),
        ('nav2_waypoint_follower', 'waypoint_follower', []),
    ]
    nav2_nodes = [nav2_node(*spec) for spec in nav2_specs]

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [executable for _, executable, _ in nav2_specs],
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

    # RViz's Map display is broken on this GPU (ros2/rviz#1279, see ARCHITECTURE.md §10),
    # and costmaps are OccupancyGrids too — show them as point clouds instead.
    costmap_clouds = [
        Node(
            package='mecanum_agv',
            executable='map_to_pointcloud',
            name=f'{which}_costmap_to_pointcloud',
            output='screen',
            parameters=[{
                'input_topic': f'/{which}_costmap/costmap',
                'output_topic': f'/{which}_costmap/costmap_points',
                'mode': 'costmap',
                'z_offset': z,
            }],
        )
        for which, z in (('global', 0.01), ('local', 0.02))
    ]

    return LaunchDescription([
        slam_arg,
        map_arg,
        params_arg,
        slam_params_arg,
        open_rviz_arg,
        localization_group,
        slam_group,
        *nav2_nodes,
        lifecycle_manager_node,
        rviz_node,
        *costmap_clouds,
    ])
