"""Runs slam_toolbox (online async) + RViz. Run this ON THE DEV LAPTOP, with
ROS_DOMAIN_ID matching the Pi so /scan, /odom and /tf are visible over the network."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_agv')
    params_path = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    open_rviz_arg = DeclareLaunchArgument(
        'open_rviz', default_value='true', description='Launch RViz alongside SLAM')

    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[params_path],
    )

    # async_slam_toolbox_node is a lifecycle node: without a lifecycle manager it stays
    # "unconfigured" and never subscribes to /scan, so drive it through configure -> activate
    # ourselves (same pattern as slam_toolbox's own online_async_launch.py).
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('open_rviz')),
    )

    return LaunchDescription([
        open_rviz_arg,
        slam_toolbox_node,
        configure_event,
        activate_event,
        rviz_node,
    ])
