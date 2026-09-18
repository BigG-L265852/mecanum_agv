"""Bringup for testing SLAM/AMCL with only a LiDAR on the dev laptop — no Arduino,
no mecanum_drive_node, no Pi. Use this while the robot chassis isn't built yet.

Publishes a static, never-changing odom->base_footprint transform, since there is
no wheel encoder odometry source. This is fine for slam.launch.py run with
config/mapper_params_no_odom_test.yaml (which ignores odom deltas and relies on
scan-to-scan correlation instead), and for a stationary AMCL smoke test, but it is
NOT real odometry: carrying the laptop around does not update this transform, so
anything that trusts it for motion prediction (AMCL's motion model) will not track
movement between updates."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_agv')
    xacro_path = os.path.join(pkg_share, 'urdf', 'mecanum_agv.urdf.xacro')

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='RPLIDAR serial port')
    lidar_baudrate_arg = DeclareLaunchArgument(
        'lidar_baudrate', default_value='115200',
        description='RPLIDAR baud rate: 115200 for A1/A2, 256000 for A3/S1')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', xacro_path]), value_type=str)}],
    )

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': LaunchConfiguration('lidar_baudrate'),
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    # Stand-in for the odom->base_footprint transform mecanum_drive_node normally
    # publishes from encoder feedback. Identity and static — see module docstring.
    fake_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fake_odom_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
    )

    return LaunchDescription([
        lidar_port_arg,
        lidar_baudrate_arg,
        robot_state_publisher,
        rplidar_node,
        fake_odom_tf,
    ])
