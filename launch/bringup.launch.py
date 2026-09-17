"""Brings up robot_state_publisher, the mecanum drive/odom node, and the RPLIDAR.
Run this ON THE RASPBERRY PI (it needs the serial links to the Arduino and the lidar)."""
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
    params_path = os.path.join(pkg_share, 'config', 'robot_params.yaml')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Arduino serial port (motor control + encoders)')
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

    mecanum_drive_node = Node(
        package='mecanum_agv',
        executable='mecanum_drive_node',
        name='mecanum_drive_node',
        output='screen',
        parameters=[params_path, {'serial_port': LaunchConfiguration('serial_port')}],
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

    return LaunchDescription([
        serial_port_arg,
        lidar_port_arg,
        lidar_baudrate_arg,
        robot_state_publisher,
        mecanum_drive_node,
        rplidar_node,
    ])
