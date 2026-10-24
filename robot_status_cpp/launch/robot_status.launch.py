"""Start turtlesim, the fake status sensor and the drive-to-point action server.

Usage:
  ros2 launch robot_status_cpp robot_status.launch.py
  ros2 launch robot_status_cpp robot_status.launch.py publish_rate_hz:=5.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('robot_status_cpp'), 'config', 'params.yaml')

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz', default_value='1.0',
        description='Rate at which /robot_status is published')

    # Launch arguments are strings; ParameterValue makes sure the node receives a double.
    publish_rate = ParameterValue(LaunchConfiguration('publish_rate_hz'), value_type=float)

    return LaunchDescription([
        publish_rate_arg,
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim'),
        Node(
            package='robot_status_cpp',
            executable='status_publisher',
            name='status_publisher',
            parameters=[params_file, {'publish_rate_hz': publish_rate}],
            output='screen',
        ),
        Node(
            package='robot_status_cpp',
            executable='drive_to_point_server',
            name='drive_to_point_server',
            parameters=[params_file],
            output='screen',
        ),
    ])
