import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # recorder node 示例节点pkg
    recorder_node = Node(
        package='recorder_node',
        executable='recorder_node',
        output='screen',
        parameters=[
            {"cache_path": "/home/hobot/recorder/"},
            {"cache_time": 60000},
            {"cycle_time": 60},
            {"format": "mcap"},
            {"mag_bag_size": 524288000},
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([
        # recorder node 示例节点pkg
        recorder_node,
    ])