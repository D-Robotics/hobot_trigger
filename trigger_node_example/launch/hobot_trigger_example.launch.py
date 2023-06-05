import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    dnn_node_example = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dnn_node_example'),
                'launch/hobot_dnn_node_example.launch.py')),
        launch_arguments={
            'config_file': 'config/ppyoloworkconfig.json',
            'msg_pub_topic_name': 'ai_msg_mono2d_trash_detection',
            'image_width': str(1920),
            'image_height': str(1080)
        }.items()
    )

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

    # jpeg图片编码&发布pkg
    jpeg_compressed_codec_node = Node(
        package='hobot_codec',
        executable='hobot_codec_republish',
        output='screen',
        parameters=[
            {"channel": 1},
            {"in_mode": "shared_mem"},
            {"in_format": "nv12"},
            {"out_mode": "ros"},
            {"out_format": "jpeg-compressed"},
            {"sub_topic": "/hbmem_img"},
            {"pub_topic": "/image_raw/compressed"}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # trigger node 示例节点pkg
    trigger_node_example = Node(
        package='trigger_node_example',
        executable='trigger_node_example',
        output='screen',
        parameters=[
            {"cache_path": "/home/hobot/recorder/"},
            {"config_file": "config/trigger_config.json"},
            {"format": "mcap"},
            {"isRecord": 1},
            {"agent_msg_sub_topic_name": "/hobot_agent"},
            {"event_msg_sub_topic_name": "/ai_msg_mono2d_trash_detection"},
            {"msg_pub_topic_name": "/hobot_trigger"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([
        # dnn_node_example推理示例
        dnn_node_example,
        # recorder node 管理
        recorder_node,
        # jpeg图片编码&发布pkg
        jpeg_compressed_codec_node,
        # trigger node 示例节点pkg
        trigger_node_example,
    ])