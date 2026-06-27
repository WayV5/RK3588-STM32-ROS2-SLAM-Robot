"""AI perception — RKNN object detection + optional obstacle costmap injection.

Usage:
    ros2 launch robot_ai ai.launch.py                             # detect only
    ros2 launch robot_ai ai.launch.py enable_obstacle_costmap:=true  # with costmap
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_ai')

    enable_oc = LaunchConfiguration('enable_obstacle_costmap', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_obstacle_costmap', default_value='false',
            description='Enable obstacle_publisher for Nav2 costmap (depth-based)'),

        Node(
            package='robot_ai',
            executable='object_detector_node',
            name='object_detector',
            parameters=[{
                'model_path': os.path.join(pkg_share, 'model', 'yolov8n_fp16_rk3588.rknn'),
                'conf_threshold': 0.5,
                'nms_threshold': 0.45,
                'publish_annotated_image': True,
            }],
            output='screen',
        ),

        Node(
            package='robot_ai',
            executable='obstacle_publisher',
            name='obstacle_publisher',
            condition=IfCondition(enable_oc),
            parameters=[{
                'depth_topic': '/depth/image_raw',
            }],
            output='screen',
        ),
    ])
