"""AI perception — RKNN object detection.

Usage:
    ros2 launch robot_ai ai.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_ai')
    return LaunchDescription([
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
            parameters=[{
                'depth_topic': '/camera/depth/image_raw',
            }],
            output='screen',
        ),
    ])
