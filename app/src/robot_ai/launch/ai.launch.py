"""AI perception — RKNN object detection.

Usage:
    ros2 launch robot_ai ai.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_ai',
            executable='object_detector_node',
            name='object_detector',
            parameters=[{
                'model_path': 'model/yolov8n_fp16.rknn',
                'conf_threshold': 0.5,
                'nms_threshold': 0.45,
                'publish_annotated_image': True,
            }],
            output='screen',
        ),
    ])
