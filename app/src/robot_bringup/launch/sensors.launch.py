"""Real-robot sensor bringup: RPLIDAR A1 + Astra Pro (depth + color).

Usage:
    ros2 launch robot_bringup sensors.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
	# RPLIDAR A1
	lidar = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('rplidar_ros'),
				'launch', 'rplidar_a1_launch.py')
		)
	)

	# Astra Pro depth (OpenNI2)
	astra_depth = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('astra_camera'),
				'launch', 'astra.launch.xml')
		),
		launch_arguments={
			'enable_color': 'false',
			'enable_ir': 'false',
		}.items(),
	)

	# Astra Pro color (UVC, YUYV)
	astra_color = Node(
		package='v4l2_camera',
		executable='v4l2_camera_node',
		name='v4l2_camera',
		arguments=['--ros-args', '-p', 'video_device:=/dev/video41',
			'-p', 'pixel_format:=YUYV',
			'-p', 'image_size:=[640,480]'],
		output='screen',
	)

	return LaunchDescription([
		lidar,
		astra_depth,
		astra_color,
	])
