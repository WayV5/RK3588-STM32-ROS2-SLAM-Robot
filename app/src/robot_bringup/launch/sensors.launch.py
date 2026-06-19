"""Real-robot sensor bringup: RPLIDAR A1 + Astra Pro depth.

Usage:
    ros2 launch robot_bringup sensors.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
	# RPLIDAR A1 — 360° on top plate, no body occlusion
	lidar = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('rplidar_ros'),
				'launch', 'rplidar_a1_launch.py')
		),
		launch_arguments={'frame_id': 'laser_frame'}.items(),
	)

	# Astra Pro depth (OpenNI2)
	astra_depth = Node(
		package='astra_camera',
		executable='astra_camera_node',
		name='astra_camera',
		output='screen',
		arguments=['--ros-args',
			'-p', 'enable_color:=false',
			'-p', 'enable_ir:=false',
		],
	)

	return LaunchDescription([
		lidar,
		TimerAction(period=2.0, actions=[astra_depth]),
	])
