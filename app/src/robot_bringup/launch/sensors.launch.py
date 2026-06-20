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
	pkg_bringup = get_package_share_directory('robot_bringup')

	# RPLIDAR A1 — 360° on top plate, no body occlusion
	# frame_id defaults to 'laser', matches URDF link name
	lidar = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('rplidar_ros'),
				'launch', 'rplidar_a1_launch.py')
		),
	)

	# Astra Pro depth (OpenNI2) — YAML params (--ros-args -p incompatible)
	astra_depth = Node(
		package='astra_camera',
		executable='astra_camera_node',
		name='astra_camera',
		parameters=[os.path.join(pkg_bringup, 'config', 'astra_params.yaml')],
		output='screen',
	)

	return LaunchDescription([
		lidar,
		TimerAction(period=2.0, actions=[astra_depth]),
	])
