"""Sensor bringup — RPLIDAR A1 + Astra Pro.

Algorithm stack:
  - robot_rplidar (自研: SLAMTEC SDK v2.1.0): Sensitivity scan + angle_compensate
  - astra_camera (开源: OpenNI2):             depth stream

Usage:
    ros2 launch robot_bringup sensors.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
	pkg_bringup = get_package_share_directory('robot_bringup')

	# RPLIDAR A1 — 360° on top plate, no body occlusion
	# SLAMTEC official SDK (robot_rplidar), Sensitivity mode, angle_compensate
	lidar = Node(
		package='robot_rplidar',
		executable='rplidar_node',
		name='rplidar_node',
		parameters=[{
			'channel_type': 'serial',
			'serial_port': '/dev/ttyUSB0',
			'serial_baudrate': 115200,
			'frame_id': 'laser',
			'angle_compensate': True,
			'scan_mode': 'Sensitivity',
			'scan_frequency': 10.0,
		}],
		output='screen',
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
