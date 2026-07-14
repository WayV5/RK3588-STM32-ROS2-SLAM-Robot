"""Base bringup — chassis + TF + state estimation.

Algorithm stack:
  - robot_can_gateway (自研):      SPSC Ring Buffer + CAN v3 + 四驱运动学
  - robot_state_publisher (开源):  URDF → tf_static
  - ekf_node (开源):              robot_localization Extended Kalman Filter 2D

Usage:
    ros2 launch robot_bringup base.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
	pkg_bringup = get_package_share_directory('robot_bringup')
	xacro_file = os.path.join(pkg_bringup, 'urdf', 'robot.xacro')

	# CAN gateway (STM32 <-> ROS2) — bound to A76 cores 4,5
	can_gateway = Node(
		package='robot_can_gateway',
		executable='can_gateway_node',
		name='can_gateway',
		arguments=['--ros-args', '-p', 'can_interface:=can0'],
		output='screen',
		prefix='taskset -c 4,5',
	)

	# robot_state_publisher: URDF TFs (base_link -> sensors)
	state_pub = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		parameters=[{
			'robot_description': ParameterValue(
				Command(['xacro ', xacro_file]), value_type=str),
			'use_sim_time': False,
		}],
		output='screen',
	)

	# EKF: /odom_raw + /imu -> /odom + odom->base_footprint TF
	# Bound to A76 cores 6,7 with SCHED_FIFO 70
	ekf_node = Node(
		package='robot_localization',
		executable='ekf_node',
		name='ekf_filter_node',
		parameters=[os.path.join(pkg_bringup, 'config', 'ekf.yaml')],
		remappings=[('odometry/filtered', 'odom')],
		output='screen',
		prefix='taskset -c 6,7 chrt -f 70',
	)

	return LaunchDescription([
		can_gateway,
		state_pub,
		ekf_node,
	])
