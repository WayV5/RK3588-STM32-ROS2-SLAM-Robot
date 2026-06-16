"""Launch Gazebo simulation with URDF model + SLAM (no ros2_control)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
	pkg_dir = get_package_share_directory('robot_description')
	xacro_file = os.path.join(pkg_dir, 'urdf', 'robot_sim.xacro')

	gazebo = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('gazebo_ros'),
				'launch', 'gazebo.launch.py')
		),
		launch_arguments={
			'world': os.path.join(pkg_dir, 'worlds', 'empty.world')
		}.items(),
	)

	state_pub = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		parameters=[{
			'robot_description': ParameterValue(
				Command(['xacro ', xacro_file]), value_type=str),
			'use_sim_time': True,
		}],
	)

	spawn = Node(
		package='gazebo_ros',
		executable='spawn_entity.py',
		arguments=['-entity', 'diff_drive_robot',
			'-topic', 'robot_description', '-z', '0.05'],
		output='screen',
	)

	slam = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('slam_toolbox'),
				'launch', 'online_async_launch.py')
		),
		launch_arguments={
			'slam_params_file': os.path.join(pkg_dir, 'config', 'mapper_params_sim.yaml'),
			'use_sim_time': 'true',
		}.items(),
	)

	return LaunchDescription([
		gazebo,
		state_pub,
		TimerAction(period=2.0, actions=[spawn]),
		TimerAction(period=4.0, actions=[slam]),
	])
