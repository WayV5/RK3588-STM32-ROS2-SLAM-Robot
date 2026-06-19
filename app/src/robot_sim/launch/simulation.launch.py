"""Launch Gazebo simulation with URDF model + SLAM mapping.

Usage:
    ros2 launch robot_sim simulation.launch.py            # simulation + SLAM
    ros2 launch robot_sim simulation.launch.py slam:=false  # simulation only (for Nav2)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
	pkg_dir = get_package_share_directory('robot_sim')
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

	# SLAM mapping (run by default, slam:=false to disable)
	slam = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('slam_toolbox'),
				'launch', 'online_async_launch.py')
		),
		launch_arguments={
			'slam_params_file': os.path.join(pkg_dir, 'config', 'mapper_params_sim.yaml'),
			'use_sim_time': 'true',
		}.items(),
		condition=IfCondition(LaunchConfiguration('slam')),
	)

	return LaunchDescription([
		DeclareLaunchArgument('slam', default_value='true',
			description='set to false to disable SLAM (e.g. when using Nav2)'),
		gazebo,
		state_pub,
		TimerAction(period=2.0, actions=[spawn]),
		TimerAction(period=4.0, actions=[slam]),
	])
