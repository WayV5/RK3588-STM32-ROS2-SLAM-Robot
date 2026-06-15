"""Launch Gazebo simulation with SDF model (no ros2_control)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
	pkg_dir = get_package_share_directory('robot_description')

	gazebo = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('gazebo_ros'),
				'launch', 'gazebo.launch.py')
		),
		launch_arguments={
			'world': os.path.join(pkg_dir, 'worlds', 'empty.world')
		}.items(),
	)

	spawn = Node(
		package='gazebo_ros',
		executable='spawn_entity.py',
		arguments=['-entity', 'diff_drive_robot',
			'-file', os.path.join(pkg_dir, 'urdf', 'robot.sdf')],
		output='screen',
	)

	return LaunchDescription([
		gazebo,
		TimerAction(period=2.0, actions=[spawn]),
	])
