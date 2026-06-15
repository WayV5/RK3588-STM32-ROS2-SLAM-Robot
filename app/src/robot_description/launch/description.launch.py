"""Launch robot_state_publisher with URDF xacro model."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
	pkg_dir = get_package_share_directory('robot_description')
	xacro_file = os.path.join(pkg_dir, 'urdf', 'robot.xacro')

	return LaunchDescription([
		DeclareLaunchArgument('use_sim_time', default_value='false',
			description='Use simulation (Gazebo) clock if true'),

		Node(
			package='robot_state_publisher',
			executable='robot_state_publisher',
			name='robot_state_publisher',
			parameters=[{
				'robot_description': ParameterValue(
					Command(['xacro ', xacro_file]), value_type=str),
				'use_sim_time': LaunchConfiguration('use_sim_time'),
			}],
			output='screen',
		),
	])
