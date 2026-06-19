"""Nav2 bringup on real robot — AMCL + planner + controller.

Prerequisite: map saved (ros2 run nav2_map_server map_saver_cli -f ~/map)
Usage:
    ros2 launch robot_bringup navigation.launch.py map:=/home/ww/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
	pkg_bringup = get_package_share_directory('robot_bringup')

	nav2 = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('nav2_bringup'),
				'launch', 'bringup_launch.py')
		),
		launch_arguments={
			'params_file': os.path.join(pkg_bringup, 'config', 'nav2_params.yaml'),
			'map': LaunchConfiguration('map'),
			'use_sim_time': 'false',
		}.items(),
	)

	return LaunchDescription([
		DeclareLaunchArgument('map', description='Path to map YAML file'),
		nav2,
	])
