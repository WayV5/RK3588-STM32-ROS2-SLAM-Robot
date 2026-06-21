"""Nav2 bringup on real robot — AMCL + planner + controller.

Usage:
    ros2 launch robot_bringup navigation.launch.py
(map path hardcoded in nav2_params.yaml)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
	pkg_bringup = get_package_share_directory('robot_bringup')

	nav2 = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('nav2_bringup'),
				'launch', 'bringup_launch.py')
		),
		launch_arguments={
			'params_file': os.path.join(pkg_bringup, 'config', 'nav2_params.yaml'),
			'map': '/app/src/robot_bringup/maps/map.yaml',
			'use_sim_time': 'false',
		}.items(),
	)

	return LaunchDescription([nav2])
