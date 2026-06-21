"""SLAM mapping — slam_toolbox (Karto SPA 图优化 + scan-to-map).

Algorithm: Karto Sparse Pose Adjustment
  - 每帧 laser scan 与已有子图 scan-to-map 配准
  - 超过距离/角度阈值 → 新增子图节点
  - 回环检测 → SPA 全局图优化

Usage:
    ros2 launch robot_bringup slam.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
	pkg_bringup = get_package_share_directory('robot_bringup')

	slam = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('slam_toolbox'),
				'launch', 'online_async_launch.py')
		),
		launch_arguments={
			'slam_params_file': os.path.join(pkg_bringup, 'config',
				'mapper_params_real.yaml'),
			'use_sim_time': 'false',
		}.items(),
	)

	return LaunchDescription([slam])
