"""Nav2 navigation — AMCL + Navfn + DWB + BT.CPP.

Algorithm stack (all open-source, ros-humble-navigation2):
  - map_server:       静态地图加载
  - amcl:             粒子滤波定位 (likelihood_field + KLD采样 + recovery_alpha)
  - planner_server:   Navfn Dijkstra 全局路径
  - controller_server: DWB (Dynamic Window Based) 局部轨迹跟踪
  - smoother_server:  B样条路径平滑
  - velocity_smoother: cmd_vel 加速度限幅
  - bt_navigator:     BT.CPP 行为树调度
  - behavior_server:  spin/backup/wait 恢复行为

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
