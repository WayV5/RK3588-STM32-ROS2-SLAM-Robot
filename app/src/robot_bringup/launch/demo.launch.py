"""Interview demo — 一键启动全栈自主导航.

Algorithm stack (one launch, full autonomous pipeline):
  - robot_can_gateway (自研):      CAN↔ROS2 SPSC Ring Buffer
  - robot_state_publisher (开源):  URDF → static TF
  - ekf_node (开源):              EKF 2D sensor fusion
  - robot_rplidar (自研 SDK v2.1): /scan
  - astra_camera (开源 OpenNI2):  /depth/image_raw
  - nav2_bringup (开源):
      amcl (粒子滤波) + Navfn (Dijkstra) + DWB (动态窗口) + BT.CPP (行为树)
  - object_detector_node (自研):
      YOLOv8n FP16 NPU 推理 → /detections + /detection_image

  Timing: base(t=0) → sensors(t=3s: lidar+深度, t=5s: RGB) → nav(t=8s) → ai(t=12s: 等RGB就绪)

Usage:
    ros2 launch robot_bringup demo.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
	pkg_bringup = get_package_share_directory('robot_bringup')
	pkg_ai = get_package_share_directory('robot_ai')

	base = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_bringup, 'launch', 'base.launch.py')
		)
	)

	sensors = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_bringup, 'launch', 'sensors.launch.py')
		)
	)

	nav = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_bringup, 'launch', 'navigation.launch.py')
		)
	)

	ai = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ai, 'launch', 'ai.launch.py')
		)
	)

	return LaunchDescription([
		base,
		TimerAction(period=3.0, actions=[sensors]),
		TimerAction(period=8.0, actions=[nav]),
		TimerAction(period=12.0, actions=[ai]),
	])
