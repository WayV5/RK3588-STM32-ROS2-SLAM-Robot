# Tools — PC ↔ RK3588 常用命令速查

## 部署代码到 RK3588

```bash
# PC: 打包 src/ 并启动 HTTP 服务
cd ~/code/RK3588-STM32-ROS2-SLAM-Robot
tools/serve_app.sh

# RK3588: 拉取源码 (只替换 src/，不动 build/install/)
/root/fetch_app.sh 192.168.0.129:8080

# 编译
cd /app
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /app/install/setup.bash

# 只编译指定包
colcon build --symlink-install \
  --packages-select robot_bringup
```

> 注意: `fetch_app.sh` 只更新 src/，STM32 固件需单独烧录（ST-Link + Keil MDK）。

## 真机启动 (RK3588)

```bash
source /app/install/setup.bash

# 终端1: 底盘 + EKF (can_gateway + robot_state_publisher + EKF)
ros2 launch robot_bringup base.launch.py

# 终端2: 传感器 (RPLIDAR A1 + Astra Pro depth)
ros2 launch robot_bringup sensors.launch.py

# 终端3: SLAM 建图
ros2 launch robot_bringup slam.launch.py
```

## 手柄遥控 (PC 端)

```bash
# 终端1: 手柄 → /joy (100Hz autorepeat)
ros2 run joy joy_node --ros-args -p dev:="/dev/input/js1" \
  -p autorepeat_rate:=100.0 -p coalesce_interval:=0.005

# 终端2: /joy → /cmd_vel
ros2 run teleop_twist_joy teleop_node \
  --ros-args --params-file ~/code/RK3588-STM32-ROS2-SLAM-Robot/tools/betop_gamepad.yaml
```

手柄模式: X (Xbox 360)，按 HOME 切换，X 灯亮。
松 RT = 刹车。

## CAN Gateway (RK3588 端)

```bash
source /app/install/setup.bash
ros2 run robot_can_gateway can_gateway_node
```

## 调试命令 (RK3588)

```bash
# 看里程计
ros2 topic echo /odom --no-arr

# 看 IMU
ros2 topic echo /imu --no-arr

# 看诊断 (帧率/心跳/滴答)
ros2 topic echo /diagnostics --once

# 看 CAN 原始帧
candump can0

# 发单条 CAN 帧
cansend can0 101#6400000064000000     # 0x101 四轮 100mm/s

# 发 /cmd_vel (50Hz 原地转)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 1.0}}" --rate 50

# CAN 接口状态
ip -details link show can0
```

## 编译 (RK3588)

```bash
cd /app
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /app/install/setup.bash
```

## 常用 topic 检查

```bash
ros2 topic list                    # 所有话题
ros2 topic hz /odom                # /odom 发布频率
ros2 topic hz /joy                 # 手柄发布频率
ros2 topic info /cmd_vel           # 话题 QoS 和发布者
```

## 系统服务 (RK3588)

```bash
systemctl status can_gateway       # 查看 CAN Gateway 服务
systemctl stop can_gateway         # 停止 (手动调试时用)
systemctl start can_gateway        # 启动
```

## 网络 (PC ↔ RK3588)

```bash
# PC IP
ip -4 addr show | grep inet

# RK3588 IP (通常 192.168.137.10 有线 / 192.168.0.x WiFi)
ip -4 addr show | grep inet

# 互 ping
ping 192.168.0.129
```

## 标定

### 轮径 (WHEEL_RADIUS)

```bash
# 1. 标记 1m 起点/终点
# 2. 重启节点归零
pkill can_gateway_node && ros2 run robot_can_gateway can_gateway_node &
# 3. 手柄推直线到终点
# 4. 读数 / 1m → 修正 WHEEL_RADIUS *= d_actual / d_odom
ros2 topic echo /odom --once | grep "position:" | grep "x:"
```

### 协方差 (twist.covariance)

```bash
# 直线速度方差 → twist.covariance[0]
tools/calib_linear.sh 0.3

# 角速度方差 → twist.covariance[35]
tools/calib_angular.sh 0.5
```

### 陀螺零偏验证 (RK3588)

```bash
# 静止时 gyro z 应 < 0.005 rad/s
ros2 topic echo /imu --no-arr --once | grep -A3 angular_velocity
```

### 标定结果 (2026-06-20, 陀螺校准后)

| 参数 | 值 | 方法 |
|------|-----|------|
| gyro_bias | STM32 启动 200 帧采样 | 静止 1s, Gz 0.6→<0.2°/s |
| WHEEL_BASE | 0.275m | 卷尺实测 (±3mm) |
| WHEEL_RADIUS | 0.0323m | 直线 1m×3 平均 (复验有效) |
| WHEEL_BALANCE | **0.001** | ~~0.003 被陀螺零偏污染~~ 陀螺校准后 1m×3 yaw≈-0.21° |
| twist.cov[0] | 0.0226 | 0.3m/s 方差 (复验有效) |
| twist.cov[35] | 0.0613 | 0.5rad/s 方差 (复验有效) |

> 旧 BALANCE=0.003 在用轮速差对冲陀螺假旋转信号，陀螺校准后仅需 0.001。

## Astra Pro 相机 (RK3588)

Astra Pro 深度走 OpenNI2，彩色走 UVC，需两个驱动分开跑。

```bash
# 深度 (OpenNI2, 640x480@30Hz)
ros2 launch astra_camera astra.launch.xml enable_color:=false enable_ir:=false &

# 彩色 (UVC, 640x480@30Hz, YUYV)
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:="/dev/video41" \
  -p pixel_format:="YUYV" \
  -p image_size:="[640,480]" &
```

### 调试

```bash
# 查看 Astra Pro USB 设备
lsusb | grep -i orbbec        # 2bc5:0401 (深度) + 2bc5:0501 (彩色)
# 或
cat /sys/bus/usb/devices/*/idVendor | sort -u | grep 2bc5

# 查找彩色 UVC 设备
v4l2-ctl --list-devices | grep -A2 "Astra"

# 查看 topic
ros2 topic list | grep -E "color|depth|image"
ros2 topic hz /camera/depth/image_raw    # 深度 ~30Hz
ros2 topic hz /image_raw                 # 彩色 ~30Hz

# 看一帧
ros2 topic echo /camera/depth/image_raw --qos-reliability reliable --once
ros2 topic echo /image_raw --qos-reliability reliable --once

# 检查厂商标定
ros2 topic echo /camera/depth/camera_info --once | grep -E "k:|d:"
```

### 已知问题

- **IR/Color 互斥**: Astra Pro 的 IR 和 Color 共用一个传感器，不能同时开。关 IR 才能出 Color。
- **MJPG 不支持**: v4l2_camera 的 MJPG 解码有问题，必须用 YUYV 格式。
- **需两个驱动**: 深度走 OpenNI2 (astra_camera)，彩色走 UVC (v4l2_camera)，不能一个节点搞定。

## RPLIDAR A1 (RK3588)

```bash
# 确认串口
ls /dev/ttyUSB0

# 验证
ros2 topic hz /scan    # ~8.8Hz
ros2 topic echo /scan --no-arr --once | grep frame_id  # 应为 'laser'
```

> frame_id 默认 `laser`，与 URDF link 名一致。内核需开启 `CONFIG_USB_SERIAL_CH341=y`。

## TF 调试 (RK3588)

```bash
# 验证 TF 树: map → odom → base_footprint → base_link → laser/camera/imu
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom laser
```

## Gazebo 仿真 (PC)

### 依赖

```bash
sudo apt install ros-humble-nav2-bringup
```

### 编译

```bash
cd ~/code/RK3588-STM32-ROS2-SLAM-Robot/app
colcon build --symlink-install --packages-select robot_sim
source install/setup.bash
```

### SLAM 建图

```bash
# 启动仿真 + SLAM
ros2 launch robot_sim simulation.launch.py

# 键盘遥控 (i前进 ,后退 j左转 l右转)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 手柄遥控
ros2 run joy joy_node --ros-args -p dev:="/dev/input/js1"
ros2 run teleop_twist_joy teleop_node \
  --ros-args --params-file tools/betop_gamepad.yaml
```

走一圈覆盖所有区域，最后回到起点触发回环。

### 保存地图

```bash
ros2 run nav2_map_server map_saver_cli -f ~/sim_map
# 生成 ~/sim_map.yaml + ~/sim_map.pgm
```

### Nav2 导航 (加载已保存地图)

```bash
# 终端1: 仿真 (关闭 SLAM, Nav2 自带 AMCL)
ros2 launch robot_sim simulation.launch.py slam:=false

# 终端2: Nav2 (map_server + AMCL + planner + controller)
ros2 launch nav2_bringup bringup_launch.py \
  map:=/home/ww/sim_map.yaml \
  use_sim_time:=true \
  slam:=False

# 终端3: Rviz
rviz2
```

**Rviz 操作:**

1. Fixed Frame → `map`
2. Add → `/map` `/scan` `/plan` `/local_plan`
3. 工具栏 "2D Pose Estimate"(紫色箭头) → 地图上拖箭头给初始位姿
4. 工具栏 "Nav2 Goal"(红色旗帜) → 地图上拖箭头选目标

> Goal 拖拽不触发时用命令行:
> `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"`

### 验证

```bash
ros2 action list | grep nav
ros2 topic echo /cmd_vel --once     # Nav2 输出给机器人
ros2 topic echo /plan --once 2>&1 | head -5
```

### TF 调试

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map laser
```

| 文件 | 用途 | 位置 |
|------|------|------|
| `robot.xacro` | 核心 URDF (真机/仿真共用) | RK3588/PC |
| `robot_sim.xacro` | 仿真 wrapper (=robot.xacro+robot.gazebo) | PC |
| `robot.gazebo` | Gazebo 插件+材质+摩擦 | PC |
| `robot.sdf` | 旧 SDF 模型 (已废弃) | — |
