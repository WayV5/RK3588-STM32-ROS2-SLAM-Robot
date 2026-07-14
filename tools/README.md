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
colcon build
source /app/install/setup.bash

# 只编译指定包
colcon build \
  --packages-select robot_bringup
```

> 注意: `fetch_app.sh` 只更新 src/，STM32 固件需单独烧录（ST-Link + Keil MDK）。

## 真机启动 (RK3588)

```bash
source /app/install/setup.bash

# ★ 一键 demo 
ros2 launch robot_bringup demo.launch.py

# --- 或分步启动 ---
# 终端1: 底盘 + EKF
ros2 launch robot_bringup base.launch.py

# 终端2: 传感器
ros2 launch robot_bringup sensors.launch.py

# 终端3: SLAM 建图
ros2 launch robot_bringup slam.launch.py

# 终端3: 导航 (map 已内置)
ros2 launch robot_bringup navigation.launch.py

# 终端4：启动ai 推理
ros2 launch robot_ai ai.launch.py
```

## 快速杀进程

```bash
bash /app/src/tools/kill_ros2.sh
```

## 导航控制

```bash
# 设初始位姿 (AMCL 丢失时)
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}},
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]
  }
}" --once

# 发导航目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}
  }
}" --feedback

# 取消导航
ros2 action cancel /navigate_to_pose

# Rviz2: Fixed Frame=map, Map Durability=Transient Local
```

## 巡航脚本 (PC 端执行)

```bash
cd ~/code/RK3588-STM32-ROS2-SLAM-Robot/tools

# 跑一圈 4 个途经点
python3 patrol.py

# 无限循环
python3 patrol.py --loop 
```

途经点坐标见 `patrol.py` 内 WAYPOINTS 定义。PC 端需安装 `ros-humble-navigation2`。

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

## 实时优先级授权 (首次部署必做)

`can_gateway_node` 内部调用 `pthread_setschedparam(SCHED_FIFO)` 设置线程实时优先级，
`ekf_node` 通过 launch prefix `chrt -f` 设置。这两个操作都需要 `CAP_SYS_NICE` 权限，
否则会降级为普通 SCHED_OTHER 调度。

```bash
# 每次 colcon build 后需重新执行 (install 目录下的二进制被覆盖)
sudo setcap cap_sys_nice=ep /app/lib/robot_can_gateway/can_gateway_node
sudo setcap cap_sys_nice=ep /opt/ros/humble/lib/robot_localization/ekf_node

# 验证
getcap /app/lib/robot_can_gateway/can_gateway_node
# 应输出: /app/lib/robot_can_gateway/can_gateway_node cap_sys_nice=ep

getcap /opt/ros/humble/lib/robot_localization/ekf_node
# 应输出: /opt/ros/humble/lib/robot_localization/ekf_node cap_sys_nice=ep
```

> `=ep`: e=进程启动后立即可用, p=允许加入 effective 集合。
> 比 `sudo` 或 `chmod u+s` 更安全——**只给这一个权限，遵循最小权限原则**。

**优先级层级** (SCHED_FIFO 数值越大优先级越高):

| 线程 | 调度策略 | 优先级 | 设置方式 |
|------|---------|:---:|---------|
| CAN read thread | SCHED_FIFO | 80 | `pthread_setschedparam` (代码内) |
| CAN executor thread | SCHED_FIFO | 75 | `pthread_setschedparam` (代码内) |
| EKF node | SCHED_FIFO | 70 | `chrt -f` (launch prefix) |
| IRQ threads | SCHED_FIFO | 50 | PREEMPT_RT 自动 |
| Nav2/SLAM/NPU/... | SCHED_OTHER | 0 | 默认 |

```bash
# 运行时确认线程优先级
ps -eL -o tid,cls,rtprio,comm | grep -E "can_gateway|ekf_node"
# cls=FF = SCHED_FIFO, rtprio=优先级数字
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
colcon build
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
| WHEEL_BALANCE | **0.005** | ~~0.001~~ 地面打滑肉眼标定 (B=0.004偏左 B=0.006偏右→0.005) |
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
ros2 topic hz /depth/image_raw    # 深度 ~30Hz
ros2 topic hz /image_raw                 # 彩色 ~30Hz

# 看一帧
ros2 topic echo /depth/image_raw --qos-reliability reliable --once
ros2 topic echo /image_raw --qos-reliability reliable --once

# 检查厂商标定
ros2 topic echo /camera/depth/camera_info --once | grep -E "k:|d:"
```

### 已知问题

- **IR/Color 互斥**: Astra Pro 的 IR 和 Color 共用一个传感器，不能同时开。关 IR 才能出 Color。
- **MJPG 不支持**: v4l2_camera 的 MJPG 解码有问题，必须用 YUYV 格式。
- **需两个驱动**: 深度走 OpenNI2 (astra_camera)，彩色走 UVC (v4l2_camera)，不能一个节点搞定。

## RPLIDAR A1 (RK3588)

**驱动**: SLAMTEC 官方 SDK `robot_rplidar` (v2.1.0)，非 apt 的 `rplidar_ros`。
Sensitivity 模式 + angle_compensate + HQ scan + CRC32 校验。

```bash
# 确认串口
ls /dev/ttyUSB0

# 单跑验证
ros2 run robot_rplidar rplidar_node --ros-args \
  -p serial_port:="/dev/ttyUSB0" \
  -p serial_baudrate:=115200 \
  -p frame_id:="laser" \
  -p angle_compensate:=true \
  -p scan_mode:="Sensitivity" \
  -p scan_frequency:=10.0

# 验证
ros2 topic hz /scan    # ~8.6Hz
ros2 topic echo /scan --no-arr --once | grep frame_id  # 应为 'laser'
```

> frame_id `laser`，与 URDF link 名一致。内核需开启 `CONFIG_USB_SERIAL_CH341=y`。
> 旧 apt 驱动 (`rplidar_ros`) 已废弃，建图质量差是**驱动问题**。

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
colcon build --packages-select robot_sim
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
