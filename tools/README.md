# Tools — PC ↔ RK3588 常用命令速查

## 部署代码到 RK3588

```bash
# PC: 打包 src/ 并启动 HTTP 服务
cd ~/code/RK3588-STM32-ROS2-SLAM-Robot
tools/serve_app.sh

# RK3588: 下载 → 校验 → 编译 → 重启服务
/root/fetch_app.sh 192.168.0.129:8080
```

## 手柄遥控 (PC 端)

```bash
# 终端1: 手柄 → /joy (100Hz autorepeat)
ros2 run joy joy_node --ros-args -p dev:="/dev/input/js1" \
  -p autorepeat_rate:=100.0 -p coalesce_interval:=0.005

# 终端2: /joy → /cmd_vel
ros2 run teleop_twist_joy teleop_node \
  --ros-args --params-file tools/betop_gamepad.yaml
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

## 轮径/轮距标定

```bash
# 重启节点 → odom 归零
pkill can_gateway_node && ros2 run robot_can_gateway can_gateway_node &

# 起点
ros2 topic echo /odom --once | grep "x:"

# ... 推直线 1m ...

# 终点 (读数 / 实际距离 → 修正 WHEEL_RADIUS)
ros2 topic echo /odom --once | grep "x:"
```
