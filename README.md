# RK3588-STM32-ROS2-SLAM-Robot

基于 RK3588 + STM32F407 异构架构的 ROS2 全栈自主导航机器人

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![FreeRTOS](https://img.shields.io/badge/FreeRTOS-10.3.1-green)](https://www.freertos.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)
[![PREEMPT_RT](https://img.shields.io/badge/Kernel-PREEMPT__RT-red)](https://wiki.linuxfoundation.org/realtime/)
[![CAN](https://img.shields.io/badge/CAN-500kbps-lightgrey)](#)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

> 从零设计并实现的四驱差速自主导航机器人 — 独立完成硬件选型采购、通信总线调试、STM32 FreeRTOS 实时固件开发、RK3588 Linux BSP 与 PREEMPT_RT 实时优化、ROS2 导航栈与 NPU AI 推理部署。

---

## 硬件拓扑

```
┌─────────────────────────────────────────────────────┐
│                   ATK-DLRK3588B 主控板              │
│  CPU: 4×A76@2.4GHz + 4×A55@1.8GHz  │  NPU: 6 TOPS   │
│  OS: Ubuntu 22.04 + ROS2 Humble + Cyclone DDS       │
└──┬───────┬──────────┬────────────────┬──────────────┘
   │USB    │USB       │CAN1 (原生)     │WiFi
   ▼       ▼          ▼                ▼
┌───────┐ ┌────────┐ ┌─────────┐    ┌────────┐
│ Astra │ │RPLIDAR │ │TPT1051V │    │ 远程PC  │
│ Pro   │ │A1      │ │CAN 收发器│    │ Rviz2  │
│ RGB-D │ │2D LiDAR│ └────┬─────┘   │ 监控    │
└───────┘ └────────┘      │         └────────┘
                          │ CAN Bus (500 kbps)
                          ▼
                    ┌─────────────┐
                    │  TJA1050    │
                    │  CAN 收发器  │
                    └──────┬──────┘
                           ▼
         ┌─────────────────────────────────┐
         │      STM32F407VET6 (MCU)        │
         │   Cortex-M4 @168MHz             │
         │   实时控制 & 传感器采集           │
         └──┬───────┬──────────┬───────────┘
            │       │          │
    ┌───────┘       │          └──────────┐
    ▼               ▼                     ▼
┌────────┐    ┌───────────┐    ┌──────────────┐
│TB6612  │    │ MPU9250   │    │ 4× 编码器电机 │
│四路驱动 │    │ 9轴 IMU   │    │ M1 M2 M3 M4  │
│PWM×4   │    │ I2C 400kHz│    │ 直流减速 30:1 │
└──┬──┬──┘    └───────────┘    └──────────────┘
   │  │
   ▼  ▼
┌──────────────┐
│ 四驱差速底盘  │
│ LB LF RF RB  │
└──────────────┘
```

---

## 核心特性

- 🤖 **异构架构** — RK3588 (ROS2 + NPU) + STM32F407 (FreeRTOS) 通过 CAN 500kbps 实时通信
- 🗺️ **自主导航** — slam_toolbox 激光 SLAM 建图 + Navigation2 (AMCL 定位 + DWB 局部规划)
- ⚡ **硬实时控制** — STM32 1kHz PID 四轮速度闭环，前馈补偿 + 积分分离/钳位/抗饱和
- 👁️ **AI 视觉推理** — YOLOv8n RKNN 部署于 RK3588 NPU (6 TOPS)，CPU 算力留给导航
- 🔧 **Linux 实时性优化** — PREEMPT_RT 内核 + CPU 隔离 + IRQ 亲和性，cyclictest 延迟改善 **33 倍** (2809µs → 84µs)
- 🔌 **全栈自研** — 独立完成硬件选型/采购/接线/调试，CAN/I2C/USB 总线全部独立调通
- 📡 **CAN 协议 v3** — 1kHz 8-slot 非阻塞发送调度 + 硬件 Filter Bank 过滤 + BUS-OFF 自动恢复
- 🎮 **仿真支持** — Gazebo + URDF，真机/仿真 topic 接口一致，参数可复用

---

## 硬件规格

| 模块 | 型号 | 接口 | 说明 |
|------|------|------|------|
| **主控** | ATK-DLRK3588B | — | 4×A76 + 4×A55, 6 TOPS NPU, 8/16GB LPDDR4x |
| **MCU** | STM32F407VET6 | CAN + I2C + GPIO | Cortex-M4 @168MHz, FreeRTOS |
| **激光雷达** | SLAMTEC RPLIDAR A1 | UART→USB | 360° 2D 扫描, 8.8Hz |
| **深度相机** | 奥比中光 Astra Pro | USB 2.0 | RGB-D, 640×480@30Hz |
| **IMU** | MPU9250 | I2C (400kHz) | 9 轴 (加速度计+陀螺仪+磁力计) |
| **电机驱动** | TB6612 | PWM + GPIO | 四路 H 桥, 最大 1.2A/路 |
| **编码器电机** | 520 编码器电机 ×4 | HX254 | 线数 11, 减速比 ~30:1 |
| **CAN 收发器** | TJA1050 + TPT1051V | CAN | STM32 侧 + RK3588 板载 |
| **电池** | 12V 10A 21000mAh | XT60 | 系统总电源 |
| **降压模块** | LM2596 | — | 12V→5V/1.8V 多路输出 |

---

## 软件架构

```
┌──────────────────────────────────────────────────────────────┐
│                      应用层 (ROS2 Nodes)                      │
│                                                              │
│  ┌─────────────────┐  ┌─────────────────┐                    │
│  │   导航与感知     │  │   硬件接口       │                    │
│  │  · SLAM Toolbox │  │  · CAN Gateway  │                    │
│  │  · AMCL 定位    │  │  · RPLIDAR A1   │                    │
│  │  · Costmap 代价 │  │  · Astra Camera │                    │
│  │  · Planner 规划 │  │  · Teleop 手柄  │                    │
│  │  · DWB 控制     │  │  · Diagnostics  │                    │
│  │  · BehaviorTree │  └────────┬────────┘                    │
│  └────────┬────────┘           │                             │
│           │           ┌────────┴────────┐                    │
│           │           │   AI 推理        │                    │
│           │           │  · YOLO NPU     │                    │
│           │           └────────┬────────┘                    │
│           │                    │                             │
├──────────────────────────────────────────────────────────────┤
│                    中间件层 (Middleware)                      │
│  ┌──────────────────────────────────────────────────────┐    │
│  │  Cyclone DDS                                         │    │
│  │  Pub/Sub · Service/Action · QoS · Shared Memory      │    │
│  └──────────────────────────────────────────────────────┘    │
├──────────────────────────────────────────────────────────────┤
│                    操作系统层 (OS)                            │
│  ┌──────────────────────────────────────────────────────┐    │
│  │  Ubuntu 22.04 + Linux Kernel 5.10 + PREEMPT_RT       │    │
│  │  · socketCAN · UVC USB · I2C · GPIO                  │    │
│  └──────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────┘
```

### STM32 固件 (FreeRTOS)

| 任务 | 频率 | CPU 占比 | 优先级 | 说明 |
|------|------|:---:|:---:|------|
| `vMotorControlTask` | 1kHz | 2% | High | 编码器读取 → 滑动窗口滤波 → 前馈+PID → PWM |
| `vIMUAcquireTask` | 250Hz | 9% | AboveNormal | I2C 读取 MPU9250, Mutex 保护共享数据 |
| `vCANTxSchedulerTask` | 1kHz | 5% | Normal | 8-slot 非阻塞 CAN 发送调度器 |
| `vCommandDispatchTask` | 事件驱动 | <1% | Normal | CAN RX ISR → TaskNotify 唤醒, 命令分发 |
| `vRTTTelemetryTask` | 2Hz | <1% | Low | 栈水位 + 运行时统计 + 遥测输出 |

**总 CPU 占用 < 17%**，IDLE 时间 > 83%。

---

## 性能指标

### 电机控制

| 指标 | 数值 |
|------|------|
| 控制频率 | **1kHz** PID 速度闭环 |
| 编码器分辨率 | 11 线 × 4 倍频, TIM 硬件正交解码 |
| 有效速度范围 | -500 ~ +400 mm/s |
| 稳态误差 (中低速) | **< 10 mm/s** (\|V\| ≤ 200) |
| 四电机一致性 | 差异 < 3% |

### CAN 总线

| 指标 | 数值 |
|------|------|
| 位速率 | 500 kbps |
| 总线利用率 | ~6% (峰值 ~900 帧/秒) |
| 帧调度 | 1kHz 8-slot 非阻塞 |
| 急停延迟 | < 100µs (硬件仲裁) |

### Linux 实时性 (cyclictest, 全 ROS2 负载)

| 配置 | 最大延迟 | 改善 |
|------|------|:---:|
| PREEMPT_VOLUNTARY (标准) | 2809 µs | 基线 |
| PREEMPT_RT | 247 µs | **11×** |
| PREEMPT_RT + CPU 隔离 + IRQ 亲和性 | **84 µs** | **33×** |

### AI 推理 (RK3588 NPU)

| 指标 | 数值 |
|------|------|
| 模型 | YOLOv8n (FP16 量化) |
| 精度 (COCO mAP) | 37.3% |
| 纯 NPU 推理 | ~90 ms |
| 含前后处理 (CPU) | ~175 ms (~5-6 FPS) |
| NPU 负载 | ~30% (单核) |

---

## 快速开始

### 前置要求

- **RK3588**: Ubuntu 22.04, ROS2 Humble, colcon, PREEMPT_RT 内核
- **STM32**: Keil MDK-ARM + J-Link 调试器
- **硬件**: CAN 总线已连接, 终端电阻 120Ω ×2

### 1. 克隆仓库

```bash
git clone https://github.com/<your-org>/RK3588-STM32-ROS2-SLAM-Robot.git
cd RK3588-STM32-ROS2-SLAM-Robot
```

### 2. STM32 固件编译与烧录

用 Keil MDK 打开 `stm32/prj/MDK-ARM/prj.uvprojx`，Build (F7) → Download (F8)。

或使用命令行:

```bash
cd stm32/prj
JLinkExe -device STM32F407VE -if SWD -speed 4000 -autoconnect 1 \
  -CommanderScript flash.jlink
```

### 3. ROS2 构建

```bash
cd app/
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 4. 启动机器人

```bash
# 一键全栈启动 (CAN + EKF + 传感器 + Nav2 + AI)
ros2 launch robot_bringup demo.launch.py

# 或分步启动:
ros2 launch robot_bringup base.launch.py       # CAN 网关 + EKF
ros2 launch robot_bringup sensors.launch.py    # LiDAR + 相机
ros2 launch robot_bringup navigation.launch.py # Nav2 导航
ros2 launch robot_ai ai.launch.py              # AI 目标检测
```

### 5. 手柄遥控

```bash
ros2 launch robot_bringup teleop.launch.py
```

---

## ROS2 功能包

| 包名 | 描述 | 类型 |
|------|------|------|
| `robot_can_gateway` | socketCAN ↔ ROS2 桥接, SPSC 环形缓冲区, 四驱运动学 | 自研 |
| `robot_bringup` | 真机启动编排 (CAN + EKF + 传感器 + SLAM + Nav2) | 自研 |
| `robot_ai` | YOLOv8n RKNN NPU 推理, 目标检测 + 障碍物发布 | 自研 |
| `robot_sim` | Gazebo 仿真 (URDF + diff_drive 插件 + 世界) | 自研 |
| `robot_rplidar` | RPLIDAR A1/A2/A3/S1/S2/S3/T1 驱动 | 适配 |
| `astra_camera` | 奥比中光 Astra Pro RGB-D 驱动 (OpenNI2) | 适配 |

---

## 项目目录结构

```
RK3588-STM32-ROS2-SLAM-Robot/
├── stm32/                  # STM32F407 固件 (FreeRTOS + HAL)
│   └── prj/
│       ├── Core/           # 应用代码 (PID/编码器/电机/IMU/CAN/RTT)
│       ├── Drivers/        # STM32F4xx HAL + CMSIS
│       ├── Middlewares/    # FreeRTOS 10.3.1
│       └── MDK-ARM/        # Keil 工程 + 编译产物
├── app/                    # ROS2 工作空间
│   └── src/
│       ├── robot_can_gateway/  # CAN ↔ ROS2 桥接
│       ├── robot_bringup/      # 启动文件 + 配置 (YAML)
│       ├── robot_ai/           # NPU AI 推理
│       ├── robot_sim/          # Gazebo 仿真
│       └── robot_rplidar/      # RPLIDAR 驱动
├── targets/
│   ├── rk3588/             # RK3588 内核/设备树/U-Boot 配置
│   └── stm32/              # STM32 烧录脚本
├── sdk/                    # BSP SDK (内核源码, U-Boot, rootfs)
├── tools/                  # 辅助工具 (标定/部署/巡线)
├── docs/                   # 文档 (部署指南/仿真指南)
```

---

## License

MIT License — 详见 [LICENSE](LICENSE)
