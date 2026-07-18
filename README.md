# SMART2 — 智能车自动驾驶竞赛系统

**SMART2** 是面向 **第21届全国大学生智能汽车竞赛（地瓜机器人赛项）** 的自主驾驶智能车项目。基于 OriginCar 阿克曼底盘（STM32 下位机 + RDK X5 上位机），使用 ROS2 Humble 构建全栈自动驾驶系统，融合深度相机感知、Pure Pursuit 导航、YOLO 避障与多阶段状态机控制。

---

## 项目概述

| 项目         | 内容                                                                                  |
| ------------ | ------------------------------------------------------------------------------------- |
| **硬件平台** | RDK X5 + STM32 + 阿克曼转向底盘 + Aurora930 深度相机                                  |
| **操作系统** | Ubuntu 22.04 (ROS2 Humble)                                                            |
| **开发语言** | C++ / Python                                                                          |
| **核心算法** | Pure Pursuit 路径跟踪、四阶段竞赛状态机、YOLOv5 锥桶检测、Aurora 深度相机人形立牌识别 |
| **竞赛任务** | 自主导航出发 → 二维码扫描 → 赛道环行 → 人形立牌识别 → 返回起点停车                    |

---

## 目录结构

```
SMART2/
├── start_origicar.sh                    # 底盘+IMU+摄像头 一键启动
├── start_yolo.sh                        # YOLO 障碍物检测启动
├── start_aurora_qr_detection.sh         # Aurora930 二维码检测启动
├── stop_aurora_qr_detection.sh          # 停止二维码检测
├── setup_origicar_env.sh                # 工作环境配置
│
├── src/
│   ├── origincar/                       # 核心 ROS2 元包
│   │   ├── origincar_base/             # 底盘驱动节点 (STM32串口通信、IMU、里程计、EKF)
│   │   ├── origincar_bringup/          # 启动管理 (摄像头、WebSocket、可视化)
│   │   ├── origincar_description/      # URDF/XACRO 机器人模型
│   │   ├── origincar_msg/              # 自定义消息接口
│   │   ├── utils/                      # 图像传输工具
│   │   └── 3rdparty/                   # 第三方依赖 (ackermann_msgs, serial_ros2, aurora930 驱动)
│   │
│   ├── racing/                          # 竞赛功能包
│   │   ├── racing_control_2026/        # 竞赛主控节点 (四阶段状态机 + Pure Pursuit 导航 + 避障)
│   │   ├── racing_obstacle_detection_yolo/  # YOLOv5s 障碍物检测（锥桶）
│   │   └── qr_mem/                     # QR 码 BPU 推理检测
│   │
│   ├── sign_recognition/                # 人形立牌识别（通过 Aurora 深度相机检测 A5 标志牌）
│   ├── deptrum-ros-driver/              # Aurora930 深度相机独立驱动
│   └── imu_tools/                       # IMU 滤波工具 (Madgwick/Complementary)
│
└── README.md                            # 本文档
```

---

## 系统架构

### 整体架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    racing_control_2026 (FSM)                     │
│  PHASE_1[大厅] → PHASE_2[二维码] → PHASE_3[赛道环行] → PHASE_4[返回]  │
│          C++ Node · 50Hz 控制循环 · 四阶段状态机                  │
└────────┬────────────┬────────────────┬─────────────────────────┘
         │ /cmd_vel   │ 扫码/人形触发    │ /racing_phase 广播
         ▼            ▼                 ▼
┌──────────────────┐  ┌────────────────────┐  ┌──────────────────┐
│  origincar_base  │  │   感知子系统        │  │   外部服务协调     │
│  STM32 串口通信   │  │                    │  │  external_services │
│  EKF /odom_combined│ │ qr_mem QR码检测    │  │  ├ /qr_scan_trigger│
│  RDK IMU BMI088   │  │ sign_recognition   │  │  ├ /sign_detect    │
│  /imu/data_raw    │  │ 人形立牌 A5 检测    │  │  │ _trigger        │
└────────┬─────────┘  └─────────┬──────────┘  └──────────────────┘
         │                      │
         │         ┌────────────────────┐
         │         │  Aurora930 深度相机 │
         └────────▶│  /aurora/rgb/image │
                   │    _raw  RGB 图像    │
                   │  QR 码检测 · 人形识别 │
                   └────────────────────┘
```

### 传感器与数据流

```
┌──────────────┐  /odom        ┌──────────┐  /odom_combined
│  轮式里程计    │─────────────▶│   EKF    │──────────────▶ racing_control
│  (STM32)     │              │  Fusion  │      + /tf     navigator
└──────────────┘              └────┬─────┘
                                   │
┌──────────────┐  /imu/data_raw    │
│ RDK BMI088   │───────────────────┘
│ 板载 IMU     │
└──────────────┘

┌──────────────┐  /aurora/rgb/  ┌─────────────────┐  /qr_scan_result
│  Aurora930   │  image_raw     │  qr_mem          │────▶ racing_control
│  深度相机     │──────────────▶│  BPU QR 码检测    │
│  RGB + Depth │               └─────────────────┘
│              │  /aurora/rgb/  ┌─────────────────┐  /sign_detect_trigger
│              │  image_raw     │  sign_recognition│◀─── racing_control
│              │──────────────▶│  A5 标志牌截取    │     (Phase 3 触发)
└──────────────┘               └─────────────────┘

┌──────────────┐  /hbmem_img   ┌────────────────────┐  /racing_obstacle_
│  USB 摄像头   │─────────────▶│  YOLOv5s          │  detection
│  (备选/辅助)  │               │  BPU 锥桶检测+避障  │────▶ racing_control
└──────────────┘               └────────────────────┘
                                                           │/cmd_vel
                                                           ▼
                                                    ┌──────────────┐
                                                    │  STM32       │
                                                    │  舵机/电机   │
                                                    └──────────────┘
```

---

## 核心功能包

### 1. racing_control_2026 — 竞赛主控节点

| 项目     | 内容                                                                                                  |
| -------- | ----------------------------------------------------------------------------------------------------- |
| 语言     | C++                                                                                                   |
| 功能     | 四阶段竞赛状态机、Pure Pursuit 路径跟踪、航点序列导航、动态预瞄、倒车控制、YOLO避障绕行、外部服务协调 |
| 发布     | `/cmd_vel`, `/racing_phase`                                                                           |
| 订阅     | `/odom_combined`, `/racing_obstacle_detection`, `/qr_scan_result`                                     |
| 关键文件 | `src/racing_controller.cpp`, `config/params.yaml`                                                     |

**四阶段竞赛流程：**

| 阶段    | 名称     | 说明                                      | 退出条件                 |
| ------- | -------- | ----------------------------------------- | ------------------------ |
| Phase 1 | 大厅     | 导航到大厅航点                            | x 超过阈值 / 到达 / 超时 |
| Phase 2 | 二维码   | 导航到二维码 + 触发扫码                   | 扫码成功 / 到达 / 超时   |
| Phase 3 | 赛道环行 | 顺/逆时针经过 10 个航点，途中识别人形立牌 | 全部航点完成             |
| Phase 4 | 返回起点 | 导航回起点，YOLO 停车                     | YOLO 停车 / 到达 / 超时  |

**优先级：**
```
Geofence（最高） > 避障绕行 > 阶段逻辑 > 导航
```

### 2. racing_obstacle_detection_yolo — YOLO 障碍物检测

| 项目 | 内容                               |
| ---- | ---------------------------------- |
| 语言 | C++                                |
| 模型 | YOLOv5s (TensorRT .bin，BPU 推理)  |
| 功能 | 锥桶目标检测，含透视校正和距离估算 |
| 发布 | `/racing_obstacle_detection`       |
| 配置 | `config/yolov5sconfig.json`        |

### 3. sign_recognition — 人形立牌识别

| 项目     | 内容                                                                                            |
| -------- | ----------------------------------------------------------------------------------------------- |
| 语言     | Python                                                                                          |
| 功能     | 订阅 Aurora930 RGB 图像，检测画面中符合 A5 比例 (1:1.414) 的矩形标志牌区域，自动截取 3 次并保存 |
| 订阅     | `/aurora/rgb/image_raw`                                                                         |
| 触发     | 竞赛主控在 Phase 3 经过指定航点（直道中段）后通过 `/sign_detect_trigger` 服务触发               |
| 停止     | 经过第三直角航点后自动关闭识别，进入冷却期                                                      |
| 关键文件 | `sign_recognition/sign_capture_node.py`                                                         |

> 利用 Aurora930 深度相机的 RGB 图像，通过计算机视觉检测画面中的 A5 纸张比例矩形区域，实现人形立牌的自动化检测与截取。

### 4. qr_mem — QR 码 BPU 检测

| 项目 | 内容                            |
| ---- | ------------------------------- |
| 语言 | Python                          |
| 推理 | RDK X5 BPU 推理，零 CPU 占用    |
| 功能 | 图像中 QR 码检测 + ROI 解码     |
| 发布 | `/qr_scan_result` (Int32)       |
| 相机 | Aurora930 深度相机 / USB 摄像头 |

### 5. origincar_base — 底盘驱动核心

| 项目 | 内容                                                                                |
| ---- | ----------------------------------------------------------------------------------- |
| 语言 | C++                                                                                 |
| 功能 | STM32 串口通信、RDK IMU 数据解析、EKF 传感器融合、轮式里程计计算、舵机/电机指令下发 |
| 发布 | `/odom`, `/odom_combined`, `/imu/data_raw`, `/PowerVoltage`                         |
| 订阅 | `/cmd_vel`                                                                          |
| 启动 | `ros2 launch origincar_base origincar_bringup.launch.py`                            |

### 6. 辅助包

| 包名                       | 语言   | 功能                        |
| -------------------------- | ------ | --------------------------- |
| `origincar_bringup`        | Python | 摄像头、WebSocket、启动管理 |
| `origincar_description`    | URDF   | 阿克曼底盘 3D 模型描述      |
| `origincar_msg`            | 消息   | 自定义 ROS2 消息接口        |
| `utils`                    | C++    | 图像传输协议转换            |
| `imu_filter_madgwick`      | C++    | Madgwick IMU 滤波           |
| `imu_complementary_filter` | C++    | 互补 IMU 滤波               |
| `deptrum-ros-driver`       | C++    | Aurora930 深度相机驱动      |
| `rdk_imu-library`          | C/C++  | RDK X5 板载 IMU 底层通信库  |

---

## 技术栈

| 层级       | 技术                             |
| ---------- | -------------------------------- |
| 操作系统   | Ubuntu 22.04 (RDK X5 arm64)      |
| 中间件     | ROS2 Humble (colcon build)       |
| AI 推理    | tros-humble BPU (YOLOv5s .bin)   |
| 深度相机   | Aurora930 深度相机 (RGB + Depth) |
| 视觉       | OpenCV (A5 矩形检测、图像截取)   |
| 传感器融合 | robot_localization (EKF)         |
| 下位机     | STM32 (串口协议)                 |
| 底盘       | 阿克曼转向 (Ackermann Drive)     |
| 远程调试   | ROSBridge WebSocket + Foxglove   |

---

## 快速启动

### 1. 启动底盘与传感器

```bash
# 阿克曼模式完整启动（底盘+IMU+摄像头+ROSBridge）
bash start_origicar.sh --akmcar

# 如需禁用 WebSocket 图传（节省资源）
bash start_origicar.sh --akmcar --no-websocket
```

### 2. 启动 YOLO 障碍物检测

```bash
# 新开终端
bash start_yolo.sh
```

### 3. 启动人脸立牌识别（独立调试）

```bash
# 启动 Aurora930 相机 + 二维码检测
bash start_aurora_qr_detection.sh

# 启动人形立牌识别节点
ros2 run sign_recognition sign_capture_node
```

### 4. 启动竞赛主控

```bash
# 新开终端
source /opt/tros/humble/setup.bash
source dev_ws/install/setup.bash
ros2 launch racing_control_2026 racing_control.launch.py
```

### 调试启动参数

```bash
# 从 Phase 3 开始（跳过大厅和二维码）
ros2 launch racing_control_2026 racing_control.launch.py start_phase:=3

# 强制顺时针路线（不依赖扫码结果）
ros2 launch racing_control_2026 racing_control.launch.py direction:=cw

# 禁用避障
ros2 launch racing_control_2026 racing_control.launch.py enable_obstacle:=false

# 禁用外部服务（扫码/人形识别均不触发）
ros2 launch racing_control_2026 racing_control.launch.py enable_external_services:=false
```

---

## 关键参数调优

所有导航参数在 `src/racing/racing_control_2026/config/params.yaml` 中配置，修改后重启节点生效，无需重新编译。

| 参数                   | 默认值     | 说明                        |
| ---------------------- | ---------- | --------------------------- |
| `max_linear_speed`     | 0.8 m/s    | 最大前进速度                |
| `lookahead_distance`   | 0.5 m      | Pure Pursuit 预瞄距离       |
| `min_turning_radius`   | 0.3 m      | 最小转弯半径                |
| `position_tolerance`   | 0.1 m      | 航点位置容差                |
| `control_frequency`    | 50 Hz      | 控制循环频率                |
| `avoid_area`           | 15000 px²  | 避障触发面积阈值            |
| `fence_x_max`          | 4.3 m      | 坐标围栏 X 上限（安全约束） |
| `sign_detect_after_wp` | "直道中段" | Phase 3 触发人形识别的航点  |

---

## 外部 ROS2 接口

| 话题/服务                    | 类型                      | 用途                           |
| ---------------------------- | ------------------------- | ------------------------------ |
| `/cmd_vel`                   | Twist                     | 底盘速度控制指令               |
| `/odom_combined`             | Odometry                  | EKF 融合里程计（导航主数据源） |
| `/odom`                      | Odometry                  | 轮式里程计                     |
| `/imu/data_raw`              | Imu                       | RDK IMU 原始数据               |
| `/racing_obstacle_detection` | PerceptionTargets         | YOLO 障碍物检测结果            |
| `/racing_phase`              | String                    | 当前竞赛阶段广播               |
| `/qr_scan_trigger`           | Trigger (srv)             | 触发 QR 码扫描                 |
| `/qr_scan_result`            | Int32                     | QR 码扫描结果（数字方向）      |
| `/sign_detect_trigger`       | Trigger (srv)             | 触发人形立牌识别               |
| `/sign_detect_stop`          | Trigger (srv)             | 关闭人形立牌识别               |
| `/aurora/rgb/image_raw`      | Image                     | Aurora930 RGB 相机画面         |
| `/PowerVoltage`              | Float32                   | 电池电压                       |
| `/set_pose`                  | PoseWithCovarianceStamped | EKF 位姿重置                   |

---

## 人形立牌识别流程

1. 竞赛进入 **Phase 3（赛道环行）** 后，车辆沿航点行驶
2. 到达**直道中段**航点时，竞赛主控通过 `/sign_detect_trigger` 服务触发人形识别
3. `sign_recognition` 节点订阅 Aurora930 RGB 图像，实时检测画面中 **A5 比例 (1:1.414)** 的矩形区域
4. 检测到后自动截取 **3 次**，每次间隔 **3 秒**，仅保存裁剪后的标志牌区域图像
5. 到达**第三直角**航点后，自动关闭识别进入 **30 秒冷却期**，防止重复触发
6. 截取的标志牌图像保存至 `sign_recognition/captured_images/` 目录，可用于后续大模型图生文分析

---

## 构建命令

```bash
# 全量编译
cd /home/sunrise/RacingDev/dev_ws
colcon build --symlink-install

# 仅编译特定包
colcon build --packages-select racing_control_2026
colcon build --packages-select sign_recognition
colcon build --packages-select racing_obstacle_detection_yolo
```

---

## 调试命令

```bash
# 查看融合里程计
ros2 topic echo /odom_combined | grep position

# 查看控制输出
ros2 topic echo /cmd_vel --once

# 查看 YOLO 检测
ros2 topic echo /racing_obstacle_detection --once

# 查看当前竞赛阶段
ros2 topic echo /racing_phase

# 查看 Aurora 相机画面
ros2 topic echo /aurora/rgb/image_raw --once

# 运行时修改参数（无需重启）
ros2 param set /racing_controller max_linear_speed 0.6
ros2 param set /racing_controller lookahead_distance 0.7
```

---

## 当前开发状态

| 模块                                | 状态     |
| ----------------------------------- | -------- |
| 底盘驱动 (STM32 通信)               | ✅ 已完成 |
| EKF 传感器融合 (轮式里程计+RDK IMU) | ✅ 已完成 |
| RDK X5 板载 IMU 驱动                | ✅ 已完成 |
| Aurora930 深度相机驱动              | ✅ 已完成 |
| QR 码 BPU 推理检测                  | ✅ 已完成 |
| Pure Pursuit 导航                   | ✅ 已完成 |
| 动态预瞄调节                        | ✅ 已完成 |
| 倒车控制                            | ✅ 已完成 |
| YOLOv5 锥桶障碍物检测               | ✅ 已完成 |
| 避障绕行                            | ✅ 已完成 |
| 四阶段竞赛状态机                    | ✅ 已完成 |
| 人形立牌 A5 矩形检测截取            | ✅ 已完成 |
| 人形识别竞赛集成触发                | ✅ 已完成 |
| 坐标围栏安全约束                    | ✅ 已完成 |
| ROSBridge 远程调试                  | ✅ 已完成 |
| 实车赛道联调                        | 🟡 进行中 |
