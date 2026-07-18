# racing_control_2026 使用说明

## 竞赛主控包

实现四阶段自主竞赛流程：大厅 → 二维码 → 赛道环行 → 返回起点。

---

## 快速启动

```bash
# 1. 启动底盘 + 传感器（必须先启动）
bash dev_ws/start_origicar.sh

# 2. 启动 YOLO 避障（可选）
bash dev_ws/start_yolo.sh

# 3. 启动竞赛主控
source /opt/tros/humble/setup.bash
source dev_ws/install/setup.bash
ros2 launch racing_control_2026 racing_control.launch.py
```

---

## 竞赛阶段

| 阶段 | 名称 | 说明 | 退出条件 |
|------|------|------|----------|
| Phase 1 | 大厅 | 导航到大厅航点 | x 超过阈值 / 到达 / 超时 |
| Phase 2 | 二维码 | 导航到二维码 + 触发扫码 | 扫码成功 / 到达 / 超时 |
| Phase 3 | 赛道环行 | 顺/逆时针经过 10 个航点 | 全部航点完成 |
| Phase 4 | 返回起点 | 导航回起点 | YOLO 停车 / 到达 / 超时 |

### 优先级

```
Geofence（最高） > 避障绕行 > 阶段逻辑 > 导航
```

---

## 调试启动参数

无需修改 YAML，通过 launch 参数快速切换调试模式：

```bash
# 从 Phase 3 开始（跳过大厅和二维码）
ros2 launch racing_control_2026 racing_control.launch.py start_phase:=3

# 强制顺时针路线（不依赖扫码结果）
ros2 launch racing_control_2026 racing_control.launch.py start_phase:=3 direction:=cw

# 强制逆时针
ros2 launch racing_control_2026 racing_control.launch.py start_phase:=3 direction:=ccw

# 禁用避障（不调用 YOLO 检测话题）
ros2 launch racing_control_2026 racing_control.launch.py enable_obstacle:=false

# 禁用所有外部服务（扫码/人形识别均不触发）
ros2 launch racing_control_2026 racing_control.launch.py enable_external_services:=false

# 组合：Phase 3 顺时针 + 禁用避障 + 禁用外部服务
ros2 launch racing_control_2026 racing_control.launch.py \
    start_phase:=3 direction:=cw \
    enable_obstacle:=false enable_external_services:=false
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `start_phase` | `1` | 起始阶段 (1-4)，跳过之前阶段 |
| `direction` | `auto` | Phase 3 方向: `auto`/`cw`/`ccw` |
| `enable_obstacle` | `true` | 是否启用 YOLO 避障绕行 |
| `enable_external_services` | `true` | 是否启用扫码/人形识别服务 |

---

## 航点配置

航点在 `config/params.yaml` 中配置，启动时一次性全部加载，阶段切换仅切换索引范围。

### 航点格式

```yaml
waypoint_names: ["大厅", "二维码", ...]   # 名称数组
waypoints: [                                # 数值数组（每个航点 7 个值）
  x, y, yaw, yaw_tol, timeout, dist_tol, pause_duration,
  ...
]
```

- `yaw = -999.0` 表示不指定朝向（自主决定）
- `pause_duration > 0` 触发接近减速 + 到达后暂停
- Phase 3 航点用 `_ccw` / `_cw` 后缀区分方向

### 航点名称约定

| 名称 | 所属阶段 |
|------|----------|
| `大厅` | Phase 1 |
| `二维码` | Phase 2 |
| `*_ccw` | Phase 3 逆时针 |
| `*_cw` | Phase 3 顺时针 |
| `起点` | Phase 4 |

---

## 外部 ROS2 接口

| 接口 | 类型 | 用途 |
|------|------|------|
| `/racing_phase` | `std_msgs/String` pub | 发布当前阶段名称 |
| `/qr_scan_trigger` | `std_srvs/Trigger` client | 触发扫码 |
| `/qr_scan_result` | `std_msgs/Int32` sub | 接收二维码数字 |
| `/sign_detect_trigger` | `std_srvs/Trigger` client | 触发人形识别 |
| `/sign_detect_stop` | `std_srvs/Trigger` client | 关闭人形识别 |
| `/odom_combined` | `nav_msgs/Odometry` sub | EKF 融合里程计 |
| `/cmd_vel` | `geometry_msgs/Twist` pub | 速度指令 |
| `/racing_obstacle_detection` | `ai_msgs/PerceptionTargets` sub | YOLO 检测 |

---

## 保存运行日志

使用 `run_inertial_nav.sh` 的 `--log` 参数保存日志：

```bash
bash dev_ws/run_inertial_nav.sh --log
# 日志保存至 dev_ws/logs/inertial_nav_<时间戳>.log

# 查看日志（带颜色）
less -R dev_ws/logs/inertial_nav_*.log

# 搜索关键事件
grep "Phase" dev_ws/logs/inertial_nav_*.log
grep "Reached waypoint" dev_ws/logs/inertial_nav_*.log
```

---

## 代码结构

```
include/
├── racing_controller.hpp   # 主控节点（状态机 + 控制循环）
├── navigator.hpp           # Pure Pursuit 导航
├── route_manager.hpp       # 路线管理（索引分区）
├── obstacle_avoider.hpp    # 避障绕行
├── phase_manager.hpp       # 阶段管理器
├── external_services.hpp   # 外部服务协调
└── pid_controller.hpp      # PID 控制器

src/
├── main.cpp                # 入口
├── racing_controller.cpp   # 主控实现
├── navigator.cpp           # 导航实现
├── route_manager.cpp       # 路线管理
├── obstacle_avoider.cpp    # 避障实现
├── phase_manager.cpp       # 阶段管理
├── external_services.cpp   # 外部服务
└── pid_controller.cpp      # PID 实现
```

---

## 重新编译

```bash
cd dev_ws
source /opt/tros/humble/setup.bash
source install/setup.bash
colcon build --packages-select racing_control_2026
```
