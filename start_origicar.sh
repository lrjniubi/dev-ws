#!/bin/bash
# ============================================
# Origicar ROS2 一键启动脚本（完整版）
# ============================================
# 功能：启动底盘节点、USB 摄像头、ROSBridge、IMU 驱动、EKF 融合、里程计重置
# 支持：阿克曼模式 + 纯图像监控 + IMU 切换 + 里程计重置服务
#
# 更新日志:
# - 2026-04-08: 添加 RDK X5 板载 IMU 支持
#              新增 --rdk-imu 参数，使用 RDK IMU Module 作为唯一 IMU 数据源
# - 2026-03-31: 更新 EKF 节点名称为 ekf_filter_node
#              使用官方 /set_pose topic 方案重置里程计
#              添加 image_transport 节点清理
# ============================================

# 参数配置（可通过命令行参数覆盖）
AKMCAR=${AKMCAR:-false}  # 默认差速模式，设为 true 使用阿克曼模式
CAMERA_TYPE=${CAMERA_TYPE:-compressed}  # 默认压缩图像模式，可设为 "hobot"、"usb_only" 或 "normal"
DISABLE_WEBSOCKET=${DISABLE_WEBSOCKET:-false}  # 默认启用WebSocket图传，设为 true 禁用

echo "============================================"
echo "  🚗 Origicar ROS2 系统启动"
echo "============================================"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 错误处理函数
error_exit() {
    echo -e "${RED}❌ 错误：$1${NC}" >&2
    exit 1
}

success_msg() {
    echo -e "${GREEN}✅ $1${NC}"
}

info_msg() {
    echo -e "${YELLOW}ℹ️  $1${NC}"
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --akmcar)
            AKMCAR=true
            shift
            ;;
        --hobot-cam)
            CAMERA_TYPE="hobot"
            shift
            ;;
        --usb-only)
            CAMERA_TYPE="usb_only"
            shift
            ;;
        --normal)  # 切换到普通模式（实际上与默认模式相同）
            CAMERA_TYPE="normal"
            shift
            ;;
        --no-websocket|--no-camera)
            DISABLE_WEBSOCKET=true
            shift
            ;;
        *)
            echo -e "${YELLOW}未知参数：$1${NC}"
            echo "可用参数:"
            echo "  --akmcar              阿克曼转向模式（默认差速）"
            echo "  --hobot-cam           使用 Hobot 摄像头节点"
            echo "  --usb-only            仅使用 USB 摄像头（无 WebSocket）"
            echo "  --normal              普通模式（与默认模式相同，都发布压缩图像）"
            echo "  --no-websocket        禁用 WebSocket 图传（保留摄像头+HB转换，节省资源）"
            exit 1
            ;;
    esac
done

# 清理旧进程
info_msg "清理旧进程..."
pkill -f "origincar_base" 2>/dev/null || true
pkill -f "hobot_usb_cam" 2>/dev/null || true
pkill -f "rosbridge_websocket" 2>/dev/null || true
pkill -f "websocket" 2>/dev/null || true
sleep 1  # 等待进程完全退出
success_msg "进程清理完成"

# Source 环境
info_msg "加载 ROS2 环境..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    error_exit "ROS2 Humble 未安装"
fi

cd /home/sunrise/RacingDev/dev_ws

if [ -f setup_origicar_env.sh ]; then
    source setup_origicar_env.sh
    success_msg "Origicar 环境已加载"
else
    error_exit "找不到 setup_origicar_env.sh"
fi

export AMENT_PREFIX_PATH=/opt/tros/humble:$AMENT_PREFIX_PATH
success_msg "TROS 环境已加载"

# 检查必要文件
if [ ! -f install/setup.bash ]; then
    error_exit "工作空间未编译，请先执行 colcon build"
fi

source install/setup.bash

# 启动服务
echo ""
echo "============================================"
echo "  正在启动服务..."
echo "============================================"
echo ""

# 1. 启动底盘节点（包含 RDK IMU、EKF 融合、TF 等）
if [ "$AKMCAR" = true ]; then
    info_msg "启动 Origicar 底盘节点（阿克曼模式）..."
else
    info_msg "启动 Origicar 底盘节点（差速模式）..."
fi

# 使用 RDK IMU 作为唯一 IMU 数据源，EKF 融合轮式里程计和 IMU
info_msg "⭐ 使用 RDK X5 板载 IMU + EKF 融合"
ros2 launch origincar_base origincar_bringup.launch.py akmcar:=$AKMCAR &
BASE_PID=$!

# 轮询等待里程计话题就绪（最多8秒）
info_msg "等待里程计话题 /odom_combined 就绪..."
ODOM_READY=false
for i in {1..16}; do
    if ros2 topic info /odom_combined &>/dev/null 2>&1; then
        ODOM_READY=true
        break
    fi
    sleep 0.5
done

if ps -p $BASE_PID > /dev/null && $ODOM_READY; then
    success_msg "底盘节点已启动 (PID: $BASE_PID)"
    info_msg "EKF 融合节点：ekf_filter_node"
    info_msg "里程计话题：/odom_combined ⭐ (已就绪)"
else
    error_exit "底盘节点启动失败或里程计话题未就绪"
fi

# 2. 启动 USB 摄像头节点（可选）
if [ "$DISABLE_WEBSOCKET" = true ]; then
    info_msg "⚠️  WebSocket 图传已禁用（--no-websocket 参数）"
    info_msg "📹 仅启动摄像头驱动 + HB转换节点（提供 /hbmem_img 供AI使用）"
    # 只启动摄像头驱动和NV12转换，不启动WebSocket/JPEG编码
    ros2 launch origincar_bringup usb_websocket_display_only.launch.py &
    CAMERA_PID=$!
    sleep 2

    if ps -p $CAMERA_PID > /dev/null; then
        success_msg "摄像头驱动+HB转换已启动 (PID: $CAMERA_PID)"
        info_msg "可用话题：/image (MJPEG), /hbmem_img (共享内存NV12)"
    else
        info_msg "摄像头节点可能未正常启动，继续其他服务..."
    fi
else
    info_msg "启动 USB 摄像头节点 (类型：$CAMERA_TYPE)..."
    if [ "$CAMERA_TYPE" = "hobot" ]; then
        ros2 launch origincar_bringup usb_websocket_display.launch.py &
    elif [ "$CAMERA_TYPE" = "usb_only" ]; then
        ros2 launch origincar_bringup usb_camera_only.launch.py &
    elif [ "$CAMERA_TYPE" = "normal" ]; then
        # 普通模式：使用标准显示（已包含压缩图像 /image）
        ros2 launch origincar_bringup usb_websocket_display_only.launch.py &
    else
        # 默认模式：也使用标准显示（hobot_usb_cam 已经发布压缩格式到 /image）
        ros2 launch origincar_bringup usb_websocket_display_only.launch.py &
    fi
    CAMERA_PID=$!
    sleep 2

    if ps -p $CAMERA_PID > /dev/null; then
        success_msg "USB 摄像头已启动 (PID: $CAMERA_PID)"
    else
        info_msg "摄像头节点可能未正常启动，继续其他服务..."
    fi
fi

# 3. 启动 ROSBridge（使用优化配置）
info_msg "启动 ROSBridge WebSocket 服务器..."
ros2 launch origincar_base rosbridge_optimized.launch.py &
ROSBRIDGE_PID=$!
sleep 1  # ROSBridge 启动较快，1秒足够

if ps -p $ROSBRIDGE_PID > /dev/null; then
    success_msg "ROSBridge 已启动 (PID: $ROSBRIDGE_PID, 端口 9090)"
else
    error_exit "ROSBridge 启动失败"
fi

# 获取 IP 地址
IP_ADDR=$(hostname -I | awk '{print $1}')

# 显示启动信息
echo ""
echo "============================================"
echo "  🎉 所有服务启动成功！"
echo "============================================"
echo ""
echo "📊 运行状态:"
echo "  - 🚗 车辆底盘：✅ PID $BASE_PID"
echo "  - 🤖 EKF 融合：✅ ekf_filter_node"
if [ "$DISABLE_WEBSOCKET" = true ]; then
    echo "  - 📹 USB 摄像头：✅ PID $CAMERA_PID (驱动+HB转换，无WebSocket图传)"
else
    echo "  - 📹 USB 摄像头：✅ PID $CAMERA_PID"
fi
echo "  - 🔌 ROSBridge: ✅ PID $ROSBRIDGE_PID (端口 9090)"
echo ""
echo "⚙️  系统配置:"
echo "  - IMU 类型：RDK X5 板载 BMI088 (唯一数据源)"
echo "  - EKF 融合：轮式里程计 + RDK IMU → /odom_combined ⭐"
if [ "$AKMCAR" = true ]; then
    echo "  - 底盘模式：阿克曼转向"
else
    echo "  - 底盘模式：差速驱动"
fi
if [ "$DISABLE_WEBSOCKET" = true ]; then
    echo "  - 图像模式：📹 摄像头驱动+HB转换（无WebSocket图传）"
else
    echo "  - 图像模式：压缩模式（hobot_usb_cam 发布到 /image）"
fi
echo ""
echo "🌐 ROSBridge 访问地址:"
echo "  - ws://localhost:9090"
echo "  - ws://${IP_ADDR}:9090"
echo ""
echo "📡 主要话题:"
echo "  - 🚗 /ackermann_cmd (底盘控制)"
echo "  - 🚗 /odom_combined (融合里程计) ⭐"
echo "  - 🚗 /odom (轮式里程计)"
echo "  - 🚗 /imu/data_raw (IMU 原始数据)"
echo "  - 🚗 /PowerVoltage (电池电压)"
if [ "$DISABLE_WEBSOCKET" = false ]; then
    echo "  - 📹 /image (摄像头画面 - CompressedImage) ⭐ ← racing_image_collect 订阅此话题"
fi
echo "  - 📹 /hbmem_img (共享内存图像 - NV12) ← AI 推理使用（始终可用）"
echo ""
echo "🔧 位姿重置:"
echo "  - 📡 /set_pose (geometry_msgs/PoseWithCovarianceStamped) - 发布到此 topic 重置 EKF 位姿"
echo "  - 📖 详细说明：dev_ws/src/origincar/origincar_base/SET_POSE_USAGE.md"
echo ""
echo "🔧 使用说明:"
echo "  - 使用 Foxglove Studio 或其他 ROSBridge 客户端连接"
echo "  - 连接到 ws://localhost:9090"
echo "  - 订阅上述话题查看数据"
echo "  - 发布消息到 /set_pose topic 重置 EKF 位姿和 TF"
echo ""
echo "💡 高级选项:"
echo "  - 阿克曼模式：bash $0 --akmcar"
echo "  - Hobot 摄像头：bash $0 --hobot-cam"
echo "  - 仅 USB 摄像头：bash $0 --usb-only"
echo "  - 禁用图传：bash $0 --no-websocket （保留摄像头驱动+HB转换，关闭WebSocket传输）"
echo ""
echo "⚠️  停止服务:"
echo "  按 Ctrl+C 或执行：bash stop_origicar.sh"
echo ""
echo "============================================"
echo ""

# 等待用户中断
trap "echo ''; info_msg '正在停止所有服务...'; kill $BASE_PID $CAMERA_PID $ROSBRIDGE_PID 2>/dev/null; pkill -f origincar_base 2>/dev/null; pkill -f hobot_usb_cam 2>/dev/null; pkill -f rosbridge_websocket 2>/dev/null; pkill -f ekf_filter_node 2>/dev/null; pkill -f imu_filter_madgwick 2>/dev/null; pkill -f hobot_codec 2>/dev/null; sleep 2; success_msg '所有服务已停止'; exit 0" INT TERM

# 保持运行
wait
