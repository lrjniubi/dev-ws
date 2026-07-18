#!/bin/bash

# ============================================
# Aurora930 二维码检测系统一键启动脚本
# ============================================
# 功能：启动 Aurora930 相机驱动、屏幕显示、二维码识别
#
# 使用方式:
#   bash start_aurora_qr_detection.sh              # 默认模式：aurora相机+显示+微信二维码
#   bash start_aurora_qr_detection.sh --usb        # 使用 USB 摄像头（默认索引 8）
#   bash start_aurora_qr_detection.sh --usb 0      # 使用 USB 摄像头（索引 0）
#   bash start_aurora_qr_detection.sh --no-display # 启动但不显示图像窗口
#   bash start_aurora_qr_detection.sh --usb --no-display  # USB摄像头+无显示
# ============================================

# 参数配置
CAMERA_TYPE="aurora"    # aurora 或 usb
USB_INDEX=8              # USB 摄像头设备索引
SHOW_DISPLAY=true        # 是否显示图像窗口
DISPLAY=:0

# 工作空间路径
DEV_WS_PATH="/home/sunrise/RacingDev/dev_ws"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

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

echo "========================================="
echo "  Aurora930 二维码检测系统启动脚本"
echo "========================================="
echo ""

# --- 解析命令行参数 ---
while [[ $# -gt 0 ]]; do
    case $1 in
        --usb)
            CAMERA_TYPE="usb"
            # 检查下一个参数是否为数字（USB 索引）
            if [[ $# -gt 1 && "$2" =~ ^[0-9]+$ ]]; then
                USB_INDEX=$2
                shift
            fi
            shift
            ;;
        --no-display)
            SHOW_DISPLAY=false
            shift
            ;;
        -h|--help)
            echo "使用方式:"
            echo "  bash $0                          # 默认模式"
            echo "  bash $0 --usb                    # USB 摄像头（索引 8）"
            echo "  bash $0 --usb 0                  # USB 摄像头（索引 0）"
            echo "  bash $0 --no-display             # 无图像窗口"
            echo "  bash $0 --usb --no-display       # USB 摄像头 + 无窗口"
            exit 0
            ;;
        *)
            echo -e "${RED}未知参数: $1${NC}"
            echo "使用 bash $0 --help 查看帮助"
            exit 1
            ;;
    esac
done

# --- 显示配置信息 ---
echo "📋 启动配置:"
echo "  相机类型: $( [ "$CAMERA_TYPE" = "aurora" ] && echo 'Aurora930' || echo "USB (索引 $USB_INDEX)" )"
echo "  屏幕显示: $( [ "$SHOW_DISPLAY" = true ] && echo '开启' || echo '关闭' )"
echo ""

# --- 加载 ROS2 环境 ---
info_msg "加载 ROS2 环境..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    error_exit "ROS2 Humble 未安装"
fi

# 加载工作空间环境
if [ -f "$DEV_WS_PATH/install/local_setup.bash" ]; then
    source "$DEV_WS_PATH/install/local_setup.bash"
    success_msg "工作空间环境已加载"
else
    error_exit "工作空间未编译，请先执行 colcon build"
fi

# 设置库路径（Aurora930 驱动依赖）
export LD_LIBRARY_PATH="$DEV_WS_PATH/install/deptrum-ros-driver-aurora930/lib:$LD_LIBRARY_PATH"
export DISPLAY=:0
export XAUTHORITY=/home/sunrise/.Xauthority

# --- 启动相机驱动 ---
if [ "$CAMERA_TYPE" = "aurora" ]; then
    # 启动 Aurora930 相机驱动
    if pgrep -f "aurora930_node" > /dev/null; then
        success_msg "Aurora930 相机驱动已在运行"
    else
        info_msg "启动 Aurora930 相机驱动..."
        cd "$DEV_WS_PATH"
        ros2 launch "$DEV_WS_PATH/install/deptrum-ros-driver-aurora930/share/deptrum-ros-driver-aurora930/aurora930_camera.launch.py" &
        AURORA_PID=$!
        sleep 5
        
        if pgrep -f "aurora930_node" > /dev/null; then
            success_msg "Aurora930 相机驱动已启动"
        else
            error_exit "Aurora930 相机驱动启动失败"
        fi
    fi
else
    # USB 模式不需要启动 Aurora 相机驱动
    info_msg "使用 USB 摄像头模式（跳过 Aurora 相机驱动）"
fi

# 等待相机就绪
sleep 2

# --- 启动二维码检测节点 ---
info_msg "启动二维码检测节点..."
QR_SCRIPT="$DEV_WS_PATH/src/racing/qr_mem/src/aurora_qr_detection.py"

if [ ! -f "$QR_SCRIPT" ]; then
    error_exit "找不到二维码检测脚本: $QR_SCRIPT"
fi

# 构造 ROS2 参数
QR_PARAMS=""

# 添加相机类型参数
if [ "$CAMERA_TYPE" = "usb" ]; then
    QR_PARAMS="$QR_PARAMS --ros-args -p camera_type:=usb -p usb_camera_index:=$USB_INDEX"
fi

# 添加显示参数
if [ "$SHOW_DISPLAY" = false ]; then
    QR_PARAMS="$QR_PARAMS --ros-args -p show_display:=false"
fi

# 启动二维码检测节点
if [ -n "$QR_PARAMS" ]; then
    python3 "$QR_SCRIPT" $QR_PARAMS &
else
    python3 "$QR_SCRIPT" &
fi
QR_PID=$!
sleep 2

if ps -p $QR_PID > /dev/null 2>&1; then
    success_msg "二维码检测节点已启动 (PID: $QR_PID)"
else
    error_exit "二维码检测节点启动失败"
fi

# --- 显示启动信息 ---
echo ""
echo "========================================="
echo "  🎉 系统已启动！"
echo "========================================="
echo ""
echo "📊 运行状态:"
if [ "$CAMERA_TYPE" = "aurora" ]; then
    echo "  - 📷 Aurora930 相机驱动：✅"
fi
echo "  - 🏷️  二维码检测节点：✅ (PID $QR_PID)"
echo "  - 🖥️  屏幕显示：$([ "$SHOW_DISPLAY" = true ] && echo '✅ 开启' || echo '⛔ 关闭')"
echo ""
echo "📡 可用话题:"
echo "  - /aurora/rgb/image_raw    (相机 RGB 图像) $([ "$CAMERA_TYPE" != "aurora" ] && echo '[仅 Aurora 模式]')"
echo "  - /qr_code_result          (二维码检测结果)"
echo ""
echo "🔧 使用说明:"
echo "  ros2 topic echo /qr_code_result          # 查看检测结果"
echo "  bash stop_aurora_qr_detection.sh         # 停止所有节点"
echo ""
echo "💡 启动参数:"
echo "  bash $0 --usb [索引]       # 使用 USB 摄像头"
echo "  bash $0 --no-display       # 关闭屏幕显示"
echo "  bash $0 --help             # 查看帮助"
echo "========================================="
echo ""

# 保持脚本运行（如果按 Ctrl+C 不会停止后台节点）
wait
