#!/bin/bash
# Origicar Dev 工作环境配置脚本
# 使用方法：source setup_origicar_env.sh
#
# 功能说明:
# - 加载 ROS2 Humble 基础环境
# - 加载 TROS 环境 (用于 hbm_img_msgs)
# - 加载 Origicar 工作空间环境
# - 配置库路径
# ============================================

set -e  # 遇到错误立即退出

echo "正在配置 Origicar 开发环境..."

# Source ROS2 基础环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS2 Humble 环境已加载"
else
    echo "✗ 未找到 ROS2 Humble 环境，请检查安装"
    return 1
fi

# Source TROS 环境 (用于 hbm_img_msgs)
if [ -f "/opt/tros/humble/setup.sh" ]; then
    source /opt/tros/humble/setup.sh
    echo "✓ TROS Humble 环境已加载"
else
    echo "⚠ 未找到 TROS 环境 (utils 包可能无法使用)"
fi

# Source 工作空间环境
WORKSPACE_PATH="/home/sunrise/RacingDev/dev_ws"
if [ -f "$WORKSPACE_PATH/install/setup.bash" ]; then
    source $WORKSPACE_PATH/install/setup.bash
    echo "✓ Origicar 工作空间环境已加载"
else
    echo "✗ 工作空间未编译，请先执行：cd $WORKSPACE_PATH && colcon build"
    return 1
fi

# 设置库路径
export LD_LIBRARY_PATH=/opt/tros/humble/lib:$LD_LIBRARY_PATH
export AMENT_PREFIX_PATH=/opt/tros/humble:$AMENT_PREFIX_PATH
echo "✓ 库路径已配置"

echo ""
echo "========================================"
echo "Origicar 开发环境配置完成!"
echo "========================================"
echo ""
echo "可用的 ROS2 包:"
if command -v ros2 &> /dev/null; then
    ros2 pkg list | grep -E "(origincar|utils|racing)" | sort
else
    echo "  (ROS2 命令不可用)"
fi
echo ""
echo "示例命令:"
echo "  ros2 run origincar_base origincar_base_node"
echo "  ros2 run utils image_transport_node"
echo "  ros2 launch origincar_bringup <launch_file>"
echo "  ros2 launch origincar_base origincar_bringup.launch.py"
echo ""
echo "快速启动:"
echo "  bash start_origicar.sh              # 差速模式 + RDK IMU"
echo "  bash start_origicar.sh --akmcar     # 阿克曼模式 + RDK IMU"
echo ""
echo "停止服务:"
echo "  bash stop_origicar.sh"
echo "========================================"
