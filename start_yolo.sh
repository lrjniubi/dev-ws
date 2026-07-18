#!/bin/bash
# ============================================================================
# YOLO 障碍物检测快速启动脚本
# 用途: 一键启动 racing_obstacle_detection_yolo 节点及相关依赖
# ============================================================================

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 工作空间路径
WORKSPACE_DIR="/home/sunrise/RacingDev/dev_ws"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  YOLO 障碍物检测启动脚本${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 步骤 1: 加载环境
echo -e "${YELLOW}[1/4] 加载 ROS2 和工作空间环境...${NC}"
source /opt/tros/humble/setup.bash
source ${WORKSPACE_DIR}/install/setup.bash
echo -e "${GREEN}      ✓ 环境加载完成${NC}"

# 步骤 2: 验证包是否存在
echo -e "${YELLOW}[2/4] 验证 racing_obstacle_detection_yolo 包...${NC}"
if ! ros2 pkg list | grep -q "racing_obstacle_detection_yolo"; then
    echo -e "${RED}      ✗ 错误: 找不到 racing_obstacle_detection_yolo 包${NC}"
    echo -e "${RED}      请先编译: cd ${WORKSPACE_DIR} && colcon build --packages-select racing_obstacle_detection_yolo${NC}"
    exit 1
fi
echo -e "${GREEN}      ✓ 包验证通过${NC}"

# 步骤 3: 检查配置文件
echo -e "${YELLOW}[3/4] 检查配置文件...${NC}"
CONFIG_FILE="${WORKSPACE_DIR}/install/racing_obstacle_detection_yolo/lib/racing_obstacle_detection_yolo/config/yolov5sconfig.json"
if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}      ✗ 错误: 配置文件不存在: $CONFIG_FILE${NC}"
    exit 1
fi
echo -e "${GREEN}      ✓ 配置文件存在${NC}"

# 步骤 4: 启动节点
echo -e "${YELLOW}[4/4] 启动 YOLO 障碍物检测节点...${NC}"
echo -e "${BLUE}----------------------------------------${NC}"
echo -e "${GREEN}即将启动以下组件:${NC}"
echo -e "  • racing_obstacle_detection_yolo (YOLOv5 检测)"
echo -e "  • hobot_codec_encoder (图像编码)"
echo -e "  • websocket (Web 服务)"
echo -e "${BLUE}----------------------------------------${NC}"
echo ""
echo -e "${GREEN}Web 可视化地址: http://192.168.7.160:8000/${NC}"
echo -e "${YELLOW}提示: 按 Ctrl+C 停止所有节点${NC}"
echo ""
sleep 2

# 启动 launch 文件
ros2 launch racing_obstacle_detection_yolo racing_obstacle_detection_yolo.launch.py
