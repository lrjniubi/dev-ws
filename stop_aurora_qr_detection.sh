#!/bin/bash

# ============================================
# Aurora930 二维码检测系统停止脚本
# ============================================
# 停止所有与本系统相关的节点和进程

echo "========================================="
echo "  停止 Aurora930 二维码检测系统"
echo "========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

success_msg() {
    echo -e "${GREEN}✅ $1${NC}"
}

info_msg() {
    echo -e "${YELLOW}ℹ️  $1${NC}"
}

STOPPED=false

# 1. 停止二维码检测节点
if pgrep -f "aurora_qr_detection.py" > /dev/null; then
    info_msg "停止二维码检测节点..."
    pkill -f "aurora_qr_detection.py"
    sleep 1
    if pgrep -f "aurora_qr_detection.py" > /dev/null; then
        pkill -9 -f "aurora_qr_detection.py" 2>/dev/null
        sleep 1
    fi
    success_msg "二维码检测节点已停止"
    STOPPED=true
else
    echo "  ○ 二维码检测节点未运行"
fi

# 2. 停止 Aurora930 相机驱动
if pgrep -f "aurora930_node" > /dev/null; then
    info_msg "停止 Aurora930 相机驱动..."
    pkill -f "aurora930_node"
    sleep 2
    if pgrep -f "aurora930_node" > /dev/null; then
        pkill -9 -f "aurora930_node" 2>/dev/null
        sleep 1
    fi
    success_msg "Aurora930 相机驱动已停止"
    STOPPED=true
else
    echo "  ○ Aurora930 相机驱动未运行"
fi

echo ""
if [ "$STOPPED" = true ]; then
    echo "========================================="
    echo -e "${GREEN}  ✅ 所有节点已停止${NC}"
    echo "========================================="
else
    echo "  ℹ️  没有正在运行的相关节点"
fi
echo ""
