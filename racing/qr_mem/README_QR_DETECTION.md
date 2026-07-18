# Aurora930 二维码检测系统

## 概述
本系统使用 Aurora930 深度相机的实时 RGB 图像进行二维码检测。

## 系统组成

### 1. Aurora930 相机驱动
- **包名**: deptrum-ros-driver-aurora930
- **启动文件**: aurora930_camera.launch.py
- **发布话题**: 
  - `/aurora/rgb/image_raw` (sensor_msgs/Image) - RGB 图像
  - `/aurora/ir/image_raw` (sensor_msgs/Image) - IR 图像
  - `/aurora/depth/image_raw` (sensor_msgs/Image) - 深度图像
  - `/aurora/points2` (sensor_msgs/PointCloud2) - 点云数据

### 2. 二维码检测节点
- **文件位置**: `/home/sunrise/RacingDev/dev_ws/src/racing/qr_mem/src/aurora_qr_detection.py`
- **节点名称**: aurora_qrcode_detect
- **订阅话题**: `/aurora/rgb/image_raw`
- **发布话题**: `/qr_code_result` (std_msgs/String) - 检测到的二维码内容

## 启动步骤

### 方法一：分别启动（推荐）

#### 1. 启动 Aurora930 相机驱动
```bash
cd /home/sunrise/RacingDev/aurora_ws
source install/setup.bash
ros2 launch deptrum-ros-driver-aurora930 aurora930_camera.launch.py
```

#### 2. 启动二维码检测节点
```bash
export XAUTHORITY=/home/sunrise/.Xauthority
export DISPLAY=:0
cd /home/sunrise/RacingDev/dev_ws
source install/setup.bash
python3 src/racing/qr_mem/src/aurora_qr_detection.py
```

### 方法二：使用一键启动脚本（待创建）

## 功能特性

1. **实时检测**: 使用 OpenCV 的 QRCodeDetector 进行实时二维码检测
2. **可视化显示**: 在窗口中显示检测结果，包括：
   - 实时摄像头画面
   - 检测到的二维码边界框（绿色）
   - 检测状态信息
3. **结果发布**: 将检测到的二维码内容发布到 `/qr_code_result` 话题

## 使用方法

1. 启动系统后，会打开一个名为 "QR Code Detection" 的窗口
2. 将二维码放置在相机视野内
3. 检测到二维码时：
   - 终端会显示检测到的内容
   - 图像上会绘制绿色边界框
   - 二维码内容会发布到 `/qr_code_result` 话题
4. 按 `q` 或 `ESC` 键退出程序

## 查看检测结果

在另一个终端中运行：
```bash
cd /home/sunrise/RacingDev/dev_ws
source install/setup.bash
ros2 topic echo /qr_code_result
```

## 注意事项

1. **显示环境**: 需要设置正确的 DISPLAY 和 XAUTHORITY 环境变量才能显示图像窗口
2. **二维码格式**: 支持标准 QR Code 格式
3. **检测性能**: 使用 OpenCV 默认检测器，如需更高性能可配置微信二维码检测模型
4. **光照条件**: 确保二维码有足够的光照和清晰度

## 故障排除

### 问题：无法显示图像窗口
**解决方案**: 
```bash
export XAUTHORITY=/home/sunrise/.Xauthority
export DISPLAY=:0
```

### 问题：没有检测到二维码
**可能原因**:
- 二维码距离相机太远或太近
- 光照不足
- 二维码模糊或有反光
- 二维码不在视野中心

### 问题：相机驱动启动失败
**解决方案**:
- 检查 USB 连接
- 确认设备权限
- 查看日志：`tail -f /tmp/deptrum-stream.log`

## 技术细节

- **图像分辨率**: 640x400
- **帧率**: 15 FPS
- **检测算法**: OpenCV QRCodeDetector
- **消息类型**: std_msgs/String
