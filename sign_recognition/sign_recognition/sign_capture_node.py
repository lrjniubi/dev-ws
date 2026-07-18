"""
标志牌图像截取节点

功能：
1. 订阅 RGB 相机图像（/aurora/rgb/image_raw）
2. 检测画面中符合 A5 比例（1:1.414）的矩形区域
3. 检测到后自动截取 3 次，每次间隔 3 秒
4. 仅保存矩形区域的裁剪图像
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
from datetime import datetime


class SignCaptureNode(Node):
    """标志牌图像截取节点"""

    # A5 纸张比例: 148mm × 210mm → 比例约 1:1.414
    A5_RATIO = 210.0 / 148.0  # ≈ 1.414
    RATIO_TOLERANCE = 0.20     # 允许 ±20% 的比例偏差

    # 截取参数
    CAPTURE_COUNT = 3          # 总共截取 3 次
    CAPTURE_INTERVAL = 3.0     # 每次间隔 3 秒
    COOLDOWN_AFTER_CAPTURE = 30.0  # 截取完成后冷却 30 秒，防止重复触发

    # 检测参数
    MIN_CONTOUR_AREA = 5000    # 最小轮廓面积（像素），过滤小噪点
    MIN_RECT_SIDE = 80         # 矩形最短边（像素），排除过小的误检
    APPROX_EPSILON = 0.04      # 多边形近似精度系数

    def __init__(self):
        super().__init__('sign_capture_node')

        # ====================== 【声明参数】======================
        self.declare_parameter('image_topic', '/aurora/rgb/image_raw')
        self.declare_parameter('save_dir', '/home/sunrise/RacingDev/dev_ws/src/sign_recognition/captured_images')
        self.declare_parameter('a5_ratio_tolerance', self.RATIO_TOLERANCE)
        self.declare_parameter('min_contour_area', self.MIN_CONTOUR_AREA)
        self.declare_parameter('capture_count', self.CAPTURE_COUNT)
        self.declare_parameter('capture_interval', self.CAPTURE_INTERVAL)
        self.declare_parameter('cooldown_after_capture', self.COOLDOWN_AFTER_CAPTURE)

        # ====================== 【读取参数】======================
        image_topic = self.get_parameter('image_topic').value
        self.save_dir = self.get_parameter('save_dir').value
        self.RATIO_TOLERANCE = self.get_parameter('a5_ratio_tolerance').value
        self.MIN_CONTOUR_AREA = self.get_parameter('min_contour_area').value
        self.CAPTURE_COUNT = self.get_parameter('capture_count').value
        self.CAPTURE_INTERVAL = self.get_parameter('capture_interval').value
        self.COOLDOWN_AFTER_CAPTURE = self.get_parameter('cooldown_after_capture').value

        # ====================== 【初始化状态】======================
        self.bridge = CvBridge()
        self.latest_frame = None
        self.capturing = False          # 是否正在截取序列
        self.capture_index = 0          # 当前截取序号
        self.last_capture_time = 0.0    # 上次截取时间
        self.detected_rect = None       # 检测到的矩形 (x, y, w, h)
        self.cooldown_until = 0.0       # 冷却截止时间，防止截取后立刻再次触发

        # 创建保存目录
        os.makedirs(self.save_dir, exist_ok=True)

        # ====================== 【订阅图像话题】======================
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f'标志牌截取节点已启动')
        self.get_logger().info(f'订阅话题: {image_topic}')
        self.get_logger().info(f'A5 比例: 1:{self.A5_RATIO:.3f} (容差 ±{self.RATIO_TOLERANCE*100:.0f}%)')
        self.get_logger().info(f'截取计划: {self.CAPTURE_COUNT} 次, 间隔 {self.CAPTURE_INTERVAL}s')
        self.get_logger().info(f'截取后冷却: {self.COOLDOWN_AFTER_CAPTURE}s')
        self.get_logger().info(f'保存目录: {self.save_dir}')

    def image_callback(self, msg):
        """图像回调：接收帧并处理"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        self.latest_frame = frame
        current_time = time.time()

        # 如果正在截取序列
        if self.capturing:
            # 检查是否到了下一次截取时间
            if current_time - self.last_capture_time >= self.CAPTURE_INTERVAL:
                # 重新检测矩形位置（目标可能移动了）
                rect = self._detect_a5_rectangle(frame)
                if rect is not None:
                    self.detected_rect = rect

                if self.detected_rect is not None:
                    self._crop_and_save(frame, current_time)
                    self.capture_index += 1
                    self.last_capture_time = current_time

                    if self.capture_index >= self.CAPTURE_COUNT:
                        self.get_logger().info(f'截取序列完成，共 {self.CAPTURE_COUNT} 张，进入冷却期 {self.COOLDOWN_AFTER_CAPTURE}s')
                        self.capturing = False
                        self.capture_index = 0
                        self.detected_rect = None
                        self.cooldown_until = current_time + self.COOLDOWN_AFTER_CAPTURE
                else:
                    self.get_logger().warn('丢失目标矩形，截取序列中断')
                    self.capturing = False
                    self.capture_index = 0
                    self.detected_rect = None
            return

        # 冷却期内不检测
        if current_time < self.cooldown_until:
            return

        # 未截取时，持续检测 A5 矩形
        rect = self._detect_a5_rectangle(frame)
        if rect is not None:
            x, y, w, h = rect
            ratio = max(w, h) / max(min(w, h), 1)
            self.get_logger().info(
                f'检测到 A5 矩形! 位置=({x},{y}), 尺寸={w}x{h}, 比例=1:{ratio:.3f}'
            )
            # 启动截取序列
            self.capturing = True
            self.capture_index = 0
            self.detected_rect = rect
            self.last_capture_time = current_time - self.CAPTURE_INTERVAL  # 立即截取第一张

    def _detect_a5_rectangle(self, frame):
        """
        检测画面中符合 A5 比例的矩形

        Returns:
            (x, y, w, h) 或 None
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 自适应阈值 + Canny 双重检测，减少误检
        thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2
        )
        edges = cv2.Canny(blurred, 50, 150)
        combined = cv2.bitwise_or(thresh, edges)

        # 查找轮廓
        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_rect = None
        best_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.MIN_CONTOUR_AREA:
                continue

            # 多边形近似
            epsilon = self.APPROX_EPSILON * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # 检查是否为四边形
            if len(approx) != 4:
                continue

            # 获取外接矩形
            x, y, w, h = cv2.boundingRect(approx)
            if w == 0 or h == 0:
                continue

            # 最短边约束：排除过小的误检
            short_side = min(w, h)
            if short_side < self.MIN_RECT_SIDE:
                continue

            # 凸性检查：A5 纸张应为凸四边形
            if not cv2.isContourConvex(approx):
                continue

            # 计算比例
            long_side = max(w, h)
            ratio = long_side / short_side

            # 检查是否符合 A5 比例（允许竖放或横放）
            if abs(ratio - self.A5_RATIO) <= self.RATIO_TOLERANCE * self.A5_RATIO:
                if area > best_area:
                    best_area = area
                    best_rect = (x, y, w, h)

        return best_rect

    def _crop_and_save(self, frame, current_time):
        """裁剪矩形区域并保存"""
        x, y, w, h = self.detected_rect

        # 边界检查
        h_img, w_img = frame.shape[:2]
        x1 = max(0, x)
        y1 = max(0, y)
        x2 = min(w_img, x + w)
        y2 = min(h_img, y + h)

        cropped = frame[y1:y2, x1:x2]

        if cropped.size == 0:
            self.get_logger().warn('裁剪区域为空，跳过保存')
            return

        # 生成文件名：时间戳 + 序号
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'sign_{timestamp}_{self.capture_index + 1}.png'
        filepath = os.path.join(self.save_dir, filename)

        cv2.imwrite(filepath, cropped)
        self.get_logger().info(
            f'截取 [{self.capture_index + 1}/{self.CAPTURE_COUNT}]: {filepath} '
            f'(尺寸: {cropped.shape[1]}x{cropped.shape[0]})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SignCaptureNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
