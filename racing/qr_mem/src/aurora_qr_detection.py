#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Aurora930 二维码检测节点（竞赛适配版）
- 启动后待命，等待 racing_control 通过 /qr_scan_trigger 服务触发扫码
- 触发后持续检测并发布结果到 /qr_scan_result (Int32) 和 /qr_code_result (String)
- 收到 /qr_scan_stop 服务调用后停止检测并回到待命
- 支持 Aurora930 相机和 USB 摄像头
"""

import rclpy
import cv2
import cv_bridge
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory


class AuroraQrCodeDetection(Node):
    def __init__(self):
        super().__init__('aurora_qrcode_detect')
        self.get_logger().info("启动 Aurora930 二维码检测节点（竞赛适配版）")

        # ========== 状态标志 ==========
        self.scanning = False          # 是否正在扫码（待命时为 False）
        self.last_result = ""          # 上一次检测结果

        # ========== 发布者 ==========
        # 保留原始 String 话题（兼容）
        self.pub_qrcode_info = self.create_publisher(String, '/qr_code_result', 10)
        # 新增 Int32 话题（racing_control 订阅此话题）
        self.pub_qr_scan_result = self.create_publisher(Int32, '/qr_scan_result', 10)

        # ========== 参数声明 ==========
        self.declare_parameter('camera_type', 'aurora')
        camera_type = self.get_parameter('camera_type').get_parameter_value().string_value

        self.declare_parameter('show_display', True)
        self.show_display = self.get_parameter('show_display').get_parameter_value().bool_value

        # ========== 相机初始化 ==========
        if camera_type == 'usb':
            self.use_usb_camera = True
            self.declare_parameter('usb_camera_index', 8)
            self.usb_camera_index = self.get_parameter('usb_camera_index').get_parameter_value().integer_value

            try:
                self.usb_cap = cv2.VideoCapture(self.usb_camera_index)
                if not self.usb_cap.isOpened():
                    self.get_logger().error(f"无法打开 USB 摄像头 {self.usb_camera_index}")
                    raise Exception(f"USB 摄像头 {self.usb_camera_index} 打开失败")

                self.usb_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.usb_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.usb_cap.set(cv2.CAP_PROP_FPS, 30)
                self.get_logger().info(f"成功打开 USB 摄像头 {self.usb_camera_index}")

                self.timer = self.create_timer(0.033, self.usb_camera_callback)
                self.image_sub = None
            except Exception as e:
                self.get_logger().error(f"USB 摄像头初始化失败: {str(e)}")
                raise
        else:
            self.use_usb_camera = False
            self.usb_cap = None

            self.image_sub = self.create_subscription(
                Image,
                "/aurora/rgb/image_raw",
                self.image_callback,
                10
            )
            self.get_logger().info("使用 Aurora930 相机")

        self.bridge = cv_bridge.CvBridge()

        # ========== 二维码检测器 ==========
        try:
            qr_pkg_dir = get_package_share_directory('qr_code_detection')
            modelPath = os.path.join(qr_pkg_dir, 'model/')

            if os.path.exists(modelPath):
                self.detect_obj = cv2.wechat_qrcode_WeChatQRCode(
                    modelPath + 'detect.prototxt',
                    modelPath + 'detect.caffemodel',
                    modelPath + 'sr.prototxt',
                    modelPath + 'sr.caffemodel'
                )
                self.use_wechat_detector = True
                self.get_logger().info("使用微信二维码检测器")
            else:
                self.use_wechat_detector = False
                self.get_logger().warn("未找到微信二维码模型，使用 OpenCV 默认检测器")
        except Exception as e:
            self.use_wechat_detector = False
            self.get_logger().warn(f"加载微信二维码检测器失败: {str(e)}，使用 OpenCV 默认检测器")

        # ========== 显示窗口 ==========
        if self.show_display:
            cv2.namedWindow('QR Code Detection', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('QR Code Detection', 640, 480)
            self.get_logger().info("屏幕显示已开启")
        else:
            self.get_logger().info("屏幕显示已关闭")

        # ========== 服务接口 ==========
        # /qr_scan_trigger: racing_control 调用此服务启动扫码
        self.srv_trigger = self.create_service(
            Trigger, '/qr_scan_trigger', self.handle_trigger)
        # /qr_scan_stop: racing_control 调用此服务停止扫码
        self.srv_stop = self.create_service(
            Trigger, '/qr_scan_stop', self.handle_stop)

        self.get_logger().info("="*50)
        self.get_logger().info("二维码检测节点就绪（待命模式）")
        self.get_logger().info("  服务: /qr_scan_trigger (启动扫码)")
        self.get_logger().info("  服务: /qr_scan_stop    (停止扫码)")
        self.get_logger().info("  发布: /qr_scan_result  (Int32)")
        self.get_logger().info("  发布: /qr_code_result  (String, 兼容)")
        self.get_logger().info("="*50)

    # ==================== 服务回调 ====================

    def handle_trigger(self, request, response):
        """收到扫码触发请求"""
        if self.scanning:
            self.get_logger().warn("已在扫码中，忽略重复触发")
            response.success = True
            response.message = "already scanning"
            return response

        self.scanning = True
        self.last_result = ""
        self.get_logger().info(">>> 扫码已启动，开始检测二维码...")
        response.success = True
        response.message = "scanning started"
        return response

    def handle_stop(self, request, response):
        """收到停止扫码请求"""
        was_scanning = self.scanning
        self.scanning = False
        if was_scanning:
            self.get_logger().info("<<< 扫码已停止，回到待命模式")
        else:
            self.get_logger().info("<<< 未在扫码中，保持待命")
        response.success = True
        response.message = "scanning stopped"
        return response

    # ==================== 图像回调 ====================

    def usb_camera_callback(self):
        """USB 摄像头定时回调"""
        try:
            ret, frame = self.usb_cap.read()
            if not ret:
                self.get_logger().warn("无法从 USB 摄像头读取图像")
                return
            self.process_image(frame)
        except Exception as e:
            self.get_logger().error(f'处理 USB 摄像头图像时出错: {str(e)}')

    def image_callback(self, msg):
        """Aurora930 图像回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}')

    # ==================== 核心检测逻辑 ====================

    def process_image(self, cv_image):
        """通用图像处理函数"""
        try:
            detected_codes = []

            # 待命模式：只做显示，不做检测
            if not self.scanning:
                if self.show_display:
                    status_text = "STANDBY - waiting for trigger"
                    cv2.putText(cv_image, status_text, (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 0), 2)
                    cv2.imshow('QR Code Detection', cv_image)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == 27:
                        self.get_logger().info('退出程序')
                        cv2.destroyAllWindows()
                        rclpy.shutdown()
                return

            # ===== 扫码模式：执行检测 =====
            if self.use_wechat_detector:
                qrInfo, qrPoints = self.detect_obj.detectAndDecode(cv_image)
                if qrInfo:
                    for code in qrInfo:
                        detected_codes.append(code)
                        if qrPoints is not None and len(qrPoints) > 0:
                            points = qrPoints[0]
                            pts = points.astype(int)
                            cv2.polylines(cv_image, [pts], True, (0, 255, 0), 2)
            else:
                detector = cv2.QRCodeDetector()
                data, bbox, _ = detector.detectAndDecode(cv_image)
                if data:
                    detected_codes.append(data)
                    if bbox is not None:
                        pts = bbox.astype(int)
                        cv2.polylines(cv_image, [pts], True, (0, 255, 0), 2)

            # 发布检测结果
            for code in detected_codes:
                self.get_logger().info(f'检测到二维码: {code}')

                # 发布 String（兼容）
                str_msg = String()
                str_msg.data = code
                self.pub_qrcode_info.publish(str_msg)

                # 发布 Int32（racing_control 接口）
                try:
                    int_value = int(code)
                except ValueError:
                    # 非纯数字二维码，取 hash 值确保稳定
                    int_value = hash(code) % 10000
                    self.get_logger().warn(
                        f"二维码内容 '{code}' 非纯数字，转换为 {int_value}")

                int_msg = Int32()
                int_msg.data = int_value
                self.pub_qr_scan_result.publish(int_msg)

                self.last_result = code

            # 显示
            status_text = f"SCANNING | Detected: {len(detected_codes)} | Last: {self.last_result}"
            cv2.putText(cv_image, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if self.show_display:
                cv2.imshow('QR Code Detection', cv_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    self.get_logger().info('退出程序')
                    cv2.destroyAllWindows()
                    rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}')

    def destroy_node(self):
        if self.use_usb_camera and self.usb_cap is not None:
            self.usb_cap.release()
            self.get_logger().info("USB 摄像头已释放")
        if self.show_display:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    qr_detector = AuroraQrCodeDetection()

    try:
        rclpy.spin(qr_detector)
    except KeyboardInterrupt:
        pass
    finally:
        qr_detector.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
