#include "rclcpp/rclcpp.hpp"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
// #include "qr_mem/msg/sign_switch.hpp" // 替换为我的包名
#include "origincar_msg/msg/sign.hpp" 
#include <memory>
#include <zbar.h>

class ImageCompressor : public rclcpp::Node {
 public:
  ImageCompressor():Node("image_compressor"){

    // 为/hbmem_img订阅者创建best_effort QoS设置，以便与图像发布者兼容
    auto hbmem_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    subscription_ =
      this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
      "/hbmem_img", hbmem_qos,
      std::bind(&ImageCompressor ::imageCallback, this,std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/image_jpeg", 1);
      
    // 创建一个发布者用于发送data=5消息，自动进入遥控状态
    foxglove_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
      "/sign_foxglove", 10);

    // 为图像订阅者创建best_effort QoS设置，以便与图像发布者兼容
    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    qr_subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/image", image_qos, 
      std::bind(&ImageCompressor::qrCallback, this, std::placeholders::_1));

    // qr_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
    //   "/sign_switch", 10);
    // qr_publisher_ = this->create_publisher<qr_mem::msg::SignSwitch>(
    //   "/sign_switch", 10);

    // 为发布者创建自定义QoS配置，使用TransientLocal耐久性策略
    auto publisher_qos = rclcpp::QoS(10);
    publisher_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    
    // 为订阅者创建TransientLocal QoS配置，与发布者保持一致
    auto subscriber_qos = rclcpp::QoS(10);
    subscriber_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    
    qr_publisher_ = this->create_publisher<origincar_msg::msg::Sign>(
      "/sign_switch", publisher_qos);

    qr_racing_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/qr_racing", publisher_qos);
      
    // 创建二维码原始内容发布者
    qr_content_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/qr_content", publisher_qos);

    control_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/racing_qr", subscriber_qos, 
      std::bind(&ImageCompressor::controlCallback, this, std::placeholders::_1));

    foxglove_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "/sign_foxglove", 10, 
      std::bind(&ImageCompressor::foxgloveCallback, this, std::placeholders::_1));

    this->declare_parameter<int>("width", 480);
    this->declare_parameter<int>("height", 320);
    this->get_parameter("width", width_);
    this->get_parameter("height", height_);
  }

  void publishSignData(int32_t sign_data) {
    // auto message = std::make_shared<qr_mem::msg::SignSwitch>();
    auto message = std::make_shared<origincar_msg::msg::Sign>();
    message->sign_data = sign_data;
    qr_publisher_->publish(*message);
  }

 private:
  void imageCallback(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg) const {
    if (!image_processing_enabled_) {
        return;
    }
    try
    {
      // 将 hbm_img_msgs::msg::HbmMsg1080P 转换为 OpenCV 图像
      cv::Mat nv12_image(cv::Size(640, 480 + 480 / 2), CV_8UC1, const_cast<unsigned char*>(msg->data.data()));
      if (nv12_image.empty()) {
          RCLCPP_ERROR(this->get_logger(), "解码图像失败");
          return;
      }
      // RCLCPP_INFO(this->get_logger(), "成功订阅图片");
      // 将 NV12 图像转换为 BGR 格式
      cv::Mat bgr_image;
      cv::cvtColor(nv12_image, bgr_image, cv::COLOR_YUV2BGR_NV12);
      // 缩放图像，降低分辨率
      cv::Mat resized_image;
      cv::resize(bgr_image, resized_image, cv::Size(), 0.5, 0.5); // 0.7 0.7
      // 压缩图像，降低质量参数以减少压缩时间
      std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 30}; //  45   设置JPEG质量参数为50
      std::vector<uchar> compressed_image;
      cv::imencode(".jpg", resized_image, compressed_image, params);
      // 创建并发布压缩图像消息
      auto compressed_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
      compressed_msg->format = "jpeg";
      compressed_msg->data = compressed_image;
      compressed_msg->header.stamp = this->now();
      compressed_msg->header.frame_id = "camera_frame";
      // 发布压缩图像消息
      publisher_->publish(*compressed_msg);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "图像处理异常: %s", e.what());
    }
    
    // if (!image_processing_enabled_) {
    //     return;
    // }
    // try
    // {
    //   // 将 hbm_img_msgs::msg::HbmMsg1080P 转换为 OpenCV 图像
    //   cv::Mat nv12_image(cv::Size(640,480), CV_8UC1, const_cast<unsigned char*>(msg->data.data()));
    //   if (nv12_image.empty()) {
    //       RCLCPP_ERROR(this->get_logger(), "解码图像失败");
    //       return;
    //   }
    //   RCLCPP_INFO(this->get_logger(), "成功订阅图片");
    //   // 缩放图像，降低分辨率
    //   cv::Mat resized_image;
    //   cv::resize(nv12_image, resized_image, cv::Size(), 0.7, 0.7);
    //   // 压缩图像，降低质量参数以减少压缩时间
    //   std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 50}; //原本30
    //   std::vector<uchar> compressed_image;
    //   cv::imencode(".jpg", resized_image, compressed_image, params);
    //   // 创建并发布压缩图像消息
    //   auto compressed_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
    //   compressed_msg->format = "jpeg";
    //   compressed_msg->data = compressed_image;
    //   compressed_msg->header.stamp = this->now();
    //   compressed_msg->header.frame_id = "camera_frame";
    //   // 发布压缩图像消息
    //   publisher_->publish(*compressed_msg);
    // } catch (const std::exception &e) {
    //     RCLCPP_ERROR(this->get_logger(), "图像处理异常: %s", e.what());
    // }

    // if (!image_processing_enabled_) {
    //     return;
    // }
    // try
    // {
    //   // 将 hbm_img_msgs::msg::HbmMsg1080P 转换为 OpenCV 图像
    //   cv::Mat nv12_image(cv::Size(640,480), CV_8UC1, const_cast<unsigned char*>(msg->data.data()));
    //   if (nv12_image.empty()) {
    //       RCLCPP_ERROR(this->get_logger(), "解码图像失败");
    //       return;
    //   }
    //   RCLCPP_INFO(this->get_logger(), "成功订阅图片");
    //   // 压缩图像，降低质量参数以减少压缩时间
    //   std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 30}; //原本30
    //   std::vector<uchar> compressed_image;
    //   cv::imencode(".jpg", nv12_image, compressed_image, params);
    //   // 创建并发布压缩图像消息
    //   auto compressed_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
    //   compressed_msg->format = "jpeg";
    //   compressed_msg->data = compressed_image;
    //   compressed_msg->header.stamp = this->now();
    //   compressed_msg->header.frame_id = "camera_frame";
    //   // 发布压缩图像消息
    //   publisher_->publish(*compressed_msg);
    // } catch (const std::exception &e) {
    //     RCLCPP_ERROR(this->get_logger(), "图像处理异常: %s", e.what());
    // }
  }

  void qrCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg){
    if (!qr_processing_enabled_) {
      //RCLCPP_INFO(this->get_logger(), "二维码检测功能未启用，忽略图像");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "正在处理压缩图像，尝试识别二维码");
    cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    zbar::Image zbar_image(image.cols, image.rows, "Y800", image.data, image.cols * image.rows);
    scanner.scan(zbar_image);
    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {
      std_msgs::msg::String qr_msg;
      qr_msg.data = symbol->get_data();
      // if (qr_msg.data == "ClockWise" || qr_msg.data == "AntiClockWise") {
      //   auto message_ = std::make_shared<std_msgs::msg::Int32>();
      //   if(qr_msg.data == "ClockWise"){
      //     message_->data = 3;
      //     qr_publisher_->publish(*message_);
      //   }
      //   else {
      //     message_->data = 4;
      //     qr_publisher_->publish(*message_);
      //   }

      // 将字符串转换为整数
      int qr_value = std::stoi(qr_msg.data);
      
      if (1) {

        // auto message_ = std::make_shared<qr_mem::msg::SignSwitch>();
        auto message_ = std::make_shared<origincar_msg::msg::Sign>();
        
        // 奇数为顺时针(3)，偶数为逆时针(4)
        if (qr_value % 2 == 1) {
          // 奇数 - 顺时针
          message_->sign_data = 3;
          RCLCPP_INFO(this->get_logger(), "奇数二维码 %d - 顺时针", qr_value);
        } else {
          // 偶数 - 逆时针
          message_->sign_data = 4;
          RCLCPP_INFO(this->get_logger(), "偶数二维码 %d - 逆时针", qr_value);
        }
        qr_publisher_->publish(*message_);
        
        // 发布二维码原始内容和方向信息到上位机
        auto qr_content_msg = std::make_shared<std_msgs::msg::String>();
        if (qr_value % 2 == 1) {
            qr_content_msg->data = "二维码识别结果为: " + qr_msg.data + " 顺时针";
        } else {
            qr_content_msg->data = "二维码识别结果为: " + qr_msg.data + " 逆时针";
        }
        qr_content_publisher_->publish(*qr_content_msg);
        
        RCLCPP_INFO(this->get_logger(), "QR Code Detected: %s", qr_msg.data.c_str());
        qr_processing_enabled_ = false; // Stop after detecting specific QR codes
        image_processing_enabled_ = true;
        // 发布 "get_qr" 消息
      auto qr_racing_msg = std::make_shared<std_msgs::msg::String>();
      qr_racing_msg->data = "get_qr";
      RCLCPP_INFO(this->get_logger(), "识别到二维码，发布get_qr消息");
      qr_racing_publisher_->publish(*qr_racing_msg);
      RCLCPP_INFO(this->get_logger(), "get_qr消息发布完成");
      
      // 自动发送data=5消息，进入遥控状态
      auto foxglove_msg = std::make_shared<std_msgs::msg::Int32>();
      foxglove_msg->data = 5;
      RCLCPP_INFO(this->get_logger(), "自动发送data=5消息，进入遥控状态");
      foxglove_publisher_->publish(*foxglove_msg);
      RCLCPP_INFO(this->get_logger(), "data=5消息发布完成");
      
      // 取消订阅的进程
      qr_subscription_.reset(); 
      control_subscription_.reset();
        break;
      }
    }
  }

  void controlCallback(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "收到消息: %s", msg->data.c_str());
    if (msg->data == "start_scan") {
      RCLCPP_INFO(this->get_logger(), "开始二维码检测模式");
      qr_processing_enabled_ = true;//开始二维码检测
    }
  }

  void foxgloveCallback(const std_msgs::msg::Int32::SharedPtr msg){
    if (msg->data == 6) {
      image_processing_enabled_ = false;//停止图像转发
      foxglove_subscription_.reset();
      subscription_.reset();
    }
  }

  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr qr_subscription_;
  // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr qr_publisher_;
  // rclcpp::Publisher<qr_mem::msg::SignSwitch>::SharedPtr qr_publisher_;
  rclcpp::Publisher<origincar_msg::msg::Sign>::SharedPtr qr_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr foxglove_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_racing_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_content_publisher_; // 发布二维码原始内容
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr foxglove_publisher_; // 用于发送data=5消息
  int width_;
  int height_;
  bool image_processing_enabled_;
  bool qr_processing_enabled_;
  int frame_count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageCompressor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}