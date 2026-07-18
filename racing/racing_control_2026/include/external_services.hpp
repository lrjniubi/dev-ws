#ifndef EXTERNAL_SERVICES_HPP
#define EXTERNAL_SERVICES_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace racing_control_2026
{

/**
 * @brief 外部服务协调器
 *
 * 封装所有外部 ROS2 服务客户端和话题，包括：
 * - 扫码服务
 * - 人形立牌识别服务
 * - 阶段广播话题
 *
 * 服务不可用时 graceful fallback（日志警告，继续默认行为）。
 */
class ExternalServices {
public:
    ExternalServices();

    /**
     * @brief 初始化：创建服务客户端和话题发布/订阅
     * @param node 节点指针
     */
    void init(rclcpp::Node* node);

    /**
     * @brief 发布当前竞赛阶段
     * @param phase_name 阶段名称
     */
    void publish_phase(const std::string& phase_name);

    /**
     * @brief 触发扫码节点开始识别
     * @return true 表示服务调用成功
     */
    bool trigger_qr_scan();

    /**
     * @brief 停止二维码节点检测
     * @return true 表示服务调用成功
     */
    bool stop_qr_scan();

    /**
     * @brief 获取二维码识别结果
     * @return 二维码数字，-1 表示未收到
     */
    int get_qr_result() const { return qr_result_; }

    /**
     * @brief 是否收到了二维码结果
     */
    bool has_qr_result() const { return qr_result_ >= 0; }

    /**
     * @brief 清除二维码结果
     */
    void clear_qr_result() { qr_result_ = -1; }

    /**
     * @brief 触发人形立牌识别节点
     * @return true 表示服务调用成功
     */
    bool trigger_sign_detection();

    /**
     * @brief 关闭人形立牌识别节点
     * @return true 表示服务调用成功
     */
    bool stop_sign_detection();

    // Configuration
    bool enabled_ = true;

private:
    void qr_result_callback(const std_msgs::msg::Int32::SharedPtr msg);

    // Phase topic publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub_;

    // QR scan service clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr qr_scan_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr qr_scan_stop_client_;

    // QR result subscriber
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr qr_result_sub_;
    int qr_result_ = -1;

    // Sign detection service clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sign_trigger_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sign_stop_client_;

    rclcpp::Logger logger_{rclcpp::get_logger("external_services")};
    rclcpp::Node* node_ = nullptr;
    bool initialized_ = false;
};

} // namespace racing_control_2026

#endif // EXTERNAL_SERVICES_HPP
