#ifndef OBSTACLE_AVOIDER_HPP
#define OBSTACLE_AVOIDER_HPP

#include "navigator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ai_msgs/msg/perception_targets.hpp>
#include <vector>

namespace racing_control_2026
{

/**
 * @brief 避障绕行管理器
 *
 * 订阅 YOLO 检测结果，当检测到障碍物时生成绕行航点。
 * 绕行完成后恢复原始航点序列。
 */
class ObstacleAvoider {
public:
    ObstacleAvoider();

    /**
     * @brief 初始化参数
     * @param node 节点指针
     */
    void init(rclcpp::Node* node);

    /**
     * @brief 处理 YOLO 检测结果（在 obstacle_callback 中调用）
     * @param msg 检测结果
     */
    void process_detection(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);

    /**
     * @brief 检查是否需要启动绕行
     * @param current_x 当前 X
     * @param current_y 当前 Y
     * @param current_yaw 当前朝向
     * @return true 表示已激活绕行，绕行航点已生成
     */
    bool check_and_enter_detour(double current_x, double current_y, double current_yaw);

    /**
     * @brief 检查绕行是否完成
     * @param current_wp_idx 当前航点索引（在绕行航点序列中）
     * @return true 表示绕行完成，应恢复原始航点
     */
    bool is_detour_complete(int current_wp_idx) const;

    /**
     * @brief 获取绕行航点列表
     */
    const std::vector<Waypoint>& get_detour_waypoints() const { return detour_waypoints_; }

    /**
     * @brief 是否正在绕行
     */
    bool is_active() const { return detour_active_; }

    /**
     * @brief 激活绕行模式
     */
    void set_active(bool active) { detour_active_ = active; }

    /**
     * @brief 获取绕行前的航点索引（用于恢复）
     */
    int get_saved_wp_idx() const { return saved_wp_idx_; }
    void set_saved_wp_idx(int idx) { saved_wp_idx_ = idx; }

    // Park detection result (Phase 4 YOLO stop)
    bool park_detected_ = false;
    int park_center_x_ = 0;
    int park_area_ = 0;

    // Parameters
    int avoid_area_ = 15000;
    int avoid_trigger_bottom_ = 230;
    double avoid_detour_lateral_offset_ = 0.35;
    double avoid_detour_forward_step_ = 0.8;
    double avoid_detour_cooldown_s_ = 3.0;
    double avoid_linear_speed_ = 0.35;
    bool enabled_ = true;

private:
    int calculate_actual_area(int max_barrel_area, int a_x);
    std::vector<Waypoint> compute_detour_waypoints(
        double current_x, double current_y, double current_yaw);

    // Detection state
    int max_barrel_area_ = 0;
    int actual_barrel_area_ = 0;
    int max_barrel_bottom_ = 0;
    int a_x_ = 0, a_y_ = 0;
    bool barrel_processed_ = false;

    // Detour state
    bool detour_active_ = false;
    bool is_left_ = false;
    std::vector<Waypoint> detour_waypoints_;
    int saved_wp_idx_ = 0;
    rclcpp::Time detour_cooldown_;

    bool initialized_ = false;
    rclcpp::Logger logger_{rclcpp::get_logger("obstacle_avoider")};
    rclcpp::Clock::SharedPtr clock_;
};

} // namespace racing_control_2026

#endif // OBSTACLE_AVOIDER_HPP
