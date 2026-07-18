#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <vector>
#include <cmath>

namespace racing_control_2026
{

/**
 * @brief 航点结构体，包含位置、朝向、容差、超时、暂停和名称
 */
struct Waypoint {
    double x;                   // 目标 X 坐标（米，相对原点）
    double y;                   // 目标 Y 坐标（米，相对原点）
    double yaw;                 // 目标朝向（弧度），NAN 表示不指定
    double yaw_tolerance;       // 朝向容差（弧度）
    double timeout;             // 单航点超时（秒），0=不超时
    double distance_tolerance;  // 欧氏距离容差（米）
    double pause_duration;      // 到达后暂停秒数（>0 触发接近减速）
    std::string name;           // 航点名称，用于日志和阶段触发判断

    Waypoint(double x_, double y_, double yaw_ = NAN,
             double yaw_tol = 0.1, double timeout_ = 10.0,
             double dist_tol = 0.2, double pause_ = 0.0,
             const std::string& name_ = "")
        : x(x_), y(y_), yaw(yaw_),
          yaw_tolerance(yaw_tol), timeout(timeout_),
          distance_tolerance(dist_tol > 0 ? dist_tol : 0.1),
          pause_duration(pause_), name(name_) {}

    bool has_yaw_target() const { return !std::isnan(yaw); }
    bool has_pause() const { return pause_duration > 0.0; }
    double get_distance_tolerance() const {
        return distance_tolerance > 0 ? distance_tolerance : 0.1;
    }
};

/**
 * @brief Pure Pursuit 导航器
 *
 * 接收当前位姿和目标航点，输出 Twist 指令。
 * 不包含阶段管理逻辑，只负责"如何到达某个航点"。
 */
class Navigator {
public:
    Navigator();

    /**
     * @brief 初始化导航参数（从 ROS2 参数读取）
     * @param node 节点指针，用于声明/获取参数
     */
    void init(rclcpp::Node* node);

    /**
     * @brief 导航到目标航点
     * @param waypoint 目标航点
     * @param current_x 当前本地 X 坐标
     * @param current_y 当前本地 Y 坐标
     * @param current_yaw 当前本地朝向
     * @param current_wp_idx 当前航点索引（用于预瞄点计算）
     * @param waypoints 全部航点列表（用于路径段计算）
     * @param is_last_waypoint 是否为最后一个航点
     * @param dt 时间步长
     * @param[out] twist 输出的速度指令
     * @return true 表示已到达航点
     */
    bool navigate_to_waypoint(
        const Waypoint& waypoint,
        double current_x, double current_y, double current_yaw,
        int current_wp_idx,
        const std::vector<Waypoint>& waypoints,
        bool is_last_waypoint,
        double dt,
        geometry_msgs::msg::Twist& twist);

    /**
     * @brief 重置导航状态（加速度、预瞄滤波等）
     */
    void reset();

    // Getter for current linear speed (for external use)
    double get_current_linear_speed() const { return current_linear_speed_; }
    void set_current_linear_speed(double speed) { current_linear_speed_ = speed; }

    // Parameters (public for direct access by controller)
    double max_linear_speed_ = 0.8;
    double max_angular_speed_ = 3.0;
    double max_acceleration_ = 0.5;
    double lookahead_distance_ = 0.5;
    double min_lookahead_distance_ = 0.1;
    double min_turning_radius_ = 0.3;
    double dynamic_lookahead_max_ = 1.0;
    double dynamic_lookahead_min_ = 0.15;
    double dist_far_threshold_ = 1.0;
    double dist_near_threshold_ = 0.3;
    double lookahead_lpf_alpha_ = 0.15;
    double decel_distance_ = 1.0;
    double yaw_blend_start_distance_ = 1.0;
    double default_waypoint_tolerance_ = 0.1;
    double min_waypoint_tolerance_ = 0.05;
    bool enable_reverse_ = true;
    double max_reverse_speed_ = 0.3;
    double reverse_angle_threshold_ = 0.8;

private:
    // Pure Pursuit helpers
    bool find_lookahead_point(
        double lookahead_dist,
        double current_x, double current_y,
        int current_wp_idx,
        const std::vector<Waypoint>& waypoints,
        double& target_x, double& target_y);

    double compute_pure_pursuit_control(
        double target_x, double target_y,
        double current_x, double current_y, double current_yaw,
        double& linear_cmd, double dt);

    double decide_target_yaw(
        const Waypoint& waypoint, int current_idx,
        const std::vector<Waypoint>& waypoints,
        double current_yaw);

    static double normalize_angle(double angle);

    // State
    double current_linear_speed_ = 0.0;
    double lookahead_filtered_ = 0.5;
    bool initialized_ = false;

    // Logger reference (set during init)
    rclcpp::Logger logger_{rclcpp::get_logger("navigator")};
    rclcpp::Clock::SharedPtr clock_;
};

} // namespace racing_control_2026

#endif // NAVIGATOR_HPP
