#ifndef RACING_CONTROLLER_HPP
#define RACING_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ai_msgs/msg/perception_targets.hpp>

#include "navigator.hpp"
#include "route_manager.hpp"
#include "obstacle_avoider.hpp"
#include "phase_manager.hpp"
#include "external_services.hpp"

namespace racing_control_2026
{

/**
 * @brief 竞赛主控节点
 *
 * 整合 Navigator、RouteManager、ObstacleAvoider、PhaseManager、ExternalServices，
 * 实现 RacingLogic.txt 描述的四阶段竞赛流程。
 *
 * 优先级: Geofence > Obstacle Avoidance > Phase Logic > Navigation
 */
class RacingController : public rclcpp::Node {
public:
    RacingController();
    ~RacingController() = default;

    void publish_stop_command();

private:
    // ROS2 callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void obstacle_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);
    void control_loop();

    // Core functions
    void initialize_origin();
    void stop_robot();
    void emergency_stop();
    bool check_geofence();
    double normalize_angle(double angle);
    void global_to_local(double gx, double gy, double gyaw,
                         double& lx, double& ly, double& lyaw);

    // Sub-modules
    Navigator navigator_;
    RouteManager route_mgr_;
    ObstacleAvoider avoider_;
    PhaseManager phase_mgr_;
    ExternalServices ext_svc_;

    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr obstacle_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Origin
    bool origin_initialized_ = false;
    double origin_global_x_ = 0.0, origin_global_y_ = 0.0, origin_global_yaw_ = 0.0;

    // Current pose (global)
    double current_global_x_ = 0.0, current_global_y_ = 0.0, current_global_yaw_ = 0.0;

    // Current pose (local, relative to origin)
    double current_local_x_ = 0.0, current_local_y_ = 0.0, current_local_yaw_ = 0.0;

    // Navigation state
    int current_wp_idx_ = 0;
    rclcpp::Time waypoint_start_time_;
    bool is_pausing_ = false;
    rclcpp::Time pause_start_time_;

    // Detour state (for waypoint list management)
    bool detour_active_ = false;
    std::vector<Waypoint> saved_waypoints_;
    int saved_wp_idx_ = 0;

    // Parameters
    double control_frequency_ = 50.0;
    double waypoint_timeout_ = 20.0;

    // Geofence
    bool fence_enabled_ = true;
    double fence_x_min_ = -0.1, fence_x_max_ = 4.3;
    double fence_y_min_ = -0.15, fence_y_max_ = 4.3;

    // Timing
    rclcpp::Time last_control_time_;
    rclcpp::Time last_odom_time_;
};

} // namespace racing_control_2026

#endif // RACING_CONTROLLER_HPP
