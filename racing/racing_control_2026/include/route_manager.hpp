#ifndef ROUTE_MANAGER_HPP
#define ROUTE_MANAGER_HPP

#include "navigator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <map>

namespace racing_control_2026
{

enum class RouteDirection { CW, CCW, AUTO };

/**
 * @brief 路线管理器
 *
 * 启动时从 YAML 一次性加载全部航点，按名称后缀建立各阶段的索引范围。
 * 阶段切换时只切换索引，不重新加载航点。
 */
class RouteManager {
public:
    RouteManager();

    /**
     * @brief 初始化：从 ROS2 参数加载全部航点并建立索引分区
     * @param node 节点指针
     */
    void init(rclcpp::Node* node);

    /**
     * @brief 设置当前阶段和方向，更新激活的航点索引范围
     * @param phase 阶段编号 (1-4)
     * @param direction 方向（仅 Phase 3 有效）
     */
    void set_phase(int phase, RouteDirection direction = RouteDirection::AUTO);

    /**
     * @brief 获取当前激活的航点子集
     */
    const std::vector<Waypoint>& get_active_waypoints() const { return active_waypoints_; }

    /**
     * @brief 获取当前激活的起始索引
     */
    int get_active_start_idx() const { return active_start_idx_; }

    /**
     * @brief 获取全部航点列表
     */
    const std::vector<Waypoint>& get_all_waypoints() const { return all_waypoints_; }

    /**
     * @brief 根据二维码数字选择方向
     * @return CW 或 CCW
     */
    RouteDirection select_direction(int qr_code_number) const;

    /**
     * @brief 获取默认方向
     */
    RouteDirection get_default_direction() const { return default_direction_; }

    /**
     * @brief 查找航点名称在当前激活航点中的索引
     * @param name 航点名称（不含 _cw/_ccw 后缀）
     * @return 索引，未找到返回 -1
     */
    int find_waypoint_index(const std::string& name) const;

    bool is_initialized() const { return initialized_; }

private:
    void load_waypoints(rclcpp::Node* node);
    void build_phase_indices();
    void update_active_waypoints();

    // All waypoints (loaded once at startup)
    std::vector<Waypoint> all_waypoints_;

    // Phase index ranges
    struct PhaseRange {
        int start_idx = -1;
        int end_idx = -1;  // inclusive
    };
    std::map<int, PhaseRange> phase_ranges_;       // phase 1, 2, 4
    PhaseRange phase3_cw_range_;
    PhaseRange phase3_ccw_range_;

    // Currently active
    int active_start_idx_ = 0;
    int active_end_idx_ = -1;
    std::vector<Waypoint> active_waypoints_;
    int current_phase_ = 1;
    RouteDirection current_direction_ = RouteDirection::AUTO;
    RouteDirection default_direction_ = RouteDirection::CW;

    bool initialized_ = false;
    rclcpp::Logger logger_{rclcpp::get_logger("route_manager")};
};

} // namespace racing_control_2026

#endif // ROUTE_MANAGER_HPP
