#ifndef PHASE_MANAGER_HPP
#define PHASE_MANAGER_HPP

#include "route_manager.hpp"
#include "external_services.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace racing_control_2026
{

/**
 * @brief 竞赛阶段枚举
 */
enum class Phase {
    INIT = 0,
    PHASE_1 = 1,  // 大厅
    PHASE_2 = 2,  // 二维码
    PHASE_3 = 3,  // 赛道环行
    PHASE_4 = 4,  // 返回起点
    COMPLETED = 5
};

/**
 * @brief 阶段管理器
 *
 * 管理四阶段竞赛状态机，处理阶段转换、超时、特殊触发逻辑。
 * 支持调试模式：可指定起始阶段和强制方向。
 */
class PhaseManager {
public:
    PhaseManager();

    /**
     * @brief 初始化阶段管理器
     * @param node 节点指针
     * @param route_mgr 路线管理器
     * @param ext_svc 外部服务协调器
     */
    void init(rclcpp::Node* node, RouteManager* route_mgr, ExternalServices* ext_svc);

    /**
     * @brief 更新阶段状态（每个控制循环调用）
     * @param current_x 当前本地 X
     * @param current_y 当前本地 Y
     * @param current_wp_idx 当前航点索引（在当前激活航点中）
     * @param wp_reached 当前航点是否已到达
     * @param phase_timeout 当前航点是否超时
     * @return 当前阶段
     */
    Phase update(double current_x, double current_y,
                 int current_wp_idx, bool wp_reached, bool phase_timeout);

    /**
     * @brief 获取当前阶段
     */
    Phase get_current_phase() const { return current_phase_; }

    /**
     * @brief 获取阶段名称字符串
     */
    std::string get_phase_name() const;

    /**
     * @brief 阶段完成时调用（通知外部服务等）
     */
    void on_phase_enter(Phase new_phase);

    /**
     * @brief 检查航点名称是否匹配触发条件（人形识别等）
     */
    void check_waypoint_triggers(const std::string& wp_name);

    // Debug parameters
    int start_phase_ = 1;
    std::string direction_str_ = "auto";
    bool enable_external_services_ = true;

    // Phase-specific config
    double phase1_skip_x_ = 1.95;
    double phase1_timeout_ = 20.0;
    double phase2_timeout_ = 20.0;
    double phase2_qr_pause_ = 0.5;
    std::string phase3_default_dir_ = "cw";
    std::string phase3_sign_after_wp_ = "直道中段";
    std::string phase3_sign_stop_wp_ = "第三直角";
    double phase3_sign_stop_delay_ = 1.0;
    bool phase4_yolo_stop_ = true;
    double phase4_timeout_ = 20.0;

    // Phase timers
    rclcpp::Time phase_start_time_;
    bool phase_timer_set_ = false;

    // Sign detection state
    bool sign_detection_triggered_ = false;
    bool sign_detection_stopped_ = false;
    rclcpp::Time sign_stop_timer_;
    bool sign_stop_timer_set_ = false;

private:
    Phase current_phase_ = Phase::INIT;
    RouteManager* route_mgr_ = nullptr;
    ExternalServices* ext_svc_ = nullptr;
    rclcpp::Logger logger_{rclcpp::get_logger("phase_manager")};
    rclcpp::Clock::SharedPtr clock_;
    bool initialized_ = false;
};

} // namespace racing_control_2026

#endif // PHASE_MANAGER_HPP
