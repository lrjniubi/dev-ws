#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

namespace racing_control_2026
{

/**
 * @brief PID 控制器（带抗积分饱和）
 * 用于角速度控制、姿态调整等场景
 */
class PIDController {
public:
    PIDController(double kp, double ki, double kd,
                  double output_min, double output_max);

    /**
     * @brief 计算 PID 输出
     * @param error 当前误差
     * @param dt 时间步长（秒）
     * @return PID 控制量
     */
    double compute(double error, double dt);

    /**
     * @brief 重置积分和误差历史
     */
    void reset();

private:
    double kp_, ki_, kd_;
    double output_min_, output_max_;
    double integral_;
    double prev_error_;
    double prev_time_;
    bool first_run_;
};

} // namespace racing_control_2026

#endif // PID_CONTROLLER_HPP
