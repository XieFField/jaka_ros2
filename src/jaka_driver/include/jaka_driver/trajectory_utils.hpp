#ifndef JAKA_DRIVER__TRAJECTORY_UTILS_HPP_
#define JAKA_DRIVER__TRAJECTORY_UTILS_HPP_

#include <optional>
#include <cstddef>
#include <string>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace jaka_driver
{

constexpr double kJakaServoInterpolationCycle = 0.008;
constexpr unsigned int kMaximumServoStepNum = 50U;

struct TimedServoSetpoint
{
    double command_time{0.0};
    double reference_time{0.0};
    unsigned int step_num{1U};
    std::vector<double> positions;
};

struct TimedServoSchedule
{
    bool valid{false};
    std::string error;
    double planned_duration{0.0};
    double scheduled_duration{0.0};
    std::vector<TimedServoSetpoint> setpoints;
};

struct ServoCallTiming
{
    double scheduled_time{0.0};
    double call_started_time{0.0};
    double call_finished_time{0.0};
};

struct ServoTimingSummary
{
    std::size_t samples{0U};
    std::size_t late_samples{0U};
    double maximum_lateness{0.0};
    double call_duration_p50{0.0};
    double call_duration_p95{0.0};
    double call_duration_p99{0.0};
    double maximum_call_duration{0.0};
};

// 与 JAKA S5 轨迹执行相关的纯校验和换序函数，保持与 SDK 解耦，便于离线测试。
bool validate_trajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::vector<std::string> & expected_joint_names,
    double maximum_duration,
    std::string & error);

std::vector<double> reorder_joint_values(
    const std::vector<std::string> & source_names,
    const std::vector<double> & source_values,
    const std::vector<std::string> & expected_joint_names);

// 返回从 previous_time 到 current_time 所需的 JAKA 8 ms 插补周期数。
std::optional<unsigned int> interpolation_steps(
    double previous_time,
    double current_time,
    double servo_period);

// 限制单次 servo_j 调用覆盖的插补周期数，避免异常长的单段命令阻塞 SDK。
bool validate_servo_segments(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    double servo_period,
    unsigned int maximum_servo_steps,
    std::string & error);

// 将 MoveIt 轨迹按 step_num * 8 ms 的主机命令周期重采样。step_num 同时
// 传给 servo_j，让控制柜在相同持续时间内完成本段内部插补。
TimedServoSchedule build_timed_servo_schedule(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::vector<std::string> & expected_joint_names,
    const std::vector<double> & initial_positions,
    double interpolation_cycle,
    unsigned int step_num,
    std::size_t maximum_samples);

ServoTimingSummary summarize_servo_timing(
    const std::vector<ServoCallTiming> & calls,
    double lateness_threshold);

// 返回 true 表示连续超限次数已经超过允许值，应停止发送过期设定点。
bool update_servo_overrun_state(
    double lateness,
    double maximum_lateness,
    std::size_t maximum_consecutive_overruns,
    std::size_t & consecutive_overruns);

std::optional<double> endpoint_deadline_offset(
    double scheduled_duration,
    double send_completed_time,
    double goal_timeout);

}  // namespace jaka_driver

#endif  // JAKA_DRIVER__TRAJECTORY_UTILS_HPP_
