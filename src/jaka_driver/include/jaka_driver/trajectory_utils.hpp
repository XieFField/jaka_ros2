#ifndef JAKA_DRIVER__TRAJECTORY_UTILS_HPP_
#define JAKA_DRIVER__TRAJECTORY_UTILS_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace jaka_driver
{

constexpr double kJakaServoInterpolationCycle = 0.008;
constexpr unsigned int kMaximumServoStepNum = 50U;

struct QueuedServoSetpoint
{
    double controller_start_time{0.0};
    double controller_finish_time{0.0};
    double source_reference_time{0.0};
    unsigned int step_num{1U};
    std::vector<double> positions;
    std::size_t source_point_index{0U};
    unsigned int split_segment_index{0U};
    unsigned int split_segment_count{1U};
    double planned_segment_duration{0.0};
    double scheduled_segment_duration{0.0};
    std::vector<double> implicit_velocities;
    std::vector<double> implicit_accelerations;
    std::vector<double> source_velocities;
    std::vector<double> source_accelerations;
};

struct QueuedServoSchedule
{
    bool valid{false};
    std::string error;
    double planned_duration{0.0};
    double scheduled_duration{0.0};
    std::vector<double> source_start_positions;
    std::vector<QueuedServoSetpoint> setpoints;
};

struct ServoScheduleDiagnostics
{
    bool valid{false};
    std::string error;
    double duration_error{0.0};
    double maximum_start_position_error{0.0};
    std::size_t maximum_start_error_joint{0U};
    unsigned int minimum_step_num{0U};
    unsigned int maximum_step_num{0U};
    std::vector<double> maximum_absolute_velocity;
    std::vector<double> maximum_absolute_acceleration;
    std::vector<std::size_t> positive_velocity_segments;
    std::vector<std::size_t> negative_velocity_segments;
    std::vector<std::size_t> velocity_sign_changes;
};

struct ServoCallTiming
{
    double call_duration{0.0};
    double queue_starvation{0.0};
};

struct ServoTimingSummary
{
    std::size_t samples{0U};
    std::size_t starved_samples{0U};
    double maximum_queue_starvation{0.0};
    double call_duration_p50{0.0};
    double call_duration_p95{0.0};
    double call_duration_p99{0.0};
    double maximum_call_duration{0.0};
};

struct EndpointProgress
{
    bool valid{false};
    std::size_t maximum_command_joint{0U};
    std::size_t maximum_error_joint{0U};
    double maximum_commanded_delta{0.0};
    double achieved_delta_on_command_joint{0.0};
    double completion_ratio{0.0};
    double maximum_absolute_error{0.0};
    std::vector<double> absolute_errors;
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

// 返回覆盖 previous_time 到 current_time 所需的 JAKA 插补周期数；向上取整，
// 确保量化后的控制柜时间轴不会短于 MoveIt 源轨迹。
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

// 将 MoveIt 轨迹按 step_num * 8 ms 的控制柜插补周期重采样。所有分段应按
// SDK 要求连续提交，由控制柜按照 controller_start/finish_time 顺序执行。
QueuedServoSchedule build_queued_servo_schedule(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::vector<std::string> & expected_joint_names,
    const std::vector<double> & initial_positions,
    double interpolation_cycle,
    unsigned int maximum_step_num,
    std::size_t maximum_samples);

ServoScheduleDiagnostics analyze_queued_servo_schedule(
    const QueuedServoSchedule & schedule,
    const std::vector<double> & actual_initial_positions,
    double velocity_deadband = 1e-9);

bool write_queued_servo_schedule_csv(
    const std::string & path,
    const QueuedServoSchedule & schedule,
    const std::vector<std::string> & joint_names,
    std::string & error);

bool write_joint_trajectory_csv(
    const std::string & path,
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    std::string & error);

std::optional<std::vector<double>> sample_queued_servo_schedule(
    const QueuedServoSchedule & schedule,
    const std::vector<double> & initial_positions,
    double controller_elapsed);

ServoTimingSummary summarize_servo_timing(
    const std::vector<ServoCallTiming> & calls,
    double starvation_threshold);

// 返回 true 表示连续队列饥饿次数已经超过允许值，应停止填充控制柜队列。
bool update_servo_starvation_state(
    double starvation,
    double maximum_starvation,
    std::size_t maximum_consecutive_starvations,
    std::size_t & consecutive_starvations);

std::optional<double> effective_endpoint_margin(
    double configured_minimum,
    double requested_margin);

// The deadline is the later of the scheduled controller finish and the final
// queue call, plus the configured endpoint convergence margin.
std::optional<double> endpoint_deadline_offset(
    double scheduled_duration,
    double send_completed_time,
    double goal_timeout);

EndpointProgress calculate_endpoint_progress(
    const std::vector<double> & initial_positions,
    const std::vector<double> & target_positions,
    const std::vector<double> & actual_positions);

}  // namespace jaka_driver

#endif  // JAKA_DRIVER__TRAJECTORY_UTILS_HPP_
