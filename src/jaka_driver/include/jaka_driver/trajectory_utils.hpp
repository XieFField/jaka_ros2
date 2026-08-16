#ifndef JAKA_DRIVER__TRAJECTORY_UTILS_HPP_
#define JAKA_DRIVER__TRAJECTORY_UTILS_HPP_

#include <optional>
#include <string>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace jaka_driver
{

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

}  // namespace jaka_driver

#endif  // JAKA_DRIVER__TRAJECTORY_UTILS_HPP_
