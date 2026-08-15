#include "jaka_driver/trajectory_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace jaka_driver
{

namespace
{

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
    return static_cast<double>(duration.sec) +
        static_cast<double>(duration.nanosec) * 1e-9;
}

bool finite_values(const std::vector<double> & values)
{
    return std::all_of(
        values.begin(), values.end(),
        [](double value) {return std::isfinite(value);});
}

}  // namespace

bool validate_trajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::vector<std::string> & expected_joint_names,
    double maximum_duration,
    std::string & error)
{
    if (trajectory.points.empty())
    {
        error = "轨迹点不能为空";
        return false;
    }
    if (!std::isfinite(maximum_duration) || maximum_duration <= 0.0)
    {
        error = "轨迹最大持续时间配置无效";
        return false;
    }
    if (trajectory.joint_names.size() != expected_joint_names.size() ||
        std::set<std::string>(
            trajectory.joint_names.begin(), trajectory.joint_names.end()).size() !=
            expected_joint_names.size())
    {
        error = "轨迹必须且只能包含六个唯一关节名";
        return false;
    }
    for (const auto & expected_name : expected_joint_names)
    {
        if (std::find(
                trajectory.joint_names.begin(),
                trajectory.joint_names.end(),
                expected_name) == trajectory.joint_names.end())
        {
            error = "轨迹关节名与 JAKA S5 不匹配";
            return false;
        }
    }

    double previous_time = -1.0;
    for (const auto & point : trajectory.points)
    {
        if (point.positions.size() != trajectory.joint_names.size() ||
            (!point.velocities.empty() &&
             point.velocities.size() != trajectory.joint_names.size()) ||
            (!point.accelerations.empty() &&
             point.accelerations.size() != trajectory.joint_names.size()) ||
            (!point.effort.empty() &&
             point.effort.size() != trajectory.joint_names.size()) ||
            !finite_values(point.positions) ||
            !finite_values(point.velocities) ||
            !finite_values(point.accelerations) ||
            !finite_values(point.effort))
        {
            error = "轨迹点数组长度错误或包含非有限数值";
            return false;
        }

        const double current_time = duration_seconds(point.time_from_start);
        if (!std::isfinite(current_time) || current_time < 0.0 ||
            current_time <= previous_time)
        {
            error = "time_from_start 必须非负且严格递增";
            return false;
        }
        if (current_time > maximum_duration)
        {
            error = "轨迹持续时间超过驱动配置上限";
            return false;
        }
        previous_time = current_time;
    }
    return true;
}

std::vector<double> reorder_joint_values(
    const std::vector<std::string> & source_names,
    const std::vector<double> & source_values,
    const std::vector<std::string> & expected_joint_names)
{
    std::vector<double> reordered;
    reordered.reserve(expected_joint_names.size());
    for (const auto & expected_name : expected_joint_names)
    {
        const auto iterator = std::find(
            source_names.begin(), source_names.end(), expected_name);
        if (iterator == source_names.end())
        {
            return {};
        }
        const auto index = static_cast<std::size_t>(
            std::distance(source_names.begin(), iterator));
        if (index >= source_values.size())
        {
            return {};
        }
        reordered.push_back(source_values[index]);
    }
    return reordered;
}

std::optional<unsigned int> interpolation_steps(
    double previous_time,
    double current_time,
    double servo_period)
{
    const double interval = current_time - previous_time;
    if (!std::isfinite(interval) || !std::isfinite(servo_period) ||
        interval < 0.0 || servo_period <= 0.0)
    {
        return std::nullopt;
    }
    const double rounded = std::max(1.0, std::round(interval / servo_period));
    if (rounded > static_cast<double>(std::numeric_limits<unsigned int>::max()))
    {
        return std::nullopt;
    }
    return static_cast<unsigned int>(rounded);
}

}  // namespace jaka_driver
