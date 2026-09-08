#include "jaka_driver/native_joint_move_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace jaka_driver
{

bool validate_native_joint_move_goal(
    const NativeJointMoveGoal & goal,
    std::string & error)
{
    if (goal.target_positions.size() != 6U)
    {
        error = "target_positions 必须恰好包含 6 个关节";
        return false;
    }
    if (!std::all_of(
            goal.target_positions.begin(), goal.target_positions.end(),
            [](double value) {return std::isfinite(value);}))
    {
        error = "target_positions 包含非有限值";
        return false;
    }
    if (!std::isfinite(goal.speed) || goal.speed <= 0.0)
    {
        error = "speed 必须为有限正数";
        return false;
    }
    if (!std::isfinite(goal.acceleration) || goal.acceleration <= 0.0)
    {
        error = "acceleration 必须为有限正数";
        return false;
    }
    if (!std::isfinite(goal.endpoint_tolerance) ||
        goal.endpoint_tolerance <= 0.0)
    {
        error = "endpoint_tolerance 必须为有限正数";
        return false;
    }
    if (!std::isfinite(goal.timeout) || goal.timeout <= 0.0)
    {
        error = "timeout 必须为有限正数";
        return false;
    }
    error.clear();
    return true;
}

bool validate_native_motion_readiness(
    const NativeMotionReadiness & readiness,
    std::string & error)
{
    if (!readiness.powered || !readiness.enabled || readiness.error_code != 0)
    {
        error = "机器人未满足上电、使能、无错误条件";
        return false;
    }
    if (!readiness.program_idle || readiness.queue_depth != 0 ||
        readiness.active_queue_depth != 0 || readiness.in_estop ||
        readiness.in_collision || readiness.in_drag_mode)
    {
        error = "机器人不是可接收原生 PTP 的空闲状态";
        return false;
    }
    error.clear();
    return true;
}

double maximum_joint_error(
    const std::vector<double> & target,
    const std::vector<double> & actual)
{
    if (target.size() != actual.size() || target.empty())
    {
        return std::numeric_limits<double>::infinity();
    }
    double result = 0.0;
    for (std::size_t index = 0; index < target.size(); ++index)
    {
        if (!std::isfinite(target[index]) || !std::isfinite(actual[index]))
        {
            return std::numeric_limits<double>::infinity();
        }
        result = std::max(result, std::abs(target[index] - actual[index]));
    }
    return result;
}

double estimate_native_joint_move_duration(
    double maximum_joint_delta,
    double programmed_speed,
    double programmed_acceleration,
    double rapid_rate)
{
    if (!std::isfinite(maximum_joint_delta) || maximum_joint_delta < 0.0 ||
        !std::isfinite(programmed_speed) || programmed_speed <= 0.0 ||
        !std::isfinite(programmed_acceleration) ||
        programmed_acceleration <= 0.0 || !std::isfinite(rapid_rate) ||
        rapid_rate <= 0.0 || rapid_rate > 1.0)
    {
        return std::numeric_limits<double>::infinity();
    }
    const double speed = programmed_speed * rapid_rate;
    const double acceleration = programmed_acceleration * rapid_rate;
    const double ramp_distance = speed * speed / acceleration;
    return maximum_joint_delta <= ramp_distance ?
        2.0 * std::sqrt(maximum_joint_delta / acceleration) :
        2.0 * speed / acceleration +
        (maximum_joint_delta - ramp_distance) / speed;
}

}  // namespace jaka_driver
