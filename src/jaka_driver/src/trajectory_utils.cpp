#include "jaka_driver/trajectory_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
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

bool same_positions(
    const std::vector<double> & lhs,
    const std::vector<double> & rhs)
{
    return lhs.size() == rhs.size() &&
        std::equal(lhs.begin(), lhs.end(), rhs.begin());
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
    std::vector<double> previous_positions;
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
            current_time < previous_time)
        {
            error = "time_from_start 必须非负且不能倒退";
            return false;
        }
        if (current_time == previous_time &&
            !same_positions(point.positions, previous_positions))
        {
            error = "相同 time_from_start 不能对应不同关节位置";
            return false;
        }
        if (current_time > maximum_duration)
        {
            error = "轨迹持续时间超过驱动配置上限";
            return false;
        }
        previous_time = current_time;
        previous_positions = point.positions;
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

bool validate_servo_segments(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    double servo_period,
    unsigned int maximum_servo_steps,
    std::string & error)
{
    if (!std::isfinite(servo_period) || servo_period <= 0.0 ||
        maximum_servo_steps == 0U)
    {
        error = "servo 插补周期或单段步数上限配置无效";
        return false;
    }

    double previous_time = 0.0;
    for (const auto & point : trajectory.points)
    {
        const double current_time = duration_seconds(point.time_from_start);
        const auto steps = interpolation_steps(
            previous_time, current_time, servo_period);
        if (!steps)
        {
            error = "轨迹包含无法转换为 servo 周期的时间段";
            return false;
        }
        if (*steps > maximum_servo_steps)
        {
            error = "轨迹单段插补周期数超过驱动上限: steps=" +
                std::to_string(*steps) + ", maximum=" +
                std::to_string(maximum_servo_steps);
            return false;
        }
        previous_time = current_time;
    }
    return true;
}

TimedServoSchedule build_timed_servo_schedule(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::vector<std::string> & expected_joint_names,
    const std::vector<double> & initial_positions,
    double interpolation_cycle,
    unsigned int step_num,
    std::size_t maximum_samples)
{
    TimedServoSchedule schedule;
    if (!std::isfinite(interpolation_cycle) || interpolation_cycle <= 0.0 ||
        step_num == 0U || step_num > kMaximumServoStepNum ||
        maximum_samples == 0U ||
        initial_positions.size() != expected_joint_names.size() ||
        !finite_values(initial_positions) || trajectory.points.empty())
    {
        schedule.error = "servo 重采样配置、初始关节位置或轨迹无效";
        return schedule;
    }

    struct Knot
    {
        double time;
        std::vector<double> positions;
    };
    std::vector<Knot> knots{{0.0, initial_positions}};
    knots.reserve(trajectory.points.size() + 1U);
    for (const auto & point : trajectory.points)
    {
        const double time = duration_seconds(point.time_from_start);
        auto positions = reorder_joint_values(
            trajectory.joint_names, point.positions, expected_joint_names);
        if (!std::isfinite(time) || time < 0.0 ||
            positions.size() != expected_joint_names.size() ||
            !finite_values(positions) || time < knots.back().time)
        {
            schedule.error = "servo 重采样输入包含无效时间或关节位置";
            return schedule;
        }
        if (time == knots.back().time)
        {
            knots.back().positions = std::move(positions);
        }
        else
        {
            knots.push_back({time, std::move(positions)});
        }
    }

    const double command_period =
        interpolation_cycle * static_cast<double>(step_num);
    if (!std::isfinite(command_period) || command_period <= 0.0)
    {
        schedule.error = "servo 命令周期无效";
        return schedule;
    }

    schedule.planned_duration = knots.back().time;
    const double raw_sample_count = std::max(
        1.0, std::ceil(schedule.planned_duration / command_period));
    if (!std::isfinite(raw_sample_count) ||
        raw_sample_count > static_cast<double>(maximum_samples))
    {
        schedule.error = "servo 重采样点数超过配置上限";
        return schedule;
    }
    const auto sample_count = static_cast<std::size_t>(raw_sample_count);
    schedule.scheduled_duration =
        static_cast<double>(sample_count) * command_period;
    schedule.setpoints.reserve(sample_count);

    std::size_t right_index = knots.size() > 1U ? 1U : 0U;
    for (std::size_t sample_index = 0; sample_index < sample_count; ++sample_index)
    {
        const double reference_time = std::min(
            static_cast<double>(sample_index + 1U) * command_period,
            schedule.planned_duration);
        while (right_index + 1U < knots.size() &&
            knots[right_index].time < reference_time)
        {
            ++right_index;
        }

        std::vector<double> positions;
        if (knots.size() == 1U || reference_time >= knots.back().time)
        {
            positions = knots.back().positions;
        }
        else
        {
            const auto & right = knots[right_index];
            const auto & left = knots[right_index - 1U];
            const double ratio = (reference_time - left.time) /
                (right.time - left.time);
            positions.resize(expected_joint_names.size());
            for (std::size_t joint = 0; joint < positions.size(); ++joint)
            {
                positions[joint] = left.positions[joint] +
                    ratio * (right.positions[joint] - left.positions[joint]);
            }
        }

        schedule.setpoints.push_back({
            static_cast<double>(sample_index) * command_period,
            reference_time,
            step_num,
            std::move(positions)});
    }
    schedule.valid = true;
    return schedule;
}

namespace
{

double percentile(
    const std::vector<double> & sorted_values,
    double ratio)
{
    if (sorted_values.empty())
    {
        return 0.0;
    }
    const double raw_index =
        ratio * static_cast<double>(sorted_values.size() - 1U);
    const auto lower = static_cast<std::size_t>(std::floor(raw_index));
    const auto upper = static_cast<std::size_t>(std::ceil(raw_index));
    const double fraction = raw_index - static_cast<double>(lower);
    return sorted_values[lower] +
        fraction * (sorted_values[upper] - sorted_values[lower]);
}

}  // namespace

ServoTimingSummary summarize_servo_timing(
    const std::vector<ServoCallTiming> & calls,
    double lateness_threshold)
{
    ServoTimingSummary summary;
    if (!std::isfinite(lateness_threshold) || lateness_threshold < 0.0)
    {
        return summary;
    }

    std::vector<double> durations;
    durations.reserve(calls.size());
    for (const auto & call : calls)
    {
        if (!std::isfinite(call.scheduled_time) ||
            !std::isfinite(call.call_started_time) ||
            !std::isfinite(call.call_finished_time) ||
            call.call_started_time < call.scheduled_time ||
            call.call_finished_time < call.call_started_time)
        {
            continue;
        }
        const double lateness = call.call_started_time - call.scheduled_time;
        const double duration =
            call.call_finished_time - call.call_started_time;
        summary.maximum_lateness = std::max(
            summary.maximum_lateness, lateness);
        if (lateness > lateness_threshold)
        {
            ++summary.late_samples;
        }
        durations.push_back(duration);
    }

    std::sort(durations.begin(), durations.end());
    summary.samples = durations.size();
    if (!durations.empty())
    {
        summary.call_duration_p50 = percentile(durations, 0.50);
        summary.call_duration_p95 = percentile(durations, 0.95);
        summary.call_duration_p99 = percentile(durations, 0.99);
        summary.maximum_call_duration = durations.back();
    }
    return summary;
}

bool update_servo_overrun_state(
    double lateness,
    double maximum_lateness,
    std::size_t maximum_consecutive_overruns,
    std::size_t & consecutive_overruns)
{
    if (!std::isfinite(lateness) || !std::isfinite(maximum_lateness) ||
        maximum_lateness < 0.0)
    {
        return true;
    }
    if (lateness > maximum_lateness)
    {
        ++consecutive_overruns;
    }
    else
    {
        consecutive_overruns = 0U;
    }
    return consecutive_overruns > maximum_consecutive_overruns;
}

std::optional<double> endpoint_deadline_offset(
    double scheduled_duration,
    double send_completed_time,
    double goal_timeout)
{
    if (!std::isfinite(scheduled_duration) || scheduled_duration < 0.0 ||
        !std::isfinite(send_completed_time) || send_completed_time < 0.0 ||
        !std::isfinite(goal_timeout) || goal_timeout <= 0.0)
    {
        return std::nullopt;
    }
    return std::max(scheduled_duration, send_completed_time) + goal_timeout;
}

}  // namespace jaka_driver
