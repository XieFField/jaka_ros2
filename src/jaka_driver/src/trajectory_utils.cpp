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
    constexpr double kIntegerRatioTolerance = 1e-9;
    const double steps = std::max(
        1.0,
        std::ceil(interval / servo_period - kIntegerRatioTolerance));
    if (steps > static_cast<double>(std::numeric_limits<unsigned int>::max()))
    {
        return std::nullopt;
    }
    return static_cast<unsigned int>(steps);
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

QueuedServoSchedule build_queued_servo_schedule(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::vector<std::string> & expected_joint_names,
    const std::vector<double> & initial_positions,
    double interpolation_cycle,
    unsigned int maximum_step_num,
    std::size_t maximum_samples)
{
    QueuedServoSchedule schedule;
    if (!std::isfinite(interpolation_cycle) || interpolation_cycle <= 0.0 ||
        maximum_step_num == 0U || maximum_step_num > kMaximumServoStepNum ||
        maximum_samples == 0U ||
        initial_positions.size() != expected_joint_names.size() ||
        !finite_values(initial_positions) || trajectory.points.empty())
    {
        schedule.error = "servo 重采样配置、初始关节位置或轨迹无效";
        return schedule;
    }

    double previous_source_time = 0.0;
    std::vector<double> previous_positions = initial_positions;
    double controller_time = 0.0;
    for (const auto & point : trajectory.points)
    {
        const double source_time = duration_seconds(point.time_from_start);
        auto positions = reorder_joint_values(
            trajectory.joint_names, point.positions, expected_joint_names);
        if (!std::isfinite(source_time) || source_time < previous_source_time ||
            positions.size() != expected_joint_names.size() ||
            !finite_values(positions))
        {
            schedule.error = "servo 重采样输入包含无效时间或关节位置";
            return schedule;
        }

        if (source_time == previous_source_time)
        {
            if (source_time == 0.0)
            {
                previous_positions = std::move(positions);
                continue;
            }
            if (!same_positions(positions, previous_positions))
            {
                schedule.error = "相同时间的 servo 分段位置不一致";
                return schedule;
            }
            continue;
        }

        const auto total_steps = interpolation_steps(
            previous_source_time, source_time, interpolation_cycle);
        if (!total_steps)
        {
            schedule.error = "轨迹时间段无法转换为控制柜插补周期";
            return schedule;
        }
        const auto segment_count = static_cast<unsigned int>(
            (*total_steps + maximum_step_num - 1U) / maximum_step_num);
        const unsigned int base_steps = *total_steps / segment_count;
        const unsigned int extra_steps = *total_steps % segment_count;
        unsigned int accumulated_steps = 0U;
        for (unsigned int segment = 0U; segment < segment_count; ++segment)
        {
            const unsigned int segment_steps =
                base_steps + (segment < extra_steps ? 1U : 0U);
            accumulated_steps += segment_steps;
            const double ratio = static_cast<double>(accumulated_steps) /
                static_cast<double>(*total_steps);
            std::vector<double> segment_positions(positions.size());
            for (std::size_t joint = 0; joint < positions.size(); ++joint)
            {
                segment_positions[joint] = previous_positions[joint] +
                    ratio * (positions[joint] - previous_positions[joint]);
            }
            const double segment_duration = interpolation_cycle *
                static_cast<double>(segment_steps);
            if (schedule.setpoints.size() >= maximum_samples)
            {
                schedule.error = "servo 队列分段数超过配置上限";
                return schedule;
            }
            schedule.setpoints.push_back({
                controller_time,
                controller_time + segment_duration,
                previous_source_time + ratio *
                (source_time - previous_source_time),
                segment_steps,
                std::move(segment_positions)});
            controller_time += segment_duration;
        }
        previous_source_time = source_time;
        previous_positions = std::move(positions);
    }

    if (schedule.setpoints.empty())
    {
        schedule.error = "轨迹没有需要提交的非零时长 servo 分段";
        return schedule;
    }
    schedule.planned_duration = previous_source_time;
    schedule.scheduled_duration = controller_time;
    schedule.valid = true;
    return schedule;
}

std::optional<std::vector<double>> sample_queued_servo_schedule(
    const QueuedServoSchedule & schedule,
    const std::vector<double> & initial_positions,
    double controller_elapsed)
{
    if (!schedule.valid || schedule.setpoints.empty() ||
        !std::isfinite(controller_elapsed) || controller_elapsed < 0.0 ||
        initial_positions.empty() || !finite_values(initial_positions))
    {
        return std::nullopt;
    }

    double previous_time = 0.0;
    const std::vector<double> * previous_positions = &initial_positions;
    for (const auto & setpoint : schedule.setpoints)
    {
        if (setpoint.positions.size() != initial_positions.size() ||
            !finite_values(setpoint.positions) ||
            !std::isfinite(setpoint.controller_finish_time) ||
            setpoint.controller_finish_time <= previous_time)
        {
            return std::nullopt;
        }
        if (controller_elapsed <= setpoint.controller_finish_time)
        {
            const double ratio = std::clamp(
                (controller_elapsed - previous_time) /
                (setpoint.controller_finish_time - previous_time),
                0.0, 1.0);
            std::vector<double> positions(initial_positions.size());
            for (std::size_t joint = 0; joint < positions.size(); ++joint)
            {
                positions[joint] = (*previous_positions)[joint] +
                    ratio * (setpoint.positions[joint] -
                    (*previous_positions)[joint]);
            }
            return positions;
        }
        previous_time = setpoint.controller_finish_time;
        previous_positions = &setpoint.positions;
    }
    return schedule.setpoints.back().positions;
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
    double starvation_threshold)
{
    ServoTimingSummary summary;
    if (!std::isfinite(starvation_threshold) || starvation_threshold < 0.0)
    {
        return summary;
    }

    std::vector<double> durations;
    durations.reserve(calls.size());
    for (const auto & call : calls)
    {
        if (!std::isfinite(call.call_duration) || call.call_duration < 0.0 ||
            !std::isfinite(call.queue_starvation) || call.queue_starvation < 0.0)
        {
            continue;
        }
        summary.maximum_queue_starvation = std::max(
            summary.maximum_queue_starvation, call.queue_starvation);
        if (call.queue_starvation > starvation_threshold)
        {
            ++summary.starved_samples;
        }
        durations.push_back(call.call_duration);
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

bool update_servo_starvation_state(
    double starvation,
    double maximum_starvation,
    std::size_t maximum_consecutive_starvations,
    std::size_t & consecutive_starvations)
{
    if (!std::isfinite(starvation) || !std::isfinite(maximum_starvation) ||
        maximum_starvation < 0.0)
    {
        return true;
    }
    if (starvation > maximum_starvation)
    {
        ++consecutive_starvations;
    }
    else
    {
        consecutive_starvations = 0U;
    }
    return consecutive_starvations > maximum_consecutive_starvations;
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

EndpointProgress calculate_endpoint_progress(
    const std::vector<double> & initial_positions,
    const std::vector<double> & target_positions,
    const std::vector<double> & actual_positions)
{
    EndpointProgress progress;
    if (initial_positions.empty() ||
        initial_positions.size() != target_positions.size() ||
        initial_positions.size() != actual_positions.size() ||
        !finite_values(initial_positions) || !finite_values(target_positions) ||
        !finite_values(actual_positions))
    {
        return progress;
    }

    progress.absolute_errors.resize(initial_positions.size());
    for (std::size_t joint = 0; joint < initial_positions.size(); ++joint)
    {
        const double commanded_delta = std::abs(
            target_positions[joint] - initial_positions[joint]);
        if (commanded_delta > progress.maximum_commanded_delta)
        {
            progress.maximum_commanded_delta = commanded_delta;
            progress.maximum_command_joint = joint;
        }

        const double target_error = std::abs(
            target_positions[joint] - actual_positions[joint]);
        progress.absolute_errors[joint] = target_error;
        if (target_error > progress.maximum_absolute_error)
        {
            progress.maximum_absolute_error = target_error;
            progress.maximum_error_joint = joint;
        }
    }

    const auto command_joint = progress.maximum_command_joint;
    const double signed_command =
        target_positions[command_joint] - initial_positions[command_joint];
    const double signed_actual_delta =
        actual_positions[command_joint] - initial_positions[command_joint];
    progress.achieved_delta_on_command_joint =
        signed_command >= 0.0 ? signed_actual_delta : -signed_actual_delta;
    progress.completion_ratio = progress.maximum_commanded_delta > 0.0 ?
        progress.achieved_delta_on_command_joint /
        progress.maximum_commanded_delta : 1.0;
    progress.valid = true;
    return progress;
}

}  // namespace jaka_driver
