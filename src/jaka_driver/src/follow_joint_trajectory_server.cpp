#include "jaka_driver/follow_joint_trajectory_server.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "jaka_driver/trajectory_utils.hpp"

namespace jaka_driver
{

namespace
{

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
    return static_cast<double>(duration.sec) +
        static_cast<double>(duration.nanosec) * 1e-9;
}

std::string sdk_error(int code)
{
    return "JAKA SDK error " + std::to_string(code);
}

struct TrackingSample
{
    std::string phase;
    std::size_t sample_index{0U};
    double elapsed{0.0};
    double controller_segment_start{0.0};
    double call_started_time{0.0};
    double call_finished_time{0.0};
    double call_duration{0.0};
    double queue_starvation{0.0};
    unsigned int step_num{0U};
    std::vector<double> desired;
    std::vector<double> actual;
    bool actual_valid{false};
    bool powered_on{false};
    bool enabled{false};
    int error_code{0};
    bool in_servo_mode{false};
    bool status_valid{false};
};

std::string make_telemetry_path()
{
    const auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    return "/tmp/jaka_trajectory_" + std::to_string(stamp) + ".csv";
}

bool write_tracking_csv(
    const std::string & path,
    const std::vector<TrackingSample> & samples)
{
    std::ofstream stream(path);
    if (!stream)
    {
        return false;
    }
    stream << "phase,sample_index,elapsed_s,controller_segment_start_s,"
              "call_started_time_s,call_finished_time_s,call_duration_s,"
              "queue_starvation_s,step_num";
    for (std::size_t joint = 0; joint < 6U; ++joint)
    {
        stream << ",desired_joint_" << joint + 1U;
    }
    for (std::size_t joint = 0; joint < 6U; ++joint)
    {
        stream << ",actual_joint_" << joint + 1U;
    }
    for (std::size_t joint = 0; joint < 6U; ++joint)
    {
        stream << ",error_joint_" << joint + 1U;
    }
    stream << ",maximum_absolute_error,actual_valid,powered_on,enabled,"
              "error_code,in_servo_mode,status_valid\n";
    stream << std::setprecision(12);
    for (const auto & sample : samples)
    {
        stream << sample.phase
               << ',' << sample.sample_index
               << ',' << sample.elapsed
               << ',' << sample.controller_segment_start
               << ',' << sample.call_started_time
               << ',' << sample.call_finished_time
               << ',' << sample.call_duration
               << ',' << sample.queue_starvation
               << ',' << sample.step_num;
        for (std::size_t joint = 0; joint < 6U; ++joint)
        {
            if (joint < sample.desired.size())
            {
                stream << ',' << sample.desired[joint];
            }
            else
            {
                stream << ',';
            }
        }
        for (std::size_t joint = 0; joint < 6U; ++joint)
        {
            if (sample.actual_valid && joint < sample.actual.size())
            {
                stream << ',' << sample.actual[joint];
            }
            else
            {
                stream << ',';
            }
        }
        double maximum_error = 0.0;
        for (std::size_t joint = 0; joint < 6U; ++joint)
        {
            if (sample.actual_valid && joint < sample.desired.size() &&
                joint < sample.actual.size())
            {
                const double error = sample.desired[joint] - sample.actual[joint];
                maximum_error = std::max(maximum_error, std::abs(error));
                stream << ',' << error;
            }
            else
            {
                stream << ',';
            }
        }
        if (sample.actual_valid)
        {
            stream << ',' << maximum_error;
        }
        else
        {
            stream << ',';
        }
        stream << ',' << sample.actual_valid
               << ',' << sample.powered_on
               << ',' << sample.enabled
               << ',' << sample.error_code
               << ',' << sample.in_servo_mode
               << ',' << sample.status_valid << '\n';
    }
    return stream.good();
}

}  // namespace

FollowJointTrajectoryServer::FollowJointTrajectoryServer(
    const rclcpp::Node::SharedPtr & node,
    JAKAZuRobot & robot,
    std::atomic<bool> & sdk_logged_in,
    std::atomic<ControlOwner> & control_owner,
    std::mutex & session_mutex,
    std::string action_name)
    : node_(node),
      robot_(robot),
      sdk_logged_in_(sdk_logged_in),
      control_owner_(control_owner),
      session_mutex_(session_mutex)
{
    goal_tolerance_ = node_->declare_parameter<double>(
        "trajectory_goal_tolerance", goal_tolerance_);
    goal_timeout_ = node_->declare_parameter<double>(
        "trajectory_goal_timeout", goal_timeout_);
    const auto maximum_servo_step_num = node_->declare_parameter<int64_t>(
        "trajectory_maximum_servo_step_num",
        static_cast<int64_t>(maximum_servo_step_num_));
    const auto maximum_servo_samples = node_->declare_parameter<int64_t>(
        "maximum_servo_samples", static_cast<int64_t>(maximum_servo_samples_));
    maximum_trajectory_duration_ = node_->declare_parameter<double>(
        "maximum_trajectory_duration", maximum_trajectory_duration_);
    feedback_period_ = node_->declare_parameter<double>(
        "trajectory_feedback_period", feedback_period_);
    servo_filter_cutoff_hz_ = node_->declare_parameter<double>(
        "trajectory_servo_filter_cutoff_hz", servo_filter_cutoff_hz_);
    maximum_queue_starvation_ = node_->declare_parameter<double>(
        "trajectory_maximum_queue_starvation", maximum_queue_starvation_);
    const auto maximum_consecutive_starvations =
        node_->declare_parameter<int64_t>(
        "trajectory_maximum_consecutive_starvations",
        static_cast<int64_t>(maximum_consecutive_starvations_));

    if (!std::isfinite(goal_tolerance_) || goal_tolerance_ <= 0.0 ||
        !std::isfinite(goal_timeout_) || goal_timeout_ <= 0.0 ||
        !std::isfinite(maximum_trajectory_duration_) ||
        maximum_trajectory_duration_ <= 0.0 ||
        maximum_servo_step_num <= 0 ||
        maximum_servo_step_num > static_cast<int64_t>(kMaximumServoStepNum) ||
        maximum_servo_samples <= 0 || maximum_consecutive_starvations < 0 ||
        !std::isfinite(feedback_period_) || feedback_period_ <= 0.0 ||
        !std::isfinite(servo_filter_cutoff_hz_) ||
        servo_filter_cutoff_hz_ < 0.0 ||
        !std::isfinite(maximum_queue_starvation_) ||
        maximum_queue_starvation_ < 0.0)
    {
        throw std::invalid_argument("轨迹 Action 的容差和周期参数必须大于零");
    }
    maximum_servo_step_num_ =
        static_cast<unsigned int>(maximum_servo_step_num);
    maximum_servo_samples_ = static_cast<std::size_t>(maximum_servo_samples);
    maximum_consecutive_starvations_ =
        static_cast<std::size_t>(maximum_consecutive_starvations);

    server_ = rclcpp_action::create_server<Action>(
        node_,
        action_name,
        std::bind(
            &FollowJointTrajectoryServer::handle_goal,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        std::bind(
            &FollowJointTrajectoryServer::handle_cancel,
            this,
            std::placeholders::_1),
        std::bind(
            &FollowJointTrajectoryServer::handle_accepted,
            this,
            std::placeholders::_1));

    RCLCPP_INFO(
        node_->get_logger(),
        "JAKA trajectory Action ready: %s, goal_tolerance=%.6f rad, "
        "goal_timeout=%.3f s, maximum_step_num=%u, servo_filter=%.3f Hz, "
        "maximum_queue_starvation=%.3f s, allowed_consecutive_starvations=%zu",
        action_name.c_str(), goal_tolerance_, goal_timeout_,
        maximum_servo_step_num_, servo_filter_cutoff_hz_,
        maximum_queue_starvation_, maximum_consecutive_starvations_);
}

FollowJointTrajectoryServer::~FollowJointTrajectoryServer()
{
    shutting_down_.store(true);
    cancel_requested_.store(true);
    if (goal_active_.load())
    {
        abort_motion_and_exit_servo();
    }
    release_control(control_owner_, ControlOwner::kTrajectory);
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable())
    {
        worker_.join();
    }
}

int FollowJointTrajectoryServer::request_stop()
{
    cancel_requested_.store(true);
    return abort_motion_and_exit_servo();
}

rclcpp_action::GoalResponse FollowJointTrajectoryServer::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Action::Goal> goal)
{
    bool expected = false;
    if (!goal_active_.compare_exchange_strong(expected, true))
    {
        RCLCPP_WARN(node_->get_logger(), "已有轨迹 Goal 正在执行，拒绝新 Goal");
        return rclcpp_action::GoalResponse::REJECT;
    }

    std::string error;
    if (!validate_goal(*goal, error) || !robot_ready(error))
    {
        RCLCPP_ERROR(node_->get_logger(), "拒绝轨迹 Goal: %s", error.c_str());
        goal_active_.store(false);
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (!try_acquire_control(control_owner_, ControlOwner::kTrajectory))
    {
        error = std::string("控制权正由 ") +
            control_owner_name(control_owner_.load()) + " 持有";
        RCLCPP_ERROR(node_->get_logger(), "拒绝轨迹 Goal: %s", error.c_str());
        goal_active_.store(false);
        return rclcpp_action::GoalResponse::REJECT;
    }

    cancel_requested_.store(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FollowJointTrajectoryServer::handle_cancel(
    const std::shared_ptr<GoalHandle>)
{
    if (!goal_active_.load())
    {
        return rclcpp_action::CancelResponse::REJECT;
    }

    // 此处仅接受取消请求。回调返回后 ROS Action 才会把 Goal 转换为
    // CANCELING，执行线程检测到 is_canceling() 后再停止 SDK 并设置 CANCELED。
    // 若在回调返回前抢先调用 canceled()，rclcpp_action 会因状态转换非法而终止进程。
    RCLCPP_INFO(node_->get_logger(), "接受轨迹 Action 取消请求");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FollowJointTrajectoryServer::handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable())
    {
        worker_.join();
    }
    worker_ = std::thread(
        &FollowJointTrajectoryServer::execute, this, goal_handle);
}

bool FollowJointTrajectoryServer::validate_goal(
    const Action::Goal & goal,
    std::string & error) const
{
    return validate_trajectory(
        goal.trajectory,
        expected_joint_names_,
        maximum_trajectory_duration_,
        error);
}

bool FollowJointTrajectoryServer::robot_ready(std::string & error)
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (!sdk_logged_in_.load())
    {
        error = "JAKA SDK 尚未登录";
        return false;
    }

    RobotStatus_simple status{};
    const int ret = robot_.get_robot_status_simple(&status);
    if (ret != 0)
    {
        error = "读取机器人状态失败: " + sdk_error(ret);
        return false;
    }
    if (!status.powered_on || !status.enabled)
    {
        error = "机器人尚未上电并使能";
        return false;
    }
    if (status.errcode != 0)
    {
        error = "机器人存在控制器故障: " + std::to_string(status.errcode);
        return false;
    }
    return true;
}

void FollowJointTrajectoryServer::log_sdk_state_locked(
    const std::string & context)
{
    RobotStatus_simple status{};
    BOOL in_servo = FALSE;
    const int status_ret = robot_.get_robot_status_simple(&status);
    const int servo_ret = robot_.is_in_servomove(&in_servo);
    if (status_ret != 0 || servo_ret != 0)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "%s: SDK 状态读取失败, robot_status=%d, servo_status=%d",
            context.c_str(), status_ret, servo_ret);
        return;
    }
    RCLCPP_INFO(
        node_->get_logger(),
        "%s: powered=%s, enabled=%s, errcode=%d, in_servo_mode=%s",
        context.c_str(), status.powered_on ? "true" : "false",
        status.enabled ? "true" : "false", status.errcode,
        in_servo ? "true" : "false");
}

int FollowJointTrajectoryServer::abort_motion_and_exit_servo()
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (!sdk_logged_in_.load())
    {
        return 0;
    }
    log_sdk_state_locked("故障停止前");
    const int abort_ret = robot_.motion_abort();
    log_sdk_state_locked("motion_abort 后");
    const int servo_ret = robot_.servo_move_enable(FALSE);
    log_sdk_state_locked("退出 servo mode 后");
    return abort_ret != 0 ? abort_ret : servo_ret;
}

int FollowJointTrajectoryServer::exit_servo_mode()
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (!sdk_logged_in_.load())
    {
        return 0;
    }
    log_sdk_state_locked("正常退出 servo mode 前");
    const int ret = robot_.servo_move_enable(FALSE);
    log_sdk_state_locked("正常退出 servo mode 后");
    return ret;
}

std::vector<double> FollowJointTrajectoryServer::reordered_positions(
    const std::vector<std::string> & names,
    const std::vector<double> & positions) const
{
    return reorder_joint_values(names, positions, expected_joint_names_);
}

void FollowJointTrajectoryServer::publish_feedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const trajectory_msgs::msg::JointTrajectoryPoint & desired,
    const JointValue & actual,
    const std::chrono::steady_clock::duration & elapsed) const
{
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->header.stamp = node_->now();
    feedback->joint_names = expected_joint_names_;
    feedback->desired = desired;
    feedback->desired.positions = reordered_positions(
        goal_handle->get_goal()->trajectory.joint_names,
        desired.positions);
    feedback->actual.positions.resize(expected_joint_names_.size());
    feedback->error.positions.resize(expected_joint_names_.size());
    for (std::size_t index = 0; index < expected_joint_names_.size(); ++index)
    {
        feedback->actual.positions[index] = actual.jVal[index];
        feedback->error.positions[index] =
            feedback->desired.positions[index] - actual.jVal[index];
    }
    feedback->actual.time_from_start =
        static_cast<builtin_interfaces::msg::Duration>(rclcpp::Duration(
            std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed)));
    goal_handle->publish_feedback(feedback);
}

void FollowJointTrajectoryServer::execute(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    auto result = std::make_shared<Action::Result>();
    std::vector<TrackingSample> telemetry;
    std::vector<ServoCallTiming> servo_call_timings;
    const std::string telemetry_path = make_telemetry_path();
    bool telemetry_written = false;
    bool servo_mode_entered = false;
    const auto flush_telemetry = [&]() {
        if (telemetry_written)
        {
            return;
        }
        telemetry_written = true;
        if (write_tracking_csv(telemetry_path, telemetry))
        {
            RCLCPP_INFO(
                node_->get_logger(), "轨迹跟踪遥测 CSV: %s",
                telemetry_path.c_str());
        }
        else
        {
            RCLCPP_WARN(
                node_->get_logger(), "无法写入轨迹跟踪遥测 CSV: %s",
                telemetry_path.c_str());
        }
    };
    const auto finish = [this]() {
        release_control(control_owner_, ControlOwner::kTrajectory);
        goal_active_.store(false);
    };
    const auto abort_goal = [&](int code, const std::string & message) {
        if (servo_mode_entered)
        {
            abort_motion_and_exit_servo();
        }
        flush_telemetry();
        result->error_code = code;
        result->error_string = message;
        goal_handle->abort(result);
        finish();
    };
    const auto cancel_goal = [&](const std::string & message) {
        if (servo_mode_entered)
        {
            abort_motion_and_exit_servo();
        }
        flush_telemetry();
        result->error_code = Action::Result::SUCCESSFUL;
        result->error_string = message;
        goal_handle->canceled(result);
        finish();
    };

    JointValue initial{};
    int initial_ret;
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        initial_ret = robot_.get_joint_position(&initial);
    }
    if (initial_ret != 0)
    {
        abort_goal(
            Action::Result::PATH_TOLERANCE_VIOLATED,
            "读取 servo 轨迹初始关节位置失败: " + sdk_error(initial_ret));
        return;
    }

    const auto & trajectory = goal_handle->get_goal()->trajectory;
    std::vector<double> initial_positions(expected_joint_names_.size());
    for (std::size_t joint = 0; joint < initial_positions.size(); ++joint)
    {
        initial_positions[joint] = initial.jVal[joint];
    }
    const auto schedule = build_queued_servo_schedule(
        trajectory, expected_joint_names_, initial_positions,
        kJakaServoInterpolationCycle, maximum_servo_step_num_,
        maximum_servo_samples_);
    if (!schedule.valid)
    {
        abort_goal(
            Action::Result::INVALID_GOAL,
            "构建 JAKA servo 调度失败: " + schedule.error);
        return;
    }

    int filter_ret;
    int enable_ret;
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        log_sdk_state_locked("进入 servo mode 前");
        filter_ret = servo_filter_cutoff_hz_ > 0.0 ?
            robot_.servo_move_use_joint_LPF(servo_filter_cutoff_hz_) :
            robot_.servo_move_use_none_filter();
        if (filter_ret != 0)
        {
            enable_ret = filter_ret;
        }
        else
        {
            enable_ret = robot_.servo_move_enable(TRUE);
        }
        log_sdk_state_locked("进入 servo mode 后");
    }
    if (filter_ret != 0)
    {
        abort_goal(
            Action::Result::PATH_TOLERANCE_VIOLATED,
            "配置 servo 滤波器失败: " + sdk_error(filter_ret));
        return;
    }
    if (enable_ret != 0)
    {
        abort_goal(
            Action::Result::PATH_TOLERANCE_VIOLATED,
            "进入 servo mode 失败: " + sdk_error(enable_ret));
        return;
    }
    servo_mode_entered = true;

    const std::string filter_description = servo_filter_cutoff_hz_ > 0.0 ?
        std::to_string(servo_filter_cutoff_hz_) + " Hz LPF" : "none";
    RCLCPP_INFO(
        node_->get_logger(),
        "开始连续填充 JAKA servo_j 队列: source_points=%zu, segments=%zu, "
        "planned_duration=%.6f s, scheduled_duration=%.6f s, "
        "maximum_step_num=%u, filter=%s",
        trajectory.points.size(), schedule.setpoints.size(),
        schedule.planned_duration, schedule.scheduled_duration,
        maximum_servo_step_num_, filter_description.c_str());

    const auto dispatch_started_at = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point controller_started_at;
    bool controller_started = false;
    std::size_t consecutive_starvations = 0U;

    for (std::size_t sample_index = 0;
        sample_index < schedule.setpoints.size(); ++sample_index)
    {
        const auto & setpoint = schedule.setpoints[sample_index];
        if (goal_handle->is_canceling())
        {
            cancel_goal("servo 队列填充期间已取消并停止机器人");
            return;
        }
        if (shutting_down_.load() || cancel_requested_.load() || !rclcpp::ok())
        {
            abort_goal(
                Action::Result::PATH_TOLERANCE_VIOLATED,
                "servo 队列填充期间被外部停止");
            return;
        }

        JointValue target{};
        for (std::size_t joint = 0; joint < setpoint.positions.size(); ++joint)
        {
            target.jVal[joint] = setpoint.positions[joint];
        }

        const auto call_started_at = std::chrono::steady_clock::now();
        int servo_ret;
        {
            std::lock_guard<std::mutex> lock(session_mutex_);
            servo_ret = robot_.servo_j(
                &target, MoveMode::ABS, setpoint.step_num);
        }
        const auto call_finished_at = std::chrono::steady_clock::now();
        if (servo_ret != 0)
        {
            abort_goal(
                Action::Result::PATH_TOLERANCE_VIOLATED,
                "servo_j 队列填充失败: " + sdk_error(servo_ret));
            return;
        }
        if (!controller_started)
        {
            controller_started_at = call_started_at;
            controller_started = true;
        }
        const double call_started_time =
            std::chrono::duration<double>(
            call_started_at - dispatch_started_at).count();
        const double call_finished_time =
            std::chrono::duration<double>(
            call_finished_at - dispatch_started_at).count();
        const double controller_elapsed = std::max(
            0.0,
            std::chrono::duration<double>(
            call_finished_at - controller_started_at).count());
        const double queue_starvation = sample_index == 0U ? 0.0 : std::max(
            0.0, controller_elapsed - setpoint.controller_start_time);
        servo_call_timings.push_back({
            call_finished_time - call_started_time, queue_starvation});
        telemetry.push_back({
            "queue", sample_index + 1U, controller_elapsed,
            setpoint.controller_start_time,
            call_started_time, call_finished_time,
            call_finished_time - call_started_time, queue_starvation,
            setpoint.step_num, setpoint.positions, {}, false,
            false, false, 0, false, false});
        if (update_servo_starvation_state(
                queue_starvation, maximum_queue_starvation_,
                maximum_consecutive_starvations_, consecutive_starvations))
        {
            std::ostringstream message;
            message << "servo 控制柜队列连续饥饿: segment="
                    << sample_index + 1U << '/' << schedule.setpoints.size()
                    << ", starvation=" << queue_starvation
                    << " s, maximum=" << maximum_queue_starvation_
                    << " s, consecutive=" << consecutive_starvations
                    << ", allowed=" << maximum_consecutive_starvations_;
            abort_goal(
                Action::Result::PATH_TOLERANCE_VIOLATED,
                message.str());
            return;
        }
        if (queue_starvation > maximum_queue_starvation_)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "servo 队列单次饥饿: segment=%zu/%zu, starvation=%.6f s, "
                "consecutive=%zu/%zu",
                sample_index + 1U, schedule.setpoints.size(), queue_starvation,
                consecutive_starvations, maximum_consecutive_starvations_);
        }
    }

    const auto send_completed_at = std::chrono::steady_clock::now();
    const auto timing_summary = summarize_servo_timing(
        servo_call_timings, maximum_queue_starvation_);
    const double dispatch_elapsed = std::chrono::duration<double>(
        send_completed_at - dispatch_started_at).count();
    const double send_completed_time = std::chrono::duration<double>(
        send_completed_at - controller_started_at).count();
    RCLCPP_INFO(
        node_->get_logger(),
        "servo 队列填充完成: sent=%zu, starved_segments=%zu, "
        "max_starvation=%.6f s, call_ms[p50=%.3f p95=%.3f p99=%.3f "
        "max=%.3f], dispatch_elapsed=%.6f s, controller_elapsed=%.6f s, "
        "queued_duration=%.6f s",
        timing_summary.samples, timing_summary.starved_samples,
        timing_summary.maximum_queue_starvation,
        timing_summary.call_duration_p50 * 1000.0,
        timing_summary.call_duration_p95 * 1000.0,
        timing_summary.call_duration_p99 * 1000.0,
        timing_summary.maximum_call_duration * 1000.0,
        dispatch_elapsed, send_completed_time, schedule.scheduled_duration);

    double allowed_timeout = goal_timeout_;
    const double requested_timeout = duration_seconds(
        goal_handle->get_goal()->goal_time_tolerance);
    if (requested_timeout > 0.0)
    {
        allowed_timeout = requested_timeout;
    }
    const auto expected_finish = controller_started_at +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(schedule.scheduled_duration));
    const auto deadline_offset = endpoint_deadline_offset(
        schedule.scheduled_duration, send_completed_time, allowed_timeout);
    if (!deadline_offset)
    {
        abort_goal(
            Action::Result::INVALID_GOAL,
            "终点检查超时参数无效");
        return;
    }
    const auto deadline = controller_started_at +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(*deadline_offset));
    RCLCPP_INFO(
        node_->get_logger(),
        "终点监控窗口: queue_remaining=%.6f s, goal_timeout=%.6f s, "
        "deadline_from_controller_start=%.6f s",
        std::max(0.0, schedule.scheduled_duration - send_completed_time),
        allowed_timeout, *deadline_offset);
    const auto final_positions = reordered_positions(
        trajectory.joint_names, trajectory.points.back().positions);
    EndpointProgress last_progress;
    auto next_terminal_log = std::chrono::steady_clock::now();
    auto next_terminal_telemetry = std::chrono::steady_clock::now();

    while (true)
    {
        if (goal_handle->is_canceling())
        {
            cancel_goal("终点检查期间轨迹被取消");
            return;
        }
        if (shutting_down_.load() || cancel_requested_.load() ||
            !rclcpp::ok())
        {
            abort_goal(
                Action::Result::PATH_TOLERANCE_VIOLATED,
                "轨迹执行期间被外部停止");
            return;
        }

        JointValue actual{};
        int ret;
        {
            std::lock_guard<std::mutex> lock(session_mutex_);
            ret = robot_.get_joint_position(&actual);
        }
        if (ret != 0)
        {
            abort_goal(
                Action::Result::GOAL_TOLERANCE_VIOLATED,
                "终点检查读取反馈失败: " + sdk_error(ret));
            return;
        }

        std::vector<double> actual_positions(expected_joint_names_.size());
        for (std::size_t joint = 0; joint < actual_positions.size(); ++joint)
        {
            actual_positions[joint] = actual.jVal[joint];
        }
        const auto now = std::chrono::steady_clock::now();
        const double controller_elapsed = std::max(
            0.0,
            std::chrono::duration<double>(
            now - controller_started_at).count());
        const auto desired_positions = sample_queued_servo_schedule(
            schedule, initial_positions, controller_elapsed);
        if (!desired_positions)
        {
            abort_goal(
                Action::Result::GOAL_TOLERANCE_VIOLATED,
                "无法按控制柜时间轴计算期望关节位置");
            return;
        }
        const auto progress = calculate_endpoint_progress(
            initial_positions, final_positions, actual_positions);
        if (!progress.valid)
        {
            abort_goal(
                Action::Result::GOAL_TOLERANCE_VIOLATED,
                "终点进度计算输入无效");
            return;
        }
        last_progress = progress;

        trajectory_msgs::msg::JointTrajectoryPoint desired;
        desired.positions = reorder_joint_values(
            expected_joint_names_, *desired_positions, trajectory.joint_names);
        desired.time_from_start = static_cast<builtin_interfaces::msg::Duration>(
            rclcpp::Duration::from_seconds(std::min(
            controller_elapsed, schedule.scheduled_duration)));
        publish_feedback(
            goal_handle, desired, actual, now - controller_started_at);
        if (now >= next_terminal_telemetry)
        {
            RobotStatus_simple status{};
            BOOL in_servo = FALSE;
            int status_ret;
            int mode_ret;
            {
                std::lock_guard<std::mutex> lock(session_mutex_);
                status_ret = robot_.get_robot_status_simple(&status);
                mode_ret = robot_.is_in_servomove(&in_servo);
            }
            if (status_ret != 0 || mode_ret != 0)
            {
                abort_goal(
                    Action::Result::GOAL_TOLERANCE_VIOLATED,
                    "终点检查状态读取失败: status=" +
                    std::to_string(status_ret) + ", servo_mode=" +
                    std::to_string(mode_ret));
                return;
            }
            telemetry.push_back({
                now < expected_finish ? "tracking" : "endpoint",
                schedule.setpoints.size(), controller_elapsed,
                schedule.scheduled_duration, 0.0, 0.0, 0.0, 0.0,
                schedule.setpoints.back().step_num,
                *desired_positions, actual_positions, true,
                static_cast<bool>(status.powered_on),
                static_cast<bool>(status.enabled), status.errcode,
                static_cast<bool>(in_servo), true});
            next_terminal_telemetry = now +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(feedback_period_));
        }
        if (now >= next_terminal_log)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "%s: elapsed=%.3f s, wait=%.3f s, "
                "commanded=%.9f rad, achieved=%.9f rad, completion=%.2f%%, "
                "max_error=%.9f rad, error_joint=%s",
                now < expected_finish ? "轨迹跟踪" : "终点收敛",
                controller_elapsed,
                std::max(
                    0.0,
                    std::chrono::duration<double>(now - expected_finish).count()),
                progress.maximum_commanded_delta,
                progress.achieved_delta_on_command_joint,
                progress.completion_ratio * 100.0,
                progress.maximum_absolute_error,
                expected_joint_names_[progress.maximum_error_joint].c_str());
            next_terminal_log = now + std::chrono::seconds(1);
        }
        if (now >= expected_finish &&
            progress.maximum_absolute_error <= goal_tolerance_)
        {
            const int stop_ret = exit_servo_mode();
            flush_telemetry();
            if (stop_ret != 0)
            {
                result->error_code = Action::Result::GOAL_TOLERANCE_VIOLATED;
                result->error_string =
                    "到达终点但退出 servo mode 失败: " + sdk_error(stop_ret);
                goal_handle->abort(result);
            }
            else
            {
                result->error_code = Action::Result::SUCCESSFUL;
                std::ostringstream message;
                message << "轨迹执行成功: commanded="
                        << progress.maximum_commanded_delta
                        << " rad, achieved="
                        << progress.achieved_delta_on_command_joint
                        << " rad, completion="
                        << progress.completion_ratio * 100.0
                        << "%, max_error="
                        << progress.maximum_absolute_error << " rad";
                result->error_string = message.str();
                goal_handle->succeed(result);
            }
            finish();
            return;
        }
        if (now >= deadline)
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::ostringstream error_details;
    for (std::size_t joint = 0;
        joint < last_progress.absolute_errors.size(); ++joint)
    {
        if (joint > 0U)
        {
            error_details << ", ";
        }
        error_details << expected_joint_names_[joint] << '='
                      << last_progress.absolute_errors[joint];
    }
    abort_goal(
        Action::Result::GOAL_TOLERANCE_VIOLATED,
        "最终关节误差在超时前未进入容差: " +
        expected_joint_names_[last_progress.maximum_error_joint] + " error=" +
        std::to_string(last_progress.maximum_absolute_error) +
        " rad, tolerance=" + std::to_string(goal_tolerance_) +
        " rad, commanded=" +
        std::to_string(last_progress.maximum_commanded_delta) +
        " rad, achieved=" +
        std::to_string(last_progress.achieved_delta_on_command_joint) +
        " rad, completion=" +
        std::to_string(last_progress.completion_ratio * 100.0) +
        "%, all_errors=[" +
        error_details.str() + "]");
}

}  // namespace jaka_driver
