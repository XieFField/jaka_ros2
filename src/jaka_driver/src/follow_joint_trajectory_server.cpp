#include "jaka_driver/follow_joint_trajectory_server.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
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
    double scheduled_time{0.0};
    double call_started_time{0.0};
    double call_finished_time{0.0};
    double call_duration{0.0};
    double lateness{0.0};
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
    stream << "phase,sample_index,elapsed_s,scheduled_time_s,"
              "call_started_time_s,call_finished_time_s,call_duration_s,"
              "lateness_s,step_num";
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
               << ',' << sample.scheduled_time
               << ',' << sample.call_started_time
               << ',' << sample.call_finished_time
               << ',' << sample.call_duration
               << ',' << sample.lateness
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
    const auto servo_step_num = node_->declare_parameter<int64_t>(
        "trajectory_servo_step_num", static_cast<int64_t>(servo_step_num_));
    const auto maximum_servo_samples = node_->declare_parameter<int64_t>(
        "maximum_servo_samples", static_cast<int64_t>(maximum_servo_samples_));
    maximum_trajectory_duration_ = node_->declare_parameter<double>(
        "maximum_trajectory_duration", maximum_trajectory_duration_);
    feedback_period_ = node_->declare_parameter<double>(
        "trajectory_feedback_period", feedback_period_);
    maximum_lateness_ = node_->declare_parameter<double>(
        "trajectory_maximum_lateness", maximum_lateness_);
    const auto maximum_consecutive_overruns = node_->declare_parameter<int64_t>(
        "trajectory_maximum_consecutive_overruns",
        static_cast<int64_t>(maximum_consecutive_overruns_));

    if (!(goal_tolerance_ > 0.0) || !(goal_timeout_ > 0.0) ||
        !(maximum_trajectory_duration_ > 0.0) ||
        servo_step_num <= 0 ||
        servo_step_num > static_cast<int64_t>(kMaximumServoStepNum) ||
        maximum_servo_samples <= 0 || maximum_consecutive_overruns < 0 ||
        !std::isfinite(feedback_period_) || feedback_period_ <= 0.0 ||
        !std::isfinite(maximum_lateness_) || maximum_lateness_ < 0.0)
    {
        throw std::invalid_argument("轨迹 Action 的容差和周期参数必须大于零");
    }
    servo_step_num_ = static_cast<unsigned int>(servo_step_num);
    maximum_servo_samples_ = static_cast<std::size_t>(maximum_servo_samples);
    maximum_consecutive_overruns_ =
        static_cast<std::size_t>(maximum_consecutive_overruns);

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
        "JAKA trajectory Action ready: %s, step_num=%u, command_period=%.3f s, "
        "maximum_lateness=%.3f s",
        action_name.c_str(), servo_step_num_,
        kJakaServoInterpolationCycle * static_cast<double>(servo_step_num_),
        maximum_lateness_);
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

bool FollowJointTrajectoryServer::wait_until(
    const std::chrono::steady_clock::time_point & deadline,
    const std::shared_ptr<GoalHandle> & goal_handle)
{
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (shutting_down_.load() || cancel_requested_.load() ||
            goal_handle->is_canceling() || !rclcpp::ok())
        {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
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
    const auto schedule = build_timed_servo_schedule(
        trajectory, expected_joint_names_, initial_positions,
        kJakaServoInterpolationCycle, servo_step_num_, maximum_servo_samples_);
    if (!schedule.valid)
    {
        abort_goal(
            Action::Result::INVALID_GOAL,
            "构建 JAKA servo 调度失败: " + schedule.error);
        return;
    }

    int enable_ret;
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        log_sdk_state_locked("进入 servo mode 前");
        enable_ret = robot_.servo_move_enable(TRUE);
        log_sdk_state_locked("进入 servo mode 后");
    }
    if (enable_ret != 0)
    {
        abort_goal(
            Action::Result::PATH_TOLERANCE_VIOLATED,
            "进入 servo mode 失败: " + sdk_error(enable_ret));
        return;
    }
    servo_mode_entered = true;

    RCLCPP_INFO(
        node_->get_logger(),
        "开始定时下发 JAKA servo_j: source_points=%zu, servo_samples=%zu, "
        "planned_duration=%.6f s, scheduled_duration=%.6f s, "
        "step_num=%u, command_period=%.6f s",
        trajectory.points.size(), schedule.setpoints.size(),
        schedule.planned_duration, schedule.scheduled_duration,
        servo_step_num_,
        kJakaServoInterpolationCycle * static_cast<double>(servo_step_num_));

    const auto started_at = std::chrono::steady_clock::now();
    auto next_feedback_at = started_at;
    std::size_t consecutive_overruns = 0U;

    for (std::size_t sample_index = 0;
        sample_index < schedule.setpoints.size(); ++sample_index)
    {
        const auto & setpoint = schedule.setpoints[sample_index];
        if (goal_handle->is_canceling())
        {
            cancel_goal("定时轨迹下发期间已取消并停止机器人");
            return;
        }
        if (shutting_down_.load() || cancel_requested_.load() || !rclcpp::ok())
        {
            abort_goal(
                Action::Result::PATH_TOLERANCE_VIOLATED,
                "定时轨迹下发期间被外部停止");
            return;
        }
        const auto command_deadline = started_at +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(setpoint.command_time));
        if (!wait_until(command_deadline, goal_handle))
        {
            if (goal_handle->is_canceling())
            {
                cancel_goal("定时轨迹下发期间已取消并停止机器人");
            }
            else
            {
                abort_goal(
                    Action::Result::PATH_TOLERANCE_VIOLATED,
                    "定时轨迹下发期间被外部停止");
            }
            return;
        }

        const auto before_call = std::chrono::steady_clock::now();
        const double lateness = std::max(
            0.0,
            std::chrono::duration<double>(before_call - command_deadline).count());
        if (update_servo_overrun_state(
                lateness, maximum_lateness_,
                maximum_consecutive_overruns_, consecutive_overruns))
        {
            std::ostringstream message;
            message << "servo 调度连续超限，停止发送过期设定点: sample="
                    << sample_index + 1U << '/' << schedule.setpoints.size()
                    << ", lateness=" << lateness
                    << " s, maximum=" << maximum_lateness_
                    << " s, consecutive=" << consecutive_overruns
                    << ", allowed=" << maximum_consecutive_overruns_;
            abort_goal(Action::Result::PATH_TOLERANCE_VIOLATED, message.str());
            return;
        }
        if (lateness > maximum_lateness_)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "servo 调度单次超限: sample=%zu/%zu, lateness=%.6f s, "
                "consecutive=%zu/%zu",
                sample_index + 1U, schedule.setpoints.size(), lateness,
                consecutive_overruns, maximum_consecutive_overruns_);
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
        const double call_started_time =
            std::chrono::duration<double>(call_started_at - started_at).count();
        const double call_finished_time =
            std::chrono::duration<double>(call_finished_at - started_at).count();
        servo_call_timings.push_back({
            setpoint.command_time, call_started_time, call_finished_time});
        telemetry.push_back({
            "command", sample_index + 1U, call_finished_time,
            setpoint.command_time, call_started_time, call_finished_time,
            call_finished_time - call_started_time, lateness,
            setpoint.step_num, setpoint.positions, {}, false,
            false, false, 0, false, false});
        if (servo_ret != 0)
        {
            abort_goal(
                Action::Result::PATH_TOLERANCE_VIOLATED,
                "servo_j 定时下发失败: " + sdk_error(servo_ret));
            return;
        }

        const auto now = call_finished_at;
        if (now >= next_feedback_at ||
            sample_index + 1U == schedule.setpoints.size())
        {
            JointValue actual{};
            int actual_ret;
            {
                std::lock_guard<std::mutex> lock(session_mutex_);
                actual_ret = robot_.get_joint_position(&actual);
            }
            if (actual_ret != 0)
            {
                abort_goal(
                    Action::Result::PATH_TOLERANCE_VIOLATED,
                    "定时轨迹反馈读取失败: joint=" +
                    std::to_string(actual_ret));
                return;
            }

            std::vector<double> actual_positions(expected_joint_names_.size());
            for (std::size_t joint = 0; joint < actual_positions.size(); ++joint)
            {
                actual_positions[joint] = actual.jVal[joint];
            }
            const double elapsed =
                std::chrono::duration<double>(now - started_at).count();
            telemetry.push_back({
                "feedback", sample_index + 1U, elapsed,
                setpoint.command_time, 0.0, 0.0, 0.0, lateness,
                setpoint.step_num, setpoint.positions, actual_positions, true,
                false, false, 0, false, false});

            trajectory_msgs::msg::JointTrajectoryPoint desired;
            desired.positions = reorder_joint_values(
                expected_joint_names_, setpoint.positions, trajectory.joint_names);
            desired.time_from_start = static_cast<builtin_interfaces::msg::Duration>(
                rclcpp::Duration::from_seconds(setpoint.reference_time));
            publish_feedback(goal_handle, desired, actual, now - started_at);
            next_feedback_at = now +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(feedback_period_));
        }
    }

    const auto send_completed_at = std::chrono::steady_clock::now();
    const auto timing_summary = summarize_servo_timing(
        servo_call_timings, maximum_lateness_);
    RCLCPP_INFO(
        node_->get_logger(),
        "servo 定时下发完成: sent=%zu, late_samples=%zu, "
        "max_lateness=%.6f s, call_ms[p50=%.3f p95=%.3f p99=%.3f max=%.3f], "
        "elapsed=%.6f s",
        timing_summary.samples, timing_summary.late_samples,
        timing_summary.maximum_lateness,
        timing_summary.call_duration_p50 * 1000.0,
        timing_summary.call_duration_p95 * 1000.0,
        timing_summary.call_duration_p99 * 1000.0,
        timing_summary.maximum_call_duration * 1000.0,
        std::chrono::duration<double>(send_completed_at - started_at).count());

    double allowed_timeout = goal_timeout_;
    const double requested_timeout = duration_seconds(
        goal_handle->get_goal()->goal_time_tolerance);
    if (requested_timeout > 0.0)
    {
        allowed_timeout = requested_timeout;
    }
    const auto expected_finish = started_at +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(schedule.scheduled_duration));
    const double send_completed_time =
        std::chrono::duration<double>(send_completed_at - started_at).count();
    const auto deadline_offset = endpoint_deadline_offset(
        schedule.scheduled_duration, send_completed_time, allowed_timeout);
    if (!deadline_offset)
    {
        abort_goal(
            Action::Result::INVALID_GOAL,
            "终点检查超时参数无效");
        return;
    }
    const auto deadline = started_at +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(*deadline_offset));
    const auto final_positions = reordered_positions(
        trajectory.joint_names, trajectory.points.back().positions);
    double last_maximum_error = std::numeric_limits<double>::infinity();
    std::size_t last_maximum_error_joint = 0;
    std::vector<double> last_joint_errors(expected_joint_names_.size(), 0.0);
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

        double maximum_error = 0.0;
        std::size_t maximum_error_joint = 0;
        for (std::size_t index = 0; index < final_positions.size(); ++index)
        {
            const double error = std::abs(
                final_positions[index] - actual.jVal[index]);
            last_joint_errors[index] = error;
            if (error > maximum_error)
            {
                maximum_error = error;
                maximum_error_joint = index;
            }
        }
        last_maximum_error = maximum_error;
        last_maximum_error_joint = maximum_error_joint;

        publish_feedback(
            goal_handle,
            trajectory.points.back(),
            actual,
            std::chrono::steady_clock::now() - started_at);
        const auto now = std::chrono::steady_clock::now();
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
            std::vector<double> actual_positions(expected_joint_names_.size());
            for (std::size_t joint = 0; joint < actual_positions.size(); ++joint)
            {
                actual_positions[joint] = actual.jVal[joint];
            }
            telemetry.push_back({
                "endpoint", schedule.setpoints.size(),
                std::chrono::duration<double>(now - started_at).count(),
                schedule.scheduled_duration, 0.0, 0.0, 0.0, 0.0,
                servo_step_num_, final_positions, actual_positions, true,
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
                "终点收敛: elapsed=%.3f s, max_error=%.9f rad, joint=%s",
                std::chrono::duration<double>(now - started_at).count(),
                maximum_error,
                expected_joint_names_[maximum_error_joint].c_str());
            next_terminal_log = now + std::chrono::seconds(1);
        }
        if (std::chrono::steady_clock::now() >= expected_finish &&
            maximum_error <= goal_tolerance_)
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
                result->error_string = "轨迹执行成功";
                goal_handle->succeed(result);
            }
            finish();
            return;
        }
        if (std::chrono::steady_clock::now() >= deadline)
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::ostringstream error_details;
    for (std::size_t joint = 0; joint < last_joint_errors.size(); ++joint)
    {
        if (joint > 0U)
        {
            error_details << ", ";
        }
        error_details << expected_joint_names_[joint] << '='
                      << last_joint_errors[joint];
    }
    abort_goal(
        Action::Result::GOAL_TOLERANCE_VIOLATED,
        "最终关节误差在超时前未进入容差: " +
        expected_joint_names_[last_maximum_error_joint] + " error=" +
        std::to_string(last_maximum_error) + " rad, tolerance=" +
        std::to_string(goal_tolerance_) + " rad, all_errors=[" +
        error_details.str() + "]");
}

}  // namespace jaka_driver
