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
    double actual_sample_period{0.0};
    std::vector<double> commanded_velocity;
    std::vector<double> estimated_actual_velocity;
    std::vector<double> sdk_inst_velocity_raw;
    bool estimated_velocity_valid{false};
    bool sdk_velocity_valid{false};
    double sdk_velocity_sample_elapsed{0.0};
    double sdk_status_call_duration{0.0};
};

long long artifact_stamp()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string make_artifact_path(
    const std::string & prefix,
    long long stamp)
{
    return "/tmp/" + prefix + '_' + std::to_string(stamp) + ".csv";
}

bool write_tracking_csv(
    const std::string & path,
    const std::vector<TrackingSample> & samples,
    double servo_filter_cutoff_hz)
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
              "error_code,in_servo_mode,status_valid,actual_sample_period_s";
    for (std::size_t joint = 0; joint < 6U; ++joint)
    {
        stream << ",commanded_velocity_joint_" << joint + 1U;
    }
    for (std::size_t joint = 0; joint < 6U; ++joint)
    {
        stream << ",estimated_actual_velocity_joint_" << joint + 1U;
    }
    for (std::size_t joint = 0; joint < 6U; ++joint)
    {
        stream << ",sdk_inst_velocity_raw_joint_" << joint + 1U;
    }
    stream << ",estimated_velocity_valid,sdk_velocity_valid,filter_type,"
              "filter_cutoff_hz,foresight_verified,"
              "sdk_velocity_sample_elapsed_s,sdk_status_call_duration_s\n";
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
               << ',' << sample.status_valid
               << ',' << sample.actual_sample_period;
        const auto write_values = [&stream](const std::vector<double> & values) {
                for (std::size_t joint = 0U; joint < 6U; ++joint)
                {
                    stream << ',';
                    if (joint < values.size())
                    {
                        stream << values[joint];
                    }
                }
            };
        write_values(sample.commanded_velocity);
        write_values(sample.estimated_actual_velocity);
        write_values(sample.sdk_inst_velocity_raw);
        stream << ',' << sample.estimated_velocity_valid
               << ',' << sample.sdk_velocity_valid
               << ',' << (servo_filter_cutoff_hz > 0.0 ? "joint_lpf" : "none")
               << ',' << servo_filter_cutoff_hz
               << ",0"
               << ',' << sample.sdk_velocity_sample_elapsed
               << ',' << sample.sdk_status_call_duration << '\n';
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
    start_tolerance_ = node_->declare_parameter<double>(
        "trajectory_start_tolerance", start_tolerance_);
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
    capture_sdk_joint_velocity_ = node_->declare_parameter<bool>(
        "trajectory_capture_sdk_joint_velocity",
        capture_sdk_joint_velocity_);
    sdk_joint_velocity_period_ = node_->declare_parameter<double>(
        "trajectory_sdk_joint_velocity_period",
        sdk_joint_velocity_period_);
    servo_filter_cutoff_hz_ = node_->declare_parameter<double>(
        "trajectory_servo_filter_cutoff_hz", servo_filter_cutoff_hz_);
    maximum_queue_starvation_ = node_->declare_parameter<double>(
        "trajectory_maximum_queue_starvation", maximum_queue_starvation_);
    const auto maximum_consecutive_starvations =
        node_->declare_parameter<int64_t>(
        "trajectory_maximum_consecutive_starvations",
        static_cast<int64_t>(maximum_consecutive_starvations_));

    if (!std::isfinite(goal_tolerance_) || goal_tolerance_ <= 0.0 ||
        !std::isfinite(start_tolerance_) || start_tolerance_ <= 0.0 ||
        !std::isfinite(goal_timeout_) || goal_timeout_ <= 0.0 ||
        !std::isfinite(maximum_trajectory_duration_) ||
        maximum_trajectory_duration_ <= 0.0 ||
        maximum_servo_step_num <= 0 ||
        maximum_servo_step_num > static_cast<int64_t>(kMaximumServoStepNum) ||
        maximum_servo_samples <= 0 || maximum_consecutive_starvations < 0 ||
        !std::isfinite(feedback_period_) || feedback_period_ <= 0.0 ||
        !std::isfinite(sdk_joint_velocity_period_) ||
        sdk_joint_velocity_period_ <= 0.0 ||
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
        "start_tolerance=%.6f rad, "
        "endpoint_margin=%.3f s, maximum_step_num=%u, servo_filter=%.3f Hz, "
        "maximum_queue_starvation=%.3f s, allowed_consecutive_starvations=%zu, "
        "sdk_joint_velocity=%s, sdk_velocity_period=%.3f s",
        action_name.c_str(), goal_tolerance_, start_tolerance_, goal_timeout_,
        maximum_servo_step_num_, servo_filter_cutoff_hz_,
        maximum_queue_starvation_, maximum_consecutive_starvations_,
        capture_sdk_joint_velocity_ ? "enabled" : "disabled",
        sdk_joint_velocity_period_);
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
    const auto run_stamp = artifact_stamp();
    const std::string telemetry_path = make_artifact_path(
        "jaka_trajectory", run_stamp);
    const std::string source_goal_path = make_artifact_path(
        "jaka_source_goal", run_stamp);
    const std::string servo_schedule_path = make_artifact_path(
        "jaka_servo_schedule", run_stamp);
    bool telemetry_written = false;
    bool servo_mode_entered = false;
    std::size_t terminal_sample_index = 0U;
    double terminal_elapsed = 0.0;
    double terminal_controller_segment_start = 0.0;
    unsigned int terminal_step_num = 0U;
    std::vector<double> terminal_desired;
    const auto flush_telemetry = [&]() {
        if (telemetry_written)
        {
            return;
        }
        telemetry_written = true;
        if (write_tracking_csv(
                telemetry_path, telemetry, servo_filter_cutoff_hz_))
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
    const auto append_terminal_snapshot = [&](const std::string & phase) {
        JointValue actual{};
        RobotStatus_simple status{};
        BOOL in_servo = FALSE;
        int actual_ret;
        int status_ret;
        int mode_ret;
        {
            std::lock_guard<std::mutex> lock(session_mutex_);
            actual_ret = robot_.get_joint_position(&actual);
            status_ret = robot_.get_robot_status_simple(&status);
            mode_ret = robot_.is_in_servomove(&in_servo);
        }

        std::vector<double> actual_positions;
        if (actual_ret == 0)
        {
            actual_positions.resize(expected_joint_names_.size());
            for (std::size_t joint = 0; joint < actual_positions.size(); ++joint)
            {
                actual_positions[joint] = actual.jVal[joint];
            }
        }
        const bool status_valid = status_ret == 0 && mode_ret == 0;
        TrackingSample sample;
        sample.phase = phase;
        sample.sample_index = terminal_sample_index;
        sample.elapsed = terminal_elapsed;
        sample.controller_segment_start = terminal_controller_segment_start;
        sample.step_num = terminal_step_num;
        sample.desired = terminal_desired;
        sample.actual = actual_positions;
        sample.actual_valid = actual_ret == 0;
        sample.powered_on =
            status_valid && static_cast<bool>(status.powered_on);
        sample.enabled = status_valid && static_cast<bool>(status.enabled);
        sample.error_code = status_valid ? status.errcode : 0;
        sample.in_servo_mode =
            status_valid && static_cast<bool>(in_servo);
        sample.status_valid = status_valid;
        telemetry.push_back(std::move(sample));
        if (actual_ret != 0 || !status_valid)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "终态遥测读取不完整: phase=%s, joint=%d, status=%d, servo=%d",
                phase.c_str(), actual_ret, status_ret, mode_ret);
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
        append_terminal_snapshot("aborted");
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
        append_terminal_snapshot("canceled");
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
    std::string artifact_error;
    if (!write_joint_trajectory_csv(
            source_goal_path, trajectory, artifact_error) ||
        !write_queued_servo_schedule_csv(
            servo_schedule_path, schedule, expected_joint_names_, artifact_error))
    {
        abort_goal(
            Action::Result::INVALID_GOAL,
            "轨迹诊断落盘失败，未进入 servo mode: " + artifact_error);
        return;
    }
    const auto schedule_diagnostics = analyze_queued_servo_schedule(
        schedule, initial_positions);
    if (!schedule_diagnostics.valid)
    {
        abort_goal(
            Action::Result::INVALID_GOAL,
            "轨迹调度诊断失败，未进入 servo mode: " +
            schedule_diagnostics.error);
        return;
    }
    if (schedule_diagnostics.maximum_start_position_error > start_tolerance_)
    {
        abort_goal(
            Action::Result::INVALID_GOAL,
            "轨迹起点与实时关节位置不连续: joint=" +
            expected_joint_names_[
                schedule_diagnostics.maximum_start_error_joint] +
            ", error=" + std::to_string(
                schedule_diagnostics.maximum_start_position_error) +
            " rad, tolerance=" + std::to_string(start_tolerance_) +
            " rad；未进入 servo mode");
        return;
    }
    RCLCPP_INFO(
        node_->get_logger(),
        "servo 转接预检: duration_error=%.9f s, start_error=%.9f rad "
        "at %s, joint_1_velocity[max=%.9f rad/s positive=%zu negative=%zu "
        "sign_changes=%zu], source_goal=%s, schedule=%s",
        schedule_diagnostics.duration_error,
        schedule_diagnostics.maximum_start_position_error,
        expected_joint_names_[
            schedule_diagnostics.maximum_start_error_joint].c_str(),
        schedule_diagnostics.maximum_absolute_velocity[0],
        schedule_diagnostics.positive_velocity_segments[0],
        schedule_diagnostics.negative_velocity_segments[0],
        schedule_diagnostics.velocity_sign_changes[0],
        source_goal_path.c_str(), servo_schedule_path.c_str());

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
    RCLCPP_WARN(
        node_->get_logger(),
        "servo foresight 状态未验证：旧 enable_robot 路径可能调用 "
        "servo_speed_foresight(15, 0.03)，本轮 CSV 将 foresight_verified=0");

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
        terminal_sample_index = sample_index + 1U;
        terminal_elapsed = controller_elapsed;
        terminal_controller_segment_start = setpoint.controller_start_time;
        terminal_step_num = setpoint.step_num;
        terminal_desired = setpoint.positions;
        servo_call_timings.push_back({
            call_finished_time - call_started_time, queue_starvation});
        TrackingSample queue_sample;
        queue_sample.phase = "queue";
        queue_sample.sample_index = sample_index + 1U;
        queue_sample.elapsed = controller_elapsed;
        queue_sample.controller_segment_start =
            setpoint.controller_start_time;
        queue_sample.call_started_time = call_started_time;
        queue_sample.call_finished_time = call_finished_time;
        queue_sample.call_duration = call_finished_time - call_started_time;
        queue_sample.queue_starvation = queue_starvation;
        queue_sample.step_num = setpoint.step_num;
        queue_sample.desired = setpoint.positions;
        queue_sample.commanded_velocity = setpoint.implicit_velocities;
        telemetry.push_back(std::move(queue_sample));
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

    const double requested_timeout = duration_seconds(
        goal_handle->get_goal()->goal_time_tolerance);
    const auto allowed_timeout = effective_endpoint_margin(
        goal_timeout_, requested_timeout);
    if (!allowed_timeout)
    {
        abort_goal(
            Action::Result::INVALID_GOAL,
            "终点收敛余量参数无效");
        return;
    }
    const auto expected_finish = controller_started_at +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(schedule.scheduled_duration));
    const auto deadline_offset = endpoint_deadline_offset(
        schedule.scheduled_duration, send_completed_time, *allowed_timeout);
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
        "终点监控窗口: queue_remaining=%.6f s, configured_margin=%.6f s, "
        "requested_margin=%.6f s, effective_margin=%.6f s, "
        "deadline_from_controller_start=%.6f s",
        std::max(0.0, schedule.scheduled_duration - send_completed_time),
        goal_timeout_, requested_timeout, *allowed_timeout, *deadline_offset);
    const auto final_positions = reordered_positions(
        trajectory.joint_names, trajectory.points.back().positions);
    EndpointProgress last_progress;
    auto next_terminal_log = std::chrono::steady_clock::now();
    auto next_terminal_telemetry = std::chrono::steady_clock::now();
    auto next_sdk_velocity_sample = controller_started_at +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(sdk_joint_velocity_period_));
    std::vector<double> previous_actual_positions;
    std::chrono::steady_clock::time_point previous_actual_time{};
    bool previous_actual_sample_valid = false;
    std::size_t actual_velocity_samples = 0U;
    double minimum_actual_sample_period =
        std::numeric_limits<double>::infinity();
    double maximum_actual_sample_period = 0.0;
    std::size_t sdk_velocity_samples = 0U;
    double minimum_sdk_status_call_duration =
        std::numeric_limits<double>::infinity();
    double maximum_sdk_status_call_duration = 0.0;
    std::size_t estimated_joint1_positive_samples = 0U;
    std::size_t estimated_joint1_negative_samples = 0U;
    std::size_t sdk_joint1_positive_samples = 0U;
    std::size_t sdk_joint1_negative_samples = 0U;

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
        const double actual_sample_period = previous_actual_sample_valid ?
            std::chrono::duration<double>(
            now - previous_actual_time).count() : 0.0;
        const auto estimated_velocity = previous_actual_sample_valid ?
            estimate_joint_velocity(
            previous_actual_positions, actual_positions, actual_sample_period) :
            std::nullopt;
        std::vector<double> estimated_actual_velocity;
        const bool estimated_velocity_valid = estimated_velocity.has_value();
        if (estimated_velocity_valid)
        {
            estimated_actual_velocity = *estimated_velocity;
            minimum_actual_sample_period = std::min(
                minimum_actual_sample_period, actual_sample_period);
            maximum_actual_sample_period = std::max(
                maximum_actual_sample_period, actual_sample_period);
            ++actual_velocity_samples;
        }
        previous_actual_positions = actual_positions;
        previous_actual_time = now;
        previous_actual_sample_valid = true;
        const auto desired_positions = sample_queued_servo_schedule(
            schedule, initial_positions, controller_elapsed);
        const auto commanded_velocity = sample_queued_servo_velocity(
            schedule, controller_elapsed);
        if (!desired_positions || !commanded_velocity)
        {
            abort_goal(
                Action::Result::GOAL_TOLERANCE_VIOLATED,
                "无法按控制柜时间轴计算期望关节位置");
            return;
        }
        constexpr double kVelocitySignDeadband = 1e-6;
        const bool joint1_command_active =
            !commanded_velocity->empty() &&
            std::abs(commanded_velocity->front()) > kVelocitySignDeadband;
        if (joint1_command_active && estimated_velocity_valid &&
            !estimated_actual_velocity.empty())
        {
            if (estimated_actual_velocity.front() > kVelocitySignDeadband)
            {
                ++estimated_joint1_positive_samples;
            }
            else if (estimated_actual_velocity.front() < -kVelocitySignDeadband)
            {
                ++estimated_joint1_negative_samples;
            }
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
        terminal_sample_index = schedule.setpoints.size();
        terminal_elapsed = controller_elapsed;
        terminal_controller_segment_start = schedule.scheduled_duration;
        terminal_step_num = schedule.setpoints.back().step_num;
        terminal_desired = *desired_positions;

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
            RobotStatus full_status{};
            bool full_status_valid = false;
            std::vector<double> sdk_inst_velocity_raw;
            bool sdk_velocity_valid = false;
            double sdk_velocity_sample_elapsed = 0.0;
            double sdk_status_call_duration = 0.0;
            if (capture_sdk_joint_velocity_ &&
                now >= next_sdk_velocity_sample)
            {
                const auto sdk_call_started_at = std::chrono::steady_clock::now();
                int sdk_status_ret;
                {
                    std::lock_guard<std::mutex> lock(session_mutex_);
                    sdk_status_ret = robot_.get_robot_status(&full_status);
                }
                const auto sdk_call_finished_at = std::chrono::steady_clock::now();
                sdk_status_call_duration = std::chrono::duration<double>(
                    sdk_call_finished_at - sdk_call_started_at).count();
                sdk_velocity_sample_elapsed = std::chrono::duration<double>(
                    sdk_call_finished_at - controller_started_at).count();
                next_sdk_velocity_sample = sdk_call_finished_at +
                    std::chrono::duration_cast<
                    std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(
                    sdk_joint_velocity_period_));
                if (sdk_status_ret == 0)
                {
                    full_status_valid = true;
                    sdk_inst_velocity_raw.resize(expected_joint_names_.size());
                    sdk_velocity_valid = true;
                    for (std::size_t joint = 0U;
                        joint < expected_joint_names_.size(); ++joint)
                    {
                        sdk_inst_velocity_raw[joint] = full_status.
                            robot_monitor_data.jointMonitorData[joint].instVel;
                        sdk_velocity_valid = sdk_velocity_valid &&
                            std::isfinite(sdk_inst_velocity_raw[joint]);
                    }
                    if (sdk_velocity_valid)
                    {
                        ++sdk_velocity_samples;
                        minimum_sdk_status_call_duration = std::min(
                            minimum_sdk_status_call_duration,
                            sdk_status_call_duration);
                        maximum_sdk_status_call_duration = std::max(
                            maximum_sdk_status_call_duration,
                            sdk_status_call_duration);
                        if (joint1_command_active &&
                            sdk_inst_velocity_raw.front() >
                            kVelocitySignDeadband)
                        {
                            ++sdk_joint1_positive_samples;
                        }
                        else if (joint1_command_active &&
                            sdk_inst_velocity_raw.front() <
                            -kVelocitySignDeadband)
                        {
                            ++sdk_joint1_negative_samples;
                        }
                    }
                }
                else
                {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "SDK instVel 辅助采样失败: %s；继续使用位置差分速度",
                        sdk_error(sdk_status_ret).c_str());
                }
            }
            RobotStatus_simple status{};
            BOOL in_servo = FALSE;
            int status_ret = 0;
            int mode_ret;
            {
                std::lock_guard<std::mutex> lock(session_mutex_);
                if (full_status_valid)
                {
                    status.powered_on = full_status.powered_on;
                    status.enabled = full_status.enabled;
                    status.errcode = full_status.errcode;
                }
                else
                {
                    status_ret = robot_.get_robot_status_simple(&status);
                }
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
                static_cast<bool>(in_servo), true,
                actual_sample_period, *commanded_velocity,
                estimated_actual_velocity, sdk_inst_velocity_raw,
                estimated_velocity_valid, sdk_velocity_valid,
                sdk_velocity_sample_elapsed, sdk_status_call_duration});
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
            RCLCPP_INFO(
                node_->get_logger(),
                "关节速度遥测: position_samples=%zu, "
                "sample_period[min=%.6f max=%.6f] s, "
                "sdk_velocity=%s, sdk_samples=%zu, sdk_period=%.3f s, "
                "sdk_call_ms[min=%.3f max=%.3f], "
                "joint_1_sign[estimated_positive=%zu estimated_negative=%zu "
                "sdk_raw_positive=%zu sdk_raw_negative=%zu]",
                actual_velocity_samples,
                actual_velocity_samples > 0U ? minimum_actual_sample_period : 0.0,
                maximum_actual_sample_period,
                capture_sdk_joint_velocity_ ? "requested" : "disabled",
                sdk_velocity_samples, sdk_joint_velocity_period_,
                sdk_velocity_samples > 0U ?
                minimum_sdk_status_call_duration * 1000.0 : 0.0,
                maximum_sdk_status_call_duration * 1000.0,
                estimated_joint1_positive_samples,
                estimated_joint1_negative_samples,
                sdk_joint1_positive_samples,
                sdk_joint1_negative_samples);
            const int stop_ret = exit_servo_mode();
            terminal_desired = final_positions;
            append_terminal_snapshot(
                stop_ret == 0 ? "succeeded" : "exit_failed");
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
