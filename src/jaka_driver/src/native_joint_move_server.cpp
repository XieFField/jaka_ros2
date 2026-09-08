#include "jaka_driver/native_joint_move_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "jaka_driver/native_joint_move_utils.hpp"

namespace jaka_driver
{

namespace
{

std::vector<double> joint_vector(const JointValue & value)
{
    return std::vector<double>(value.jVal, value.jVal + 6);
}

}  // namespace

NativeJointMoveServer::NativeJointMoveServer(
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
    feedback_period_ = node_->declare_parameter<double>(
        "native_joint_move_feedback_period", 0.05);
    timeout_margin_ = node_->declare_parameter<double>(
        "native_joint_move_timeout_margin", 15.0);
    progress_timeout_ = node_->declare_parameter<double>(
        "native_joint_move_progress_timeout", 10.0);
    progress_epsilon_ = node_->declare_parameter<double>(
        "native_joint_move_progress_epsilon", 1.0e-5);
    endpoint_settle_timeout_ = node_->declare_parameter<double>(
        "native_joint_move_endpoint_settle_timeout", 2.0);
    status_log_period_ = node_->declare_parameter<double>(
        "native_joint_move_status_log_period", 1.0);
    if (!std::isfinite(feedback_period_) || feedback_period_ <= 0.0)
    {
        throw std::invalid_argument(
            "native_joint_move_feedback_period 必须为有限正数");
    }
    if (!std::isfinite(timeout_margin_) || timeout_margin_ < 0.0)
    {
        throw std::invalid_argument(
            "native_joint_move_timeout_margin 必须为有限非负数");
    }
    if (!std::isfinite(progress_timeout_) || progress_timeout_ <= 0.0 ||
        !std::isfinite(progress_epsilon_) || progress_epsilon_ <= 0.0 ||
        !std::isfinite(endpoint_settle_timeout_) ||
        endpoint_settle_timeout_ <= 0.0 ||
        !std::isfinite(status_log_period_) || status_log_period_ <= 0.0)
    {
        throw std::invalid_argument(
            "原生 joint_move 进度监控参数必须为有限正数");
    }
    server_ = rclcpp_action::create_server<Action>(
        node_, action_name,
        std::bind(
            &NativeJointMoveServer::handle_goal, this,
            std::placeholders::_1, std::placeholders::_2),
        std::bind(
            &NativeJointMoveServer::handle_cancel, this,
            std::placeholders::_1),
        std::bind(
            &NativeJointMoveServer::handle_accepted, this,
            std::placeholders::_1));
}

NativeJointMoveServer::~NativeJointMoveServer()
{
    shutting_down_.store(true);
    if (goal_active_.load())
    {
        request_stop();
    }
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable())
    {
        worker_.join();
    }
}

bool NativeJointMoveServer::robot_ready(std::string & error)
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (!sdk_logged_in_.load())
    {
        error = "JAKA SDK 尚未登录";
        return false;
    }
    RobotStatus_simple status{};
    MotionStatus motion{};
    ProgramState program_state = PROGRAM_IDLE;
    BOOL drag_mode = FALSE;
    const int status_ret = robot_.get_robot_status_simple(&status);
    const int motion_ret = robot_.get_motion_status(&motion);
    const int program_ret = robot_.get_program_state(&program_state);
    const int drag_ret = robot_.is_in_drag_mode(&drag_mode);
    if (status_ret != 0 || motion_ret != 0 || program_ret != 0 || drag_ret != 0)
    {
        error = "读取机器人状态失败: status=" +
            std::to_string(status_ret) + ", motion=" +
            std::to_string(motion_ret) + ", program=" +
            std::to_string(program_ret) + ", drag=" +
            std::to_string(drag_ret);
        return false;
    }
    const NativeMotionReadiness readiness{
        static_cast<bool>(status.powered_on),
        static_cast<bool>(status.enabled),
        status.errcode,
        program_state == PROGRAM_IDLE,
        motion.queue,
        motion.active_queue,
        static_cast<bool>(motion.paused),
        static_cast<bool>(motion.isOnLimit),
        static_cast<bool>(motion.isInEstop),
        static_cast<bool>(motion.isInCollision),
        static_cast<bool>(drag_mode)};
    const bool ready = validate_native_motion_readiness(readiness, error);
    const std::string snapshot =
        "powered=" + std::to_string(status.powered_on) +
        ", enabled=" + std::to_string(status.enabled) +
        ", errcode=" + std::to_string(status.errcode) +
        ", program_state=" + std::to_string(static_cast<int>(program_state)) +
        ", inpos=" + std::to_string(motion.inpos) +
        ", queue=" + std::to_string(motion.queue) +
        ", active_queue=" + std::to_string(motion.active_queue) +
        ", paused=" + std::to_string(motion.paused) +
        ", on_limit=" + std::to_string(motion.isOnLimit) +
        ", estop=" + std::to_string(motion.isInEstop) +
        ", collision=" + std::to_string(motion.isInCollision) +
        ", drag=" + std::to_string(drag_mode);
    if (!ready)
    {
        error += ": " + snapshot;
        return false;
    }
    RCLCPP_INFO(
        node_->get_logger(), "NATIVE PTP PREFLIGHT: PASS: %s",
        snapshot.c_str());
    return true;
}

rclcpp_action::GoalResponse NativeJointMoveServer::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Action::Goal> goal)
{
    bool expected = false;
    if (!goal_active_.compare_exchange_strong(expected, true))
    {
        return rclcpp_action::GoalResponse::REJECT;
    }

    NativeJointMoveGoal values{
        goal->target_positions, goal->speed, goal->acceleration,
        goal->endpoint_tolerance, goal->timeout};
    std::string error;
    if (!validate_native_joint_move_goal(values, error) || !robot_ready(error))
    {
        RCLCPP_ERROR(node_->get_logger(), "拒绝原生 joint_move Goal: %s", error.c_str());
        goal_active_.store(false);
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!try_acquire_control(control_owner_, ControlOwner::kNativeMotion))
    {
        RCLCPP_ERROR(
            node_->get_logger(), "拒绝原生 joint_move Goal: 控制权由 %s 持有",
            control_owner_name(control_owner_.load()));
        goal_active_.store(false);
        return rclcpp_action::GoalResponse::REJECT;
    }
    cancel_requested_.store(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NativeJointMoveServer::handle_cancel(
    const std::shared_ptr<GoalHandle>)
{
    return goal_active_.load() ? rclcpp_action::CancelResponse::ACCEPT :
           rclcpp_action::CancelResponse::REJECT;
}

void NativeJointMoveServer::handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable())
    {
        worker_.join();
    }
    worker_ = std::thread(&NativeJointMoveServer::execute, this, goal_handle);
}

int NativeJointMoveServer::abort_motion()
{
    cancel_requested_.store(true);
    std::lock_guard<std::mutex> lock(session_mutex_);
    return sdk_logged_in_.load() ? robot_.motion_abort() : 0;
}

int NativeJointMoveServer::request_stop()
{
    return abort_motion();
}

void NativeJointMoveServer::execute(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Action::Result>();
    result->max_joint_error = std::numeric_limits<double>::infinity();
    const auto finish = [&]() {
        release_control(control_owner_, ControlOwner::kNativeMotion);
        goal_active_.store(false);
    };
    const auto fail = [&](int code, const std::string & message) {
        result->success = false;
        result->sdk_error_code = code;
        result->message = message;
        RCLCPP_ERROR(
            node_->get_logger(), "NATIVE PTP FAIL: request=%s, code=%d, %s",
            goal->request_id.c_str(), code, message.c_str());
        goal_handle->abort(result);
        finish();
    };

    JointValue target{};
    std::copy(
        goal->target_positions.begin(), goal->target_positions.end(),
        target.jVal);
    int command_ret;
    double rapid_rate = 0.0;
    double approach_linear_speed = std::numeric_limits<double>::quiet_NaN();
    double approach_angular_speed = std::numeric_limits<double>::quiet_NaN();
    JointValue initial{};
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        const int rapid_ret = robot_.get_rapidrate(&rapid_rate);
        const int joint_ret = robot_.get_joint_position(&initial);
        const int approach_ret = robot_.get_approach_speed_limit(
            &approach_linear_speed, &approach_angular_speed);
        if (approach_ret != 0)
        {
            approach_linear_speed = std::numeric_limits<double>::quiet_NaN();
            approach_angular_speed = std::numeric_limits<double>::quiet_NaN();
        }
        if (rapid_ret != 0 || joint_ret != 0 || !std::isfinite(rapid_rate) ||
            rapid_rate <= 0.0 || rapid_rate > 1.0)
        {
            command_ret = rapid_ret != 0 ? rapid_ret :
                (joint_ret != 0 ? joint_ret : -2);
        }
        else
        {
            command_ret = robot_.joint_move(
                &target, MoveMode::ABS, FALSE, goal->speed,
                goal->acceleration, 0.0, nullptr);
        }
    }
    if (command_ret != 0)
    {
        fail(command_ret, "读取速度倍率/初始关节或下发 JAKA joint_move 失败");
        return;
    }
    double maximum_delta = 0.0;
    for (std::size_t index = 0; index < 6U; ++index)
    {
        maximum_delta = std::max(
            maximum_delta, std::abs(target.jVal[index] - initial.jVal[index]));
    }
    const double estimated_duration = estimate_native_joint_move_duration(
        maximum_delta, goal->speed, goal->acceleration, rapid_rate);
    const double reference_timeout = std::max(
        goal->timeout, estimated_duration + timeout_margin_);
    RCLCPP_INFO(
        node_->get_logger(),
        "NATIVE PTP START: request=%s, programmed_speed=%.6f rad/s, rapid_rate=%.3f, nominal_effective_speed=%.6f rad/s, programmed_acceleration=%.6f rad/s^2, approach_limits=[%.3f mm/s %.6f rad/s], estimated_duration=%.3f s, reference_timeout=%.3f s, progress_timeout=%.3f s, tolerance=%.6f rad",
        goal->request_id.c_str(), goal->speed, rapid_rate,
        goal->speed * rapid_rate, goal->acceleration,
        approach_linear_speed, approach_angular_speed, estimated_duration,
        reference_timeout, progress_timeout_, goal->endpoint_tolerance);

    const auto started = std::chrono::steady_clock::now();
    auto last_progress = started;
    auto next_status_log = started +
        std::chrono::duration<double>(status_log_period_);
    double best_error = maximum_joint_error(
        goal->target_positions, joint_vector(initial));
    bool motion_observed = false;
    bool reference_timeout_reported = false;
    std::optional<std::chrono::steady_clock::time_point> stopped_since;
    const auto period = std::chrono::duration<double>(feedback_period_);
    while (rclcpp::ok() && !shutting_down_.load())
    {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - started).count();
        if (goal_handle->is_canceling())
        {
            const int abort_ret = abort_motion();
            result->success = false;
            result->sdk_error_code = abort_ret;
            result->message = "原生 joint_move 已取消";
            goal_handle->canceled(result);
            finish();
            return;
        }
        if (cancel_requested_.load())
        {
            result->success = false;
            result->sdk_error_code = 0;
            result->message = "原生 joint_move 被外部停止";
            goal_handle->abort(result);
            finish();
            return;
        }
        RobotStatus_simple status{};
        MotionStatus motion{};
        JointValue actual{};
        int status_ret;
        int motion_ret;
        int joint_ret;
        {
            std::lock_guard<std::mutex> lock(session_mutex_);
            status_ret = robot_.get_robot_status_simple(&status);
            motion_ret = robot_.get_motion_status(&motion);
            joint_ret = robot_.get_joint_position(&actual);
        }
        if (status_ret != 0 || motion_ret != 0 || joint_ret != 0)
        {
            abort_motion();
            fail(
                status_ret != 0 ? status_ret :
                (motion_ret != 0 ? motion_ret : joint_ret),
                "原生 joint_move 状态读取失败");
            return;
        }

        result->max_joint_error = maximum_joint_error(
            goal->target_positions, joint_vector(actual));
        bool made_progress = false;
        if (best_error - result->max_joint_error >= progress_epsilon_)
        {
            best_error = result->max_joint_error;
            last_progress = now;
            motion_observed = true;
            made_progress = true;
        }
        if (!motion.inpos || motion.queue > 0 || motion.active_queue > 0)
        {
            motion_observed = true;
        }
        auto feedback = std::make_shared<Action::Feedback>();
        feedback->elapsed = elapsed;
        feedback->in_position = motion.inpos;
        feedback->queue_depth = motion.queue;
        feedback->max_joint_error = result->max_joint_error;
        feedback->powered = status.powered_on;
        feedback->enabled = status.enabled;
        feedback->controller_error_code = status.errcode;
        goal_handle->publish_feedback(feedback);

        if (!status.powered_on || !status.enabled || status.errcode != 0 ||
            motion.isInCollision || motion.isInEstop)
        {
            abort_motion();
            fail(
                status.errcode,
                "原生 joint_move 期间机器人状态异常: powered=" +
                std::to_string(status.powered_on) + ", enabled=" +
                std::to_string(status.enabled) + ", errcode=" +
                std::to_string(status.errcode));
            return;
        }
        if (motion.inpos && motion.queue == 0 &&
            result->max_joint_error <= goal->endpoint_tolerance)
        {
            result->success = true;
            result->sdk_error_code = 0;
            result->message = "原生 joint_move 执行成功";
            RCLCPP_INFO(
                node_->get_logger(),
                "NATIVE PTP PASS: request=%s, elapsed=%.3f s, max_error=%.9f rad",
                goal->request_id.c_str(), elapsed, result->max_joint_error);
            goal_handle->succeed(result);
            finish();
            return;
        }
        if (motion_observed && !made_progress && motion.inpos &&
            motion.queue == 0)
        {
            if (!stopped_since.has_value())
            {
                stopped_since = now;
            }
            else if (std::chrono::duration<double>(
                now - *stopped_since).count() >= endpoint_settle_timeout_)
            {
                const int abort_ret = abort_motion();
                fail(
                    abort_ret,
                    "原生 joint_move 已停止但终点误差未进入容差: error=" +
                    std::to_string(result->max_joint_error) +
                    ", tolerance=" +
                    std::to_string(goal->endpoint_tolerance));
                return;
            }
        }
        else
        {
            stopped_since.reset();
        }

        const double no_progress_time =
            std::chrono::duration<double>(now - last_progress).count();
        if (no_progress_time >= progress_timeout_)
        {
            const int abort_ret = abort_motion();
            fail(
                abort_ret,
                "原生 joint_move 连续无有效进展: no_progress=" +
                std::to_string(no_progress_time) + " s, error=" +
                std::to_string(result->max_joint_error) +
                ", best_error=" + std::to_string(best_error));
            return;
        }
        if (!reference_timeout_reported && elapsed >= reference_timeout)
        {
            reference_timeout_reported = true;
            RCLCPP_WARN(
                node_->get_logger(),
                "NATIVE PTP REFERENCE TIME EXCEEDED: request=%s, elapsed=%.3f s, reference=%.3f s; robot healthy and progressing, continue monitoring",
                goal->request_id.c_str(), elapsed, reference_timeout);
        }
        if (now >= next_status_log)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "NATIVE PTP PROGRESS: request=%s, elapsed=%.3f s, error=%.9f rad, best_error=%.9f rad, no_progress=%.3f s, inpos=%d, queue=%d, active_queue=%d",
                goal->request_id.c_str(), elapsed, result->max_joint_error,
                best_error, no_progress_time, motion.inpos, motion.queue,
                motion.active_queue);
            next_status_log = now +
                std::chrono::duration<double>(status_log_period_);
        }
        std::this_thread::sleep_for(period);
    }

    abort_motion();
    fail(0, "原生 joint_move 因 ROS 关闭而终止");
}

}  // namespace jaka_driver
