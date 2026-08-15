#include "jaka_driver/follow_joint_trajectory_server.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
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
    servo_period_ = node_->declare_parameter<double>(
        "trajectory_servo_period", servo_period_);
    maximum_trajectory_duration_ = node_->declare_parameter<double>(
        "maximum_trajectory_duration", maximum_trajectory_duration_);

    if (!(goal_tolerance_ > 0.0) || !(goal_timeout_ > 0.0) ||
        !(servo_period_ > 0.0) || !(maximum_trajectory_duration_ > 0.0))
    {
        throw std::invalid_argument("轨迹 Action 的容差和周期参数必须大于零");
    }

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
        "JAKA trajectory Action ready: %s",
        action_name.c_str());
}

FollowJointTrajectoryServer::~FollowJointTrajectoryServer()
{
    shutting_down_.store(true);
    cancel_requested_.store(true);
    if (goal_active_.load())
    {
        stop_motion_and_servo();
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
    return stop_motion_and_servo();
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

    const int ret = request_stop();
    if (ret != 0)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "取消轨迹时停止机器人失败: %s",
            sdk_error(ret).c_str());
    }
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

int FollowJointTrajectoryServer::stop_motion_and_servo()
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (!sdk_logged_in_.load())
    {
        return 0;
    }
    const int abort_ret = robot_.motion_abort();
    const int servo_ret = robot_.servo_move_enable(FALSE);
    return abort_ret != 0 ? abort_ret : servo_ret;
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
    const auto finish = [this]() {
        release_control(control_owner_, ControlOwner::kTrajectory);
        goal_active_.store(false);
    };
    const auto abort_goal = [&](int code, const std::string & message) {
        stop_motion_and_servo();
        result->error_code = code;
        result->error_string = message;
        goal_handle->abort(result);
        finish();
    };
    const auto cancel_goal = [&](const std::string & message) {
        stop_motion_and_servo();
        result->error_code = Action::Result::SUCCESSFUL;
        result->error_string = message;
        goal_handle->canceled(result);
        finish();
    };

    int enable_ret;
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        enable_ret = robot_.servo_move_enable(TRUE);
    }
    if (enable_ret != 0)
    {
        abort_goal(
            Action::Result::PATH_TOLERANCE_VIOLATED,
            "进入 servo mode 失败: " + sdk_error(enable_ret));
        return;
    }

    const auto & trajectory = goal_handle->get_goal()->trajectory;
    const auto started_at = std::chrono::steady_clock::now();
    double previous_time = 0.0;

    for (const auto & point : trajectory.points)
    {
        const double point_time = duration_seconds(point.time_from_start);
        const auto step_num = interpolation_steps(
            previous_time, point_time, servo_period_);
        if (!step_num)
        {
            abort_goal(
                Action::Result::INVALID_GOAL,
                "轨迹插补周期无效");
            return;
        }

        const auto positions = reordered_positions(
            trajectory.joint_names, point.positions);
        JointValue target{};
        for (std::size_t index = 0; index < positions.size(); ++index)
        {
            target.jVal[index] = positions[index];
        }

        int servo_ret;
        {
            std::lock_guard<std::mutex> lock(session_mutex_);
            servo_ret = robot_.servo_j(&target, MoveMode::ABS, *step_num);
        }
        if (servo_ret != 0)
        {
            abort_goal(
                Action::Result::PATH_TOLERANCE_VIOLATED,
                "servo_j 执行失败: " + sdk_error(servo_ret));
            return;
        }

        // 在该段起点下发终点和插补周期，然后等待段终点读取反馈。
        const auto point_deadline = started_at +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(point_time));
        if (!wait_until(
                point_deadline,
                goal_handle))
        {
            cancel_goal("轨迹已取消并停止机器人");
            return;
        }

        JointValue actual{};
        int state_ret;
        {
            std::lock_guard<std::mutex> lock(session_mutex_);
            state_ret = robot_.get_joint_position(&actual);
        }
        if (state_ret != 0)
        {
            abort_goal(
                Action::Result::PATH_TOLERANCE_VIOLATED,
                "读取关节反馈失败: " + sdk_error(state_ret));
            return;
        }

        publish_feedback(
            goal_handle,
            point,
            actual,
            std::chrono::steady_clock::now() - started_at);
        previous_time = point_time;
    }

    double allowed_timeout = goal_timeout_;
    const double requested_timeout = duration_seconds(
        goal_handle->get_goal()->goal_time_tolerance);
    if (requested_timeout > 0.0)
    {
        allowed_timeout = requested_timeout;
    }
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(allowed_timeout);
    const auto final_positions = reordered_positions(
        trajectory.joint_names, trajectory.points.back().positions);

    while (std::chrono::steady_clock::now() < deadline)
    {
        if (cancel_requested_.load() || goal_handle->is_canceling())
        {
            cancel_goal("终点检查期间轨迹被取消");
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
        for (std::size_t index = 0; index < final_positions.size(); ++index)
        {
            maximum_error = std::max(
                maximum_error,
                std::abs(final_positions[index] - actual.jVal[index]));
        }
        if (maximum_error <= goal_tolerance_)
        {
            const int stop_ret = stop_motion_and_servo();
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
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    abort_goal(
        Action::Result::GOAL_TOLERANCE_VIOLATED,
        "最终关节误差在超时前未进入容差");
}

}  // namespace jaka_driver
