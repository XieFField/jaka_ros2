#include "jaka_driver/follow_joint_trajectory_server.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
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
    const auto maximum_servo_steps = node_->declare_parameter<int64_t>(
        "maximum_servo_steps", static_cast<int64_t>(maximum_servo_steps_));
    maximum_trajectory_duration_ = node_->declare_parameter<double>(
        "maximum_trajectory_duration", maximum_trajectory_duration_);

    if (!(goal_tolerance_ > 0.0) || !(goal_timeout_ > 0.0) ||
        !(servo_period_ > 0.0) || !(maximum_trajectory_duration_ > 0.0) ||
        maximum_servo_steps <= 0 ||
        maximum_servo_steps >
        static_cast<int64_t>(std::numeric_limits<unsigned int>::max()))
    {
        throw std::invalid_argument("轨迹 Action 的容差和周期参数必须大于零");
    }
    maximum_servo_steps_ = static_cast<unsigned int>(maximum_servo_steps);

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
    if (!validate_trajectory(
        goal.trajectory,
        expected_joint_names_,
        maximum_trajectory_duration_,
        error))
    {
        return false;
    }
    return validate_servo_segments(
        goal.trajectory, servo_period_, maximum_servo_steps_, error);
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
    const double trajectory_duration = duration_seconds(
        trajectory.points.back().time_from_start);
    double previous_time = 0.0;

    RCLCPP_INFO(
        node_->get_logger(),
        "开始流式下发 JAKA servo_j 轨迹: points=%zu, duration=%.3f s",
        trajectory.points.size(), trajectory_duration);

    // servo_j 是流式接口：每条命令只描述下一个插补段，后续命令必须连续下发。
    // 整段下发期间独占 SDK 会话，避免状态读取插入相邻命令之间形成控制间隙。
    {
        std::unique_lock<std::mutex> lock(session_mutex_);
        for (std::size_t point_index = 0;
            point_index < trajectory.points.size(); ++point_index)
        {
            if (goal_handle->is_canceling())
            {
                lock.unlock();
                cancel_goal("轨迹下发期间已取消并停止机器人");
                return;
            }
            if (shutting_down_.load() || cancel_requested_.load() ||
                !rclcpp::ok())
            {
                lock.unlock();
                abort_goal(
                    Action::Result::PATH_TOLERANCE_VIOLATED,
                    "轨迹下发期间被外部停止");
                return;
            }

            const auto & point = trajectory.points[point_index];
            const double point_time = duration_seconds(point.time_from_start);
            const auto step_num = interpolation_steps(
                previous_time, point_time, servo_period_);
            if (!step_num)
            {
                lock.unlock();
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

            const int servo_ret = robot_.servo_j(
                &target, MoveMode::ABS, *step_num);
            if (servo_ret != 0)
            {
                lock.unlock();
                abort_goal(
                    Action::Result::PATH_TOLERANCE_VIOLATED,
                    "servo_j 执行失败: " + sdk_error(servo_ret));
                return;
            }

            RCLCPP_DEBUG(
                node_->get_logger(),
                "servo_j point=%zu/%zu, time=%.6f s, step_num=%u",
                point_index + 1, trajectory.points.size(), point_time,
                *step_num);
            previous_time = point_time;
        }
    }

    double allowed_timeout = goal_timeout_;
    const double requested_timeout = duration_seconds(
        goal_handle->get_goal()->goal_time_tolerance);
    if (requested_timeout > 0.0)
    {
        allowed_timeout = requested_timeout;
    }
    // 流式命令会很快完成下发，但机器人仍需 trajectory_duration 才能走完。
    // 终点超时因此必须从计划终止时刻计算，不能从下发结束时刻直接起算。
    const auto expected_finish = started_at +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(trajectory_duration));
    const auto deadline = expected_finish +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(allowed_timeout));
    const auto final_positions = reordered_positions(
        trajectory.joint_names, trajectory.points.back().positions);
    double last_maximum_error = std::numeric_limits<double>::infinity();
    std::size_t last_maximum_error_joint = 0;

    while (std::chrono::steady_clock::now() < deadline)
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
        // 即使提前进入终点容差，也必须等到轨迹计划终止时刻再结束 servo mode，
        // 否则小幅运动会在执行到一半时被提前停止。
        if (std::chrono::steady_clock::now() >= expected_finish &&
            maximum_error <= goal_tolerance_)
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
        "最终关节误差在超时前未进入容差: " +
        expected_joint_names_[last_maximum_error_joint] + " error=" +
        std::to_string(last_maximum_error) + " rad, tolerance=" +
        std::to_string(goal_tolerance_) + " rad");
}

}  // namespace jaka_driver
