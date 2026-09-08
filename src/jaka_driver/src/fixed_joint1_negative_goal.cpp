#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "jaka_driver/trajectory_utils.hpp"
#include "jaka_msgs/msg/robot_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

const std::vector<std::string> kJointNames{
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6"};

bool reorder_joint_state(
    const sensor_msgs::msg::JointState & state,
    std::vector<double> & positions)
{
    positions.clear();
    positions.reserve(kJointNames.size());
    for (const auto & name : kJointNames)
    {
        const auto iterator = std::find(state.name.begin(), state.name.end(), name);
        if (iterator == state.name.end())
        {
            return false;
        }
        const auto index = static_cast<std::size_t>(
            std::distance(state.name.begin(), iterator));
        if (index >= state.position.size() ||
            !std::isfinite(state.position[index]))
        {
            return false;
        }
        positions.push_back(state.position[index]);
    }
    return true;
}

builtin_interfaces::msg::Duration duration_from_seconds(double seconds)
{
    return static_cast<builtin_interfaces::msg::Duration>(
        rclcpp::Duration::from_seconds(seconds));
}

FollowJointTrajectory::Goal make_negative_goal(
    const std::vector<double> & initial_positions,
    double joint_delta,
    double duration,
    std::size_t point_count,
    jaka_driver::ScalarMotionProfile profile,
    double goal_time_tolerance)
{
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = kJointNames;
    for (std::size_t index = 0U; index < point_count; ++index)
    {
        const double elapsed = duration * static_cast<double>(index) /
            static_cast<double>(point_count - 1U);
        const auto sample = jaka_driver::sample_scalar_motion_profile(
            profile, elapsed, duration);
        if (!sample)
        {
            throw std::invalid_argument("无法生成固定 Goal 的运动曲线");
        }
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = initial_positions;
        point.positions[0] += joint_delta * sample->position_ratio;
        point.velocities.assign(kJointNames.size(), 0.0);
        point.accelerations.assign(kJointNames.size(), 0.0);
        point.velocities[0] = joint_delta * sample->velocity_ratio;
        point.accelerations[0] = joint_delta * sample->acceleration_ratio;
        point.time_from_start = duration_from_seconds(elapsed);
        goal.trajectory.points.push_back(std::move(point));
    }
    goal.goal_time_tolerance = duration_from_seconds(goal_time_tolerance);
    return goal;
}

template<typename FutureT>
bool ready_within(FutureT & future, double timeout)
{
    return future.wait_for(std::chrono::duration<double>(timeout)) ==
           std::future_status::ready;
}

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "fixed_joint1_negative_goal",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    bool activate = false;
    bool parameters_confirmed = false;
    double joint_delta = -0.02;
    double duration = 2.0;
    int64_t point_count_parameter = 21;
    std::string motion_profile_name = "quintic";
    double endpoint_tolerance = 0.002;
    double goal_time_tolerance = 15.0;
    double state_timeout = 3.0;
    double state_maximum_age = 0.5;
    std::string action_name = "/jaka_s5_controller/follow_joint_trajectory";
    node->get_parameter_or("activate", activate, false);
    node->get_parameter_or("parameters_confirmed", parameters_confirmed, false);
    node->get_parameter_or("joint_delta", joint_delta, -0.02);
    node->get_parameter_or("duration", duration, 2.0);
    node->get_parameter_or("point_count", point_count_parameter, int64_t{21});
    node->get_parameter_or(
        "motion_profile", motion_profile_name, std::string("quintic"));
    node->get_parameter_or("endpoint_tolerance", endpoint_tolerance, 0.002);
    node->get_parameter_or("goal_time_tolerance", goal_time_tolerance, 15.0);
    node->get_parameter_or("state_timeout", state_timeout, 3.0);
    node->get_parameter_or("state_maximum_age", state_maximum_age, 0.5);
    node->get_parameter_or(
        "trajectory_action", action_name,
        std::string("/jaka_s5_controller/follow_joint_trajectory"));

    const auto point_count = point_count_parameter > 0 ?
        static_cast<std::size_t>(point_count_parameter) : 0U;
    const auto motion_profile = jaka_driver::parse_scalar_motion_profile(
        motion_profile_name);
    if (!std::isfinite(joint_delta) || joint_delta >= -0.001 ||
        joint_delta < -0.10 || !std::isfinite(duration) || duration < 2.0 ||
        point_count < 3U || point_count > 1001U ||
        !std::isfinite(endpoint_tolerance) || endpoint_tolerance <= 0.0 ||
        endpoint_tolerance > 0.01 ||
        !std::isfinite(goal_time_tolerance) || goal_time_tolerance <= 0.0 ||
        !std::isfinite(state_timeout) || state_timeout <= 0.0 ||
        !std::isfinite(state_maximum_age) || state_maximum_age <= 0.0 ||
        action_name.empty() || !motion_profile)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "参数无效：仅允许 joint_1 负向 [-0.10,-0.001) rad、"
            "duration>=2 s、endpoint_tolerance<=0.01 rad，"
            "motion_profile 仅允许 quintic 或 linear");
        rclcpp::shutdown();
        return 2;
    }
    if (activate && !parameters_confirmed)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "activate=true 时必须显式设置 parameters_confirmed:=true");
        rclcpp::shutdown();
        return 2;
    }
    if (activate &&
        *motion_profile == jaka_driver::ScalarMotionProfile::kLinear)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "motion_profile=linear 仅允许离线诊断，不允许发送到真机；"
            "请使用 motion_profile=quintic");
        rclcpp::shutdown();
        return 2;
    }

    std::mutex state_mutex;
    std::condition_variable state_condition;
    sensor_msgs::msg::JointState latest_joint_state;
    jaka_msgs::msg::RobotMsg latest_robot_state;
    std::chrono::steady_clock::time_point joint_received_at{};
    std::chrono::steady_clock::time_point robot_received_at{};
    bool joint_received = false;
    bool robot_received = false;
    auto joint_subscription =
        node->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            [&](sensor_msgs::msg::JointState::SharedPtr message) {
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    latest_joint_state = *message;
                    joint_received_at = std::chrono::steady_clock::now();
                    joint_received = true;
                }
                state_condition.notify_all();
            });
    auto robot_subscription =
        node->create_subscription<jaka_msgs::msg::RobotMsg>(
            "/jaka_driver/robot_states", 10,
            [&](jaka_msgs::msg::RobotMsg::SharedPtr message) {
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    latest_robot_state = *message;
                    robot_received_at = std::chrono::steady_clock::now();
                    robot_received = true;
                }
                state_condition.notify_all();
            });
    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
        node, action_name);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});
    GoalHandle::SharedPtr active_goal;
    int exit_code = 1;
    try
    {
        {
            std::unique_lock<std::mutex> lock(state_mutex);
            if (!state_condition.wait_for(
                    lock, std::chrono::duration<double>(state_timeout),
                    [&]() {return joint_received && robot_received;}))
            {
                throw std::runtime_error(
                    "等待 /joint_states 或 /jaka_driver/robot_states 超时");
            }
        }
        if (!action_client->wait_for_action_server(
                std::chrono::duration<double>(state_timeout)))
        {
            throw std::runtime_error("等待 FollowJointTrajectory Action 超时");
        }
        const auto status_topic = action_name + "/_action/status";
        const auto action_publishers = node->get_publishers_info_by_topic(
            status_topic);
        if (action_publishers.size() != 1U)
        {
            throw std::runtime_error(
                "FollowJointTrajectory Action server 数量不是 1: " +
                std::to_string(action_publishers.size()));
        }

        std::vector<double> initial_positions;
        jaka_msgs::msg::RobotMsg robot_state;
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            const auto now = std::chrono::steady_clock::now();
            const double joint_age = std::chrono::duration<double>(
                now - joint_received_at).count();
            const double robot_age = std::chrono::duration<double>(
                now - robot_received_at).count();
            if (joint_age > state_maximum_age || robot_age > state_maximum_age)
            {
                throw std::runtime_error("机器人状态数据不新鲜");
            }
            if (!reorder_joint_state(latest_joint_state, initial_positions))
            {
                throw std::runtime_error("/joint_states 缺少有效的 JAKA 六轴位置");
            }
            robot_state = latest_robot_state;
        }

        const auto goal = make_negative_goal(
            initial_positions, joint_delta, duration, point_count,
            *motion_profile, goal_time_tolerance);
        RCLCPP_INFO(
            node->get_logger(),
            "固定负向 Goal 已生成: joint_1 initial=%.9f target=%.9f "
            "delta=%.9f rad, duration=%.3f s, points=%zu, profile=%s, action=%s",
            initial_positions[0], initial_positions[0] + joint_delta,
            joint_delta, duration, point_count, motion_profile_name.c_str(),
            action_name.c_str());

        if (!activate)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "JOINT1 NEGATIVE TRACKING: SAFE IDLE: activate=false；"
                "已验证状态与唯一 Action，未发送 Goal");
            exit_code = 0;
        }
        else
        {
            if (robot_state.motion_state != 0 || robot_state.power_state != 1 ||
                robot_state.servo_state != 1 || robot_state.collision_state != 0)
            {
                throw std::runtime_error(
                    "机器人未处于 motion=0,power=1,servo=1,collision=0");
            }
            auto goal_future = action_client->async_send_goal(goal);
            if (!ready_within(goal_future, state_timeout))
            {
                throw std::runtime_error("等待固定 Goal 响应超时");
            }
            active_goal = goal_future.get();
            if (!active_goal)
            {
                throw std::runtime_error("固定 Goal 被驱动拒绝");
            }
            auto result_future = action_client->async_get_result(active_goal);
            const double result_timeout = duration + goal_time_tolerance + 5.0;
            if (!ready_within(result_future, result_timeout))
            {
                auto cancel_future = action_client->async_cancel_goal(active_goal);
                (void)ready_within(cancel_future, state_timeout);
                throw std::runtime_error("等待固定 Goal 结果超时，已请求取消");
            }
            const auto result = result_future.get();
            active_goal.reset();
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
                !result.result ||
                result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
            {
                const std::string detail = result.result ?
                    result.result->error_string : "结果为空";
                throw std::runtime_error("固定 Goal 执行失败: " + detail);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            std::vector<double> final_positions;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                if (!reorder_joint_state(latest_joint_state, final_positions))
                {
                    throw std::runtime_error("终点关节状态无效");
                }
            }
            const double target = initial_positions[0] + joint_delta;
            const double endpoint_error = std::abs(target - final_positions[0]);
            if (endpoint_error > endpoint_tolerance)
            {
                throw std::runtime_error(
                    "joint_1 终点误差超限: error=" +
                    std::to_string(endpoint_error));
            }
            RCLCPP_INFO(
                node->get_logger(),
                "JOINT1 NEGATIVE TRACKING: PASS: target=%.9f actual=%.9f "
                "error=%.9f rad",
                target, final_positions[0], endpoint_error);
            exit_code = 0;
        }
    }
    catch (const std::exception & exception)
    {
        RCLCPP_ERROR(
            node->get_logger(), "JOINT1 NEGATIVE TRACKING: FAIL: %s",
            exception.what());
        if (active_goal)
        {
            auto cancel_future = action_client->async_cancel_goal(active_goal);
            (void)ready_within(cancel_future, state_timeout);
        }
        exit_code = 3;
    }

    executor.cancel();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
    rclcpp::shutdown();
    return exit_code;
}
