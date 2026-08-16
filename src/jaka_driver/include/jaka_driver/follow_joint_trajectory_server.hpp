#ifndef JAKA_DRIVER__FOLLOW_JOINT_TRAJECTORY_SERVER_HPP_
#define JAKA_DRIVER__FOLLOW_JOINT_TRAJECTORY_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/control_ownership.hpp"

namespace jaka_driver
{

// 将标准 FollowJointTrajectory Action 接入现有的唯一 JAKA SDK 会话。
// 本类不负责登录、上电或使能，这些操作仍由显式生命周期服务完成。
class FollowJointTrajectoryServer
{
public:
    using Action = control_msgs::action::FollowJointTrajectory;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

    FollowJointTrajectoryServer(
        const rclcpp::Node::SharedPtr & node,
        JAKAZuRobot & robot,
        std::atomic<bool> & sdk_logged_in,
        std::atomic<ControlOwner> & control_owner,
        std::mutex & session_mutex,
        std::string action_name);

    ~FollowJointTrajectoryServer();

    // 供驱动的全局 stop 服务复用；仅设置取消并发送 SDK 停止命令。
    int request_stop();

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Action::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle>);
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    bool validate_goal(
        const Action::Goal & goal,
        std::string & error) const;
    bool robot_ready(std::string & error);
    bool wait_until(
        const std::chrono::steady_clock::time_point & deadline,
        const std::shared_ptr<GoalHandle> & goal_handle);
    int stop_motion_and_servo();
    std::vector<double> reordered_positions(
        const std::vector<std::string> & names,
        const std::vector<double> & positions) const;
    void publish_feedback(
        const std::shared_ptr<GoalHandle> & goal_handle,
        const trajectory_msgs::msg::JointTrajectoryPoint & desired,
        const JointValue & actual,
        const std::chrono::steady_clock::duration & elapsed) const;

    rclcpp::Node::SharedPtr node_;
    JAKAZuRobot & robot_;
    std::atomic<bool> & sdk_logged_in_;
    std::atomic<ControlOwner> & control_owner_;
    std::mutex & session_mutex_;
    rclcpp_action::Server<Action>::SharedPtr server_;
    const std::vector<std::string> expected_joint_names_{
        "joint_1", "joint_2", "joint_3",
        "joint_4", "joint_5", "joint_6"};
    double goal_tolerance_{0.01};
    double goal_timeout_{2.0};
    double servo_period_{0.008};
    unsigned int maximum_servo_steps_{50};
    double maximum_trajectory_duration_{300.0};

    mutable std::mutex worker_mutex_;
    std::thread worker_;
    std::atomic<bool> goal_active_{false};
    std::atomic<bool> cancel_requested_{false};
    std::atomic<bool> shutting_down_{false};
};

}  // namespace jaka_driver

#endif  // JAKA_DRIVER__FOLLOW_JOINT_TRAJECTORY_SERVER_HPP_
