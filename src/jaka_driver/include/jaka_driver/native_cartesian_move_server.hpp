#ifndef JAKA_DRIVER__NATIVE_CARTESIAN_MOVE_SERVER_HPP_
#define JAKA_DRIVER__NATIVE_CARTESIAN_MOVE_SERVER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "jaka_msgs/action/execute_cartesian_move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/control_ownership.hpp"

namespace jaka_driver
{

class NativeCartesianMoveServer
{
public:
  using Action = jaka_msgs::action::ExecuteCartesianMove;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  NativeCartesianMoveServer(
    const rclcpp::Node::SharedPtr & node, JAKAZuRobot & robot,
    std::atomic<bool> & sdk_logged_in,
    std::atomic<ControlOwner> & control_owner,
    std::mutex & session_mutex, std::string action_name);
  ~NativeCartesianMoveServer();

  int request_stop();

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Action::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);
  bool robot_ready(std::string & error);
  int abort_motion();

  rclcpp::Node::SharedPtr node_;
  JAKAZuRobot & robot_;
  std::atomic<bool> & sdk_logged_in_;
  std::atomic<ControlOwner> & control_owner_;
  std::mutex & session_mutex_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  double feedback_period_{0.05};
  mutable std::mutex worker_mutex_;
  std::thread worker_;
  std::atomic<bool> goal_active_{false};
  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> shutting_down_{false};
};

}  // namespace jaka_driver

#endif  // JAKA_DRIVER__NATIVE_CARTESIAN_MOVE_SERVER_HPP_
