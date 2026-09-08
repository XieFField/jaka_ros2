#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "jaka_msgs/action/execute_joint_move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{

const std::vector<std::string> kJointNames{
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

std::vector<double> ordered_positions(const sensor_msgs::msg::JointState & state)
{
  std::vector<double> result;
  result.reserve(kJointNames.size());
  for (const auto & expected : kJointNames)
  {
    const auto found = std::find(state.name.begin(), state.name.end(), expected);
    if (found == state.name.end()) return {};
    const auto index = static_cast<std::size_t>(
      std::distance(state.name.begin(), found));
    if (index >= state.position.size() || !std::isfinite(state.position[index]))
    {
      return {};
    }
    result.push_back(state.position[index]);
  }
  return result;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "native_joint_move_smoke",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  int exit_code = 1;
  try
  {
    bool activate = false;
    bool parameters_confirmed = false;
    std::string joint_name = "joint_1";
    double joint_delta = 0.01;
    double maximum_joint_delta = 0.10;
    double speed = 0.02;
    double acceleration = 0.05;
    double endpoint_tolerance = 0.002;
    double timeout = 30.0;
    node->get_parameter_or("activate", activate, activate);
    node->get_parameter_or(
      "parameters_confirmed", parameters_confirmed, parameters_confirmed);
    node->get_parameter_or("joint_name", joint_name, joint_name);
    node->get_parameter_or("joint_delta", joint_delta, joint_delta);
    node->get_parameter_or(
      "maximum_joint_delta", maximum_joint_delta, maximum_joint_delta);
    node->get_parameter_or("speed", speed, speed);
    node->get_parameter_or("acceleration", acceleration, acceleration);
    node->get_parameter_or(
      "endpoint_tolerance", endpoint_tolerance, endpoint_tolerance);
    node->get_parameter_or("timeout", timeout, timeout);

    if (!activate)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "NATIVE JOINT MOVE SMOKE: SAFE IDLE: activate=false; 未读取关节角，未发送运动 Goal");
      exit_code = 0;
    }
    else
    {
      if (!parameters_confirmed)
      {
        throw std::invalid_argument(
                "activate=true 时必须显式设置 parameters_confirmed=true");
      }
      const auto joint = std::find(
        kJointNames.begin(), kJointNames.end(), joint_name);
      if (joint == kJointNames.end() || !std::isfinite(joint_delta) ||
        !std::isfinite(maximum_joint_delta) || maximum_joint_delta <= 0.0 ||
        std::abs(joint_delta) > maximum_joint_delta || joint_delta == 0.0 ||
        !std::isfinite(speed) || speed <= 0.0 || speed > 0.20 ||
        !std::isfinite(acceleration) || acceleration <= 0.0 ||
        acceleration > 0.50 || !std::isfinite(endpoint_tolerance) ||
        endpoint_tolerance <= 0.0 || !std::isfinite(timeout) || timeout <= 0.0)
      {
        throw std::invalid_argument("原生 joint_move 冒烟参数无效或超过阶段一上限");
      }

      std::mutex state_mutex;
      std::condition_variable state_condition;
      sensor_msgs::msg::JointState latest_state;
      bool state_received = false;
      auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [&](const sensor_msgs::msg::JointState::SharedPtr message)
        {
          std::lock_guard<std::mutex> lock(state_mutex);
          latest_state = *message;
          state_received = true;
          state_condition.notify_all();
        });
      rclcpp::executors::MultiThreadedExecutor executor;
      executor.add_node(node);
      std::thread spin_thread([&]() {executor.spin();});
      try
      {
        sensor_msgs::msg::JointState initial_state;
        {
          std::unique_lock<std::mutex> lock(state_mutex);
          if (!state_condition.wait_for(
              lock, std::chrono::seconds(5), [&]() {return state_received;}))
          {
            throw std::runtime_error("等待 /joint_states 超时");
          }
          initial_state = latest_state;
        }
        auto target = ordered_positions(initial_state);
        if (target.size() != kJointNames.size())
        {
          throw std::runtime_error("/joint_states 不包含完整且有限的六关节位置");
        }
        const auto joint_index = static_cast<std::size_t>(
          std::distance(kJointNames.begin(), joint));
        const double initial = target[joint_index];
        target[joint_index] += joint_delta;

        using Action = jaka_msgs::action::ExecuteJointMove;
        auto client = rclcpp_action::create_client<Action>(
          node, "/jaka_driver/execute_joint_move");
        if (!client->wait_for_action_server(std::chrono::seconds(5)))
        {
          throw std::runtime_error("等待原生 joint_move Action 超时");
        }
        Action::Goal goal;
        goal.request_id = "native_joint_move_smoke";
        goal.target_positions = target;
        goal.speed = speed;
        goal.acceleration = acceleration;
        goal.endpoint_tolerance = endpoint_tolerance;
        goal.timeout = timeout;
        RCLCPP_INFO(
          node->get_logger(),
          "NATIVE JOINT MOVE SMOKE: SEND: joint=%s initial=%.9f target=%.9f delta=%.9f speed=%.6f acceleration=%.6f",
          joint_name.c_str(), initial, target[joint_index], joint_delta,
          speed, acceleration);
        auto goal_future = client->async_send_goal(goal);
        if (goal_future.wait_for(std::chrono::seconds(5)) !=
          std::future_status::ready)
        {
          client->async_cancel_all_goals();
          throw std::runtime_error("原生 joint_move Goal 响应超时，已请求取消");
        }
        const auto goal_handle = goal_future.get();
        if (!goal_handle) throw std::runtime_error("原生 joint_move Goal 被拒绝");
        auto result_future = client->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::duration<double>(timeout + 2.0)) !=
          std::future_status::ready)
        {
          client->async_cancel_goal(goal_handle);
          throw std::runtime_error("等待原生 joint_move 终态超时，已请求取消");
        }
        const auto result = result_future.get();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
          !result.result || !result.result->success)
        {
          throw std::runtime_error(
                  result.result ? result.result->message : "原生 joint_move 执行失败");
        }
        RCLCPP_INFO(
          node->get_logger(),
          "NATIVE JOINT MOVE SMOKE: PASS: max_error=%.9f rad",
          result.result->max_joint_error);
        exit_code = 0;
      }
      catch (...)
      {
        executor.cancel();
        if (spin_thread.joinable()) spin_thread.join();
        throw;
      }
      executor.cancel();
      if (spin_thread.joinable()) spin_thread.join();
      (void)subscription;
    }
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(
      node->get_logger(), "NATIVE JOINT MOVE SMOKE: FAIL: %s", exception.what());
    exit_code = 3;
  }
  rclcpp::shutdown();
  return exit_code;
}
