#include "jaka_driver/native_cartesian_move_server.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include "jaka_driver/native_joint_move_utils.hpp"

namespace jaka_driver
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

CartesianPose sdk_pose(const std::array<double, 6> & values)
{
  CartesianPose pose{};
  pose.tran.x = values[0];
  pose.tran.y = values[1];
  pose.tran.z = values[2];
  pose.rpy.rx = values[3];
  pose.rpy.ry = values[4];
  pose.rpy.rz = values[5];
  return pose;
}

bool finite_pose(const std::array<double, 6> & values)
{
  return std::all_of(
    values.begin(), values.end(),
    [](double value) {return std::isfinite(value);});
}

double wrapped_error(double target, double actual)
{
  return std::abs(std::remainder(target - actual, 2.0 * kPi));
}

std::pair<double, double> pose_error(
  const CartesianPose & target, const CartesianPose & actual)
{
  const double dx = target.tran.x - actual.tran.x;
  const double dy = target.tran.y - actual.tran.y;
  const double dz = target.tran.z - actual.tran.z;
  const double translation = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double rotation = std::max({
      wrapped_error(target.rpy.rx, actual.rpy.rx),
      wrapped_error(target.rpy.ry, actual.rpy.ry),
      wrapped_error(target.rpy.rz, actual.rpy.rz)});
  return {translation, rotation};
}

bool valid_goal(
  const jaka_msgs::action::ExecuteCartesianMove::Goal & goal,
  std::string & error)
{
  const bool linear = goal.motion_type == goal.LINEAR;
  const bool circular = goal.motion_type == goal.CIRCULAR;
  if (!linear && !circular)
  {
    error = "motion_type 必须为 LINEAR 或 CIRCULAR";
    return false;
  }
  if (!finite_pose(goal.target_pose) ||
    (circular && (!goal.has_midpoint || !finite_pose(goal.midpoint_pose))))
  {
    error = "笛卡尔目标或圆弧中间点无效";
    return false;
  }
  if (!std::isfinite(goal.speed) || goal.speed <= 0.0 || goal.speed > 500.0 ||
    !std::isfinite(goal.acceleration) || goal.acceleration <= 0.0 ||
    goal.acceleration > 2000.0 ||
    !std::isfinite(goal.orientation_speed) ||
    goal.orientation_speed <= 0.0 || goal.orientation_speed > kPi ||
    !std::isfinite(goal.orientation_acceleration) ||
    goal.orientation_acceleration <= 0.0 ||
    goal.orientation_acceleration > 4.0 * kPi ||
    !std::isfinite(goal.translation_tolerance) ||
    goal.translation_tolerance <= 0.0 ||
    !std::isfinite(goal.rotation_tolerance) ||
    goal.rotation_tolerance <= 0.0 || !std::isfinite(goal.timeout) ||
    goal.timeout <= 0.0)
  {
    error = "原生笛卡尔运动速度、加速度、容差或超时无效";
    return false;
  }
  return true;
}

}  // namespace

NativeCartesianMoveServer::NativeCartesianMoveServer(
  const rclcpp::Node::SharedPtr & node, JAKAZuRobot & robot,
  std::atomic<bool> & sdk_logged_in,
  std::atomic<ControlOwner> & control_owner,
  std::mutex & session_mutex, std::string action_name)
: node_(node), robot_(robot), sdk_logged_in_(sdk_logged_in),
  control_owner_(control_owner), session_mutex_(session_mutex)
{
  feedback_period_ = node_->declare_parameter<double>(
    "native_cartesian_move_feedback_period", 0.05);
  if (!std::isfinite(feedback_period_) || feedback_period_ <= 0.0)
  {
    throw std::invalid_argument(
            "native_cartesian_move_feedback_period 必须为有限正数");
  }
  server_ = rclcpp_action::create_server<Action>(
    node_, action_name,
    std::bind(
      &NativeCartesianMoveServer::handle_goal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &NativeCartesianMoveServer::handle_cancel, this,
      std::placeholders::_1),
    std::bind(
      &NativeCartesianMoveServer::handle_accepted, this,
      std::placeholders::_1));
}

NativeCartesianMoveServer::~NativeCartesianMoveServer()
{
  shutting_down_.store(true);
  if (goal_active_.load()) request_stop();
  std::lock_guard<std::mutex> lock(worker_mutex_);
  if (worker_.joinable()) worker_.join();
}

bool NativeCartesianMoveServer::robot_ready(std::string & error)
{
  std::lock_guard<std::mutex> lock(session_mutex_);
  if (!sdk_logged_in_.load())
  {
    error = "JAKA SDK 尚未登录";
    return false;
  }
  RobotStatus_simple status{};
  MotionStatus motion{};
  ProgramState program = PROGRAM_IDLE;
  BOOL drag = FALSE;
  const int status_ret = robot_.get_robot_status_simple(&status);
  const int motion_ret = robot_.get_motion_status(&motion);
  const int program_ret = robot_.get_program_state(&program);
  const int drag_ret = robot_.is_in_drag_mode(&drag);
  if (status_ret != 0 || motion_ret != 0 || program_ret != 0 || drag_ret != 0)
  {
    error = "读取机器人状态失败";
    return false;
  }
  const NativeMotionReadiness readiness{
    static_cast<bool>(status.powered_on), static_cast<bool>(status.enabled),
    status.errcode, program == PROGRAM_IDLE, motion.queue,
    motion.active_queue, static_cast<bool>(motion.paused),
    static_cast<bool>(motion.isOnLimit), static_cast<bool>(motion.isInEstop),
    static_cast<bool>(motion.isInCollision), static_cast<bool>(drag)};
  if (!validate_native_motion_readiness(readiness, error)) return false;
  return true;
}

rclcpp_action::GoalResponse NativeCartesianMoveServer::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const Action::Goal> goal)
{
  bool expected = false;
  if (!goal_active_.compare_exchange_strong(expected, true))
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::string error;
  if (!valid_goal(*goal, error) || !robot_ready(error) ||
    !try_acquire_control(
      control_owner_, ControlOwner::kNativeCartesianMotion))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "拒绝原生笛卡尔 Goal: %s; owner=%s",
      error.c_str(), control_owner_name(control_owner_.load()));
    goal_active_.store(false);
    return rclcpp_action::GoalResponse::REJECT;
  }
  cancel_requested_.store(false);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NativeCartesianMoveServer::handle_cancel(
  const std::shared_ptr<GoalHandle>)
{
  return goal_active_.load() ? rclcpp_action::CancelResponse::ACCEPT :
         rclcpp_action::CancelResponse::REJECT;
}

void NativeCartesianMoveServer::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  if (worker_.joinable()) worker_.join();
  worker_ = std::thread(
    &NativeCartesianMoveServer::execute, this, goal_handle);
}

int NativeCartesianMoveServer::abort_motion()
{
  cancel_requested_.store(true);
  std::lock_guard<std::mutex> lock(session_mutex_);
  return sdk_logged_in_.load() ? robot_.motion_abort() : 0;
}

int NativeCartesianMoveServer::request_stop()
{
  return abort_motion();
}

void NativeCartesianMoveServer::execute(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<Action::Result>();
  result->translation_error = std::numeric_limits<double>::infinity();
  result->rotation_error = std::numeric_limits<double>::infinity();
  const auto finish = [&]() {
      release_control(
        control_owner_, ControlOwner::kNativeCartesianMotion);
      goal_active_.store(false);
    };
  const auto fail = [&](int code, const std::string & message) {
      result->success = false;
      result->sdk_error_code = code;
      result->message = message;
      goal_handle->abort(result);
      finish();
    };

  const CartesianPose target = sdk_pose(goal->target_pose);
  const CartesianPose midpoint = sdk_pose(goal->midpoint_pose);
  double rapid_rate = 0.0;
  int command_ret = 0;
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    command_ret = robot_.get_rapidrate(&rapid_rate);
    if (command_ret == 0 && (!std::isfinite(rapid_rate) ||
      rapid_rate <= 0.0 || rapid_rate > 1.0))
    {
      command_ret = -2;
    }
    if (command_ret == 0 && goal->motion_type == goal->LINEAR)
    {
      command_ret = robot_.linear_move(
        &target, MoveMode::ABS, FALSE, goal->speed, goal->acceleration,
        0.0, nullptr, goal->orientation_speed,
        goal->orientation_acceleration);
    }
    else if (command_ret == 0)
    {
      command_ret = robot_.circular_move(
        &target, &midpoint, MoveMode::ABS, FALSE, goal->speed,
        goal->acceleration, 0.0, nullptr, 0.0, 0);
    }
  }
  if (command_ret != 0)
  {
    fail(command_ret, "JAKA 原生 linear_move/circular_move 下发失败");
    return;
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "NATIVE CARTESIAN START: request=%s, type=%s, programmed_speed=%.3f mm/s, rapid_rate=%.3f, effective_speed=%.3f mm/s, acceleration=%.3f mm/s^2, timeout=%.3f s",
    goal->request_id.c_str(),
    goal->motion_type == goal->LINEAR ? "LIN" : "CIRC",
    goal->speed, rapid_rate, goal->speed * rapid_rate,
    goal->acceleration, goal->timeout);

  const auto started = std::chrono::steady_clock::now();
  const auto deadline = started + std::chrono::duration<double>(goal->timeout);
  const auto period = std::chrono::duration<double>(feedback_period_);
  while (rclcpp::ok() && !shutting_down_.load())
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - started).count();
    if (goal_handle->is_canceling())
    {
      const int ret = abort_motion();
      result->sdk_error_code = ret;
      result->message = "原生笛卡尔运动已取消";
      goal_handle->canceled(result);
      finish();
      return;
    }
    if (cancel_requested_.load())
    {
      fail(0, "原生笛卡尔运动被外部停止");
      return;
    }
    if (now >= deadline)
    {
      const int ret = abort_motion();
      fail(ret, "原生笛卡尔运动执行超时");
      return;
    }

    RobotStatus_simple status{};
    MotionStatus motion{};
    CartesianPose actual{};
    int status_ret;
    int motion_ret;
    int pose_ret;
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      status_ret = robot_.get_robot_status_simple(&status);
      motion_ret = robot_.get_motion_status(&motion);
      pose_ret = robot_.get_tcp_position(&actual);
    }
    if (status_ret != 0 || motion_ret != 0 || pose_ret != 0)
    {
      abort_motion();
      fail(
        status_ret != 0 ? status_ret :
        (motion_ret != 0 ? motion_ret : pose_ret),
        "原生笛卡尔运动状态读取失败");
      return;
    }
    const auto errors = pose_error(target, actual);
    result->translation_error = errors.first;
    result->rotation_error = errors.second;
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->elapsed = elapsed;
    feedback->in_position = motion.inpos;
    feedback->queue_depth = motion.queue;
    feedback->translation_error = errors.first;
    feedback->rotation_error = errors.second;
    feedback->powered = status.powered_on;
    feedback->enabled = status.enabled;
    feedback->controller_error_code = status.errcode;
    goal_handle->publish_feedback(feedback);

    if (!status.powered_on || !status.enabled || status.errcode != 0 ||
      motion.isInCollision || motion.isInEstop)
    {
      abort_motion();
      fail(status.errcode, "原生笛卡尔运动期间机器人状态异常");
      return;
    }
    if (motion.inpos && motion.queue == 0 &&
      errors.first <= goal->translation_tolerance &&
      errors.second <= goal->rotation_tolerance)
    {
      result->success = true;
      result->sdk_error_code = 0;
      result->message = "JAKA 原生笛卡尔运动成功";
      RCLCPP_INFO(
        node_->get_logger(),
        "NATIVE CARTESIAN PASS: request=%s, elapsed=%.3f s, translation_error=%.6f mm, rotation_error=%.6f rad",
        goal->request_id.c_str(), elapsed, errors.first, errors.second);
      goal_handle->succeed(result);
      finish();
      return;
    }
    std::this_thread::sleep_for(period);
  }
  abort_motion();
  fail(0, "原生笛卡尔运动因 ROS 关闭而终止");
}

}  // namespace jaka_driver
