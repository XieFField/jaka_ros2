#ifndef JAKA_DRIVER__NATIVE_JOINT_MOVE_UTILS_HPP_
#define JAKA_DRIVER__NATIVE_JOINT_MOVE_UTILS_HPP_

#include <string>
#include <vector>

namespace jaka_driver
{

struct NativeJointMoveGoal
{
    std::vector<double> target_positions;
    double speed{0.0};
    double acceleration{0.0};
    double endpoint_tolerance{0.0};
    double timeout{0.0};
};

struct NativeMotionReadiness
{
    bool powered{false};
    bool enabled{false};
    int error_code{0};
    bool program_idle{false};
    int queue_depth{0};
    int active_queue_depth{0};
    bool paused{false};
    bool on_limit{false};
    bool in_estop{false};
    bool in_collision{false};
    bool in_drag_mode{false};
};

bool validate_native_joint_move_goal(
    const NativeJointMoveGoal & goal,
    std::string & error);

bool validate_native_motion_readiness(
    const NativeMotionReadiness & readiness,
    std::string & error);

double maximum_joint_error(
    const std::vector<double> & target,
    const std::vector<double> & actual);

double estimate_native_joint_move_duration(
    double maximum_joint_delta,
    double programmed_speed,
    double programmed_acceleration,
    double rapid_rate);

}  // namespace jaka_driver

#endif  // JAKA_DRIVER__NATIVE_JOINT_MOVE_UTILS_HPP_
