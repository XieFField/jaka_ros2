#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "jaka_driver/native_joint_move_utils.hpp"

namespace
{

jaka_driver::NativeJointMoveGoal valid_goal()
{
    return {{0.0, 0.1, 0.2, 0.3, 0.4, 0.5}, 0.02, 0.05, 0.002, 15.0};
}

TEST(NativeJointMoveUtilsTest, AcceptsFiniteSixJointGoal)
{
    std::string error;
    EXPECT_TRUE(jaka_driver::validate_native_joint_move_goal(valid_goal(), error));
    EXPECT_TRUE(error.empty());
}

TEST(NativeJointMoveUtilsTest, RejectsInvalidGoalFields)
{
    std::string error;
    auto goal = valid_goal();
    goal.target_positions.pop_back();
    EXPECT_FALSE(jaka_driver::validate_native_joint_move_goal(goal, error));

    goal = valid_goal();
    goal.target_positions[2] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(jaka_driver::validate_native_joint_move_goal(goal, error));

    goal = valid_goal();
    goal.speed = 0.0;
    EXPECT_FALSE(jaka_driver::validate_native_joint_move_goal(goal, error));

    goal = valid_goal();
    goal.acceleration = -1.0;
    EXPECT_FALSE(jaka_driver::validate_native_joint_move_goal(goal, error));

    goal = valid_goal();
    goal.endpoint_tolerance = 0.0;
    EXPECT_FALSE(jaka_driver::validate_native_joint_move_goal(goal, error));

    goal = valid_goal();
    goal.timeout = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(jaka_driver::validate_native_joint_move_goal(goal, error));
}

TEST(NativeJointMoveUtilsTest, CalculatesMaximumEndpointError)
{
    EXPECT_NEAR(
        jaka_driver::maximum_joint_error(
            {0.0, 1.0, 2.0}, {0.001, 0.997, 2.002}),
        0.003, 1e-12);
    EXPECT_TRUE(std::isinf(
        jaka_driver::maximum_joint_error({0.0}, {0.0, 1.0})));
}

TEST(NativeJointMoveUtilsTest, AcceptsIdleRobotWithoutRequiringInPosition)
{
    jaka_driver::NativeMotionReadiness readiness;
    readiness.powered = true;
    readiness.enabled = true;
    readiness.program_idle = true;
    // The controller may retain these flags after an earlier command even
    // while the program is idle and both motion queues are empty.
    readiness.paused = true;
    readiness.on_limit = true;
    std::string error;
    EXPECT_TRUE(
        jaka_driver::validate_native_motion_readiness(readiness, error));
    EXPECT_TRUE(error.empty());
}

TEST(NativeJointMoveUtilsTest, RejectsActiveOrUnsafeRobot)
{
    jaka_driver::NativeMotionReadiness readiness;
    readiness.powered = true;
    readiness.enabled = true;
    readiness.program_idle = true;
    std::string error;

    readiness.active_queue_depth = 1;
    EXPECT_FALSE(
        jaka_driver::validate_native_motion_readiness(readiness, error));

    readiness.active_queue_depth = 0;
    readiness.in_collision = true;
    EXPECT_FALSE(
        jaka_driver::validate_native_motion_readiness(readiness, error));
}

}  // namespace
