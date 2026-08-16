#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "jaka_driver/trajectory_utils.hpp"

namespace
{

const std::vector<std::string> kExpectedJoints{
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6"};

trajectory_msgs::msg::JointTrajectory valid_trajectory()
{
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = kExpectedJoints;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
    point.time_from_start.sec = 1;
    trajectory.points.push_back(point);
    return trajectory;
}

TEST(TrajectoryUtilsTest, AcceptsValidTrajectory)
{
    std::string error;
    EXPECT_TRUE(jaka_driver::validate_trajectory(
        valid_trajectory(), kExpectedJoints, 60.0, error));
}

TEST(TrajectoryUtilsTest, RejectsWrongJointAndNonFiniteValue)
{
    auto trajectory = valid_trajectory();
    trajectory.joint_names.back() = "wrong_joint";
    std::string error;
    EXPECT_FALSE(jaka_driver::validate_trajectory(
        trajectory, kExpectedJoints, 60.0, error));

    trajectory = valid_trajectory();
    trajectory.points.front().positions.front() =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(jaka_driver::validate_trajectory(
        trajectory, kExpectedJoints, 60.0, error));
}

TEST(TrajectoryUtilsTest, RejectsNonMonotonicAndExcessiveDuration)
{
    auto trajectory = valid_trajectory();
    auto decreasing_point = trajectory.points.front();
    decreasing_point.time_from_start.sec = 0;
    trajectory.points.push_back(decreasing_point);
    std::string error;
    EXPECT_FALSE(jaka_driver::validate_trajectory(
        trajectory, kExpectedJoints, 60.0, error));

    trajectory = valid_trajectory();
    EXPECT_FALSE(jaka_driver::validate_trajectory(
        trajectory, kExpectedJoints, 0.5, error));
}

TEST(TrajectoryUtilsTest, AcceptsSameTimeForUnchangedStopPosition)
{
    auto trajectory = valid_trajectory();
    auto stop_point = trajectory.points.front();
    stop_point.velocities.assign(kExpectedJoints.size(), 0.0);
    trajectory.points.push_back(stop_point);

    std::string error;
    EXPECT_TRUE(jaka_driver::validate_trajectory(
        trajectory, kExpectedJoints, 60.0, error));
}

TEST(TrajectoryUtilsTest, RejectsSameTimeForDifferentPosition)
{
    auto trajectory = valid_trajectory();
    auto conflicting_point = trajectory.points.front();
    conflicting_point.positions.front() += 0.01;
    trajectory.points.push_back(conflicting_point);

    std::string error;
    EXPECT_FALSE(jaka_driver::validate_trajectory(
        trajectory, kExpectedJoints, 60.0, error));
}

TEST(TrajectoryUtilsTest, ReordersJointValues)
{
    const std::vector<std::string> reversed_names(
        kExpectedJoints.rbegin(), kExpectedJoints.rend());
    const std::vector<double> reversed_values{6, 5, 4, 3, 2, 1};
    EXPECT_EQ(
        jaka_driver::reorder_joint_values(
            reversed_names, reversed_values, kExpectedJoints),
        (std::vector<double>{1, 2, 3, 4, 5, 6}));
}

TEST(TrajectoryUtilsTest, ComputesInterpolationSteps)
{
    const auto steps = jaka_driver::interpolation_steps(0.2, 0.28, 0.008);
    ASSERT_TRUE(steps.has_value());
    EXPECT_EQ(*steps, 10U);
    EXPECT_EQ(*jaka_driver::interpolation_steps(0.0, 0.0, 0.008), 1U);
    EXPECT_FALSE(jaka_driver::interpolation_steps(1.0, 0.5, 0.008));
}

TEST(TrajectoryUtilsTest, RejectsServoSegmentAboveConfiguredStepLimit)
{
    auto trajectory = valid_trajectory();
    std::string error;
    EXPECT_FALSE(jaka_driver::validate_servo_segments(
        trajectory, 0.008, 50U, error));

    trajectory.points.front().time_from_start.sec = 0;
    trajectory.points.front().time_from_start.nanosec = 100000000;
    EXPECT_TRUE(jaka_driver::validate_servo_segments(
        trajectory, 0.008, 50U, error));
}

}  // namespace
