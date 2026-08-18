#include <algorithm>
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

TEST(TrajectoryUtilsTest, ResamplesLinearTrajectoryAtServoPeriod)
{
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = kExpectedJoints;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = std::vector<double>(6, 1.0);
    point.time_from_start.sec = 1;
    trajectory.points.push_back(point);

    const auto schedule = jaka_driver::build_timed_servo_schedule(
        trajectory, kExpectedJoints, std::vector<double>(6, 0.0),
        0.25, 1U, 10U);

    ASSERT_TRUE(schedule.valid) << schedule.error;
    ASSERT_EQ(schedule.setpoints.size(), 4U);
    EXPECT_DOUBLE_EQ(schedule.planned_duration, 1.0);
    EXPECT_DOUBLE_EQ(schedule.scheduled_duration, 1.0);
    EXPECT_DOUBLE_EQ(schedule.setpoints[0].command_time, 0.0);
    EXPECT_DOUBLE_EQ(schedule.setpoints[0].reference_time, 0.25);
    EXPECT_EQ(schedule.setpoints[0].step_num, 1U);
    EXPECT_DOUBLE_EQ(schedule.setpoints[0].positions[0], 0.25);
    EXPECT_DOUBLE_EQ(schedule.setpoints[3].command_time, 0.75);
    EXPECT_DOUBLE_EQ(schedule.setpoints[3].positions[0], 1.0);
}

TEST(TrajectoryUtilsTest, ResamplingRoundsDurationUpAndEndsAtExactTarget)
{
    auto trajectory = valid_trajectory();
    trajectory.points.front().time_from_start.sec = 0;
    trajectory.points.front().time_from_start.nanosec = 21000000U;

    const auto schedule = jaka_driver::build_timed_servo_schedule(
        trajectory, kExpectedJoints, std::vector<double>(6, 0.0),
        0.008, 1U, 10U);

    ASSERT_TRUE(schedule.valid) << schedule.error;
    ASSERT_EQ(schedule.setpoints.size(), 3U);
    EXPECT_DOUBLE_EQ(schedule.planned_duration, 0.021);
    EXPECT_DOUBLE_EQ(schedule.scheduled_duration, 0.024);
    EXPECT_DOUBLE_EQ(schedule.setpoints.back().reference_time, 0.021);
    EXPECT_EQ(
        schedule.setpoints.back().positions,
        trajectory.points.front().positions);
}

TEST(TrajectoryUtilsTest, ResamplingReordersJoints)
{
    auto trajectory = valid_trajectory();
    std::reverse(trajectory.joint_names.begin(), trajectory.joint_names.end());
    trajectory.points.front().positions = {6, 5, 4, 3, 2, 1};

    const auto schedule = jaka_driver::build_timed_servo_schedule(
        trajectory, kExpectedJoints, std::vector<double>(6, 0.0),
        1.0, 1U, 10U);

    ASSERT_TRUE(schedule.valid) << schedule.error;
    EXPECT_EQ(
        schedule.setpoints.back().positions,
        (std::vector<double>{1, 2, 3, 4, 5, 6}));
}

TEST(TrajectoryUtilsTest, ResamplingRejectsInvalidInputsAndSampleOverflow)
{
    auto trajectory = valid_trajectory();
    EXPECT_FALSE(jaka_driver::build_timed_servo_schedule(
        trajectory, kExpectedJoints, {}, 0.008, 1U, 1000U).valid);
    EXPECT_FALSE(jaka_driver::build_timed_servo_schedule(
        trajectory, kExpectedJoints, std::vector<double>(6, 0.0),
        0.0, 1U, 1000U).valid);
    EXPECT_FALSE(jaka_driver::build_timed_servo_schedule(
        trajectory, kExpectedJoints, std::vector<double>(6, 0.0),
        0.008, 1U, 10U).valid);
    EXPECT_FALSE(jaka_driver::build_timed_servo_schedule(
        trajectory, kExpectedJoints, std::vector<double>(6, 0.0),
        0.008, 0U, 1000U).valid);
    EXPECT_FALSE(jaka_driver::build_timed_servo_schedule(
        trajectory, kExpectedJoints, std::vector<double>(6, 0.0),
        0.008, jaka_driver::kMaximumServoStepNum + 1U, 1000U).valid);
}

TEST(TrajectoryUtilsTest, LongMoveItTrajectoryUsesControllerCycleMultiple)
{
    auto trajectory = valid_trajectory();
    trajectory.points.front().time_from_start.sec = 45;
    trajectory.points.front().time_from_start.nanosec = 660000000U;

    const auto schedule = jaka_driver::build_timed_servo_schedule(
        trajectory, kExpectedJoints, std::vector<double>(6, 0.0),
        jaka_driver::kJakaServoInterpolationCycle, 4U, 10000U);

    ASSERT_TRUE(schedule.valid) << schedule.error;
    EXPECT_EQ(schedule.setpoints.size(), 1427U);
    EXPECT_NEAR(schedule.scheduled_duration, 45.664, 1e-12);
    EXPECT_EQ(schedule.setpoints.front().step_num, 4U);
}

TEST(TrajectoryUtilsTest, SummarizesServoCallTiming)
{
    const std::vector<jaka_driver::ServoCallTiming> calls{
        {0.000, 0.000, 0.005},
        {0.032, 0.034, 0.054},
        {0.064, 0.074, 0.104},
    };
    const auto summary = jaka_driver::summarize_servo_timing(calls, 0.008);

    EXPECT_EQ(summary.samples, 3U);
    EXPECT_EQ(summary.late_samples, 1U);
    EXPECT_NEAR(summary.maximum_lateness, 0.010, 1e-12);
    EXPECT_NEAR(summary.call_duration_p50, 0.020, 1e-12);
    EXPECT_NEAR(summary.maximum_call_duration, 0.030, 1e-12);
}

TEST(TrajectoryUtilsTest, AbortsOnlyAfterConfiguredConsecutiveOverruns)
{
    std::size_t consecutive = 0U;
    EXPECT_FALSE(jaka_driver::update_servo_overrun_state(
        0.009, 0.008, 1U, consecutive));
    EXPECT_EQ(consecutive, 1U);
    EXPECT_FALSE(jaka_driver::update_servo_overrun_state(
        0.001, 0.008, 1U, consecutive));
    EXPECT_EQ(consecutive, 0U);
    EXPECT_FALSE(jaka_driver::update_servo_overrun_state(
        0.009, 0.008, 1U, consecutive));
    EXPECT_TRUE(jaka_driver::update_servo_overrun_state(
        0.010, 0.008, 1U, consecutive));
}

TEST(TrajectoryUtilsTest, EndpointTimeoutStartsAfterLaterFinishEvent)
{
    const auto normal = jaka_driver::endpoint_deadline_offset(
        1.416, 1.200, 2.0);
    ASSERT_TRUE(normal.has_value());
    EXPECT_DOUBLE_EQ(*normal, 3.416);

    const auto delayed = jaka_driver::endpoint_deadline_offset(
        1.416, 3.836, 2.0);
    ASSERT_TRUE(delayed.has_value());
    EXPECT_DOUBLE_EQ(*delayed, 5.836);

    EXPECT_FALSE(jaka_driver::endpoint_deadline_offset(
        1.416, 3.836, 0.0).has_value());
}

}  // namespace
