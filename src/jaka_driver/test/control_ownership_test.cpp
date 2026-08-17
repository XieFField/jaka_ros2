#include <atomic>
#include <string>

#include "gtest/gtest.h"

#include "jaka_driver/control_ownership.hpp"

TEST(ControlOwnershipTest, ToolDriveHasIndependentOwner)
{
    std::atomic<jaka_driver::ControlOwner> owner{
        jaka_driver::ControlOwner::kIdle};

    EXPECT_TRUE(jaka_driver::try_acquire_control(
        owner, jaka_driver::ControlOwner::kToolDrive));
    EXPECT_STREQ(
        jaka_driver::control_owner_name(owner.load()), "tool_drive");
    EXPECT_FALSE(jaka_driver::try_acquire_control(
        owner, jaka_driver::ControlOwner::kTrajectory));

    jaka_driver::release_control(
        owner, jaka_driver::ControlOwner::kToolDrive);
    EXPECT_EQ(owner.load(), jaka_driver::ControlOwner::kIdle);
}
