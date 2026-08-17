#ifndef JAKA_DRIVER__CONTROL_OWNERSHIP_HPP_
#define JAKA_DRIVER__CONTROL_OWNERSHIP_HPP_

#include <atomic>

namespace jaka_driver
{

// 同一 SDK 会话在任一时刻只能由一种运动后端持有。
enum class ControlOwner
{
    kIdle,
    kTrajectory,
    kCompliance,
    kToolDrive,
    kLegacyMotion,
};

inline const char * control_owner_name(ControlOwner owner)
{
    switch (owner)
    {
        case ControlOwner::kIdle:
            return "idle";
        case ControlOwner::kTrajectory:
            return "trajectory";
        case ControlOwner::kCompliance:
            return "compliance";
        case ControlOwner::kToolDrive:
            return "tool_drive";
        case ControlOwner::kLegacyMotion:
            return "legacy_motion";
    }
    return "unknown";
}

inline bool try_acquire_control(
    std::atomic<ControlOwner> & owner,
    ControlOwner requested)
{
    ControlOwner expected = ControlOwner::kIdle;
    return owner.compare_exchange_strong(expected, requested);
}

inline void release_control(
    std::atomic<ControlOwner> & owner,
    ControlOwner expected_owner)
{
    ControlOwner expected = expected_owner;
    owner.compare_exchange_strong(expected, ControlOwner::kIdle);
}

}  // namespace jaka_driver

#endif  // JAKA_DRIVER__CONTROL_OWNERSHIP_HPP_
