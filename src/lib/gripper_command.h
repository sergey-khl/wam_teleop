#pragma once

#include <algorithm>
#include <cmath>

struct GripperCommand {
    double velocity = 0.0;
    bool spread_half_open = false;

    GripperCommand() = default;
    GripperCommand(double velocity, bool spread_half_open)
        : velocity(velocity)
        , spread_half_open(spread_half_open) {
    }

    bool fingers12Only() const {
        return spread_half_open;
    }
};

namespace gripper_command {

constexpr double kEncodedBase = 10000.0;
constexpr double kModeStride = 100.0;
constexpr double kVelocityOffset = 50.0;
constexpr double kMaxEncodedVelocity = 49.0;

inline double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

inline double moveToward(double current, double target, double max_delta) {
    if (target > current) {
        return std::min(target, current + max_delta);
    }
    return std::max(target, current - max_delta);
}

inline double encode(const GripperCommand& command) {
    const double velocity = clamp(command.velocity, -kMaxEncodedVelocity, kMaxEncodedVelocity);
    const double mode_offset = command.spread_half_open ? kModeStride : 0.0;
    return kEncodedBase + mode_offset + velocity + kVelocityOffset;
}

inline GripperCommand decode(double raw_value) {
    if (raw_value < kEncodedBase || raw_value >= kEncodedBase + (2.0 * kModeStride)) {
        return GripperCommand{raw_value, false};
    }

    double payload = raw_value - kEncodedBase;
    const bool spread_half_open = payload >= kModeStride;
    if (spread_half_open) {
        payload -= kModeStride;
    }

    return GripperCommand{payload - kVelocityOffset, spread_half_open};
}

} // namespace gripper_command
