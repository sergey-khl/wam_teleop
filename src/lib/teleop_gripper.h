#pragma once

#include "gripper/barrett/barrett_hand.h"
#include "gripper/barrett/constants.h"
#include "gripper/gecko/gecko_gripper.h"
#include "teleop_config_loader.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef __linux__
#include <dirent.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#endif

namespace teleop_gripper_detail {

inline bool startsWith(const std::string& value, const std::string& prefix) {
    return value.size() >= prefix.size() && std::equal(prefix.begin(), prefix.end(), value.begin());
}

inline std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

inline std::string normalizeSocketCanPort(std::string port) {
    if (startsWith(port, "socketcan:")) {
        port = port.substr(std::string("socketcan:").size());
    } else if (startsWith(port, "can://")) {
        port = port.substr(std::string("can://").size());
    } else if (startsWith(port, "can:")) {
        port = port.substr(std::string("can:").size());
    }
    return port;
}

inline bool isAutoPort(const std::string& port) {
    return lower(port) == "auto" || lower(port) == "socketcan:auto";
}

inline bool isSocketCanPort(const std::string& port) {
    if (isAutoPort(port)) {
        return true;
    }
    const std::string normalized = normalizeSocketCanPort(port);
    return !normalized.empty() && normalized.find('/') == std::string::npos
           && (startsWith(normalized, "can") || startsWith(normalized, "vcan")
               || startsWith(normalized, "slcan"));
}

#ifdef __linux__
inline canid_t makeDirectCanId(int from, int to) {
    return static_cast<canid_t>(((from & 0x1f) << 5) | (to & 0x1f));
}

inline bool writeCanGet(int fd, int puck, uint8_t prop) {
    can_frame frame{};
    frame.can_id = makeDirectCanId(0, puck);
    frame.can_dlc = 1;
    frame.data[0] = prop & 0x7f;
    return write(fd, &frame, sizeof(frame)) == sizeof(frame);
}

inline void drainCanFrames(int fd) {
    pollfd pfd{fd, POLLIN, 0};
    can_frame frame{};
    while (poll(&pfd, 1, 0) > 0) {
        const ssize_t bytes_read = read(fd, &frame, sizeof(frame));
        if (bytes_read <= 0) {
            break;
        }
        pfd.revents = 0;
    }
}

inline bool waitForPuckReply(int fd, int puck, uint8_t prop, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        const int remaining_ms = std::max(
            1,
            static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now())
                    .count()
            )
        );

        pollfd pfd{fd, POLLIN, 0};
        const int poll_result = poll(&pfd, 1, remaining_ms);
        if (poll_result <= 0) {
            continue;
        }

        can_frame frame{};
        if (read(fd, &frame, sizeof(frame)) != sizeof(frame)) {
            continue;
        }
        if (frame.can_id & (CAN_ERR_FLAG | CAN_EFF_FLAG)) {
            continue;
        }

        const canid_t id = frame.can_id & CAN_SFF_MASK;
        const bool is_group = (id & 0x400) != 0;
        const int from = static_cast<int>((id >> 5) & 0x1f);
        const int to = static_cast<int>(id & 0x1f);
        if (is_group && from == puck && to == 6 && frame.can_dlc >= 4
            && (frame.data[0] & 0x80) && (frame.data[0] & 0x7f) == prop) {
            return true;
        }
    }
    return false;
}

inline int countBarrettPuckReplies(const std::string& port) {
    const std::string ifname = normalizeSocketCanPort(port);
    const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        return 0;
    }

    ifreq ifr{};
    strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        close(fd);
        return 0;
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(fd);
        return 0;
    }

    constexpr uint8_t kPropStat = 5;
    int replies = 0;
    for (int puck = 11; puck <= 14; ++puck) {
        drainCanFrames(fd);
        if (writeCanGet(fd, puck, kPropStat) && waitForPuckReply(fd, puck, kPropStat, 200)) {
            ++replies;
        }
    }

    close(fd);
    return replies;
}

inline std::vector<std::string> listSocketCanPorts() {
    std::vector<std::string> ports;
    DIR* dir = opendir("/sys/class/net");
    if (!dir) {
        return ports;
    }

    while (dirent* entry = readdir(dir)) {
        const std::string name = entry->d_name;
        if (startsWith(name, "can") || startsWith(name, "vcan") || startsWith(name, "slcan")) {
            ports.push_back(name);
        }
    }
    closedir(dir);
    std::sort(ports.begin(), ports.end());
    return ports;
}

inline std::string resolveBarrettSocketCanPort(const std::string& configured_port) {
    if (!isSocketCanPort(configured_port)) {
        return configured_port;
    }

    std::vector<std::string> candidates;
    if (!isAutoPort(configured_port)) {
        candidates.push_back(normalizeSocketCanPort(configured_port));
    }
    for (const std::string& port : listSocketCanPorts()) {
        if (std::find(candidates.begin(), candidates.end(), port) == candidates.end()) {
            candidates.push_back(port);
        }
    }

    std::string best_port;
    int best_replies = 0;
    for (const std::string& port : candidates) {
        const int replies = countBarrettPuckReplies(port);
        if (replies > best_replies) {
            best_replies = replies;
            best_port = port;
        }
        if (replies == 4) {
            break;
        }
    }

    if (best_replies == 0) {
        std::cerr << "No Barrett hand pucks responded on any SocketCAN interface." << std::endl;
        return "";
    }

    if (best_port != normalizeSocketCanPort(configured_port)) {
        std::cout << "Using Barrett hand on " << best_port << " (" << best_replies
                  << "/4 pucks replied; configured port was " << configured_port << ")." << std::endl;
    } else {
        std::cout << "Using Barrett hand on " << best_port << " (" << best_replies << "/4 pucks replied)."
                  << std::endl;
    }
    return best_port;
}
#else
inline std::string resolveBarrettSocketCanPort(const std::string& configured_port) {
    return configured_port;
}
#endif

} // namespace teleop_gripper_detail

class TeleopGripper {
  public:
    virtual ~TeleopGripper() = default;

    virtual bool initialize() = 0;
    virtual void setVelocity(double velocity, bool fingers_12_only = false) = 0;
    virtual void setSpread(double fraction) = 0;
    virtual double feedback() const = 0;
    virtual void shutdown() = 0;
};

class GeckoTeleopGripper : public TeleopGripper {
  public:
    bool initialize() override {
        return gripper_.initialize();
    }

    void setVelocity(double velocity, bool) override {
        gripper_.setVelocity(velocity);
        gripper_.controlLoopCallback();
    }

    void setSpread(double) override {
    }

    double feedback() const override {
        return gripper_.getLatestState().torque;
    }

    void shutdown() override {
        gripper_.setVelocity(0.0);
        gripper_.shutdown();
    }

  private:
    gripper::gecko::GeckoGripper gripper_;
};

class BarrettTeleopGripper : public TeleopGripper {
  public:
    explicit BarrettTeleopGripper(const GripperConfig& config)
        : port_(config.port)
        , force_init_(config.force_init)
        , finger_velocity_scale_(config.finger_velocity_scale)
        , spread_velocity_(config.spread_velocity) {
    }

    bool initialize() override {
        const std::string resolved_port = teleop_gripper_detail::resolveBarrettSocketCanPort(port_);
        if (resolved_port.empty()) {
            return false;
        }
        port_ = resolved_port;
        return hand_.initialize(port_, force_init_);
    }

    void setVelocity(double velocity, bool fingers_12_only) override {
        const double finger_velocity = velocity * finger_velocity_scale_;
        if (std::abs(finger_velocity) <= kStoppedVelocityEpsilon) {
            stopHand();
            return;
        }

        stopped_ = false;
        if (fingers_12_only) {
            const std::array<double, 4> velocities{{finger_velocity, finger_velocity, 0.0, 0.0}};
            hand_.setVelocity(velocities);
            return;
        }
        const std::array<double, 4> velocities{
            {finger_velocity, finger_velocity, finger_velocity, spread_velocity_}
        };
        hand_.setVelocity(velocities);
    }

    void setSpread(double fraction) override {
        stopHand();
        const double clamped_fraction = std::max(0.0, std::min(1.0, fraction));
        const double spread_position = gripper::barrett::RADIAN_SPREAD_MIN
                                       + clamped_fraction
                                             * (gripper::barrett::RADIAN_SPREAD_MAX
                                                - gripper::barrett::RADIAN_SPREAD_MIN);
        hand_.moveTo(gripper::barrett::MotorGroup::Spread, spread_position);
    }

    double feedback() const override {
        return 0.0;
    }

    void shutdown() override {
        stopHand();
        hand_.shutdown();
    }

  private:
    void stopHand() {
        if (stopped_) {
            return;
        }
        hand_.stopMotion();
        stopped_ = true;
    }

    static constexpr double kStoppedVelocityEpsilon = 1e-4;

    gripper::barrett::BarrettHand hand_;
    std::string port_;
    bool force_init_;
    double finger_velocity_scale_;
    double spread_velocity_;
    bool stopped_{true};
};

inline std::string lowercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

inline std::unique_ptr<TeleopGripper> createTeleopGripper(const GripperConfig& config) {
    const std::string type = lowercase(config.type);
    if (type == "gecko") {
        return std::unique_ptr<TeleopGripper>(new GeckoTeleopGripper());
    }
    if (type == "barrett" || type == "bhand" || type == "bh8" || type == "bh8-282") {
        return std::unique_ptr<TeleopGripper>(new BarrettTeleopGripper(config));
    }
    if (type == "none" || type == "disabled") {
        return nullptr;
    }
    throw std::runtime_error("Unsupported gripper type: " + config.type);
}
