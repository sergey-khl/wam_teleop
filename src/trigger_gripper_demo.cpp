#include "haptic_wrist/haptic_wrist.h"
#include "lib/teleop_config_loader.h"
#include "lib/teleop_gripper.h"
#include "lib/utils.h"

#include <algorithm>
#include <atomic>
#include <boost/optional.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

namespace {

std::atomic<bool> running{true};

void signalHandler(int) {
    running.store(false);
}

double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

double moveToward(double current, double target, double max_delta) {
    if (target > current) {
        return std::min(target, current + max_delta);
    }
    return std::max(target, current - max_delta);
}

} // namespace

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const std::string config_dir = get_teleop_config_directory();
    if (config_dir.empty()) {
        return 1;
    }

    const TeleopConfig config = load_teleop_config(config_dir);
    std::unique_ptr<TeleopGripper> gripper = createTeleopGripper(config.gripper);
    if (!gripper) {
        std::cerr << "Gripper control is disabled in teleop_config.yaml." << std::endl;
        return 1;
    }

    if (!gripper->initialize()) {
        std::cerr << "Failed to initialize " << config.gripper.type << " gripper." << std::endl;
        return 1;
    }

    haptic_wrist::HapticWrist wrist;
    wrist.gravityCompensate(false);
    wrist.run();

    std::cout << "Trigger/gripper demo running. Press Ctrl-C to stop." << std::endl;
    std::cout << "Gripper type: " << config.gripper.type << ", port: " << config.gripper.port << std::endl;
    std::cout << "Double-click bumper/R1 to toggle spread between 0% and 50%." << std::endl;

    auto last_print = std::chrono::steady_clock::now();
    auto last_update = std::chrono::steady_clock::now();
    auto last_bumper_click = std::chrono::steady_clock::time_point::min();
    const auto double_click_window = std::chrono::milliseconds(config.gripper.bumper_double_click_ms);
    bool previous_bumper_pressed = false;
    bool ignore_bumper_until_release = false;
    bool spread_open = false;
    double filtered_velocity = 0.0;
    double commanded_velocity = 0.0;

    while (running.load()) {
        double trigger = 0.0;
        bool bumper_pressed = false;

        if (boost::optional<haptic_wrist::handle_type> opt_handle = wrist.getHandle()) {
            const haptic_wrist::handle_type handle = *opt_handle;
            trigger = clamp(handle[3], 0.0, 1.0);
            bumper_pressed = handle[2] > 0.5;
        }

        const auto now = std::chrono::steady_clock::now();
        if (bumper_pressed && !previous_bumper_pressed) {
            if (last_bumper_click != std::chrono::steady_clock::time_point::min()
                && now - last_bumper_click <= double_click_window) {
                spread_open = !spread_open;
                gripper->setVelocity(0.0);
                gripper->setSpread(spread_open ? 0.5 : 0.0);
                filtered_velocity = 0.0;
                commanded_velocity = 0.0;
                ignore_bumper_until_release = true;
                last_bumper_click = std::chrono::steady_clock::time_point::min();
                std::cout << "spread=" << (spread_open ? "50%" : "0%") << std::endl;
            } else {
                last_bumper_click = now;
            }
        }
        if (!bumper_pressed) {
            ignore_bumper_until_release = false;
        }
        previous_bumper_pressed = bumper_pressed;

        double velocity = 0.0;
        if (trigger > config.leader.haptics.trigger_rest_pos) {
            velocity = config.leader.haptics.target_velocity * trigger;
        } else if (bumper_pressed && !ignore_bumper_until_release) {
            velocity = -config.leader.haptics.target_velocity;
        }
        if (std::abs(velocity) < config.gripper.velocity_deadband) {
            velocity = 0.0;
        }

        const double alpha = clamp(config.gripper.velocity_filter_alpha, 0.0, 1.0);
        filtered_velocity = alpha * velocity + (1.0 - alpha) * filtered_velocity;

        const double dt = std::chrono::duration<double>(now - last_update).count();
        last_update = now;
        const double max_delta = std::max(0.0, config.gripper.velocity_slew_rate) * dt;
        commanded_velocity = moveToward(commanded_velocity, filtered_velocity, max_delta);

        if (std::abs(commanded_velocity) < config.gripper.velocity_deadband) {
            commanded_velocity = 0.0;
        }

        gripper->setVelocity(commanded_velocity, spread_open);

        if (now - last_print >= std::chrono::milliseconds(250)) {
            std::cout << std::fixed << std::setprecision(3)
                      << "trigger=" << trigger
                      << " bumper=" << (bumper_pressed ? 1 : 0)
                      << " velocity=" << commanded_velocity
                      << " spread=" << (spread_open ? 0.5 : 0.0)
                      << " fingers=" << (spread_open ? "1+2" : "1+2+3")
                      << " feedback=" << gripper->feedback()
                      << std::endl;
            last_print = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    gripper->setVelocity(0.0);
    gripper->shutdown();
    wrist.setTriggerHaptics(0);
    wrist.stop();
    return 0;
}
