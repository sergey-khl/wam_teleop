#include "haptic_wrist/haptic_wrist.h"
#include "gripper/gecko/gecko_gripper.h"
#include <iostream>
#include <thread>
#include <boost/optional.hpp>

using namespace gripper::gecko;

int main(int argc, char** argv) {
    haptic_wrist::HapticWrist hw;
    hw.gravityCompensate(false);
    hw.run();

    GeckoGripper gripper;
    std::cout << "starting gripper and handle test" << std::endl;

    if (!gripper.initialize()) {
        std::cerr << "ERROR: Failed to initialize Gecko Gripper." << std::endl;
        return -1;
    }

    const double trigger_rest_pos = 0.25;
    float target_velocity = 0.3;
    const float torque_scaling = 1.5;
    const float minStiffness = 0.1;  // Base spring force for moving through empty air
    const float maxStiffness = 1.0;  // Max pushback when gripper is stalled/crushing
    float trigger = 0.0;
    bool bumper_pressed = false;

    // ema to smooth torque
    const float alpha = 0.15f; 
    float smoothed_torque = 0.0f;


    float trigger_pos = 0.0f;
    float trigger_vel = 0.0f;
    float trigger_torque = 0.0f;

    float gripper_max_pos = 0.05;
    float gripper_min_pos = -0.17;

    while (true) {
        if (boost::optional<haptic_wrist::handle_type> opt_handle = hw.getHandle()) {
            haptic_wrist::handle_type handle = *opt_handle;
            trigger_pos = static_cast<float>(handle[0]); // see haptic_wrist_impl.cpp for trigger range, TOOD: add to config
            trigger_vel = static_cast<float>(handle[1]);
            trigger_torque = static_cast<float>(handle[2]);
        }

        double new_gripper_pos = gripper_max_pos;
        float local_gripper_pos = trigger_pos / 0.88 - 1;
        local_gripper_pos = local_gripper_pos * 0.17;

        new_gripper_pos = std::max(gripper_min_pos, std::min(local_gripper_pos, gripper_max_pos));

        std::cout << "new gripper pos " << new_gripper_pos << std::endl;
        std::cout << "target gripper pos " << local_gripper_pos << std::endl;
        gripper.setPosition(new_gripper_pos);
        gripper.controlLoopCallback();

        GripperState state = gripper.getLatestState();

        smoothed_torque = (alpha * state.torque) + ((1.0f - alpha) * smoothed_torque);
        
        std::cout << "\rPos: " << state.position << " | Trq: " << smoothed_torque << "    " << std::endl;
        if (smoothed_torque > minStiffness) {
            float dynamicStiffness = smoothed_torque * torque_scaling * (maxStiffness - minStiffness) + minStiffness;
            float raw_haptics = 255.0f * dynamicStiffness;
            if (raw_haptics > 255.0f) raw_haptics = 255.0f;

            uint8_t haptics = static_cast<uint8_t>(raw_haptics);

            std::cout << "haptics " << static_cast<int>(haptics) << " stiffness " << dynamicStiffness << " torque " << smoothed_torque << std::endl;

            hw.setTriggerHaptics(haptics);
        } else {
            hw.setTriggerHaptics(0); 
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    gripper.shutdown();
    hw.stop();
    
    return 0;
}
