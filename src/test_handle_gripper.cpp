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

    const float gripper_speed = 0.3f;

    double bumper = 0.0;
    double trigger = 0.0;
    float gripper_max_pos = 0.05;
    float gripper_min_pos = -0.17;

    while (true) {
        if (boost::optional<haptic_wrist::handle_type> opt_handle = hw.getHandle()) {
            haptic_wrist::handle_type handle = *opt_handle;

            bumper      = handle[0];
            trigger     = handle[1];
        }


        GripperState state = gripper.getLatestState();
        float target_velocity = 0.0f;

        if (bumper && !trigger) {
            if (state.position < gripper_max_pos) {
                target_velocity = -gripper_speed;
            }
        } else if (trigger && !bumper) {
            if (state.position > gripper_min_pos) {
                target_velocity = gripper_speed;
            }
        }

        gripper.setVelocity(target_velocity);
        gripper.controlLoopCallback();

        std::cout << "\rGripper Pos: " << state.position 
                  << " | Target Vel: " << target_velocity 
                  << "        " << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    gripper.shutdown();
    hw.stop();
    
    return 0;
}
