#include "haptic_wrist/handle.h"
#include "gripper/gecko/gecko_gripper.h"
#include <iostream>
#include <thread>
#include <boost/optional.hpp>

using namespace gripper::gecko;

int main(int argc, char** argv) {
    haptic_wrist::Handle handle;

    GeckoGripper gripper;
    std::cout << "starting gripper and handle test" << std::endl;

    if (!gripper.initialize()) {
        std::cerr << "ERROR: Failed to initialize Gecko Gripper." << std::endl;
        return -1;
    }

    const float gripper_speed = 0.3f;

    double bumper = 0.0;
    double trigger = 0.0;
    double x_button = 0.0;
    float gripper_max_pos = 0.05;
    float gripper_min_pos = -0.17;

    float target_position = 0.0f;
    float target_velocity = 0.0f;

    while (true) {
        handle.poll();
        if (boost::optional<haptic_wrist::handle_type> opt_handle = handle.getHandle()) {
            haptic_wrist::handle_type handle = *opt_handle;

            bumper      = handle[0];
            trigger     = handle[1];
            x_button    = handle[2];
        }


        GripperState state = gripper.getLatestState();

        
        // still position controlled. just send to max and min gripper pos
        if (bumper && !trigger) {
            target_position = -1;
        } else if (trigger && !bumper) {
            target_position = 1;
        }


        gripper.setPosition(target_position);
        gripper.controlLoopCallback();

        std::cout << "\rGripper Pos: " << state.position 
                  << " | Target Vel: " << target_velocity 
                  << "        " << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    gripper.shutdown();
    
    return 0;
}
