/*
 * ex11_master_master.cpp
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 *      Author: Brian Zenowich
 */

// A version of 7-DOF follower.

#include <barrett/systems/pid_controller.h>
#include <barrett/systems/tool_torque_to_joint_torques.h>
#include <iostream>
#include <libconfig.h++>
#include <string>

#include <boost/thread.hpp>

#include <barrett/detail/stl_utils.h>
#include <barrett/os.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/units.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

#include "lib/follower.h"
#include "lib/background_state_publisher.h"
#include "lib/follower_dynamics_4dof.h"
#include "lib/dynamic_external_torque.h"
#include "lib/policy_torque.h"
#include "lib/follower_vertical_dynamics.h"
// #include "lib/trajectory_smoother.h"

using namespace barrett;
using detail::waitForEnter;

bool validate_args(int argc, char** argv) {
    const std::string config_dir = get_teleop_config_directory();
    if (config_dir.empty()) {
        throw std::runtime_error("No valid configuration directory found.");
    }

    try {
        TeleopConfig config = load_teleop_config(config_dir);
        print_follower_banner(config);
    } catch (...) {
        printf("ERROR: could not print follower config... exiting\n.");
        return false;
    }
    return true;
}

template <size_t DOF> int wam_main(int argc, char **argv, ProductManager &pm, systems::Wam<DOF> &wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    const std::string config_dir = get_teleop_config_directory();
    if (config_dir.empty()) {
        throw std::runtime_error("No valid configuration directory found.");
    }

    const TeleopConfig config = load_teleop_config(config_dir);

    jp_type SYNC_POS; // the position each WAM should move to before linking
    if (DOF == 7) {
        for (int i = 0; i < 7; ++i) {
            SYNC_POS[i] = config.follower.sync_pos[i];
        }

    } else {
        printf("Error: 7 DOF supported\n");
        return false;
    }

    GeckoGripper gripper;
    bool gripper_initialized = false;
    try {
        if (config.gripper.usable) {
            gripper_initialized = gripper.initialize();
        }
    } catch (const std::exception& e) {
        std::cerr << "WARNING: Gecko gripper init threw exception: " << e.what() << std::endl;
    }
    if (!gripper_initialized) {
        std::cerr << "WARNING: Gecko gripper not initialized. Trigger/bumper commands will be ignored." << std::endl;
    }


    ros::init(argc, argv, "follower");
    // BackgroundStatePublisher<DOF> state_publisher(pm.getExecutionManager(), wam);

    barrett::systems::Summer<jt_type, 3> customjtSum;
    pm.getExecutionManager()->startManaging(customjtSum);

    barrett::systems::PIDController<jp_type, jt_type> base_policy_controller(get_barrett_gains<DOF>(config.policy.base));
    barrett::systems::PIDController<jp_type, jt_type> res_policy_controller(get_barrett_gains<DOF>(config.policy.res));
    barrett::systems::PIDController<jt_type, jt_type> torque_policy_controller(get_barrett_gains<DOF>(config.policy.torque));

    FollowerDynamics<DOF> followerDynamics(pm.getExecutionManager());
    DynamicExternalTorque<DOF> dynamicExternalTorque(pm.getExecutionManager());
    PolicyTorque<DOF> policyTorque(pm.getExecutionManager());

    FollowerDynamics<DOF>* horizontalGravity = nullptr;
    FollowerVerticalDynamics<DOF>* followerVerticalDynamics = nullptr;
    if (config.leader.vertical) {
        horizontalGravity = new FollowerDynamics<DOF>(pm.getExecutionManager());
        followerVerticalDynamics = new FollowerVerticalDynamics<DOF>(pm.getExecutionManager());
    }
    
    barrett::systems::FirstOrderFilter<jt_type> extFilter;
    jt_type omega_p(80.0);
    extFilter.setLowPass(omega_p);
    pm.getExecutionManager()->startManaging(extFilter);

    jp_type jp;
    jp.setConstant(0.0);
    systems::Constant<jp_type> zeroPosition(jp);
    pm.getExecutionManager()->startManaging(zeroPosition);

    jv_type jv;
    jv.setConstant(0.0);
    systems::Constant<jv_type> zeroVelocity(jv);
    pm.getExecutionManager()->startManaging(zeroVelocity);

    ja_type ja;
    ja.setConstant(0.0);
    systems::Constant<ja_type> zeroAcceleration(ja);
    pm.getExecutionManager()->startManaging(zeroAcceleration);

    Follower<DOF> follower(pm.getExecutionManager(), &gripper, config);

    // systems::PrintToStream<jt_type> printTOQ(pm.getExecutionManager(), "TOQ: ");

    // filters
    double h_omega_p = 25.0;
    barrett::systems::FirstOrderFilter<jv_type> hp1;
    hp1.setHighPass(jv_type(h_omega_p), jv_type(h_omega_p));
    systems::Gain<jv_type, double, ja_type> jaWAM(1.0);
    pm.getExecutionManager()->startManaging(hp1);

    barrett::systems::FirstOrderFilter<ja_type> jaFilter;
    ja_type l_omega_p = ja_type::Constant(50.0);
    jaFilter.setLowPass(l_omega_p);
    pm.getExecutionManager()->startManaging(jaFilter);

    // values needed for dynamics. zero vel and acc is just grav comp.
    systems::connect(wam.jpOutput, followerDynamics.jpInputDynamics);
    systems::connect(wam.jvOutput, followerDynamics.jvInputDynamics);
    // filtered acc for dynamics
    // systems::connect(wam.jvOutput, hp1.input);
    // systems::connect(hp1.output, jaWAM.input);
    // systems::connect(jaWAM.output, jaFilter.input);
    // systems::connect(jaFilter.output, followerDynamics.jaInputDynamics);
    systems::connect(zeroAcceleration.output, followerDynamics.jaInputDynamics);

    if (config.follower.vertical) {
        systems::connect(wam.jpOutput, horizontalGravity->jpInputDynamics);
        systems::connect(zeroVelocity.output, horizontalGravity->jvInputDynamics);
        systems::connect(zeroAcceleration.output, horizontalGravity->jaInputDynamics);

        systems::connect(followerDynamics.dynamicsFeedFWD, followerVerticalDynamics->followerDynamicsIn);
        systems::connect(horizontalGravity->dynamicsFeedFWD, followerVerticalDynamics->horizontalGravityIn);
        systems::connect(wam.gravity.output, followerVerticalDynamics->gravityIn);
    }

    // follower info
    systems::connect(wam.jpOutput, follower.wamJPIn);
    systems::connect(wam.jvOutput, follower.wamJVIn);
    systems::connect(dynamicExternalTorque.wamExternalTorqueOut, follower.dyngravcompTorqueIn);
    systems::connect(wam.gravity.output, follower.wamGravIn);
    systems::connect(wam.toolPose.output, follower.wamTPIn);
    systems::connect(base_policy_controller.controlOutput, follower.basePolicyJtIn);
    systems::connect(res_policy_controller.controlOutput, follower.resPolicyJtIn);
    systems::connect(torque_policy_controller.controlOutput, follower.refTorquePolicyJtIn);
    systems::connect(policyTorque.extTorqueOutput, follower.environmentTorqueIn);
    systems::connect(extFilter.output, follower.filteredEnvironmentTorqueIn);
    if (config.follower.vertical) {
        systems::connect(followerVerticalDynamics->followerVerticalDynamicsOut, follower.wamDynIn);
    } else {
        systems::connect(followerDynamics.dynamicsFeedFWD, follower.wamDynIn);
    }

    // if using dyn_comp-grav_comp as feedforward then customjtSum will find the the non dynamically compensated ext torque
    systems::connect(follower.wamJTOutput, customjtSum.getInput(0));
    systems::connect(wam.gravity.output, customjtSum.getInput(1));
    systems::connect(wam.supervisoryController.output, customjtSum.getInput(2));
    systems::connect(customjtSum.output, dynamicExternalTorque.wamTorqueSumIn);

    // pass dynamics for other systems
    if (config.follower.vertical) {
        systems::connect(followerVerticalDynamics->followerVerticalDynamicsOut, dynamicExternalTorque.wamDynamicsIn);
    } else {
        systems::connect(followerDynamics.dynamicsFeedFWD, dynamicExternalTorque.wamDynamicsIn);
    }

    // find and rate limit the scale
    systems::connect(dynamicExternalTorque.wamExternalTorqueOut, policyTorque.wamExtTorqueIn);
    systems::connect(base_policy_controller.controlOutput, policyTorque.policyExtTorqueIn);

    // filter torques
    systems::connect(policyTorque.extTorqueOutput, extFilter.input);

    // policy impedance control
    systems::connect(follower.basePolicyJpOutput, base_policy_controller.referenceInput);
    systems::connect(wam.jpOutput, base_policy_controller.feedbackInput);
    systems::connect(follower.resPolicyJpOutput, res_policy_controller.referenceInput);
    systems::connect(zeroPosition.output, res_policy_controller.feedbackInput);
    systems::connect(follower.refPolicyJtOutput, torque_policy_controller.referenceInput);
    systems::connect(follower.filteredHumanTorqueOutput, torque_policy_controller.feedbackInput);

    // systems::connect(customjtSum.output, printTOQ.input);

    wam.gravityCompensate();

    std::string line;
    v_type gainTmp;

    bool going = true;

    while (going) {
        printf(">>> ");
        std::getline(std::cin, line);

        switch (line[0]) {
        case 'l':
            if (follower.isLinked()) {
                follower.unlink();
                printf("unlinked");
            } else {
                wam.moveTo(SYNC_POS, true);

                printf("Press [Enter] to link with the other WAM.");
                waitForEnter();
                follower.tryLink();
                wam.trackReferenceSignal(follower.theirJPOutput);
                systems::connect(follower.wamJTOutput, wam.input); // CAREFUL WITH THIS. CAN BE IN BOTH LINK AND IN INFERENCE

                btsleep(0.1); // wait an execution cycle or two
                if (follower.isLinked()) {
                    printf("Linked with remote WAM.\n");
                } else {
                    printf("WARNING: Linking was unsuccessful.\n");
                }
            }

            break;

        case 't':
            size_t jointIndex;
            {
                size_t jointNumber;
                std::cout << "\tJoint: ";
                std::cin >> jointNumber;
                jointIndex = jointNumber - 1;

                if (jointIndex >= DOF) {
                    std::cout << "\tBad joint number: " << jointNumber;
                    break;
                }
            }

            char gainId;
            std::cout << "\tGain identifier (p, i, or d): ";
            std::cin >> line;
            gainId = line[0];

            std::cout << "\tCurrent value: ";
            switch (gainId) {
            case 'p':
                gainTmp = wam.jpController.getKp();
                break;
            case 'i':
                gainTmp = wam.jpController.getKi();
                break;
            case 'd':
                gainTmp = wam.jpController.getKd();
                break;

            default:
                std::cout << "\tBad gain identifier.";
            }
            std::cout << gainTmp[jointIndex] << std::endl;

            std::cout << "\tNew value: ";
            std::cin >> gainTmp[jointIndex];
            switch (gainId) {
            case 'p':
                wam.jpController.setKp(gainTmp);
                break;
            case 'i':
                wam.jpController.setKi(gainTmp);
                break;
            case 'd':
                wam.jpController.setKd(gainTmp);
                break;

            default:
                std::cout << "\tBad gain identifier.";
            }

            break;
        case 'x':
            going = false;
            break;

        default:
            printf("\n");
            printf("    'l'  start/stop teleop linking. press enter for both robots after they are in linking position\n");
            printf("    'p'  start/stop policy on follower\n");
            printf("    't'  tune control gains\n");
            printf("    'x'  exit\n");
            printf("\n");

            break;
        }
    }

    gripper.shutdown();

    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

    return 0;
}
