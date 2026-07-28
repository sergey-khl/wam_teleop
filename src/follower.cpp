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
#include "lib/leader_dynamics_4dof.h"
// #include "lib/follower_dynamics_7dof.h"
#include "lib/dynamic_external_torque.h"
#include "lib/policy_external_torque.h"
#include "lib/follower_vertical_dynamics.h"
#include "lib/trajectory_smoother.h"

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

// TODO: this is gross. fix later
template <size_t DOF>
barrett::math::Matrix<DOF, 1> toBarrettVec(const std::vector<double>& v) {
    if (v.size() != DOF) {
        throw std::runtime_error("Config vector size (" + std::to_string(v.size()) +
                                  ") does not match DOF (" + std::to_string(DOF) + ")");
    }
    barrett::math::Matrix<DOF, 1> out;
    for (size_t i = 0; i < DOF; ++i) {
        out[i] = v[i];
    }
    return out;
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
        gripper_initialized = gripper.initialize();
    } catch (const std::exception& e) {
        std::cerr << "WARNING: Gecko gripper init threw exception: " << e.what() << std::endl;
    }
    if (!gripper_initialized) {
        std::cerr << "WARNING: Gecko gripper not initialized. Trigger/bumper commands will be ignored." << std::endl;
    }


    ros::init(argc, argv, "follower");
    BackgroundStatePublisher<DOF> state_publisher(pm.getExecutionManager(), wam);

    barrett::systems::Summer<jt_type, 3> customjtSum;
    pm.getExecutionManager()->startManaging(customjtSum);

    // barrett::systems::Summer<jt_type, 2> policyJtSum;
    // pm.getExecutionManager()->startManaging(policyJtSum);

    TrajectorySmoother<DOF> policyFilter(2.0, 2.0, 0.0);
    pm.getExecutionManager()->startManaging(policyFilter);

    barrett::systems::PIDController<jp_type, jt_type> policy_controller;
    policy_controller.setKp(toBarrettVec<DOF>(config.policy.kp));
    policy_controller.setKi(toBarrettVec<DOF>(config.policy.ki));
    policy_controller.setKd(toBarrettVec<DOF>(config.policy.kd));
    policy_controller.setIntegratorLimit(toBarrettVec<DOF>(config.policy.integrator_limit));
    policy_controller.setControlSignalLimit(toBarrettVec<DOF>(config.policy.control_signal_limit));

    FollowerDynamics<DOF> followerDynamics(pm.getExecutionManager());
    DynamicExternalTorque<DOF> dynamicExternalTorque(pm.getExecutionManager());
    PolicyExternalTorque<DOF> policyExternalTorque(pm.getExecutionManager());

    FollowerDynamics<DOF>* horizontalGravity = nullptr;
    FollowerVerticalDynamics<DOF>* followerVerticalDynamics = nullptr;
    if (config.leader.vertical) {
        horizontalGravity = new FollowerDynamics<DOF>(pm.getExecutionManager());
        followerVerticalDynamics = new FollowerVerticalDynamics<DOF>(pm.getExecutionManager());
    }
    
    barrett::systems::FirstOrderFilter<jt_type> extFilter;
    jt_type omega_p(180.0);
    extFilter.setLowPass(omega_p);
    pm.getExecutionManager()->startManaging(extFilter);

    jv_type jv;
    jv.setConstant(0.0);
    systems::Constant<jv_type> zeroVelocity(jv);
    pm.getExecutionManager()->startManaging(zeroVelocity);

    ja_type ja;
    ja.setConstant(0.0);
    systems::Constant<ja_type> zeroAcceleration(ja);
    pm.getExecutionManager()->startManaging(zeroAcceleration);

    Follower<DOF> follower(pm.getExecutionManager(), &gripper, config);

    jt_type maxRate; // Nm · s-1 per joint
    maxRate << 50, 50, 50, 50;
    systems::RateLimiter<jt_type> wamJPOutputRamp(maxRate, "ffRamp");

    systems::PrintToStream<jt_type> printdynamicextTorque(pm.getExecutionManager(), "dynamicextTorque: ");
    systems::PrintToStream<jt_type> printSC(pm.getExecutionManager(), "SC: ");
    systems::PrintToStream<jp_type> printPOS(pm.getExecutionManager(), "POS: ");
    systems::PrintToStream<jt_type> printFOR(pm.getExecutionManager(), "FOR: ");
    systems::PrintToStream<jt_type> printTOQ(pm.getExecutionManager(), "TOQ: ");
    systems::PrintToStream<ja_type> printACC(pm.getExecutionManager(), "ACC: ");

    double h_omega_p = 25.0;
    barrett::systems::FirstOrderFilter<jv_type> hp1;
    hp1.setHighPass(jv_type(h_omega_p), jv_type(h_omega_p));
    systems::Gain<jv_type, double, ja_type> jaWAM(1.0);
    pm.getExecutionManager()->startManaging(hp1);

    barrett::systems::FirstOrderFilter<ja_type> jaFilter;
    ja_type l_omega_p = ja_type::Constant(50.0);
    jaFilter.setLowPass(l_omega_p);
    pm.getExecutionManager()->startManaging(jaFilter);

    // convert VLA actions to joint torques
    systems::ToolForceToJointTorques<DOF> tf2jt;
    systems::ToolTorqueToJointTorques<DOF> tt2jt;

    systems::connect(wam.jvOutput, hp1.input);
    systems::connect(hp1.output, jaWAM.input);
    systems::connect(jaWAM.output, jaFilter.input);
    systems::connect(jaFilter.output, followerDynamics.jaInputDynamics);

    if (config.follower.vertical) {
        systems::connect(wam.jpOutput, horizontalGravity->jpInputDynamics);
        systems::connect(zeroVelocity.output, horizontalGravity->jvInputDynamics);
        systems::connect(zeroAcceleration.output, horizontalGravity->jaInputDynamics);

        systems::connect(followerDynamics.dynamicsFeedFWD, followerVerticalDynamics->followerDynamicsIn);
        systems::connect(horizontalGravity->dynamicsFeedFWD, followerVerticalDynamics->horizontalGravityIn);
        systems::connect(wam.gravity.output, followerVerticalDynamics->gravityIn);
    }

    systems::connect(wam.jpOutput, follower.wamJPIn);
    systems::connect(wam.jvOutput, follower.wamJVIn);
    systems::connect(extFilter.output, follower.extTorqueIn);
    // systems::connect(customjtSum.output, follower.extTorqueIn);
    // systems::connect(dynamicExternalTorque.wamExternalTorqueOut, follower.extTorqueIn);

    systems::connect(wam.jpOutput, followerDynamics.jpInputDynamics);
    systems::connect(wam.jvOutput, followerDynamics.jvInputDynamics);
    // systems::connect(zeroVelocity.output, followerDynamics.jvInputDynamics);
    // systems::connect(zeroAcceleration.output, followerDynamics.jaInputDynamics);

    systems::connect(follower.wamJTOutput, customjtSum.getInput(0));
    systems::connect(wam.gravity.output, customjtSum.getInput(1));
    systems::connect(wam.supervisoryController.output, customjtSum.getInput(2));
    // systems::connect(policy_controller.controlOutput, customjtSum.getInput(3));

    systems::connect(customjtSum.output, dynamicExternalTorque.wamTorqueSumIn);
    if (config.follower.vertical) {
        systems::connect(followerVerticalDynamics->followerVerticalDynamicsOut, dynamicExternalTorque.wamDynamicsIn);
    } else {
        systems::connect(followerDynamics.dynamicsFeedFWD, dynamicExternalTorque.wamDynamicsIn);
    }

    systems::connect(wam.gravity.output, follower.wamGravIn);
    if (config.follower.vertical) {
        systems::connect(followerVerticalDynamics->followerVerticalDynamicsOut, follower.wamDynIn);
    } else {
        systems::connect(followerDynamics.dynamicsFeedFWD, follower.wamDynIn);
    }

    systems::connect(dynamicExternalTorque.wamExternalTorqueOut, extFilter.input);
    // systems::connect(customjtSum.output, extFilter.input);

    systems::connect(wam.toolPose.output, follower.wamTPIn);

    // systems::connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
    // systems::connect(wam.kinematicsBase.kinOutput, tt2jt.kinInput);
    // systems::connect(follower.policyToolForceOutput, tf2jt.input);
    // systems::connect(follower.policyToolTorqueOutput, tt2jt.input);
    //
    // systems::connect(tf2jt.output, policyJtSum.getInput(0));
    // systems::connect(tt2jt.output, policyJtSum.getInput(1));
    // systems::connect(policyJtSum.output, follower.policyJtIn);


    systems::connect(follower.policyJpOutput, policyFilter.input);

    systems::connect(follower.policyJpOutput, policy_controller.referenceInput);
    // systems::connect(policyFilter.output, policy_controller.referenceInput);
    // systems::connect(follower.wamJPOutput, policy_controller.feedbackInput);
    systems::connect(follower.theirJPOutput, policy_controller.feedbackInput);

    // systems::connect(dynamicExternalTorque.wamExternalTorqueOut, policyExternalTorque.wamCompTorqIn);
    // systems::connect(follower.theirExtTorqueOutput, policyExternalTorque.wamCompTorqIn);
    // systems::connect(follower.policyJaOutput, policyExternalTorque.policyJaIn);
    // systems::connect(policy_controller.controlOutput, policyExternalTorque.policyTorqueIn);
    // systems::connect(follower.policyJtScaleOutput, policyExternalTorque.policyTorqueScaleIn);

    systems::connect(policy_controller.controlOutput, follower.policyJtIn);
    // systems::connect(policyExternalTorque.output, follower.policyJtIn);
    // systems::connect(policy_controller.controlOutput, follower.policyJtIn);

    // systems::connect(extFilter.output, printdynamicextTorque.input);
    // systems::connect(dynamicExternalTorque.wamExternalTorqueOut, printdynamicextTorque.input);
    // systems::connect(wam.supervisoryController.output, printSC.input);
    // systems::connect(customjtSum.output, printTOQ.input);
    // systems::connect(followerDynamics.dynamicsFeedFWD, printTOQ.input);
    // systems::connect(policyFilter.output, printPOS.input);
    // systems::connect(policy_controller.controlOutput, printTOQ.input);
    // systems::connect(policyExternalTorque.output, printTOQ.input);
    // systems::connect(follower.policyJtScaleOutput, printTOQ.input);
    // systems::connect(follower.policyJaOutput, printACC.input);

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
            } else if (follower.isInference()) {
                follower.disableInference();
                printf("disabled inference");
            } else {
                wam.moveTo(SYNC_POS, true);

                printf("Press [Enter] to link with the other WAM.");
                waitForEnter();
                follower.tryLink();

                btsleep(0.1); // wait an execution cycle or two
                if (follower.isLinked()) {
                    wam.trackReferenceSignal(follower.theirJPOutput);
                    // wam.trackReferenceSignal(wam.jpOutput);
                    systems::connect(follower.wamJTOutput, wam.input); // CAREFUL WITH THIS. CAN BE IN BOTH LINK AND IN INFERENCE
                    printf("Linked with remote WAM.\n");
                } else {
                    printf("WARNING: Linking was unsuccessful.\n");
                }
            }

            break;

        case 'p':
            if (follower.isInference()) {
                follower.disableInference();
                printf("disabled inference");
            } else {
                follower.enableInference();

                btsleep(0.1); // wait an execution cycle or two
                if (follower.isInference()) {
                    // wam.trackReferenceSignal(policyFilter.output);
                    // systems::connect(policyExternalTorque.output, wam.input);
                    printf("Running policy.\n");
                } else {
                    printf("WARNING: inference was unsuccessful.\n");
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
