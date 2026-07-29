/*
 * ex11_master_master.cpp  (wrist-enabled)
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 *      Author: Brian Zenowich
 *  Updated: 2025-11-07  (add haptic wrist support)
 */

// This a version of 7dof-7dof control

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

#include <haptic_wrist/handle.h>
#include "lib/leader.h"
#include "lib/background_state_publisher.h"
#include "lib/leader_dynamics_4dof.h"
#include "lib/dynamic_external_torque.h"
#include "lib/leader_vertical_dynamics.h"

using namespace barrett;
using detail::waitForEnter;


bool validate_args(int argc, char** argv) {
    const std::string config_dir = get_teleop_config_directory();
    if (config_dir.empty()) {
        throw std::runtime_error("No valid configuration directory found.");
    }

    try {
        TeleopConfig config = load_teleop_config(config_dir);
        print_leader_banner(config);
    } catch (...) {
        printf("ERROR: could not print leader config... exiting\n.");
        return false;
    }
    return true;
}

template <size_t DOF>
int wam_main(int argc, char **argv, ProductManager &pm, systems::Wam<DOF> &wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    const std::string config_dir = get_teleop_config_directory();
    if (config_dir.empty()) {
        throw std::runtime_error("No valid configuration directory found.");
    }

    const TeleopConfig config = load_teleop_config(config_dir);

    jp_type SYNC_POS; // the position each WAM should move to before linking
    if (DOF == 7) {
        for (int i = 0; i < 7; ++i) {
            SYNC_POS[i] = config.leader.sync_pos[i];
        }
    } else {
        printf("Error: 7 DOF supported\n");
        return false;
    }

    ros::init(argc, argv, "leader");

    haptic_wrist::Handle handle;

    // BackgroundStatePublisher<DOF> state_publisher(pm.getExecutionManager(), wam, &hw);

    barrett::systems::Summer<jt_type, 3> customjtSum;
    pm.getExecutionManager()->startManaging(customjtSum);

    LeaderDynamics<DOF> leaderDynamics(pm.getExecutionManager());
    DynamicExternalTorque<DOF> dynamicExternalTorque(pm.getExecutionManager());

    LeaderDynamics<DOF>* horizontalGravity = nullptr;
    LeaderVerticalDynamics<DOF>* leaderVerticalDynamics = nullptr;
    if (config.leader.vertical) {
        horizontalGravity = new LeaderDynamics<DOF>(pm.getExecutionManager());
        leaderVerticalDynamics = new LeaderVerticalDynamics<DOF>(pm.getExecutionManager());
    }

    // filters
    barrett::systems::FirstOrderFilter<jt_type> extFilter;
    jt_type omega_p(180.0);
    extFilter.setLowPass(omega_p);
    pm.getExecutionManager()->startManaging(extFilter);

    barrett::systems::FirstOrderFilter<jt_type> dynamicExtFilter;
    dynamicExtFilter.setLowPass(omega_p);
    pm.getExecutionManager()->startManaging(dynamicExtFilter);

    jv_type jv; jv.setConstant(0.0);
    systems::Constant<jv_type> zeroVelocity(jv);
    pm.getExecutionManager()->startManaging(zeroVelocity);

    ja_type ja; ja.setConstant(0.0);
    systems::Constant<ja_type> zeroAcceleration(ja);
    pm.getExecutionManager()->startManaging(zeroAcceleration);

    Leader<DOF> leader(pm.getExecutionManager(), &handle, config);

    jt_type maxRate; // Nm·s^-1 per joint
    maxRate << 50, 50, 50, 50;
    systems::RateLimiter<jt_type> wamJPOutputRamp(maxRate, "ffRamp");

    // systems::PrintToStream<jt_type> printTOQ(pm.getExecutionManager(), "TOQ: ");

    double h_omega_p = 25.0;
    barrett::systems::FirstOrderFilter<jv_type> hp1;
    hp1.setHighPass(jv_type(h_omega_p), jv_type(h_omega_p));
    systems::Gain<jv_type, double, ja_type> jaWAM(1.0);
    pm.getExecutionManager()->startManaging(hp1);

    barrett::systems::FirstOrderFilter<ja_type> jaFilter;
    ja_type l_omega_p = ja_type::Constant(50.0);
    jaFilter.setLowPass(l_omega_p);
    pm.getExecutionManager()->startManaging(jaFilter);

    // filtered acc for dynamics
    systems::connect(wam.jvOutput, hp1.input);
    systems::connect(hp1.output, jaWAM.input);
    systems::connect(jaWAM.output, jaFilter.input);
    systems::connect(jaFilter.output, leaderDynamics.jaInputDynamics);

    if (config.leader.vertical) {
        systems::connect(wam.jpOutput, horizontalGravity->jpInputDynamics);
        systems::connect(zeroVelocity.output, horizontalGravity->jvInputDynamics);
        systems::connect(zeroAcceleration.output, horizontalGravity->jaInputDynamics);

        systems::connect(leaderDynamics.dynamicsFeedFWD, leaderVerticalDynamics->leaderDynamicsIn);
        systems::connect(horizontalGravity->dynamicsFeedFWD, leaderVerticalDynamics->horizontalGravityIn);
        systems::connect(wam.gravity.output, leaderVerticalDynamics->gravityIn);
    }

    // leader info
    systems::connect(wam.jpOutput, leader.wamJPIn);
    systems::connect(wam.jvOutput, leader.wamJVIn);
    systems::connect(dynamicExternalTorque.wamExternalTorqueOut, leader.extTorqueIn);
    systems::connect(wam.gravity.output, leader.wamGravIn);
    systems::connect(wam.toolPose.output, leader.wamTPIn);
    if (config.leader.vertical) {
        systems::connect(leaderVerticalDynamics->leaderVerticalDynamicsOut, leader.wamDynIn);
    } else {
        systems::connect(leaderDynamics.dynamicsFeedFWD, leader.wamDynIn);
    }

    // the other current values needed for dynamics. zero vel and acc is just grav comp.
    systems::connect(wam.jpOutput, leaderDynamics.jpInputDynamics);
    systems::connect(wam.jvOutput, leaderDynamics.jvInputDynamics);
    // systems::connect(zeroVelocity.output, leaderDynamics.jvInputDynamics);
    // systems::connect(zeroAcceleration.output, leaderDynamics.jaInputDynamics);

    systems::connect(leader.wamJTOutput, customjtSum.getInput(0));
    systems::connect(wam.gravity.output, customjtSum.getInput(1));
    systems::connect(wam.supervisoryController.output, customjtSum.getInput(2)); // this will be 0 initially until we command the wam
    systems::connect(customjtSum.output, dynamicExternalTorque.wamTorqueSumIn);

    // pass dynamics for other systems
    if (config.leader.vertical) {
        systems::connect(leaderVerticalDynamics->leaderVerticalDynamicsOut, dynamicExternalTorque.wamDynamicsIn);
    } else {
        systems::connect(leaderDynamics.dynamicsFeedFWD, dynamicExternalTorque.wamDynamicsIn);
    }

    // filter torques
    systems::connect(dynamicExternalTorque.wamExternalTorqueOut, dynamicExtFilter.input);
    systems::connect(customjtSum.output, extFilter.input);

    wam.gravityCompensate();

    std::string line;
    v_type gainTmp;
    bool going = true;

    while (going) {
        printf(">>> ");
        std::getline(std::cin, line);
        if (line.empty()) { printf("\n"); continue; }

        switch (line[0]) {
        case 'l':
            if (leader.isLinked()) {
                leader.unlink();
            } else {
                // Sync both arm and wrist before link
                wam.moveTo(SYNC_POS, true);

                printf("Press [Enter] to link with the other WAM.");
                waitForEnter();
                leader.tryLink();
                wam.trackReferenceSignal(leader.theirJPOutput);
                // NOTE: avoid connecting multiple signals to wam.input because it causes free motion to be worse. even if the signal is 0
                connect(leader.wamJTOutput, wam.input);

                btsleep(0.1); // wait an execution cycle or two
                if (leader.isLinked()) {
                    printf("Linked with remote WAM.\n");
                } else {
                    printf("WARNING: Linking was unsuccessful.\n");
                }
            }
            break;

        case 't': {
            size_t jointNumber;
            std::cout << "\tJoint: ";
            std::cin >> jointNumber;
            size_t jointIndex = jointNumber - 1;

            if (jointIndex >= DOF) {
                std::cout << "\tBad joint number: " << jointNumber;
                break;
            }

            char gainId;
            std::cout << "\tGain identifier (p, i, or d): ";
            std::cin >> line;
            gainId = line[0];

            std::cout << "\tCurrent value: ";
            switch (gainId) {
            case 'p': gainTmp = wam.jpController.getKp(); break;
            case 'i': gainTmp = wam.jpController.getKi(); break;
            case 'd': gainTmp = wam.jpController.getKd(); break;
            default:  std::cout << "\tBad gain identifier.";
            }
            std::cout << gainTmp[jointIndex] << std::endl;

            std::cout << "\tNew value: ";
            std::cin >> gainTmp[jointIndex];
            switch (gainId) {
            case 'p': wam.jpController.setKp(gainTmp); break;
            case 'i': wam.jpController.setKi(gainTmp); break;
            case 'd': wam.jpController.setKd(gainTmp); break;
            default:  std::cout << "\tBad gain identifier.";
            }
            } break;

        case 'x':
            going = false;
            break;

        default:
            printf("\n");
            printf("    'l' to toggle linking with other WAM (and wrist)\n");
            printf("    't' to tune WAM JP control gains\n");
            printf("    'x' to exit\n");
            break;
        }
    }

    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

    return 0;
}
