/*
 * ex11_master_master.cpp  (wrist-enabled)
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 *      Author: Brian Zenowich
 *  Updated: 2025-11-07  (add haptic wrist support)
 */

// This a version of 4-DOF leader equiped with a haptic wrist.

#include "lib/external_torque.h"
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

// ==== CHANGED: use wrist-capable Leader and wrist headers ====
#include <haptic_wrist/haptic_wrist.h>
#include "lib/leader.h"                         // was "leader_nowrist.h"
#include "lib/background_state_publisher.h"
#include "lib/leader_dynamics.h"
#include "lib/dynamic_external_torque.h"
// #include "lib/leader_vertical_dynamics.h"

using namespace barrett;
using detail::waitForEnter;

void printUsage(const std::string& programName, const std::string& remoteHost, int recPort, int sendPort) {
    std::cout << "Usage: " << programName << " [remoteHost] [recPort] [sendPort]" << std::endl;
    std::cout << "       Defaults: remoteHost=" << remoteHost << ", recPort=" << recPort << ", sendPort=" << sendPort
              << std::endl;
    std::cout << "       -h or --help: Display this help message." << std::endl;
}

bool validate_args(int argc, char** argv) {
    if ((argc == 2 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) || (argc > 4)) {
        printUsage(argv[0], "127.0.0.1", 5555, 5554);
        return 0;
    }
    return true;
}

template <size_t DOF>
int wam_main(int argc, char **argv, ProductManager &pm, systems::Wam<DOF> &wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    jp_type SYNC_POS; // the position each WAM should move to before linking
    if (DOF == 4) {
        SYNC_POS[0] = 0.0;
        SYNC_POS[1] = -1.57;
        SYNC_POS[2] = 0.0;
        SYNC_POS[3] = 2.0;
    } else {
        printf("Error: 4 DOF supported\n");
        return false;
    }

    std::string remoteHost = "127.0.0.1";
    int rec_port = 5555;
    int send_port = 5554;

    if (argc >= 2) remoteHost = std::string(argv[1]);
    if (argc >= 3) rec_port   = std::atoi(argv[2]);
    if (argc >= 4) send_port  = std::atoi(argv[3]);

    // ==== CHANGED: node name to reflect wrist leader ====
    ros::init(argc, argv, "leader");

    // ==== NEW: start the haptic wrist device ====
    haptic_wrist::HapticWrist hw;
    hw.gravityCompensate(false);
    hw.run();

    // ==== CHANGED: pass &hw to state publisher so it can publish wrist states ====
    BackgroundStatePublisher<DOF> state_publisher(pm.getExecutionManager(), wam, &hw);

    barrett::systems::Summer<jt_type, 3> customjtSum;
    pm.getExecutionManager()->startManaging(customjtSum);

    LeaderDynamics<DOF> leaderDynamics(pm.getExecutionManager());
    // LeaderDynamics<DOF> horizontalGravity(pm.getExecutionManager());
    ExternalTorque<DOF> externalTorque(pm.getExecutionManager());
    DynamicExternalTorque<DOF> dynamicExternalTorque(pm.getExecutionManager());
    // LeaderVerticalDynamics<DOF> leaderVerticalDynamics(pm.getExecutionManager());

    // Filters (unchanged)
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

    // ==== CHANGED: instantiate wrist-capable Leader ====
    Leader<DOF> leader(pm.getExecutionManager(), &hw, remoteHost, rec_port, send_port);

    jt_type maxRate; // Nm·s^-1 per joint
    maxRate << 50, 50, 50, 50;
    systems::RateLimiter<jt_type> wamJPOutputRamp(maxRate, "ffRamp");

    systems::PrintToStream<jt_type> printdynamicextTorque(pm.getExecutionManager(), "dynamicextTorque: ");
    systems::PrintToStream<jt_type> printextTorque(pm.getExecutionManager(), "extTorque: ");
    systems::PrintToStream<jt_type> printdynamicoutput(pm.getExecutionManager(), "dynamicoutput: ");
    systems::PrintToStream<jt_type> printSC(pm.getExecutionManager(), "SC: ");

    double h_omega_p = 25.0;
    barrett::systems::FirstOrderFilter<jv_type> hp1;
    hp1.setHighPass(jv_type(h_omega_p), jv_type(h_omega_p));
    systems::Gain<jv_type, double, ja_type> jaWAM(1.0);
    pm.getExecutionManager()->startManaging(hp1);

    barrett::systems::FirstOrderFilter<ja_type> jaFilter;
    ja_type l_omega_p = ja_type::Constant(50.0);
    jaFilter.setLowPass(l_omega_p);
    pm.getExecutionManager()->startManaging(jaFilter);

    // === Wiring (same as your original "no-wrist" main) ===
    systems::connect(wam.jvOutput, hp1.input);
    systems::connect(hp1.output, jaWAM.input);
    systems::connect(jaWAM.output, jaFilter.input);
    systems::connect(jaFilter.output, leaderDynamics.jaInputDynamics);

    systems::connect(wam.jpOutput, leader.wamJPIn);
    systems::connect(wam.jvOutput, leader.wamJVIn);
    // systems::connect(dynamicExtFilter.output, leader.extTorqueIn);
    systems::connect(dynamicExternalTorque.wamExternalTorqueOut, leader.extTorqueIn);

    systems::connect(wam.jpOutput, leaderDynamics.jpInputDynamics);
    systems::connect(wam.jvOutput, leaderDynamics.jvInputDynamics);
    // systems::connect(zeroAcceleration.output, leaderDynamics.jaInputDynamics);

    // BEAR
    // systems::connect(leader.wamJPOutput, customjtSum.getInput(0));
    systems::connect(wam.gravity.output, customjtSum.getInput(1));
    systems::connect(wam.supervisoryController.output, customjtSum.getInput(2));

    // systems::connect(wam.gravity.output, externalTorque.wamGravityIn);
    // systems::connect(customjtSum.output, externalTorque.wamTorqueSumIn);
    // systems::connect(externalTorque.wamExternalTorqueOut, extFilter.input);

    systems::connect(customjtSum.output, dynamicExternalTorque.wamTorqueSumIn);
    systems::connect(leaderDynamics.dynamicsFeedFWD, dynamicExternalTorque.wamDynamicsIn);
    systems::connect(dynamicExternalTorque.wamExternalTorqueOut, dynamicExtFilter.input);

    systems::connect(wam.gravity.output, leader.wamGravIn);
    systems::connect(leaderDynamics.dynamicsFeedFWD, leader.wamDynIn);

    // Optional prints (leave commented to avoid loop jitter)
    // systems::connect(dynamicExternalTorque.wamExternalTorqueOut, printdynamicextTorque.input);
    // systems::connect(extFilter.output, printextTorque.input);
    // systems::connect(wam.supervisoryController.output, printSC.input);
    // systems::connect(leaderDynamics.dynamicsFeedFWD, printdynamicoutput.input);

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
                haptic_wrist::jp_type wrist_sync; 
                wrist_sync.setZero();
                hw.jointMoveTo(wrist_sync);

                printf("Press [Enter] to link with the other WAM.");
                waitForEnter();
                leader.tryLink();

                // Track peer’s arm joints (Leader publishes them)
                wam.trackReferenceSignal(leader.theirJPOutput);
                // connect(leader.wamJPOutput, wam.input); // BEAR

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

    // ==== NEW: stop the wrist thread cleanly ====
    hw.stop();

    return 0;
}
