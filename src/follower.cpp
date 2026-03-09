/*
 * ex11_master_master.cpp
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 *      Author: Brian Zenowich
 */

// A version of 7-DOF follower.

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

#include "lib/follower.h"
#include "lib/background_state_publisher.h"
#include "lib/follower_dynamics.h"
#include "lib/dynamic_external_torque.h"

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
        printUsage(argv[0], "127.0.0.1", 5554, 5555);
        return 0;
    }

    return true;
}

template <size_t DOF> int wam_main(int argc, char **argv, ProductManager &pm, systems::Wam<DOF> &wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    jp_type SYNC_POS; // the position each WAM should move to before linking
    if (DOF == 7) {
        SYNC_POS[0] = 0.0;
        SYNC_POS[1] = -1.70;
        SYNC_POS[2] = 0.0;
        SYNC_POS[3] = 2.97;
        SYNC_POS[4] = 0.0;
        SYNC_POS[5] = 0.0;
        SYNC_POS[6] = 0.0;

    } else {
        printf("Error: 7 DOF supported\n");
        return false;
    }

    std::string remoteHost = "127.0.0.1";
    int rec_port = 5554;
    int send_port = 5555;

    if (argc >= 2) {
        remoteHost = std::string(argv[1]);
    }
    if (argc >= 3) {
        rec_port = std::atoi(argv[2]);
    }
    if (argc >= 4) {
        send_port = std::atoi(argv[3]);
    }

    ros::init(argc, argv, "follower");
    BackgroundStatePublisher<DOF> state_publisher(pm.getExecutionManager(), wam);

    barrett::systems::Summer<jt_type, 3> customjtSum;
    pm.getExecutionManager()->startManaging(customjtSum);

    FollowerDynamics<DOF> followerDynamics(pm.getExecutionManager());

    ExternalTorque<DOF> externalTorque(pm.getExecutionManager());

    DynamicExternalTorque<DOF> dynamicExternalTorque(pm.getExecutionManager());
    
    barrett::systems::FirstOrderFilter<jt_type> extFilter;
    jt_type omega_p(180.0);
    extFilter.setLowPass(omega_p);
    pm.getExecutionManager()->startManaging(extFilter);

    ja_type ja;
    ja.setConstant(0.0);
    systems::Constant<ja_type> zeroAcceleration(ja);
    pm.getExecutionManager()->startManaging(zeroAcceleration);

    Follower<DOF> follower(pm.getExecutionManager(), remoteHost, rec_port, send_port);

    jt_type maxRate; // Nm · s-1 per joint
    maxRate << 50, 50, 50, 50;
    systems::RateLimiter<jt_type> wamJPOutputRamp(maxRate, "ffRamp");

    systems::PrintToStream<jt_type> printdynamicextTorque(pm.getExecutionManager(), "dynamicextTorque: ");
    systems::PrintToStream<jt_type> printSC(pm.getExecutionManager(), "SC: ");

    // systems::PrintToStream<jt_type> printcustomjtSum(pm.getExecutionManager(), "customjtSum: ");

    double h_omega_p = 25.0;
    barrett::systems::FirstOrderFilter<jv_type> hp1;
    hp1.setHighPass(jv_type(h_omega_p), jv_type(h_omega_p));
    systems::Gain<jv_type, double, ja_type> jaWAM(1.0);
    pm.getExecutionManager()->startManaging(hp1);

    barrett::systems::FirstOrderFilter<ja_type> jaFilter;
    ja_type l_omega_p = ja_type::Constant(50.0);
    jaFilter.setLowPass(l_omega_p);
    pm.getExecutionManager()->startManaging(jaFilter);


    systems::connect(wam.jvOutput, hp1.input);
    systems::connect(hp1.output, jaWAM.input);
    systems::connect(jaWAM.output, jaFilter.input);
    systems::connect(jaFilter.output, followerDynamics.jaInputDynamics);

    systems::connect(wam.jpOutput, follower.wamJPIn);
    systems::connect(wam.jvOutput, follower.wamJVIn);
    // systems::connect(extFilter.output, follower.extTorqueIn);
    systems::connect(dynamicExternalTorque.wamExternalTorqueOut, follower.extTorqueIn);

    systems::connect(wam.jpOutput, followerDynamics.jpInputDynamics);
    systems::connect(wam.jvOutput, followerDynamics.jvInputDynamics);
    // systems::connect(zeroAcceleration.output, followerDynamics.jaInputDynamics);

    systems::connect(follower.wamJPOutput, customjtSum.getInput(0));
    systems::connect(wam.gravity.output, customjtSum.getInput(1));
    systems::connect(wam.supervisoryController.output, customjtSum.getInput(2));

    systems::connect(customjtSum.output, dynamicExternalTorque.wamTorqueSumIn);
    systems::connect(followerDynamics.dynamicsFeedFWD, dynamicExternalTorque.wamDynamicsIn);

    systems::connect(wam.gravity.output, follower.wamGravIn);
    systems::connect(followerDynamics.dynamicsFeedFWD, follower.wamDynIn);

    systems::connect(dynamicExternalTorque.wamExternalTorqueOut, extFilter.input);

    // systems::connect(extFilter.output, printdynamicextTorque.input);
    // systems::connect(dynamicExternalTorque.wamExternalTorqueOut, printdynamicextTorque.input);
    // systems::connect(wam.supervisoryController.output, printSC.input);
    // systems::connect(extFilter.output, printcustomjtSum.input);

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
            } else {
                wam.moveTo(SYNC_POS, true);

                printf("Press [Enter] to link with the other WAM.");
                waitForEnter();
                follower.tryLink();
                wam.trackReferenceSignal(follower.theirJPOutput);
                systems::connect(follower.wamJPOutput, wam.input);
                // connect(follower.wamJPOutput, wamJPOutputRamp.input); // one of the problem with the joint limiter is that it adds delay in applying external torque to the robot.
                // connect(wamJPOutputRamp.output, wam.input);
                // systems::forceConnect(wam.jtSum.output, externalTorque.wamTorqueSumIn);

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
            printf("    'l' to toggle linking with other WAM\n");
            printf("    't' to tune control gains\n");
            printf("    'x' to exit\n");

            break;
        }
    }


    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

    return 0;
}