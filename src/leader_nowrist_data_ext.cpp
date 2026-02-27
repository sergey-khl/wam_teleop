/*
 * ex11_master_master.cpp
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 *      Author: Brian Zenowich
 */

// A version of leader_nowrist_data.cpp, with the difference that a virtual external torque is applied to the leader robot (free motion experiments for ICRA 2026 paper).

#include <cstdlib>  // For mkstmp()
#include <cstdio>  // For remove()

#include "lib/external_torque.h"
#include <iostream>
#include <string>

#include <boost/thread.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/detail/stl_utils.h>
#include <barrett/os.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/units.h>
#include <barrett/log.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

#include "lib/leader_nowrist_data.h"
#include "lib/background_state_publisher.h"
#include "lib/leader_dynamics.h"
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

    if ((argc == 2 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) || (argc > 5)) {
        printUsage(argv[0], "127.0.0.1", 5555, 5554);
        return 0;
    }

    return true;
}


template <size_t DOF> int wam_main(int argc, char **argv, ProductManager &pm, systems::Wam<DOF> &wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    char tmpFile_kinematics[] = "/tmp/btXXXXXX";
    if (mkstemp(tmpFile_kinematics) == -1) {
        printf("Error: Couldn't create temporary file!\n");
        return 1;
    }

    char tmpFile_dynamics[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile_dynamics) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}

    jp_type SYNC_POS; // the position each WAM should move to before linking
    if (DOF == 4) {
        SYNC_POS[0] = 0.0;
        SYNC_POS[1] = 0.0;
        SYNC_POS[2] = 0.0;
        SYNC_POS[3] = 0.0;
        // SYNC_POS[4] = 0.0;
        // SYNC_POS[5] = 0.0;
        // SYNC_POS[6] = 0.0;

    } else {
        printf("Error: 4 DOF supported\n");
        return false;
    }

    std::string remoteHost = "127.0.0.1";
    int rec_port = 5555;
    int send_port = 5554;

    if (argc >= 2) {
        remoteHost = std::string(argv[1]);
    }
    if (argc >= 3) {
        rec_port = std::atoi(argv[2]);
    }
    if (argc >= 4) {
        send_port = std::atoi(argv[3]);
    }

    ros::init(argc, argv, "leader_nowrist");
    BackgroundStatePublisher<DOF> state_publisher(pm.getExecutionManager(), wam);

    barrett::systems::Summer<jt_type, 3> customjtSum;
    pm.getExecutionManager()->startManaging(customjtSum);

    LeaderDynamics<DOF> leaderDynamics(pm.getExecutionManager());

    ExternalTorque<DOF> externalTorque(pm.getExecutionManager());

    DynamicExternalTorque<DOF> dynamicExternalTorque(pm.getExecutionManager());
    
    barrett::systems::FirstOrderFilter<jt_type> extFilter;
    jt_type omega_p(180.0);
    extFilter.setLowPass(omega_p);
    pm.getExecutionManager()->startManaging(extFilter);

    barrett::systems::FirstOrderFilter<jt_type> dynamicExtFilter;
    // jt_type omega_p(180.0);
    dynamicExtFilter.setLowPass(omega_p);
    pm.getExecutionManager()->startManaging(dynamicExtFilter);

    ja_type ja;
    ja.setConstant(0.0);
    systems::Constant<ja_type> zeroAcceleration(ja);
    pm.getExecutionManager()->startManaging(zeroAcceleration);

    Leader<DOF> leader(pm.getExecutionManager(), remoteHost, rec_port, send_port);

    jt_type maxRate; // Nm · s-1 per joint
    maxRate << 50, 50, 50, 50;
    systems::RateLimiter<jt_type> wamJPOutputRamp(maxRate, "ffRamp");

    systems::PrintToStream<jt_type> printdynamicextTorque(pm.getExecutionManager(), "dynamicextTorque: ");
    systems::PrintToStream<jt_type> printextTorque(pm.getExecutionManager(), "extTorque: ");
    systems::PrintToStream<jt_type> printdynamicoutput(pm.getExecutionManager(), "dynamicoutput: ");
    systems::PrintToStream<jt_type> printSC(pm.getExecutionManager(), "SC: ");
    // systems::PrintToStream<jt_type> printjtSum(pm.getExecutionManager(), "jtSum: ");
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


    //Applied External Torque
    jt_type A;
	A << 0.0, 9.0, 0.0, 3.0;
	double f = 0.3;
	sinextorq<DOF> extorqFeedFWD(A, f);
	systems::Summer<jt_type, 2> feedFwdSum;
	systems::Ramp time(pm.getExecutionManager(), 1.0);
	connect(time.output, extorqFeedFWD.timef);
	connect(leader.wamJPOutput, feedFwdSum.getInput(0));
	connect(extorqFeedFWD.extorq, feedFwdSum.getInput(1));

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

    systems::connect(leader.wamJPOutput, customjtSum.getInput(0));
    systems::connect(wam.gravity.output, customjtSum.getInput(1));
    systems::connect(wam.supervisoryController.output, customjtSum.getInput(2));

    // systems::connect(wam.gravity.output, externalTorque.wamGravityIn);
    // systems::connect(customjtSum.output, externalTorque.wamTorqueSumIn);
    // systems::connect(externalTorque.wamExternalTorqueOut, extFilter.input);

    systems::connect(customjtSum.output, dynamicExternalTorque.wamTorqueSumIn);
    systems::connect(leaderDynamics.dynamicsFeedFWD, dynamicExternalTorque.wamDynamicsIn);
    // systems::connect(dynamicExternalTorque.wamExternalTorqueOut, dynamicExtFilter.input);

    systems::connect(wam.gravity.output, leader.wamGravIn);
    systems::connect(leaderDynamics.dynamicsFeedFWD, leader.wamDynIn);

    // systems::connect(dynamicExternalTorque.wamExternalTorqueOut, printdynamicextTorque.input);
    // systems::connect(extFilter.output, printextTorque.input);
    // systems::connect(wam.supervisoryController.output, printSC.input);
    // systems::connect(leaderDynamics.dynamicsFeedFWD, printdynamicoutput.input);

    // systems::connect(extFilter.output, printjtSum.input);
    // systems::connect(extFilter.output, printcustomjtSum.input);

    systems::Ramp timelog(pm.getExecutionManager(), 1.0);
    systems::TupleGrouper<double, jp_type, jp_type> tg_kinematics;
    systems::connect(timelog.output, tg_kinematics.template getInput<0>());
    systems::connect(leader.theirJPOutput, tg_kinematics.template getInput<1>());
    systems::connect(wam.jpOutput, tg_kinematics.template getInput<2>());

    typedef boost::tuple<double, jp_type, jp_type> tuple_type_kinematics;
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type_kinematics> logger_kinematics(
        pm.getExecutionManager(),
		new log::RealTimeWriter<tuple_type_kinematics>(tmpFile_kinematics, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
		PERIOD_MULTIPLIER);
    
    systems::TupleGrouper<double, jt_type, jt_type, jt_type, jt_type, jt_type, jt_type, jt_type> tg_dynamics;
    systems::connect(timelog.output, tg_dynamics.template getInput<0>());
    systems::connect(wam.jtSum.output, tg_dynamics.template getInput<1>());
    systems::connect(customjtSum.output, tg_dynamics.template getInput<2>());
    systems::connect(wam.gravity.output, tg_dynamics.template getInput<3>());
    systems::connect(leaderDynamics.dynamicsFeedFWD, tg_dynamics.template getInput<4>());
    systems::connect(dynamicExternalTorque.wamExternalTorqueOut, tg_dynamics.template getInput<5>());
    systems::connect(leader.theirextTorqueOutput, tg_dynamics.template getInput<6>());
    systems::connect(extorqFeedFWD.extorq, tg_dynamics.template getInput<7>());
    
    typedef boost::tuple<double, jt_type, jt_type, jt_type, jt_type, jt_type, jt_type, jt_type> tuple_type_dynamics;
	systems::PeriodicDataLogger<tuple_type_dynamics> logger_dynamics(
        pm.getExecutionManager(),
        new log::RealTimeWriter<tuple_type_dynamics>(tmpFile_dynamics, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
        PERIOD_MULTIPLIER);


    wam.gravityCompensate();


    std::string line;
    v_type gainTmp;

    bool going = true;

    while (going) {
        printf(">>> ");
        std::getline(std::cin, line);

        switch (line[0]) {
        case 'l':
            if (leader.isLinked()) {
                leader.unlink();
            } else {
                wam.moveTo(SYNC_POS, true);

                printf("Press [Enter] to link with the other WAM.");
                waitForEnter();
                leader.tryLink();
                wam.trackReferenceSignal(leader.theirJPOutput);
                connect(feedFwdSum.output, wam.input);
                // connect(leader.wamJPOutput, wamJPOutputRamp.input); // one of the problem with the joint limiter is that it adds delay in applying external torque to the robot.
                // connect(wamJPOutputRamp.output, wam.input);
                // systems::forceConnect(wam.jtSum.output, externalTorque.wamTorqueSumIn);
                

                btsleep(0.1); // wait an execution cycle or two
                if (leader.isLinked()) {
                    printf("Linked with remote WAM.\n");
                } else {
                    printf("WARNING: Linking was unsuccessful.\n");
                }

                timelog.start();
                connect(tg_kinematics.output, logger_kinematics.input);
                connect(tg_dynamics.output, logger_dynamics.input);
				printf("Logging started.\n");
                time.stop();
				time.reset();
				time.start();
				btsleep((2/f));
				time.stop();
				time.reset();
				logger_kinematics.closeLog();
				logger_dynamics.closeLog();
				printf("Logging stopped.\n");
				timelog.stop();
				timelog.reset();

            }


            break;

        // case 'r':
        //     timelog.start();
        //     connect(tg_kinematics.output, logger_kinematics.input);
        //     connect(tg_dynamics.output, logger_dynamics.input);
        //     printf("Logging started.\n");
        //     break;

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
            logger_kinematics.closeLog();
            logger_dynamics.closeLog();
            printf("Logging stopped.\n");
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


    // Create the data directory using the provided name
	std::string folderName = argv[4];
	// Create the data directory using the provided name
	std::string command = std::string("mkdir -p ../data/") + folderName; // -p flag ensures it doesn't fail if the directory exists
	if (system(command.c_str()) != 0) {
    	std::cerr << "Error: Could not create directory." << std::endl;
    	return 1;
	}

    std::string kinematicsFilename = "../data/" + folderName + "/kinematics.txt";
	std::string dynamicsFilename = "../data/" + folderName + "/dynamics.txt";
	std::string configFilename = "../data/" + folderName + "/config.txt";
	std::ofstream kinematicsFile(kinematicsFilename);
	std::ofstream dynamicsFile(dynamicsFilename);
	std::ofstream configFile(configFilename);

    //Config File Writing
	configFile << "Teleop\n";
	configFile << "Kinematics data: time, desired joint pos, feedback joint pos\n";
	configFile << "Dynamics data: time, wam joint torque input, custom joint torque input, wam gravity input, dynamic input, leader external torque, follower external torque, leader virtual external torque\n";
	configFile << "Joint Position PID Controller: \nkp: " << wam.jpController.getKp() << "\nki: " << wam.jpController.getKi()<<  "\nkd: "<< wam.jpController.getKd() <<"\nControl Signal Limit: " << wam.jpController.getControlSignalLimit() <<".\n";
	configFile << "Sync Pos:" << SYNC_POS;

	log::Reader<tuple_type_kinematics> lr_kinematics(tmpFile_kinematics);
	lr_kinematics.exportCSV(kinematicsFile);
	log::Reader<tuple_type_dynamics> lr_Dynamics(tmpFile_dynamics);
	lr_Dynamics.exportCSV(dynamicsFile);
	configFile.close();
	printf("Output written to %s folder.\n", folderName.c_str());

	unlink(tmpFile_kinematics);
	unlink(tmpFile_dynamics);

    std::remove(tmpFile_kinematics);
    std::remove(tmpFile_dynamics);

    return 0;
}