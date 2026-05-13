#pragma once

#include <boost/asio.hpp>
#include <iostream>
#include <cmath>
#include <atomic>
#include <thread>
#include <chrono>
#include <iomanip>


#include "udp_handler.h"
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>
#include "teleop_config_loader.h"
#include "gripper_command.h"
#include "teleop_gripper.h"
#include "utils.h"

template <size_t DOF>
class Follower : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Input<jt_type> extTorqueIn;
    Input<jt_type> wamGravIn;
    Input<jt_type> wamDynIn;
    Output<jt_type> wamJPOutput;
    Output<jp_type> theirJPOutput;

    enum class State { INIT, LINKED, UNLINKED };

    explicit Follower(barrett::systems::ExecutionManager* em, TeleopGripper* gripper,
                  const TeleopConfig& config,
                  const std::string& sysName = "Follower")
        : System(sysName)
        , config(config)
        , theirJp(0.0)
        , theirJv(0.0)
        , theirExtTorque(0.0)
        , control(0.0)
        , wamJPIn(this)
        , wamJVIn(this)
        , extTorqueIn(this)
        , wamGravIn(this)
        , wamDynIn(this)
        , wamJPOutput(this, &jtOutputValue)
        , theirJPOutput(this, &theirJPOutputValue)
        , udp_handler(config.network.leader_host, config.network.teleop_recv, config.network.teleop_send, 
                      config.network.mode, config.network.inference_host, config.network.follower_inference_send, config.network.inference_recv)
        , gripper(gripper)
        , target_gripper_command(gripper_command::encode(GripperCommand{}))
        , current_gripper_torque(0.0f)
        , io_running(false)
        , state(State::INIT) {

        for (size_t i = 0; i < DOF; i++) {
            kp[i] = config.follower.gains.kp[i];
            kd[i] = config.follower.gains.kd[i];
            cf[i] = config.follower.gains.cf[i];
        }

        last_op_time = std::chrono::steady_clock::now();

        if (em != NULL) {
            em->startManaging(*this);
        }
        io_running.store(true);
        io_thread = std::thread(&Follower::pollGripper, this);
    }

    virtual ~Follower() {
        stopGripperControl();
        this->mandatoryCleanUp();
    }

    virtual bool inputsValid() {return true;}

    bool isLinked() const {
        return state == State::LINKED;
    }
    void tryLink() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        state = State::LINKED;
    }
    void unlink() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        state = State::UNLINKED;
    }

    void stopGripperControl() {
        io_running.store(false);
        if (io_thread.joinable()) {
            io_thread.join();
        }
    }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    typename Output<jp_type>::Value* theirJPOutputValue;
    jp_type wamJP;
    jv_type wamJV;
    jt_type extTorque;
    jt_type wamGrav;
    jt_type wamDyn;
    Eigen::Matrix<double, DOF, 1> sendJpMsg;
    Eigen::Matrix<double, DOF, 1> sendJvMsg;
    Eigen::Matrix<double, DOF, 1> sendExtTorqueMsg;

    TeleopConfig config;
    
    int loop_counter = 0;
    std::chrono::time_point<std::chrono::steady_clock> last_op_time;

    using ReceivedData = typename UDPHandler<DOF>::ReceivedData;

    virtual void operate() {
        auto now_op = std::chrono::steady_clock::now();
        double loop_dt = std::chrono::duration<double, std::milli>(now_op - last_op_time).count();
        last_op_time = now_op;

        wamJP = wamJPIn.getValue();
        wamJV = wamJVIn.getValue();
        wamGrav = wamGravIn.getValue();
        wamDyn = wamDynIn.getValue();

        if (extTorqueIn.valueDefined()) {
            extTorque = extTorqueIn.getValue();
            // std::cout << "defined" << std::endl;
        } else {
            // std::cout << "not defined" << std::endl;
            extTorque << 0.0, 0.0, 0,0, 0.0;
        }

        sendJpMsg << wamJP;
        sendJvMsg << wamJV;
        sendExtTorqueMsg << extTorque;

        boost::optional<ReceivedData> received_data = udp_handler.getLatestTeleopReceived();
        uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        uint64_t timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(TIMEOUT_DURATION).count();
        double udp_rx_age = 0.0;
        if (received_data && (now_ns >= received_data->timestamp) && (now_ns - received_data->timestamp <= timeout_ns)) {
            udp_rx_age = static_cast<double>(now_ns - received_data->timestamp) / 1000000.0;

            theirJp = received_data->jp;
            theirJv = received_data->jv;
            theirExtTorque = received_data->extTorque;
            target_gripper_command.store(received_data->gripper);

            // mirror and offset some of the wam joints
            for (size_t i = 0; i < DOF; i++) {
                theirJp[i] = theirJp[i] * config.sync_mapping.scales[i] + config.sync_mapping.offsets[i];
                theirJv[i] = theirJv[i] * config.sync_mapping.scales[i];
                theirExtTorque[i] = theirExtTorque[i] * config.sync_mapping.scales[i];
            }

            theirJPOutputValue->setData(&theirJp);
        } else {
            if (state == State::LINKED) {
                std::cout << "lost link" << std::endl;
                state = State::UNLINKED;
            }
        }

        switch (state) {
            case State::INIT:
                control.setZero();
                jtOutputValue->setData(&control);
                break;
            case State::LINKED:
                // // Active teleop. Only the callee can transition to LINKED
                control = compute_control(theirJp, theirJv, theirExtTorque, wamJP, wamJV, extTorque, wamGrav, wamDyn);
                jtOutputValue->setData(&control);
                break;
            case State::UNLINKED:
                // // Changed to unlinked with either timeout or callee.
                control.setZero();
                jtOutputValue->setData(&control);
                break;
        }

        // sendExtTorqueMsg << control;

        auto send_start = std::chrono::steady_clock::now();
        udp_handler.send(wamJP, wamJV, sendExtTorqueMsg, control, static_cast<double>(current_gripper_torque.load()));
        auto send_end = std::chrono::steady_clock::now();
        double send_dt = std::chrono::duration<double, std::milli>(send_end - send_start).count();

        if (++loop_counter % 500 == 0) {
        //     std::cout << "[FOLLOWER] Loop dt: " << loop_dt 
        //               << " ms | UDP Rx Age: " << udp_rx_age 
        //               << " ms | UDP Send latency: " << send_dt << " ms\n";

            // std::cout << std::fixed << std::setprecision(3);
            // std::cout << "  -> TX JP:      [" << sendJpMsg.transpose() << "]\n";
            // std::cout << "  -> TX JV:      [" << sendJvMsg.transpose() << "]\n";
            // std::cout << "  -> TX ExtTrq:  [" << sendExtTorqueMsg.transpose() << "]\n";
            // std::cout << "  -> TX MeasTrq: [" << control.transpose() << "]\n";
            // std::cout << "  -> TX GrpTrq:  " << current_gripper_torque.load() << "\n\n";
            // std::cout << "  -> ref:  " << theirExtTorque.transpose() << "\n\n";
        }
    }

    jp_type theirJp;
    jv_type theirJv;
    jt_type theirExtTorque;
    jt_type control;

    void pollGripper() {
        bool current_spread_half_open = false;
        while (io_running.load()) {
            if (gripper) {
                const GripperCommand command = gripper_command::decode(target_gripper_command.load());
                if (command.spread_half_open != current_spread_half_open) {
                    gripper->setVelocity(0.0);
                    gripper->setSpread(command.spread_half_open ? 0.5 : 0.0);
                    current_spread_half_open = command.spread_half_open;
                    std::cout << "gripper spread=" << (current_spread_half_open ? "50%" : "0%") << std::endl;
                }
                gripper->setVelocity(command.velocity, command.fingers12Only());
                current_gripper_torque.store(static_cast<float>(gripper->feedback()));
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        if (gripper) {
            gripper->setVelocity(0.0);
        }
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(Follower);
    std::mutex state_mutex;
    jp_type joint_positions;
    UDPHandler<DOF> udp_handler;
    const std::chrono::milliseconds TIMEOUT_DURATION = std::chrono::milliseconds(20);
    State state;
    Eigen::Matrix<double, DOF, 1> kp;
    Eigen::Matrix<double, DOF, 1> kd;
    Eigen::Matrix<double, DOF, 1> cf;

    TeleopGripper* gripper;
    std::thread io_thread;
    std::atomic<bool> io_running;
    std::atomic<double> target_gripper_command;
    std::atomic<float> current_gripper_torque;

    jt_type compute_control(const jp_type& ref_pos, const jv_type& ref_vel, const jt_type& ref_extTorque,
                            const jp_type& cur_pos, const jv_type& cur_vel, const jt_type& cur_extTorque,
                            const jt_type& cur_grav, const jt_type& cur_dyn) {
        
        // cases where the follower and leader have the same control law

        jt_type u1 = 0.0 * cur_extTorque; // zero feedforward (equal to default P-P with gravity compensation)

        jt_type u2 = cur_dyn - cur_grav; // P-P with dynamic compensation

        jt_type u3 = -0.5 * ref_extTorque; // PF-PF with ref external torque feedback

        jt_type u4 = -0.1 * ref_extTorque + cur_dyn - cur_grav; // PF-PF with ref external torque feedback and dynamic compensation (Lawrence's perfect transparency architecture);


        jt_type u5 = -0.5 * ref_extTorque -0.15 * (ref_extTorque + cur_extTorque); // PF-PF with ref external torque and cur external torque feedback

        jt_type u6 = -0.1 * ref_extTorque -0.03 * (ref_extTorque + cur_extTorque) + cur_dyn - cur_grav; // it has the best performance


        // cases that the leader side has differnt controller that the follower

        jt_type u7 = -0.0 * cur_extTorque; // zero

        jt_type u8 = -0.0 * (ref_extTorque + cur_extTorque); // zero

        // jt_type u9 = -0.0 * cur_extTorque - 0.0 * (ref_extTorque + cur_extTorque);

        jt_type u9 = -0.5 * ref_extTorque; // PF-PF with ref external torque as feedback

        jt_type u10 = -0.5 * ref_extTorque;

        jt_type u = u6;

        for (size_t i = 4; i < 7; ++i) {
            u[i] = 0.0;
        }
        return u;
    };
};
