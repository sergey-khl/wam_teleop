#pragma once

#include <haptic_wrist/haptic_wrist.h>
#include <boost/asio.hpp>
#include <iostream>
#include <cmath>
#include <cstdint>
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
#include "utils.h"

template <size_t DOF>
class Leader : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    // Inputs (same as your first file)
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Input<jt_type> extTorqueIn;   // may be undefined
    Input<jt_type> wamGravIn;
    Input<jt_type> wamDynIn;

    // Outputs (same as your first file)
    Output<jt_type> wamJPOutput;      // control torque command for the WAM arm (DOF)
    Output<jp_type> theirJPOutput;    // peer arm JP (DOF) for logging/monitoring

    enum class State { INIT, LINKED, UNLINKED };

    // we dont actually do inference on the leader but we still record data from it
    explicit Leader(barrett::systems::ExecutionManager* em, haptic_wrist::HapticWrist* hw, 
                const TeleopConfig& config,
                const std::string& sysName = "Leader")
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
        , udp_handler(config.network.follower_host, config.network.teleop_send, config.network.teleop_recv, 
                      config.network.mode, config.network.inference_host, config.network.leader_inference_send, config.network.inference_recv)
        , hw(hw)
    	, bumper(0.0f)
    	, trigger(0.0f)
    	, btn_x(0.0f)
    	, btn_o(0.0f)
    	, dpad_up(0.0f)
    	, dpad_down (0.0f)
    	, dpad_left(0.0f)
    	, dpad_right(0.0f)
        , desired_gripper_vel(0.0f)
        , remote_gripper_torque(0.0f)
        , io_running(false)
        , state(State::INIT) {

        torque_scaling   = config.leader.haptics.torque_scaling;
        minStiffness     = config.leader.haptics.minStiffness;
        maxStiffness     = config.leader.haptics.maxStiffness;
        alpha            = config.leader.haptics.alpha;

        last_op_time = std::chrono::steady_clock::now();

        if (em != NULL) {
            em->startManaging(*this);
        }
        io_running.store(true);
        io_thread = std::thread(&Leader::pollHandle, this);
    }

    virtual ~Leader() {
        io_running.store(false);
        if (io_thread.joinable()) {
            io_thread.join();
        }
        this->mandatoryCleanUp();
    }

    virtual bool inputsValid() { return true; }

    bool isLinked() const { return state == State::LINKED; }
    void tryLink() { BARRETT_SCOPED_LOCK(this->getEmMutex()); state = State::LINKED; }
    void unlink() { BARRETT_SCOPED_LOCK(this->getEmMutex()); state = State::UNLINKED; }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    typename Output<jp_type>::Value* theirJPOutputValue;

    TeleopConfig config;

    // Local copies
    jp_type wamJP;
    jv_type wamJV;
    jt_type extTorque;
    jt_type wamGrav;
    jt_type wamDyn;

    const float gripper_speed = 0.3f;

    // TODO: these should be bool
    std::atomic<double> bumper;
    std::atomic<double> trigger;
    std::atomic<double> btn_x;
    std::atomic<double> btn_o;
    std::atomic<double> dpad_up;
    std::atomic<double> dpad_down ;
    std::atomic<double> dpad_left;
    std::atomic<double> dpad_right;

    std::atomic<float> desired_gripper_vel;
    std::atomic<float> remote_gripper_torque;


    float torque_scaling;
    float minStiffness;
    float maxStiffness;
    float alpha;

    int loop_counter = 0;
    std::chrono::time_point<std::chrono::steady_clock> last_op_time;

    // Wrist state
    haptic_wrist::jp_type wristJP;      // local wrist pos
    haptic_wrist::jv_type wristJV;      // local wrist vel
    haptic_wrist::jp_type theirWristJp; // received wrist pos

    // Network payloads: arm (DOF) + wrist (3) = DOF+3
    Eigen::Matrix<double, DOF + 3, 1> sendJpMsg;
    Eigen::Matrix<double, DOF + 3, 1> sendJvMsg;
    Eigen::Matrix<double, DOF + 3, 1> sendExtTorqueMsg;
    Eigen::Matrix<double, DOF + 3, 1> sendMeasTorqueMsg;

    using ReceivedData = typename UDPHandler<DOF + 3>::ReceivedData;

    virtual void operate() {
        auto now_op = std::chrono::steady_clock::now();
        double loop_dt = std::chrono::duration<double, std::milli>(now_op - last_op_time).count();
        last_op_time = now_op;

        double dt_sec = loop_dt / 1000.0;
        if (dt_sec < 0.0) dt_sec = 0.0;
        if (dt_sec > 0.1) dt_sec = 0.1;

        // Read WAM inputs
        wamJP  = wamJPIn.getValue();
        wamJV  = wamJVIn.getValue();
        wamGrav = wamGravIn.getValue();
        wamDyn  = wamDynIn.getValue();

        if (extTorqueIn.valueDefined()) {
            extTorque = extTorqueIn.getValue();
        } else {
            extTorque.setZero();
        }

        // Read wrist device (if available)
        haptic_wrist::jp_type wristJP = hw->getPosition();
        haptic_wrist::jp_type wristJV = hw->getVelocity();


        // Receive (non-blocking)
        boost::optional<ReceivedData> received_data = udp_handler.getLatestTeleopReceived();
        uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        uint64_t timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(TIMEOUT_DURATION).count();
        double udp_rx_age = 0.0;
        if (received_data && (now_ns >= received_data->timestamp) && (now_ns - received_data->timestamp <= timeout_ns)) {
            udp_rx_age = static_cast<double>(now_ns - received_data->timestamp) / 1000000.0;

            // Split received arm & wrist
            theirJp        = received_data->jp.template head<DOF>();
            theirJv        = received_data->jv.template head<DOF>();
            theirExtTorque = received_data->extTorque.template head<DOF>();
            remote_gripper_torque.store(static_cast<double>(received_data->gripper));

            // mirror and offset some wrist joints
            // for (size_t i = 0; i < 3; i++) {
            //     theirWristJp[i] = received_data->jp(DOF + i) * config.sync_mapping.scales[DOF + i] + config.sync_mapping.offsets[DOF + i];
            // }
            theirWristJp[0] = received_data->jp(DOF) * config.sync_mapping.scales[DOF] + config.sync_mapping.offsets[DOF];
            theirWristJp[1] = 0;
            theirWristJp[2] = 0;

            // mirror and offset some of the wam joints
            for (size_t i = 0; i < DOF; i++) {
                theirJp[i] = theirJp[i] * config.sync_mapping.scales[i] + config.sync_mapping.offsets[i];
                theirJv[i] = theirJv[i] * config.sync_mapping.scales[i];
                theirExtTorque[i] = theirExtTorque[i] * config.sync_mapping.scales[i];
            }

            // Publish peer arm JP (as before)
            theirJPOutputValue->setData(&theirJp);
        } else {
            if (state == State::LINKED) {
                std::cout << "lost link" << std::endl;
                state = State::UNLINKED;
            }
        }

        // Pack outgoing messages (arm + wrist)
        sendJpMsg << wamJP, wristJP;
        sendJvMsg << wamJV, wristJV;

        // Only arm external torque is meaningful here; pad wrist torques with zeros
        sendExtTorqueMsg << extTorque, 0.0, 0.0, 0.0;

        // State machine
        switch (state) {
            case State::INIT:
                control.setZero();
                jtOutputValue->setData(&control);
                break;

            case State::LINKED:

                // // Drive wrist to peer wrist pose
                hw->setTarget(theirWristJp);

                // Your arm control law (kept intact)
                control = compute_control(
                    theirJp, theirJv, theirExtTorque,
                    wamJP,   wamJV,   extTorque,
                    wamGrav, wamDyn
                );
                jtOutputValue->setData(&control);
                break;

            case State::UNLINKED:
                control.setZero();
                jtOutputValue->setData(&control);
                break;
        }

        sendMeasTorqueMsg << control, 0.0, 0.0, 0.0;

        auto send_start = std::chrono::steady_clock::now();
        udp_handler.send(sendJpMsg, sendJvMsg, sendExtTorqueMsg, sendMeasTorqueMsg, static_cast<double>(desired_gripper_vel.load()));
        auto send_end = std::chrono::steady_clock::now();
        double send_dt = std::chrono::duration<double, std::milli>(send_end - send_start).count();

        if (++loop_counter % 500 == 0) {
        //     std::cout << "[LEADER] Loop dt: " << loop_dt 
        //               << " ms | UDP Rx Age: " << udp_rx_age 
        //               << " ms | UDP Send latency: " << send_dt << " ms\n";
               
            std::cout << std::fixed << std::setprecision(3);
            // std::cout << "  -> TX JP:      [" << sendJpMsg.transpose() << "]\n";
            // std::cout << "  -> TX JV:      [" << sendJvMsg.transpose() << "]\n";
            // std::cout << "  -> TX ExtTrq:  [" << sendExtTorqueMsg.transpose() << "]\n";
            // std::cout << "  -> TX MeasTrq: [" << sendMeasTorqueMsg.transpose() << "]\n";
            // std::cout << "  -> TX GrpVel:  " << desired_gripper_vel.load() << "\n";
            std::cout << "  -> Their wrist:  " << theirWristJp.transpose() << "\n";
            std::cout << "  -> My wrist:  " << wristJP.transpose() << "\n\n";
            // std::cout << "  -> leader ext:  " << extTorque.transpose() << "\n";
            // std::cout << "  -> ref:  " << theirExtTorque.transpose() << "\n\n";
        }
    }

    // Peer (arm) state & internal
    jp_type theirJp;
    jv_type theirJv;
    jt_type theirExtTorque;
    jt_type control;
    std::thread io_thread;
    std::atomic<bool> io_running;

    void pollHandle() {
        float local_smoothed_torque = 0.0f;
        while (io_running.load()) {
            if (boost::optional<haptic_wrist::handle_type> opt_handle = hw->getHandle()) {
                haptic_wrist::handle_type handle = *opt_handle;
                bumper.store(handle[0]);
                trigger.store(handle[1]);
                btn_x.store(handle[2]);
                btn_o.store(handle[3]);
                dpad_up.store(handle[4]);
                dpad_down.store(handle[5]);
                dpad_left.store(handle[6]);
                dpad_right.store(handle[7]);
            }

            float target_velocity = 0.0f;

            if (bumper && !trigger) {
                target_velocity = -gripper_speed;
            } else if (trigger && !bumper) {
                target_velocity = gripper_speed;
            }

            desired_gripper_vel.store(target_velocity);

            float remote_torque = remote_gripper_torque.load(); // TODO: do something with this.

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        hw->setTriggerHaptics(0);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(Leader);

    haptic_wrist::HapticWrist* hw;     // NEW
    std::mutex state_mutex;
    jp_type joint_positions;
    UDPHandler<DOF + 3> udp_handler;
    const std::chrono::milliseconds TIMEOUT_DURATION = std::chrono::milliseconds(20);
    State state;

    // Gains you had
    Eigen::Matrix<double, DOF, 1> kp;
    Eigen::Matrix<double, DOF, 1> kd;
    Eigen::Matrix<double, DOF, 1> cf;

    // === Your original control choices preserved ===
    jt_type compute_control(const jp_type& ref_pos, const jv_type& ref_vel, const jt_type& ref_extTorque,
                            const jp_type& cur_pos, const jv_type& cur_vel, const jt_type& cur_extTorque,
                            const jt_type& cur_grav, const jt_type& cur_dyn) {

        jt_type u1 = 0.0 * cur_extTorque;                        // zero FF (P-P + g-comp only if you add it)
        jt_type u2 = cur_dyn - cur_grav;                          // P-P with dynamic comp (your comment)
        jt_type u3 = -0.5 * ref_extTorque;                        // PF-PF (ref ext torque FF)
        jt_type u4 = -0.1 * ref_extTorque + cur_dyn - cur_grav;   // PF-PF + dyn comp (Lawrence ideal)
        jt_type u5 = -0.5 * ref_extTorque - 0.15 * (ref_extTorque + cur_extTorque);
        jt_type u6 = -0.1 * ref_extTorque - 0.03 * (ref_extTorque + cur_extTorque) + cur_dyn - cur_grav;

        // Follower-different cases (kept for completeness)
        jt_type u7 = -0.5 * cur_extTorque;
        jt_type u8 = -0.25 * (ref_extTorque + cur_extTorque);

        // Default: u4 as you had
        return u1;
    };
};
