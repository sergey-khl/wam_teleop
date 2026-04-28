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
                const TeleopConfig& cfg,
                const std::string& sysName = "Leader")
        : System(sysName)
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
        , udp_handler(cfg.network.follower_host, cfg.network.teleop_send, cfg.network.teleop_recv, 
                      cfg.network.mode, cfg.network.inference_host, cfg.network.inference_send, cfg.network.inference_recv)
        , hw(hw)
        , joy_x(0.0f)
        , trigger(0.0f)
        , bumper_pressed(false)
        , desired_gripper_vel(0.0f)
        , remote_gripper_torque(0.0f)
        , io_running(false)
        , state(State::INIT) {

        for (size_t i = 0; i < DOF; i++) {
            kp[i] = config.leader.gains.kp[i];
            kd[i] = config.leader.gains.kd[i];
            cf[i] = config.leader.gains.cf[i];
        }

        trigger_rest_pos = config.leader.haptics.trigger_rest_pos;
        target_velocity  = config.leader.haptics.target_velocity;
        torque_scaling   = config.leader.haptics.torque_scaling;
        minStiffness     = config.leader.haptics.minStiffness;
        maxStiffness     = config.leader.haptics.maxStiffness;
        alpha            = config.leader.haptics.alpha;
        j7_joy_deadband            = config.leader.haptics.j7_joy_deadband;
        j7_max_velocity_rad_s            = config.leader.haptics.j7_max_velocity_rad_s;

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

    std::atomic<float> joy_x;
    std::atomic<float> trigger;
    std::atomic<bool> bumper_pressed;
    std::atomic<float> desired_gripper_vel;
    std::atomic<float> remote_gripper_torque;


    double trigger_rest_pos;
    float target_velocity;
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
        auto now = std::chrono::steady_clock::now();
        double udp_rx_age = 0.0;
        if (received_data && (now - received_data->timestamp <= TIMEOUT_DURATION)) {
            udp_rx_age = std::chrono::duration<double, std::milli>(now - received_data->timestamp).count();

            // Split received arm & wrist
            theirJp        = received_data->jp.template head<DOF>();
            theirJv        = received_data->jv.template head<DOF>();
            theirExtTorque = received_data->extTorque.template head<DOF>();
            remote_gripper_torque.store(static_cast<double>(received_data->gripper));

            // theirWristJp = hw->getPosition();

            // get and mirror follower wrist
            for (size_t i = 0; i < 2; i++) {
                theirWristJp[i] = received_data->jp(DOF + i) * config.sync_mapping.scales[DOF + i] + config.sync_mapping.offsets[DOF + i];
            }

            remote_j7_pos = received_data->jp(6);

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

        updateJ7Command(dt_sec);

        // Pack outgoing messages (arm + wrist)
        sendJpMsg << wamJP, wristJP, j7_command_pos;
        sendJvMsg << wamJV, wristJV, j7_command_vel;

        // Only arm external torque is meaningful here; pad wrist torques with zeros
        sendExtTorqueMsg << extTorque, 0.0, 0.0, 0.0;

        // Example scaling of J5 and J7 (index 4 and 6 in the concatenated vector)
        sendJpMsg(4) = sendJpMsg(4);
        sendJpMsg(6) = sendJpMsg(6);

        // State machine
        switch (state) {
            case State::INIT:
                // Hold wrist (don’t move) and zero arm torque
                hw->setTarget(wristJP);
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
                // // Release to local wrist pose and zero arm torque
                hw->setTarget(wristJP);
                control.setZero();
                jtOutputValue->setData(&control);
                break;
        }

        // BEAR
        sendMeasTorqueMsg << control, 0.0, 0.0, 0.0;

        auto send_start = std::chrono::steady_clock::now();
        udp_handler.send(sendJpMsg, sendJvMsg, sendExtTorqueMsg, sendMeasTorqueMsg, static_cast<double>(desired_gripper_vel.load()));
        auto send_end = std::chrono::steady_clock::now();
        double send_dt = std::chrono::duration<double, std::milli>(send_end - send_start).count();

        if (++loop_counter % 500 == 0) {
        //     std::cout << "[LEADER] Loop dt: " << loop_dt 
        //               << " ms | UDP Rx Age: " << udp_rx_age 
        //               << " ms | UDP Send latency: " << send_dt << " ms\n";
               
            // std::cout << std::fixed << std::setprecision(3);
            // std::cout << "  -> TX JP:      [" << sendJpMsg.transpose() << "]\n";
            // std::cout << "  -> TX JV:      [" << sendJvMsg.transpose() << "]\n";
            // std::cout << "  -> TX ExtTrq:  [" << sendExtTorqueMsg.transpose() << "]\n";
            // std::cout << "  -> TX MeasTrq: [" << sendMeasTorqueMsg.transpose() << "]\n";
            // std::cout << "  -> TX GrpVel:  " << desired_gripper_vel.load() << "\n";
            // std::cout << "  -> Their wrist:  " << theirWristJp.transpose() << "\n";
            // std::cout << "  -> My wrist:  " << wristJP.transpose() << "\n\n";
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
                joy_x.store(static_cast<float>(handle[0]));
                trigger.store(static_cast<float>(handle[3]));
                bumper_pressed.store(static_cast<int>(handle[2]) == 1);
            }

            const float local_trigger = trigger.load();
            const bool local_bumper_pressed = bumper_pressed.load();
            float vel_command = 0.0f;
            if (local_trigger > trigger_rest_pos) {
                vel_command = target_velocity * local_trigger;
            } else if (local_bumper_pressed) {
                vel_command = -target_velocity;
            }
            desired_gripper_vel.store(vel_command);

            float remote_torque = remote_gripper_torque.load();

            local_smoothed_torque = (alpha * remote_torque) + ((1.0f - alpha) * local_smoothed_torque);
            if (local_smoothed_torque > minStiffness) {
                float dynamicStiffness =
                    local_smoothed_torque * torque_scaling * (maxStiffness - minStiffness) + minStiffness;
                float raw_haptics = 255.0f * dynamicStiffness;
                if (raw_haptics > 255.0f) {
                    raw_haptics = 255.0f;
                }
                hw->setTriggerHaptics(static_cast<uint8_t>(raw_haptics));
            } else {
                hw->setTriggerHaptics(0);
            }

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

    double j7_joy_deadband;
    double j7_max_velocity_rad_s;
    static constexpr size_t J7_INDEX = 6;
    bool j7_initialized = false;
    bool j7_joystick_active = false;
    double j7_command_pos = 0.0;
    double j7_command_vel = 0.0;
    double remote_j7_pos = 0.0;
    std::chrono::steady_clock::time_point j7_last_update;


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
        return u6;
    };

    void updateJ7Command(double dt) {
        if (state == State::INIT) {
            j7_initialized = false;
            j7_command_pos = 0.0;
            j7_command_vel = 0.0;
            return;
        }

        if (!j7_initialized) {
            j7_command_pos = remote_j7_pos; // Initialize to the follower's current J7
            j7_initialized = true;
        }

        double joy_cmd = joy_x.load();
        bool active = std::abs(joy_cmd) > j7_joy_deadband;

        if (active) {
            j7_command_vel = j7_max_velocity_rad_s * joy_cmd;
            j7_command_pos += j7_command_vel * dt;
            j7_joystick_active = true;
        } else {
            j7_command_vel = 0.0;

            j7_joystick_active = false;
        }
    }
};
