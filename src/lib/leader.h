#pragma once

// #include <haptic_wrist/handle.h>
#include <boost/asio.hpp>
#include <iostream>
#include <cmath>
#include <cstdint>
#include <atomic>
#include <thread>
#include <chrono>
#include <iomanip>

#include "leader_udp_handler.h"
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
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Input<boost::tuple<cp_type, Eigen::Quaterniond>> wamTPIn;
    Input<jt_type> dyngravcompTorqueIn;   // may be undefined
    Input<jt_type> wamGravIn;
    Input<jt_type> wamDynIn;
    Input<jt_type> policyJtIn;
    Input<jt_type> policyTorqueScaleIn;
    Input<jt_type> humanTorqueIn;
    Input<jt_type> filteredHumanTorqueIn;

    Output<jt_type> wamJTOutput;      // control torque command for the WAM arm (DOF)
    Output<jp_type> theirJPOutput;    // peer arm JP (DOF) for logging/monitoring
    Output<jp_type> policyJPOutput;      // control torque command for the WAM arm (DOF)

    std::atomic<bool> linked;

    explicit Leader(barrett::systems::ExecutionManager* em, haptic_wrist::Handle* handle,
                const TeleopConfig& config,
                const std::string& sysName = "Leader")
        : System(sysName)
        , config(config)
        , theirJp(0.0)
        , theirJv(0.0)
        , theirDyngravcompTorque(0.0)
        , environmentTorque(0.0)
        , theirToolPos(0.0)
        , theirToolQ(1, 0, 0, 0)
        , control(0.0)
        , wamJPIn(this)
        , wamJVIn(this)
        , wamTPIn(this)
        , dyngravcompTorqueIn(this)
        , wamGravIn(this)
        , wamDynIn(this)
        , policyJtIn(this)
        , policyTorqueScaleIn(this)
        , humanTorqueIn(this)
        , filteredHumanTorqueIn(this)
        , wamJTOutput(this, &jtOutputValue)
        , theirJPOutput(this, &theirJPOutputValue)
        , policyJPOutput(this, &policyJPOutputValue)
        , teleop_udp_handler(config.network.follower_host, config.network.teleop_send, config.network.teleop_recv)
        , policy_udp_handler(config.policy.on_leader, config.network.policy_host, config.network.policy_send, config.network.policy_leader_recv)
        , handle(handle)
    	, bumper(0.0f)
    	, trigger(0.0f)
    	, cancel_policy(0.0f)
        , desired_gripper_pos(0.0f)
        , remote_gripper_torque(0.0f)
        , remote_gripper_pos(0.0f)
        , remote_gripper_vel(0.0f)
        , io_running(false)
        , linked(false) {

        torque_scaling   = config.handle.torque_scaling;
        minStiffness     = config.handle.minStiffness;
        maxStiffness     = config.handle.maxStiffness;
        alpha            = config.handle.alpha;

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

    bool isLinked() const { return linked.load(); }
    void tryLink()  { BARRETT_SCOPED_LOCK(this->getEmMutex()); linked.store(true); }
    void unlink()   { BARRETT_SCOPED_LOCK(this->getEmMutex()); linked.store(false); }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    typename Output<jp_type>::Value* theirJPOutputValue;
    typename Output<jp_type>::Value* policyJPOutputValue;

    TeleopConfig config;

    jp_type wamJP;
    jv_type wamJV;
    boost::tuple<cp_type, Eigen::Quaterniond> wamTP;
    jt_type dyngravcompTorque;
    jt_type humanTorque;
    jt_type filteredHumanTorque;
    jt_type wamGrav;
    jt_type wamDyn;
    jp_type policyJp;

    const float gripper_speed = 0.1f;

    // TODO: these should be bool
    std::atomic<double> bumper;
    std::atomic<double> trigger;
    std::atomic<double> cancel_policy;

    std::atomic<float> desired_gripper_pos;
    std::atomic<float> remote_gripper_torque;
    std::atomic<float> remote_gripper_pos;
    std::atomic<float> remote_gripper_vel;


    float torque_scaling;
    float minStiffness;
    float maxStiffness;
    float alpha;

    int loop_counter = 0;
    std::chrono::time_point<std::chrono::steady_clock> last_op_time;

    Eigen::Matrix<double, DOF, 1> sendJpMsg;
    Eigen::Matrix<double, DOF, 1> sendJvMsg;
    Eigen::Matrix<double, DOF, 1> sendDyngravcompTorqueMsg;
    Eigen::Matrix<double, DOF, 1> sendHumanTorqueMsg;

    using TeleopReceivedData = typename LeaderUDPHandler<DOF>::TeleopReceivedData;

    virtual void operate() {
        auto now_op = std::chrono::steady_clock::now();
        // uint64_t loop_dt = std::chrono::duration<uint64_t, std::nano>(now_op - last_op_time).count();
        // last_op_time = now_op;

        // Read WAM inputs
        wamJP  = wamJPIn.getValue();
        wamJV  = wamJVIn.getValue();
        wamTP = wamTPIn.getValue();
        wamGrav = wamGravIn.getValue();
        wamDyn  = wamDynIn.getValue();

        // policy defaults
        policyJp << wamJP;
        policy_gripper_cmd.store(remote_gripper_pos.load());

        // teleop
        boost::optional<TeleopReceivedData> received_data = teleop_udp_handler.getLatestTeleopReceived();
        uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        uint64_t timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(TIMEOUT_DURATION).count();
        double udp_teleop_age = 0.0;
        if (received_data && (now_ns >= received_data->timestamp) && (now_ns - received_data->timestamp <= timeout_ns)) {
            theirJp        = received_data->jp;
            theirJv        = received_data->jv;
            theirDyngravcompTorque = received_data->dyngravcompTorque;
            environmentTorque = received_data->environmentTorque;
            filteredEnvironmentTorque = received_data->filteredEnvironmentTorque;
            theirToolPos = received_data->cart_pos.template head<3>();
            theirToolQ = received_data->quat;
            remote_gripper_torque.store(static_cast<double>(received_data->gripper_torque));
            remote_gripper_pos.store(static_cast<double>(received_data->gripper_pos));
            remote_gripper_vel.store(static_cast<double>(received_data->gripper_vel));

            // mirror and offset some of the wam joints
            // NOTE: follower does the exact opposite
            for (size_t i = 0; i < DOF; i++) {
                theirJp[i] = (theirJp[i] - config.sync_mapping.offsets[i]) / config.sync_mapping.scales[i];
                theirJv[i] = theirJv[i] / config.sync_mapping.scales[i];
                theirDyngravcompTorque[i] = theirDyngravcompTorque[i] / config.sync_mapping.scales[i];
                environmentTorque[i] = environmentTorque[i] / config.sync_mapping.scales[i];
                filteredEnvironmentTorque[i] = filteredEnvironmentTorque[i] / config.sync_mapping.scales[i];
            }

            theirJPOutputValue->setData(&theirJp);
        } else {
            if (isLinked()) {
                udp_teleop_age = static_cast<double>(now_ns - received_data->timestamp) / 1000000.0;

                std::cout << "lost link with age " << udp_teleop_age << std::endl;
                linked.store(false);
            }
        }

        // cancel a policy
        if (cancel_policy.load() == 1) {
            policy_udp_handler.clearQueueAndPause();
        }

        // inference.
        boost::optional<PolicyReceivedData> policy_data = policy_udp_handler.getLatestPolicyReceived();
        if (policy_data) {
            jp_type clipped_jp;
            clipped_jp << policy_data->jp;
            jp_type clip_val;
            clip_val << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

            bool jp_was_clipped = false;
            std::string clipped_jp_joints_str = "";
            for (size_t i = 0; i < DOF; ++i) {
                double delta = policy_data->jp[i] - wamJP[i];
                bool joint_clipped = false;

                if (delta > clip_val[i]) {
                    clipped_jp[i] = wamJP[i] + clip_val[i];
                    joint_clipped = true;
                } else if (delta < -clip_val[i]) {
                    clipped_jp[i] = wamJP[i] - clip_val[i];
                    joint_clipped = true;
                }

                if (joint_clipped) {
                    jp_was_clipped = true;
                    if (!clipped_jp_joints_str.empty()) clipped_jp_joints_str += ", ";
                    clipped_jp_joints_str += std::to_string(i);
                }
            }

            policyJp << clipped_jp;
            // policy_gripper_cmd.store(static_cast<double>(policy_data->gripper_cmd));
        }
        policyJPOutputValue->setData(&policyJp);

        // Pack outgoing messages
        sendJpMsg << wamJP;
        sendJvMsg << wamJV;

        const cp_type& toolPos  = boost::get<0>(wamTP);
        const Eigen::Quaterniond& toolQ = boost::get<1>(wamTP);

        // extTorqueIn.valueDefined() before setting a reference signal can cause bad feeling teleop
        // also cant put this before the policy read. i have no idea why
        if (dyngravcompTorqueIn.valueDefined()) {
            dyngravcompTorque = dyngravcompTorqueIn.getValue();
        } else {
            dyngravcompTorque.setZero();
        }
        sendDyngravcompTorqueMsg << dyngravcompTorque;

        if (humanTorqueIn.valueDefined()) {
            humanTorque = humanTorqueIn.getValue();
        } else {
            humanTorque.setZero();
        }
        sendHumanTorqueMsg << humanTorque;

        if (filteredHumanTorqueIn.valueDefined()) {
            filteredHumanTorque = filteredHumanTorqueIn.getValue();
        } else {
            filteredHumanTorque.setZero();
        }

        if (policyJtIn.valueDefined()) {
            policyJt = policyJtIn.getValue();
        } else {
            policyJt << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        if (policyTorqueScaleIn.valueDefined()) {
            policyTorqueScale = policyTorqueScaleIn.getValue();
        } else {
            policyTorqueScale << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        // State machine
        if (isLinked()) {
            control = compute_control(
                theirJp, theirJv, environmentTorque,
                wamJP,   wamJV,   humanTorque,
                wamGrav, wamDyn, policyTorqueScale.asDiagonal() * policyJt
            );
            jtOutputValue->setData(&control);
        } else {
            control.setZero();
            jtOutputValue->setData(&control);
        }

        sendHumanTorqueMsg << humanTorque;

        uint64_t loop_start = std::chrono::duration_cast<std::chrono::nanoseconds>(now_op.time_since_epoch()).count();
        auto send_start = std::chrono::steady_clock::now();
        teleop_udp_handler.send(sendJpMsg, sendJvMsg, sendDyngravcompTorqueMsg, sendHumanTorqueMsg, filteredHumanTorque, toolPos, toolQ, policyTorqueScale, static_cast<double>(desired_gripper_pos.load()), static_cast<double>(cancel_policy.load()), loop_start);
        // see how on_leader is used for the magic
        policy_udp_handler.send(theirJp, theirJv, theirDyngravcompTorque, environmentTorque, filteredEnvironmentTorque, sendJpMsg, sendJvMsg, sendDyngravcompTorqueMsg, sendHumanTorqueMsg, filteredHumanTorque, policyJp, policyJt, policyTorqueScale, theirToolPos, theirToolQ, toolPos, toolQ, static_cast<double>(remote_gripper_pos.load()), static_cast<double>(remote_gripper_vel.load()), static_cast<double>(remote_gripper_pos.load()));
        auto send_end = std::chrono::steady_clock::now();
        double send_dt = std::chrono::duration<double, std::milli>(send_end - send_start).count();

        if (++loop_counter % 250 == 0) {
            std::cout << std::fixed << std::setprecision(3);

            // std::cout << "[LEADER] Loop dt: " << loop_dt 
                      // << " ms | UDP teleop Age: " << udp_teleop_age 
                      // << " ms | UDP Send latency: " << send_dt << " ms\n";
               
            // std::cout << "  -> LEADER JP:      [" << sendJpMsg.transpose() << "]\n";
            // std::cout << "  -> FOLLOWER JP:    [" << theirJp.transpose() << "\n";
            // std::cout << "  -> LEADER JV:      [" << sendJvMsg.transpose() << "]\n";
            // std::cout << "  -> FOLLOWER JV:    [" << theirJv.transpose() << "]\n";
            std::cout << "  -> leader Trq:  [" << filteredHumanTorque.transpose() << "]\n";
            std::cout << "  -> follower Trq:[" << filteredEnvironmentTorque.transpose() << "]\n";
            // std::cout << "  -> Leader Tool Pos:  [" << toolPos.transpose() << "]\n";
            // std::cout << "  -> leader Tool Quat: [" << toolQ.w() << " " << toolQ.x() << " " << toolQ.y() << " " << toolQ.z() << "]\n";
            // std::cout << "  -> follower Tool Pos:  [" << theirToolPos.transpose() << "]\n";
            // std::cout << "  -> follower Tool Quat: [" << theirToolQ.w() << " " << theirToolQ.x() << " " << theirToolQ.y() << " " << theirToolQ.z() << "]\n";
            // std::cout << "  -> policy jp:      [" << policyJp.transpose() << "]\n";
            std::cout << "  -> policy jt:      [" << policyJt.transpose() << "]\n";
            std::cout << "  -> policy scale:[" << policyTorqueScale.transpose() << "]\n";
            // std::cout << "  -> human jt:    [" << humanTorque.transpose() << "]\n";
            // std::cout << "  -> leader control: [" << compute_control(theirJp, theirJv, theirTorque, wamJP, wamJV, extTorque, wamGrav, wamDyn, policyTorque) << "]\n\n";
            // std::cout << "  -> dyn: [" << wamDyn.transpose() << "]\n\n";
            // std::cout << "  -> Their wrist:  " << theirWristJp.transpose() << "\n";
            // std::cout << "  -> My wrist:  " << wristJP.transpose() << "\n\n";
            // std::cout << "  -> bumper: [" << bumper.load() << "]\n";
            // std::cout << "  -> trigger: [" << trigger.load() << "]\n";
            // std::cout << "  -> desired_gripper_pos: [" << desired_gripper_pos.load() << "]\n";
            // std::cout << "  -> remote_gripper_pos: [" << remote_gripper_pos.load() << "]\n";
            // std::cout << "  -> remote_gripper_vel: [" << remote_gripper_vel.load() << "]\n";
            // std::cout << "  -> remote_gripper_torque: [" << remote_gripper_torque.load() << "]\n";

            std::cout << std::endl;
        }
    }

    // Peer (arm) state & internal
    jp_type theirJp;
    jv_type theirJv;
    jt_type theirDyngravcompTorque;
    jt_type environmentTorque;
    jt_type filteredEnvironmentTorque;
    cp_type theirToolPos;
    jt_type policyTorqueScale;
    jt_type normalized_ext_torque;
    Eigen::Quaterniond theirToolQ;
    jt_type policyJt;
    jt_type control;
    std::thread io_thread;
    std::atomic<bool> io_running;

    void pollHandle() {
        float local_smoothed_torque = 0.0f;
        while (io_running.load()) {
            handle->poll(); // need to poll sony controller
            if (boost::optional<haptic_wrist::handle_type> opt_handle = handle->getHandle()) {
                haptic_wrist::handle_type handle = *opt_handle;
                bumper.store(handle[0]);
                trigger.store(handle[1]);
                cancel_policy.store(handle[2]); // up button on controller
            }

            float target_position = desired_gripper_pos.load();
            
            // still position controlled. just send to max and min gripper pos
            if (bumper && !trigger) {
                target_position = -1;
            } else if (trigger && !bumper) {
                target_position = 1;
            }

            desired_gripper_pos.store(target_position);

            float remote_torque = remote_gripper_torque.load(); // TODO: do something with this.

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(Leader);

    haptic_wrist::Handle* handle;
    std::mutex state_mutex;
    jp_type joint_positions;
    LeaderUDPHandler<DOF> teleop_udp_handler;
    PolicyUDPHandler<DOF> policy_udp_handler;
    std::atomic<float> policy_gripper_cmd;
    const std::chrono::milliseconds TIMEOUT_DURATION = std::chrono::milliseconds(20);

    jt_type compute_control(const jp_type& ref_pos, const jv_type& ref_vel, const jt_type& ref_extTorque,
                            const jp_type& cur_pos, const jv_type& cur_vel, const jt_type& cur_extTorque,
                            const jt_type& cur_grav, const jt_type& cur_dyn, const jt_type& policyJt) {

        jt_type u1 = 0.0 * cur_extTorque;                        // zero FF (P-P + g-comp only if you add it)
        jt_type u2 = cur_dyn - cur_grav;                          // P-P with dynamic comp (your comment)
        jt_type u3 = -0.5 * ref_extTorque;                        // PF-PF (ref ext torque FF)
        jt_type u4 = -0.1 * ref_extTorque + cur_dyn - cur_grav;   // PF-PF + dyn comp (Lawrence ideal)
        jt_type u5 = -0.5 * ref_extTorque - 0.15 * (ref_extTorque + cur_extTorque);
        jt_type u6 = -0.1 * ref_extTorque - 0.03 * (ref_extTorque + cur_extTorque) + cur_dyn - cur_grav;

        // Follower-different cases (kept for completeness)
        jt_type u7 = -0.5 * cur_extTorque;
        jt_type u8 = -0.25 * (ref_extTorque + cur_extTorque);

        jt_type u = u2;

        // j5-7 does not give a usable ext torque
        for (size_t i = 4; i < 7; ++i) {
            u[i] = 0.0;
        }

        u += policyJt;

        return u;
    };
};
