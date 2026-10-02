#pragma once

#include <boost/asio.hpp>
#include <iostream>
#include <cmath>
#include <atomic>
#include <thread>
#include <chrono>
#include "gripper/gecko/gecko_gripper.h"
#include <iomanip>


#include "udp/follower_udp_handler.h"
#include "udp/udp_policy.h"
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>
#include "utils/teleop_config_loader.h"
#include "utils/utils.h"
#include "utils/data_packets.h"

using namespace gripper::gecko;

template <size_t DOF>
class Follower : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Input<boost::tuple<cp_type, Eigen::Quaterniond>> wamTPIn;
    Input<jt_type> dyngravcompTorqueIn;   // may be undefined
    Input<jt_type> wamGravIn;
    Input<jt_type> wamDynIn;
    Input<jt_type> basePolicyJtIn;
    Input<jt_type> resPolicyJtIn;
    Input<jt_type> refTorquePolicyJtIn;
    Input<jt_type> environmentTorqueIn;
    Input<jt_type> filteredEnvironmentTorqueIn;

    Output<jt_type> wamJTOutput;
    Output<jp_type> theirJPOutput;
    Output<jp_type> basePolicyJpOutput;
    Output<jp_type> resPolicyJpOutput;
    Output<jt_type> refPolicyJtOutput;

    std::atomic<bool> linked;
    
    explicit Follower(barrett::systems::ExecutionManager* em, GeckoGripper* gripper, 
                  const TeleopConfig& config,
                  const std::string& sysName = "Follower")
        : System(sysName)
        , config(config)
        , control(0.0)
        , wamJPIn(this)
        , wamJVIn(this)
        , wamTPIn(this)
        , basePolicyJtIn(this)
        , resPolicyJtIn(this)
        , refTorquePolicyJtIn(this)
        , environmentTorqueIn(this)
        , filteredEnvironmentTorqueIn(this)
        , dyngravcompTorqueIn(this)
        , wamGravIn(this)
        , wamDynIn(this)
        , wamJTOutput(this, &jtOutputValue)
        , theirJPOutput(this, &theirJPOutputValue)
        , basePolicyJpOutput(this, &basePolicyJpOutputValue)
        , resPolicyJpOutput(this, &resPolicyJpOutputValue)
        , refPolicyJtOutput(this, &refPolicyJtOutputValue)
        , teleop_udp_handler(config.network.leader_host, config.network.teleop_recv, config.network.teleop_send,
                             config.data_routing.teleop_send_follower, config.data_routing.teleop_send_leader)
        , policy_udp_handler(config, config.policy.on_follower, config.network.policy_follower_recv,
                             config.data_routing.policy_send_wam)
        , gripper(gripper)
        , target_gripper_pos(0.0f)
        , current_gripper_pos(0.0f)
        , current_gripper_vel(0.0f)
        , current_gripper_torque(0.0f)
        , cancel_policy(0.0f)
        , io_running(false)
        , linked(false) {

        last_op_time = std::chrono::steady_clock::now();

        gripper_max_pos = gripper->getGripperClosePos();
        gripper_min_pos = gripper->getGripperOpenPos();

        if (em != NULL) {
            em->startManaging(*this);
        }
        io_running.store(true);
        io_thread = std::thread(&Follower::pollGripper, this);
    }

    virtual ~Follower() {
        io_running.store(false);
        if (io_thread.joinable()) {
            io_thread.join();
        }
        this->mandatoryCleanUp();
    }

    virtual bool inputsValid() {return true;}

    bool isLinked() const { return linked.load(); }
    void tryLink()  { BARRETT_SCOPED_LOCK(this->getEmMutex()); linked.store(true); }
    void unlink()   { BARRETT_SCOPED_LOCK(this->getEmMutex()); linked.store(false); }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    typename Output<jp_type>::Value* theirJPOutputValue;
    typename Output<jp_type>::Value* basePolicyJpOutputValue;
    typename Output<jp_type>::Value* resPolicyJpOutputValue;
    typename Output<jt_type>::Value* refPolicyJtOutputValue;
    jp_type wamJP;
    jv_type wamJV;
    boost::tuple<cp_type, Eigen::Quaterniond> wamTP;
    jt_type dyngravcompTorque;
    jt_type wamGrav;
    jt_type wamDyn;

    // All cross-thread / packet-bound state lives here and is passed around by
    // reference. See utils/data_packets.h.
    TeleopState<DOF> state;

    TeleopConfig config;
    
    int loop_counter = 0;
    std::chrono::time_point<std::chrono::steady_clock> last_op_time;

    float gripper_max_pos; // assumes 0 is the close pos of the gripper
    float gripper_min_pos;

    virtual void operate() {
        auto now_op = std::chrono::steady_clock::now();
        // double loop_dt = std::chrono::duration<double, std::milli>(now_op - last_op_time).count();
        // last_op_time = now_op;

        wamJP = wamJPIn.getValue();
        wamJV = wamJVIn.getValue();
        wamTP = wamTPIn.getValue();
        wamGrav = wamGravIn.getValue();
        wamDyn = wamDynIn.getValue();

        // One lock for the whole cycle; the shared state is handed to the UDP
        // handlers at the end.
        auto state_lock = state.lock();
        TeleopData<DOF>& st = *state_lock;

        // policy defaults
        st.policyJp << wamJP;
        st.resPolicyJp.setZero();
        st.refPolicyTorque.setZero();
        policy_gripper_cmd.store(0);
        st.policyTorqueScale.setZero();

        // teleop
        boost::optional<TeleopData<DOF>> teleop_data = teleop_udp_handler.getLatestTeleopReceived();
        uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        uint64_t timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(TELEOP_TIMEOUT_DURATION).count();
        double udp_teleop_age = 0.0;
        if (teleop_data && (now_ns >= teleop_data->timestamp) && (now_ns - teleop_data->timestamp <= timeout_ns)) {
            st.leader_jp = teleop_data->leader_jp;
            st.leader_jv = teleop_data->leader_jv;
            st.leader_dyngravcomp_torque = teleop_data->leader_dyngravcomp_torque;
            st.human_torque = teleop_data->human_torque;
            st.filtered_human_torque = teleop_data->filtered_human_torque;
            st.leader_cart_pos = teleop_data->leader_cart_pos;
            st.leader_quat = teleop_data->leader_quat;
            st.policyTorqueScale << teleop_data->policyTorqueScale;
            target_gripper_pos.store(static_cast<double>(teleop_data->gripper_cmd));
            cancel_policy.store(static_cast<double>(teleop_data->cancel_policy));

            // mirror and offset some of the wam joints
            for (size_t i = 0; i < DOF; i++) {
                st.leader_jp[i] = st.leader_jp[i] * config.sync_mapping.scales[i] + config.sync_mapping.offsets[i];
                st.leader_jv[i] = st.leader_jv[i] * config.sync_mapping.scales[i];
                st.leader_dyngravcomp_torque[i] = st.leader_dyngravcomp_torque[i] * config.sync_mapping.scales[i];
                st.human_torque[i] = st.human_torque[i] * config.sync_mapping.scales[i];
                st.filtered_human_torque[i] = st.filtered_human_torque[i] * config.sync_mapping.scales[i];
            }

            theirJPOutputValue->setData(&st.leader_jp);
        } else {
            if (isLinked()) {
                udp_teleop_age = static_cast<double>(now_ns - teleop_data->timestamp) / 1000000.0;

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
            st.policyJp << policy_data->base_policy_jp;
            st.resPolicyJp << policy_data->res_policy_jp;
            st.refPolicyTorque << policy_data->ref_torque;
            policy_gripper_cmd.store(static_cast<double>(policy_data->gripper_cmd));
        }
        basePolicyJpOutputValue->setData(&st.policyJp);
        resPolicyJpOutputValue->setData(&st.resPolicyJp);
        refPolicyJtOutputValue->setData(&st.refPolicyTorque);

        st.follower_jp << wamJP;
        st.follower_jv << wamJV;

        st.follower_cart_pos = boost::get<0>(wamTP);
        st.follower_quat     = boost::get<1>(wamTP);

        // extTorqueIn.valueDefined() before setting a reference signal can cause bad feeling teleop
        // also cant put this before the policy read. i have no idea why
        if (dyngravcompTorqueIn.valueDefined()) {
            dyngravcompTorque = dyngravcompTorqueIn.getValue();
        } else {
            dyngravcompTorque.setZero();
        }
        st.follower_dyngravcomp_torque << dyngravcompTorque;

        if (environmentTorqueIn.valueDefined()) {
            st.environment_torque = environmentTorqueIn.getValue();
        } else {
            st.environment_torque.setZero();
        }

        if (filteredEnvironmentTorqueIn.valueDefined()) {
            st.filtered_environment_torque = filteredEnvironmentTorqueIn.getValue();
        } else {
            st.filtered_environment_torque.setZero();
        }

        // impedance results
        if (basePolicyJtIn.valueDefined()) {
            st.policyJt = basePolicyJtIn.getValue();
        } else {
            st.policyJt << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        if (resPolicyJtIn.valueDefined()) {
            st.resPolicyJt = resPolicyJtIn.getValue();
        } else {
            st.resPolicyJt << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        if (refTorquePolicyJtIn.valueDefined()) {
            st.refTorquePolicyJt = refTorquePolicyJtIn.getValue();
        } else {
            st.refTorquePolicyJt << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        jt_type zero_torque;
        zero_torque.setZero();

        if (isLinked()) {
            if (config.policy.type == "dg") {
                // apply policy torque as feed forward
                control = compute_control(
                    st.leader_jp, st.leader_jv, st.human_torque,
                    wamJP,   wamJV,   st.environment_torque,
                    wamGrav, wamDyn, st.policyTorqueScale.asDiagonal() * st.policyJt,
                    zero_torque, st.refPolicyTorque
                );

            } else if (config.policy.type == "cr") {
                // treat policy torque as a desired torque and apply a pid towards it
                control = compute_control(
                    st.leader_jp, st.leader_jv, st.environment_torque,
                    wamJP,   wamJV,   st.human_torque,
                    wamGrav, wamDyn, st.policyTorqueScale.asDiagonal() * st.policyJt,
                    st.resPolicyJt, st.refTorquePolicyJt
                );
            } else if (config.policy.type == "base") {
                control = compute_control(
                    st.leader_jp, st.leader_jv, st.human_torque,
                    wamJP,   wamJV,   st.environment_torque,
                    wamGrav, wamDyn, st.policyTorqueScale.asDiagonal() * st.policyJt,
                    zero_torque, zero_torque
                );
            }
            jtOutputValue->setData(&control);
        } else {
            control.setZero();
            jtOutputValue->setData(&control);
        }

        uint64_t loop_start = std::chrono::duration_cast<std::chrono::nanoseconds>(now_op.time_since_epoch()).count();
        st.timestamp = loop_start;
        // auto send_start = std::chrono::steady_clock::now();
        // Snapshot the gripper scalars into the shared state.
        st.gripper_pos    = static_cast<double>(current_gripper_pos.load());
        st.gripper_vel    = static_cast<double>(current_gripper_vel.load());
        st.gripper_torque = static_cast<double>(current_gripper_torque.load());
        // send to leader then send to policy; packet contents are config-driven.
        teleop_udp_handler.send(st);
        // see how on_follower is used for the magic
        policy_udp_handler.send(st);

        // auto send_end = std::chrono::steady_clock::now();
        // double send_dt = std::chrono::duration<double, std::milli>(send_end - send_start).count();

        if (++loop_counter % 250 == 0) {
            std::cout << std::fixed << std::setprecision(3);

            // std::cout << "[FOLLOWER] Loop dt: " << loop_dt
                      // << " ms | UDP Rx Age: " << udp_rx_age 
                      // << " ms | UDP Send latency: " << send_dt << " ms\n";

            // std::cout << "  -> FOLLOWER JP:      [" << sendJpMsg.transpose() << "]\n";
            // std::cout << "  -> LEADER JP:    [" << theirJp.transpose() << "\n";
            // std::cout << "  -> FOLLOWER JV:      [" << sendJvMsg.transpose() << "]\n";
            // std::cout << "  -> LEADER JV:    [" << theirJv.transpose() << "]\n";
            // std::cout << "  -> FOLLOWER Trq:  [" << environmentTorque.transpose() << "]\n";
            // std::cout << "  -> leader Trq:[" << humanTorque.transpose() << "]\n";
            // std::cout << "  -> follower Tool Pos:  [" << toolPos.transpose() << "]\n";
            // std::cout << "  -> follower Tool Quat: [" << toolQ.w() << " " << toolQ.x() << " " << toolQ.y() << " " << toolQ.z() << "]\n";
            // std::cout << "  -> leader Tool Pos:  [" << theirToolPos.transpose() << "]\n";
            // std::cout << "  -> leader Tool Quat: [" << theirToolQ.w() << " " << theirToolQ.x() << " " << theirToolQ.y() << " " << theirToolQ.z() << "]\n";
            // std::cout << "  -> policy:        [" << policyJt.transpose() << "]\n";
            // std::cout << "  -> control: [" << compute_control(theirJp, theirJv, theirExtTorque, wamJP, wamJV, extTorque, wamGrav, wamDyn, policyJt) << "]\n";
            // std::cout << "  -> dyn: [" << wamDyn.transpose() << "]\n";
            // std::cout << "  -> TX GrpTrq:  " << current_gripper_torque.load() << "\n";
            // std::cout << "  -> TX GrpPos:  " << current_gripper_pos.load() << "\n";
            // std::cout << "  -> grip pos:  " << current_gripper_pos.load() << "\n";
            // std::cout << "  -> grip vel:  " << current_gripper_vel.load() << "\n";
            // std::cout << "  -> grip toq:  " << current_gripper_torque.load() << "\n";
            // std::cout << "  -> P JP:      [" << policyJp.transpose() << "]\n";
            // std::cout << "  -> base p JP:      [" << basePolicyJp.transpose() << "]\n";
            // std::cout << "  -> ff p toq:      [" << ffPolicyTorque.transpose() << "]\n";
            // std::cout << "  -> P G:      [" << policy_gripper_cmd.load() << "]\n";
            // std::cout << "  -> L G:      [" << target_gripper_pos.load() << "]\n";

            // std::cout << std::endl;
        }
    }

    jt_type control;

    void pollGripper() {
        while (io_running.load()) {
            float local_usr_gripper_pos = target_gripper_pos.load();
            float local_policy_gripper_cmd = policy_gripper_cmd.load();
            // operator can command an override
            if (policy_gripper_cmd != 0) {
                gripper->setPosition(local_policy_gripper_cmd);
            } else {
                gripper->setPosition(local_usr_gripper_pos);
            }
            gripper->controlLoopCallback();

            GripperState gripper_state = gripper->getLatestState();
            current_gripper_pos.store(gripper_state.position);
            current_gripper_vel.store(gripper_state.velocity);
            current_gripper_torque.store(gripper_state.torque);

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        gripper->setVelocity(0.0f);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(Follower);
    std::mutex state_mutex;
    FollowerUDPHandler<DOF> teleop_udp_handler;
    PolicyUDPHandler<DOF> policy_udp_handler;
    const std::chrono::milliseconds TELEOP_TIMEOUT_DURATION = std::chrono::milliseconds(20); // this needs to be larger than 2ms becuse of warmup when starting other programs

    GeckoGripper* gripper;
    std::thread io_thread;
    std::atomic<bool> io_running;
    std::atomic<float> target_gripper_pos;
    std::atomic<float> policy_gripper_cmd;
    std::atomic<float> current_gripper_pos;
    std::atomic<float> current_gripper_vel;
    std::atomic<float> current_gripper_torque;
    std::atomic<float> cancel_policy;

    jt_type compute_control(const jp_type& ref_pos, const jv_type& ref_vel, const jt_type& ref_extTorque,
                            const jp_type& cur_pos, const jv_type& cur_vel, const jt_type& cur_extTorque,
                            const jt_type& cur_grav, const jt_type& cur_dyn, const jt_type& basePolicyJt, const jt_type& resPolicyJt, const jt_type& refTorquePolicyJt) {
        
        // cases where the follower and leader have the same control law

        jt_type u1 = 0.0 * cur_extTorque; // zero feedforward (equal to default P-P with gravity compensation)

        jt_type u2 = cur_dyn - cur_grav; // P-P with dynamic compensation

        jt_type u3 = -0.5 * ref_extTorque; // PF-PF with ref external torque feedback

        jt_type u4 = -0.5 * ref_extTorque + cur_dyn - cur_grav; // PF-PF with ref external torque feedback and dynamic compensation (Lawrence's perfect transparency architecture);


        jt_type u5 = -0.5 * ref_extTorque -0.15 * (ref_extTorque + cur_extTorque); // PF-PF with ref external torque and cur external torque feedback

        jt_type u6 = -0.1 * ref_extTorque -0.03 * (ref_extTorque + cur_extTorque) + cur_dyn - cur_grav; // it has the best performance


        // cases that the leader side has differnt controller that the follower

        jt_type u7 = -0.0 * cur_extTorque; // zero

        jt_type u8 = -0.0 * (ref_extTorque + cur_extTorque); // zero

        // jt_type u9 = -0.0 * cur_extTorque - 0.0 * (ref_extTorque + cur_extTorque);

        jt_type u9 = -0.5 * ref_extTorque; // PF-PF with ref external torque as feedback

        jt_type u10 = -0.5 * ref_extTorque;

        jt_type u = u2;


        // j5-7 does not give a usable ext torque
        for (size_t i = 4; i < 7; ++i) {
            u[i] = 0.0;
        }

        u += basePolicyJt;

        u += resPolicyJt;

        u += refTorquePolicyJt;

        return u;
    };
};

