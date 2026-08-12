#pragma once

#include <boost/asio.hpp>
#include <iostream>
#include <cmath>
#include <atomic>
#include <thread>
#include <chrono>
#include "gripper/gecko/gecko_gripper.h"
#include <iomanip>


#include "follower_udp_handler.h"
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>
#include "teleop_config_loader.h"
#include "utils.h"

using namespace gripper::gecko;

template <size_t DOF>
class Follower : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Input<boost::tuple<cp_type, Eigen::Quaterniond>> wamTPIn;
    Input<jt_type> extTorqueIn;
    Input<jt_type> wamGravIn;
    Input<jt_type> wamDynIn;
    Input<jt_type> policyJtIn;
    Output<jt_type> wamJTOutput;
    Output<jp_type> theirJPOutput;
    Output<jv_type> theirJVOutput;
    Output<jt_type> theirExtTorqueOutput;
    Output<jp_type> policyJpOutput;
    Output<jt_type> policyTorqueScaleOutput;

    std::atomic<bool> linked;
    
    explicit Follower(barrett::systems::ExecutionManager* em, GeckoGripper* gripper, 
                  const TeleopConfig& config,
                  const std::string& sysName = "Follower")
        : System(sysName)
        , config(config)
        , theirJp(0.0)
        , theirJv(0.0)
        , theirExtTorque(0.0)
        , theirToolPos(0.0)
        , theirToolQ(1, 0, 0, 0)
        , control(0.0)
        , wamJPIn(this)
        , wamJVIn(this)
        , wamTPIn(this)
        , policyJtIn(this)
        , extTorqueIn(this)
        , wamGravIn(this)
        , wamDynIn(this)
        , wamJTOutput(this, &jtOutputValue)
        , theirJPOutput(this, &theirJPOutputValue)
        , theirJVOutput(this, &theirJVOutputValue)
        , theirExtTorqueOutput(this, &theirExtTorqueOutputValue)
        , policyJpOutput(this, &policyJpOutputValue)
        , policyTorqueScaleOutput(this, &policyTorqueScaleOutputValue)
        , teleop_udp_handler(config.network.leader_host, config.network.teleop_recv, config.network.teleop_send)
        , policy_udp_handler(config.policy.on_follower, config.network.policy_host, config.network.policy_send, config.network.policy_follower_recv)
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
    typename Output<jv_type>::Value* theirJVOutputValue;
    typename Output<jt_type>::Value* theirExtTorqueOutputValue;
    typename Output<jp_type>::Value* policyJpOutputValue;
    typename Output<jt_type>::Value* policyTorqueScaleOutputValue;
    jp_type wamJP;
    jv_type wamJV;
    boost::tuple<cp_type, Eigen::Quaterniond> wamTP;
    jt_type extTorque;
    jt_type wamGrav;
    jt_type wamDyn;
    jt_type policyJt;
    jt_type policyTorqueScale;
    jp_type policyJp;
    Eigen::Matrix<double, DOF, 1> sendJpMsg;
    Eigen::Matrix<double, DOF, 1> sendJvMsg;
    Eigen::Matrix<double, DOF, 1> sendExtTorqueMsg;

    TeleopConfig config;
    
    int loop_counter = 0;
    std::chrono::time_point<std::chrono::steady_clock> last_op_time;

    float gripper_max_pos; // assumes 0 is the close pos of the gripper
    float gripper_min_pos;

    using TeleopReceivedData = typename FollowerUDPHandler<DOF>::TeleopReceivedData;

    virtual void operate() {
        auto now_op = std::chrono::steady_clock::now();
        // double loop_dt = std::chrono::duration<double, std::milli>(now_op - last_op_time).count();
        // last_op_time = now_op;

        wamJP = wamJPIn.getValue();
        wamJV = wamJVIn.getValue();
        wamTP = wamTPIn.getValue();
        wamGrav = wamGravIn.getValue();
        wamDyn = wamDynIn.getValue();

        // policy defaults
        policyJp << wamJP;
        policy_gripper_cmd.store(current_gripper_pos.load());
        policyTorqueScale.setZero();

        sendJpMsg << wamJP;
        sendJvMsg << wamJV;


        const cp_type& toolPos  = boost::get<0>(wamTP);
        const Eigen::Quaterniond& toolQ = boost::get<1>(wamTP);

        // teleop
        boost::optional<TeleopReceivedData> teleop_data = teleop_udp_handler.getLatestTeleopReceived();
        uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        uint64_t timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(TELEOP_TIMEOUT_DURATION).count();
        double udp_teleop_age = 0.0;
        if (teleop_data && (now_ns >= teleop_data->timestamp) && (now_ns - teleop_data->timestamp <= timeout_ns)) {
            theirJp = teleop_data->jp;
            theirJv = teleop_data->jv;
            theirExtTorque = teleop_data->extTorque;
            theirToolPos = teleop_data->cart_pos.template head<3>();
            theirToolQ = teleop_data->quat;
            policyTorqueScale << teleop_data->policyTorqueScale;
            target_gripper_pos.store(static_cast<double>(teleop_data->gripper_cmd));
            cancel_policy.store(static_cast<double>(teleop_data->cancel_policy));

            // mirror and offset some of the wam joints
            for (size_t i = 0; i < DOF; i++) {
                theirJp[i] = theirJp[i] * config.sync_mapping.scales[i] + config.sync_mapping.offsets[i];
                theirJv[i] = theirJv[i] * config.sync_mapping.scales[i];
                theirExtTorque[i] = theirExtTorque[i] * config.sync_mapping.scales[i];
            }

            theirJPOutputValue->setData(&theirJp);
            theirJVOutputValue->setData(&theirJv);
            theirExtTorqueOutputValue->setData(&theirExtTorque);
            policyTorqueScaleOutputValue->setData(&policyTorqueScale);
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
            policy_gripper_cmd.store(static_cast<double>(policy_data->gripper_cmd));
        }
        policyJpOutputValue->setData(&policyJp);

        // extTorqueIn.valueDefined() before setting a reference signal can cause bad feeling teleop
        if (extTorqueIn.valueDefined()) {
            extTorque = extTorqueIn.getValue();
        } else {
            extTorque.setZero();
        }
        sendExtTorqueMsg << extTorque;


        if (policyJtIn.valueDefined()) {
            policyJt = policyJtIn.getValue();
        } else {
            policyJt << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        if (isLinked()) {
            control = compute_control(theirJp, theirJv, theirExtTorque, wamJP, wamJV, extTorque, wamGrav, wamDyn, policyTorqueScale.asDiagonal() * policyJt);
            jtOutputValue->setData(&control);
        } else {
            control.setZero();
            jtOutputValue->setData(&control);
        }

        uint64_t loop_start = std::chrono::duration_cast<std::chrono::nanoseconds>(now_op.time_since_epoch()).count();
        // auto send_start = std::chrono::steady_clock::now();
        // send to leader then send to policy
        teleop_udp_handler.send(sendJpMsg, sendJvMsg, sendExtTorqueMsg, toolPos, toolQ, static_cast<double>(current_gripper_torque.load()), static_cast<double>(current_gripper_pos.load()), static_cast<double>(current_gripper_vel.load()), loop_start);
        // see how on_follower is used for the magic
        policy_udp_handler.send(sendJpMsg, sendJvMsg, sendExtTorqueMsg, theirJp, theirJv, theirExtTorque, policyTorqueScale, policyJt, toolPos, toolQ, theirToolPos, theirToolQ, static_cast<double>(current_gripper_pos.load()), static_cast<double>(current_gripper_vel.load()), static_cast<double>(current_gripper_torque.load()), loop_start);

        // auto send_end = std::chrono::steady_clock::now();
        // double send_dt = std::chrono::duration<double, std::milli>(send_end - send_start).count();

        if (++loop_counter % 50 == 0) {
            std::cout << std::fixed << std::setprecision(3);

            // std::cout << "[FOLLOWER] Loop dt: " << loop_dt
                      // << " ms | UDP Rx Age: " << udp_rx_age 
                      // << " ms | UDP Send latency: " << send_dt << " ms\n";

            // std::cout << "  -> FOLLOWER JP:      [" << sendJpMsg.transpose() << "]\n";
            // std::cout << "  -> LEADER JP:    [" << theirJp.transpose() << "\n";
            // std::cout << "  -> FOLLOWER JV:      [" << sendJvMsg.transpose() << "]\n";
            // std::cout << "  -> LEADER JV:    [" << theirJv.transpose() << "]\n";
            std::cout << "  -> FOLLOWER ExtTrq:  [" << sendExtTorqueMsg.transpose() << "]\n";
            std::cout << "  -> leader ExtTrq:[" << theirExtTorque.transpose() << "]\n";
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
            // std::cout << "  -> P G:      [" << policy_gripper_pos.load() << "]\n";

            std::cout << std::endl;
        }
    }

    jp_type theirJp;
    jv_type theirJv;
    jt_type theirExtTorque;
    cp_type theirToolPos;
    Eigen::Quaterniond theirToolQ;
    jt_type control;

    void pollGripper() {
        while (io_running.load()) {
            float local_usr_gripper_pos = target_gripper_pos.load();
            float local_policy_gripper_cmd = policy_gripper_cmd.load();
            // operator can command an override
            gripper->setPosition(local_usr_gripper_pos);
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
                            const jt_type& cur_grav, const jt_type& cur_dyn, const jt_type& policyJt) {
        
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

        jt_type u = u2;


        // j5-7 does not give a usable ext torque
        for (size_t i = 4; i < 7; ++i) {
            u[i] = 0.0;
        }

        u += policyJt;

        return u;
    };
};

