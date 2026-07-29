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
    Output<jp_type> wamJPOutput;
    Output<jt_type> wamJTOutput;
    Output<jp_type> theirJPOutput;
    Output<jv_type> theirJVOutput;
    Output<jt_type> theirExtTorqueOutput;
    Output<jp_type> policyJpOutput;

    std::atomic<bool> linked;
    std::atomic<bool> inference_enabled;
    

    explicit Follower(barrett::systems::ExecutionManager* em, GeckoGripper* gripper, 
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
        , wamTPIn(this)
        , policyJtIn(this)
        , extTorqueIn(this)
        , wamGravIn(this)
        , wamDynIn(this)
        , wamJPOutput(this, &jpOutputValue)
        , wamJTOutput(this, &jtOutputValue)
        , theirJPOutput(this, &theirJPOutputValue)
        , theirJVOutput(this, &theirJVOutputValue)
        , theirExtTorqueOutput(this, &theirExtTorqueOutputValue)
        , policyJpOutput(this, &policyJpOutputValue)
        , udp_handler(config.network.leader_host, config.network.teleop_recv, config.network.teleop_send, 
                      config.network.policy_send_active, config.network.policy_host, config.network.policy_send, config.network.policy_recv)
        , gripper(gripper)
        , target_gripper_vel(0.0f)
        , current_gripper_pos(0.0f)
        , current_gripper_vel(0.0f)
        , current_gripper_torque(0.0f)
        , io_running(false)
        , linked(false)
        , inference_enabled (false) {

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

    bool isInference() const { return inference_enabled.load(); }
    void enableInference()  {
        {
            BARRETT_SCOPED_LOCK(this->getEmMutex());
            inference_enabled.store(true);
        }
    }
    void disableInference() {
        {
            BARRETT_SCOPED_LOCK(this->getEmMutex());
            inference_enabled.store(false);
        }
    }

  protected:
    typename Output<jp_type>::Value* jpOutputValue;
    typename Output<jt_type>::Value* jtOutputValue;
    typename Output<jp_type>::Value* theirJPOutputValue;
    typename Output<jv_type>::Value* theirJVOutputValue;
    typename Output<jt_type>::Value* theirExtTorqueOutputValue;
    typename Output<jp_type>::Value* policyJpOutputValue;
    jp_type wamJP;
    jv_type wamJV;
    boost::tuple<cp_type, Eigen::Quaterniond> wamTP;
    jt_type extTorque;
    jt_type wamGrav;
    jt_type wamDyn;
    jt_type policyJt;
    jp_type policyJp;
    jv_type policyJv;
    ja_type policyJa;
    Eigen::Matrix<double, DOF, 1> sendJpMsg;
    Eigen::Matrix<double, DOF, 1> sendJvMsg;
    Eigen::Matrix<double, DOF, 1> sendExtTorqueMsg;

    TeleopConfig config;
    
    int loop_counter = 0;
    std::chrono::time_point<std::chrono::steady_clock> last_op_time;

    float gripper_max_pos; // assumes 0 is the close pos of the gripper
    float gripper_min_pos;

    using TeleopReceivedData = typename FollowerUDPHandler<DOF>::TeleopReceivedData;

    // static std::ofstream& trajectoryLogFile() {
    //     static std::ofstream log_file;
    //     static bool header_written = false;
    //     if (!log_file.is_open()) {
    //         log_file.open("trajectory_log.csv", std::ios::out | std::ios::app);
    //         if (log_file.is_open() && !header_written) {
    //             log_file << "timestamp_ns";
    //             for (size_t i = 0; i < DOF; ++i) log_file << ",jp" << i;
    //             for (size_t i = 0; i < DOF; ++i) log_file << ",jv" << i;
    //             for (size_t i = 0; i < DOF; ++i) log_file << ",ja" << i;
    //             log_file << ",jp_clipped,jv_clipped,ja_clipped"
    //                       << ",clipped_jp_joints,clipped_jv_joints,clipped_ja_joints"
    //                       << "\n";
    //             header_written = true;
    //         }
    //     }
    //     return log_file;
    // }

    virtual void operate() {
        auto now_op = std::chrono::steady_clock::now();
        double loop_dt = std::chrono::duration<double, std::milli>(now_op - last_op_time).count();
        last_op_time = now_op;

        wamJP = wamJPIn.getValue();
        wamJV = wamJVIn.getValue();
        wamTP = wamTPIn.getValue();
        wamGrav = wamGravIn.getValue();
        wamDyn = wamDynIn.getValue();

        if (policyJtIn.valueDefined()) {
            policyJt = policyJtIn.getValue();
        } else {
            policyJt << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }


        // policy defaults
        policyJp << wamJP;
        policyJv << wamJV;
        policyJa << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        sendJpMsg << wamJP;
        sendJvMsg << wamJV;


        const cp_type& toolPos  = boost::get<0>(wamTP);
        const Eigen::Quaterniond& toolQ = boost::get<1>(wamTP);

        // policyJtScale.setZero();

        // teleop
        boost::optional<TeleopReceivedData> teleop_data = udp_handler.getLatestTeleopReceived();
        uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        uint64_t timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(TELEOP_TIMEOUT_DURATION).count();
        double udp_teleop_age = 0.0;
        if (teleop_data && (now_ns >= teleop_data->timestamp) && (now_ns - teleop_data->timestamp <= timeout_ns)) {
            theirJp = teleop_data->jp;
            theirJv = teleop_data->jv;
            theirExtTorque = teleop_data->extTorque;
            target_gripper_vel.store(static_cast<double>(teleop_data->gripper_cmd));

            // mirror and offset some of the wam joints
            for (size_t i = 0; i < DOF; i++) {
                theirJp[i] = theirJp[i] * config.sync_mapping.scales[i] + config.sync_mapping.offsets[i];
                theirJv[i] = theirJv[i] * config.sync_mapping.scales[i];
                theirExtTorque[i] = theirExtTorque[i] * config.sync_mapping.scales[i];
            }

            theirJPOutputValue->setData(&theirJp);
            theirJVOutputValue->setData(&theirJv);
            theirExtTorqueOutputValue->setData(&theirExtTorque);

            // help with free motion
            policyJp << theirJp;
            policyJv << theirJv;
            policyJa << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        } else {
            if (isLinked()) {
                udp_teleop_age = static_cast<double>(now_ns - teleop_data->timestamp) / 1000000.0;

                std::cout << "lost link with age " << udp_teleop_age << std::endl;
                linked.store(false);
            }
        }

        // inference
        if (isInference()) {
            boost::optional<PolicyReceivedData> policy_data = udp_handler.getLatestPolicyReceived();
            now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
            double udp_policy_age = 0.0;
            if (policy_data) {
                jt_type max_torques;
                max_torques << 3, 3, 3, 3, 1, 1, 1;

                jt_type normalized_ext_torque;
                for (size_t i = 0; i < DOF; ++i) {
                    // if (extTorque[i] < 1) {
                    //     policyJtScale[i] = 0.2;
                    // } else {
                    //     policyJtScale[i] = 1.0;
                    // }
                    normalized_ext_torque[i] = max_torques[i] / std::abs(theirExtTorque[i]+policyJt[i]);
                }

                // this should output something roughly between 0.2 and 1 which i will use to scale my policy control
                // for (size_t i = 0; i < DOF; ++i) {
                //     policyJtScale[i] = 1.0 / (1.0 + std::exp(1.0 * (-normalized_ext_torque[i] + 0.5)));
                // }

                jp_type clipped_jp;
                clipped_jp << policy_data->jp;
                jp_type clip_val;
                clip_val << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

                bool jp_was_clipped = false;
                std::string clipped_jp_joints_str = "";
                // for (size_t i = 0; i < DOF; ++i) {
                //     double delta = policy_data->jp[i] - wamJP[i];
                //     bool joint_clipped = false;
                //
                //     if (delta > clip_val[i]) {
                //         clipped_jp[i] = wamJP[i] + clip_val[i];
                //         joint_clipped = true;
                //     } else if (delta < -clip_val[i]) {
                //         clipped_jp[i] = wamJP[i] - clip_val[i];
                //         joint_clipped = true;
                //     }
                //
                //     if (joint_clipped) {
                //         was_clipped = true;
                //         if (!clipped_joints_str.empty()) clipped_joints_str += ", ";
                //         clipped_joints_str += std::to_string(i);
                //     }
                // }
                for (size_t i = 0; i < DOF; ++i) {
                    double delta = policy_data->jp[i] - theirJp[i];
                    bool joint_clipped = false;

                    if (delta > clip_val[i]) {
                        clipped_jp[i] = theirJp[i] + clip_val[i];
                        joint_clipped = true;
                    } else if (delta < -clip_val[i]) {
                        clipped_jp[i] = theirJp[i] - clip_val[i];
                        joint_clipped = true;
                    }

                    if (joint_clipped) {
                        jp_was_clipped = true;
                        if (!clipped_jp_joints_str.empty()) clipped_jp_joints_str += ", ";
                        clipped_jp_joints_str += std::to_string(i);
                    }
                }

                jp_type clipped_jv;
                clipped_jv << policy_data->jv;
                jp_type max_jv;
                max_jv << 2.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0; // rad/s, tune per joint

                bool jv_was_clipped = false;
                std::string clipped_jv_joints_str = "";
                for (size_t i = 0; i < DOF; ++i) {
                    bool joint_clipped = false;
                    if (clipped_jv[i] > max_jv[i]) {
                        clipped_jv[i] = max_jv[i];
                        joint_clipped = true;
                    } else if (clipped_jv[i] < -max_jv[i]) {
                        clipped_jv[i] = -max_jv[i];
                        joint_clipped = true;
                    }
                    if (joint_clipped) {
                        jv_was_clipped = true;
                        if (!clipped_jv_joints_str.empty()) clipped_jv_joints_str += ";";
                        clipped_jv_joints_str += std::to_string(i);
                    }
                }

                jp_type clipped_ja;
                clipped_ja << policy_data->ja;
                jp_type max_ja;
                max_ja << 8.0, 8.0, 8.0, 8.0, 10.0, 10.0, 10.0;

                bool ja_was_clipped = false;
                std::string clipped_ja_joints_str = "";
                for (size_t i = 0; i < DOF; ++i) {
                    bool joint_clipped = false;
                    if (clipped_ja[i] > max_ja[i]) {
                        clipped_ja[i] = max_ja[i];
                        joint_clipped = true;
                    } else if (clipped_ja[i] < -max_ja[i]) {
                        clipped_ja[i] = -max_ja[i];
                        joint_clipped = true;
                    }
                    if (joint_clipped) {
                        ja_was_clipped = true;
                        if (!clipped_ja_joints_str.empty()) clipped_ja_joints_str += ";";
                        clipped_ja_joints_str += std::to_string(i);
                    }
                }


                // std::cout << (was_clipped ? "  CLIPPED " : "  EXECUTING ") << clipped_jp.transpose() << " | diff: " << static_cast<uint64_t>(now_ns - policy_data->timestamp) / 1e9 << std::endl;
                // std::cout << (jp_was_clipped ? "  CLIPPED [joints: " + clipped_jp_joints_str + "] " : "  EXECUTING ") << clipped_jp.transpose() << std::endl;

                // {
                //     std::ofstream& log_file = trajectoryLogFile();
                //     if (log_file.is_open()) {
                //         log_file << policy_data->timestamp;
                //         for (size_t i = 0; i < DOF; ++i) log_file << "," << clipped_jp[i];
                //         for (size_t i = 0; i < DOF; ++i) log_file << "," << clipped_jv[i];
                //         for (size_t i = 0; i < DOF; ++i) log_file << "," << clipped_ja[i];
                //         log_file << "," << (jp_was_clipped ? 1 : 0)
                //                  << "," << (jv_was_clipped ? 1 : 0)
                //                  << "," << (ja_was_clipped ? 1 : 0)
                //                  << "," << clipped_jp_joints_str
                //                  << "," << clipped_jv_joints_str
                //                  << "," << clipped_ja_joints_str
                //                  << "\n";
                //         // flush() is somewhat expensive; see caveat below re: RT loop timing.
                //         log_file.flush();
                //     }
                // }

                policyJp << clipped_jp;
                policyJv << clipped_jv;
                policyJa << clipped_ja;
                policy_gripper_cmd.store(static_cast<double>(policy_data->gripper_cmd));
            } else {
                // udp_policy_age = static_cast<double>(now_ns - policy_data->timestamp) / 1000000.0;
                // std::cout << "No action for policy " << policy_data->jp.transpose() << ". diff: " << udp_policy_age << " | " << now_ns << " | " << policy_data->timestamp << std::endl;
            }
        }

        // extTorqueIn.valueDefined() before setting a reference signal can cause bad feeling teleop
        if (extTorqueIn.valueDefined()) {
            extTorque = extTorqueIn.getValue();
        } else {
            extTorque.setZero();
        }
        sendExtTorqueMsg << extTorque;


        if (isLinked() && isInference()) { // shared control
            control = compute_teleop_control(theirJp, theirJv, theirExtTorque, wamJP, wamJV, extTorque, wamGrav, wamDyn);
            jtOutputValue->setData(&control);
        } else if (isLinked()) { // teleop only
            control = compute_teleop_control(theirJp, theirJv, theirExtTorque, wamJP, wamJV, extTorque, wamGrav, wamDyn);
            jtOutputValue->setData(&control);
        } else if (isInference()) { // inference only
            control.setZero();
            jtOutputValue->setData(&control);
        } else {
            control.setZero();
            jtOutputValue->setData(&control);
        }

        policyJpOutputValue->setData(&policyJp);

        jpOutputValue->setData(&wamJP);
        // policyJtScaleOutputValue->setData(&policyJtScale);

        uint64_t loop_start = std::chrono::duration_cast<std::chrono::nanoseconds>(now_op.time_since_epoch()).count();
        auto send_start = std::chrono::steady_clock::now();
        // send to leader then send to policy
        udp_handler.send(sendJpMsg, sendJvMsg, sendExtTorqueMsg, policyJt, static_cast<double>(current_gripper_torque.load()), loop_start);
        udp_handler.sendToPolicy(sendJpMsg, sendJvMsg, sendExtTorqueMsg, theirJp, theirJv, theirExtTorque, toolPos, toolQ, static_cast<double>(current_gripper_pos.load()), static_cast<double>(current_gripper_vel.load()), static_cast<double>(current_gripper_torque.load()), loop_start);
        auto send_end = std::chrono::steady_clock::now();
        double send_dt = std::chrono::duration<double, std::milli>(send_end - send_start).count();

        if (++loop_counter % 50 == 0) {
            std::cout << std::fixed << std::setprecision(3);

            // std::cout << "[FOLLOWER] Loop dt: " << loop_dt
                      // << " ms | UDP Rx Age: " << udp_rx_age 
                      // << " ms | UDP Send latency: " << send_dt << " ms\n";

            // std::cout << "  -> FOLLOWER JP:      [" << sendJpMsg.transpose() << "]\n";
            // std::cout << "  -> LEADER JP:  " << theirJp.transpose() << "\n\n";
            // std::cout << "  -> TX JV:      [" << sendJvMsg.transpose() << "]\n";
            // std::cout << "  -> FOLLOWER EXT TOQ:  [" << sendExtTorqueMsg.transpose() << "]\n";
            // std::cout << "  -> LEADER EXT TOQ:  " << theirExtTorque.transpose() << "\n";
            // std::cout << "  -> control: [" << compute_teleop_control(theirJp, theirJv, theirExtTorque, wamJP, wamJV, extTorque, wamGrav, wamDyn) << "]\n";
            // std::cout << "  -> applied: [" << (sendExtTorqueMsg + compute_teleop_control(theirJp, theirJv, theirExtTorque, wamJP, wamJV, extTorque, wamGrav, wamDyn)).transpose() << "]\n\n";
            // std::cout << "  -> dyn: [" << wamDyn.transpose() << "]\n\n";
            // std::cout << "  -> inf: [" << isInference() << "]\n";
            // std::cout << "  -> teleop: [" << isLinked() << "]\n\n";
            // std::cout << "  -> TX GrpTrq:  " << current_gripper_torque.load() << "\n\n";
            // std::cout << "  -> TX GrpPos:  " << current_gripper_pos.load() << "\n\n";
            // std::cout << "  -> TX grpcmd:  " << target_gripper_vel.load() << "\n\n";
            // std::cout << "  -> P control:      [" << policyControl.transpose() << "]\n";
            // std::cout << "  -> P JT:      [" << policyJt.transpose() << "]\n";
            // std::cout << "  -> P scales:      [" << policyJtScale.transpose() << "]\n";
            // std::cout << "  -> P final:      [" << (policyJtScale.asDiagonal() * policyJt).transpose() << "]\n\n";
            std::cout << "  -> P JP:      [" << policyJp.transpose() << "]\nn";
            // std::cout << "  -> P T Force:      [" << policyToolForce.transpose() << "]\n\n";
            // std::cout << "  -> P T Torque:      [" << policyToolTorque.transpose() << "]\n\n";
            // std::cout << "  -> P G:      [" << policy_gripper_pos.load() << "]\n\n";
        }
    }

    jp_type theirJp;
    jv_type theirJv;
    jt_type theirExtTorque;
    jt_type control;

    void pollGripper() {
        while (io_running.load()) {
            if (isLinked() && isInference()) { // shared control
                float local_gripper_vel = target_gripper_vel.load();
                gripper->setVelocity(local_gripper_vel);
                gripper->controlLoopCallback();
            } else if (isLinked()) { // teleop only
                float local_gripper_vel = target_gripper_vel.load();
                gripper->setVelocity(local_gripper_vel);
                gripper->controlLoopCallback();
            } else if (isInference()) { // inference only
                // float local_gripper_cmd = policy_gripper_cmd.load();
                // if (local_gripper_cmd > 0) {
                //     gripper->setPosition(gripper_max_pos);
                // } else {
                //     gripper->setPosition(gripper_min_pos);
                // }
                // gripper->setPosition(local_gripper_cmd);
                gripper->setVelocity(0.0f);
                gripper->controlLoopCallback();
            } else {
                gripper->setVelocity(0.0f);
            }

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
    FollowerUDPHandler<DOF> udp_handler;
    const std::chrono::milliseconds TELEOP_TIMEOUT_DURATION = std::chrono::milliseconds(20); // this needs to be larger than 2ms becuse of warmup when starting other programs

    GeckoGripper* gripper;
    std::thread io_thread;
    std::atomic<bool> io_running;
    std::atomic<float> target_gripper_vel;
    std::atomic<float> policy_gripper_cmd;
    std::atomic<float> current_gripper_pos;
    std::atomic<float> current_gripper_vel;
    std::atomic<float> current_gripper_torque;

    jt_type compute_teleop_control(const jp_type& ref_pos, const jv_type& ref_vel, const jt_type& ref_extTorque,
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

        jt_type u = u2;

        for (size_t i = 4; i < 7; ++i) {
            u[i] = 0.0;
        }

        return u;
    };
};

