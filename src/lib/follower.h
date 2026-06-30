#pragma once

#include <boost/asio.hpp>
#include <iostream>
#include <cmath>
#include <atomic>
#include <thread>
#include <chrono>
#include "gripper/gecko/gecko_gripper.h"
#include <iomanip>


#include "udp_handler.h"
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
    Output<cf_type> policyToolForceOutput;
    Output<ct_type> policyToolTorqueOutput;

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
        , policyToolForceOutput(this, &policyToolForceValue)
        , policyToolTorqueOutput(this, &policyToolTorqueValue)
        , udp_handler(config.network.leader_host, config.network.teleop_recv, config.network.teleop_send, 
                      config.network.recording, config.network.inference_host, config.network.follower_inference_send, config.network.inference_recv)
        , gripper(gripper)
        , target_gripper_vel(0.0f)
        , current_gripper_pos(0.0f)
        , current_gripper_torque(0.0f)
        , io_running(false)
        , prevError_(0.0)
        , linked(false)
        , inference_enabled (false) {

        last_op_time = std::chrono::high_resolution_clock::now();

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
        udp_handler.enableInference();
    }
    void disableInference() {
        {
            BARRETT_SCOPED_LOCK(this->getEmMutex());
            inference_enabled.store(false);
        }
        udp_handler.disableInference();
    }

  protected:
    typename Output<jp_type>::Value* jpOutputValue;
    typename Output<jt_type>::Value* jtOutputValue;
    typename Output<jp_type>::Value* theirJPOutputValue;
    typename Output<cf_type>::Value* policyToolForceValue;
    typename Output<ct_type>::Value* policyToolTorqueValue;
    jp_type wamJP;
    jv_type wamJV;
    boost::tuple<cp_type, Eigen::Quaterniond> wamTP;
    jt_type extTorque;
    jt_type wamGrav;
    jt_type wamDyn;
    jt_type policyJt;
    Eigen::Matrix<double, DOF, 1> sendJpMsg;
    Eigen::Matrix<double, DOF, 1> sendJvMsg;
    Eigen::Matrix<double, DOF, 1> sendExtTorqueMsg;
    Eigen::Matrix<double, DOF, 1> sendMeasTorqueMsg;

    TeleopConfig config;
    
    int loop_counter = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_op_time;

    float gripper_max_pos;
    float gripper_min_pos; // assumes 0 is the open pos of the gripper

    using ReceivedData = typename UDPHandler<DOF>::ReceivedData;

    virtual void operate() {
        auto now_op = std::chrono::high_resolution_clock::now();
        double loop_dt = std::chrono::duration<double, std::milli>(now_op - last_op_time).count();
        last_op_time = now_op;

        wamJP = wamJPIn.getValue();
        wamJV = wamJVIn.getValue();
        wamTP = wamTPIn.getValue();
        wamGrav = wamGravIn.getValue();
        wamDyn = wamDynIn.getValue();

        if (extTorqueIn.valueDefined()) {
            extTorque = extTorqueIn.getValue();
            // std::cout << "defined" << std::endl;
        } else {
            // std::cout << "not defined" << std::endl;
            extTorque << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        if (policyJtIn.valueDefined()) {
            policyJt = policyJtIn.getValue();
        } else {
            policyJt << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        policyToolForce << 0.0, 0.0, 0.0;
        policyToolTorque << 0.0, 0.0, 0.0;

        sendJpMsg << wamJP;
        sendJvMsg << wamJV;
        sendExtTorqueMsg << extTorque;

        const cp_type& toolPos  = boost::get<0>(wamTP);
        const Eigen::Quaterniond& toolQ = boost::get<1>(wamTP);


        boost::optional<ReceivedData> teleop_data = udp_handler.getLatestTeleopReceived();
        uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        uint64_t timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(TELEOP_TIMEOUT_DURATION).count();
        double udp_rx_age = 0.0;
        if (teleop_data && (now_ns >= teleop_data->timestamp) && (now_ns - teleop_data->timestamp <= timeout_ns)) {
            udp_rx_age = static_cast<double>(now_ns - teleop_data->timestamp) / 1000000.0;

            theirJp = teleop_data->jp;
            theirJv = teleop_data->jv;
            theirExtTorque = teleop_data->extTorque;
            target_gripper_vel.store(static_cast<double>(teleop_data->gripper));

            // mirror and offset some of the wam joints
            for (size_t i = 0; i < DOF; i++) {
                theirJp[i] = theirJp[i] * config.sync_mapping.scales[i] + config.sync_mapping.offsets[i];
                theirJv[i] = theirJv[i] * config.sync_mapping.scales[i];
                theirExtTorque[i] = theirExtTorque[i] * config.sync_mapping.scales[i];
            }

            theirJPOutputValue->setData(&theirJp);
        } else {
            std::cout << "lost link" << std::endl;
            linked.store(false);
        }

        if (isInference()) {
            boost::optional<ReceivedData> policy_data = udp_handler.getLatestInferenceReceived();
            uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            uint64_t timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(INFERENCE_TIMEOUT_DURATION).count();
            double udp_rx_age = 0.0;
            if (policy_data && (now_ns >= policy_data->timestamp) && (now_ns - policy_data->timestamp <= timeout_ns)) {
                udp_rx_age = static_cast<double>(now_ns - policy_data->timestamp) / 1000000.0;

                // from inference we directly get the tool force and tool torque. Calculated in openpi policy repo
                // TODO: this is gross. properly sepearte out udp controller to handle the different cases in a not dumb way
                policyToolForce << policy_data->cart_pos.x(), policy_data->cart_pos.y(), policy_data->cart_pos.z();
                policyToolTorque << policy_data->cart_rot.w(), policy_data->cart_rot.x(), policy_data->cart_rot.y();

                policy_gripper_pos.store(static_cast<double>(policy_data->gripper));
            } else {
                // policy force and torque are already zeroed above
                policy_gripper_pos.store(static_cast<double>(current_gripper_pos));
            }
        }



        if (isLinked() && isInference()) { // shared control
            // TOOD: make this actually work
            control.setZero();
            jtOutputValue->setData(&control);
        } else if (isLinked()) { // teleop only
            control = compute_teleop_control(theirJp, theirJv, theirExtTorque, wamJP, wamJV, extTorque, wamGrav, wamDyn);
            jtOutputValue->setData(&control);
        } else if (isInference()) { // inference only
            // control = compute_policy_control(policyJp, policyJv, policyExtTorque, wamJP, wamJV, extTorque, wamGrav, wamDyn, loop_dt);
            // jtOutputValue->setData(&control);
            control.setZero();
            jtOutputValue->setData(&control);
        } else {
            control.setZero();
            jtOutputValue->setData(&control);
        }

        jpOutputValue->setData(&wamJP);
        policyToolForceValue->setData(&policyToolForce);
        policyToolTorqueValue->setData(&policyToolTorque);

        sendMeasTorqueMsg << control;

        uint64_t loop_start = std::chrono::duration_cast<std::chrono::nanoseconds>(now_op.time_since_epoch()).count();

        auto send_start = std::chrono::high_resolution_clock::now();
        udp_handler.send(wamJP, wamJV, sendExtTorqueMsg, sendMeasTorqueMsg, toolPos, toolQ, static_cast<double>(current_gripper_torque.load()), loop_start);
        auto send_end = std::chrono::high_resolution_clock::now();
        double send_dt = std::chrono::duration<double, std::milli>(send_end - send_start).count();

        if (++loop_counter % 500 == 0) {
            // std::cout << "[FOLLOWER] Loop dt: " << loop_dt << "\n" ;
        //               << " ms | UDP Rx Age: " << udp_rx_age 
        //               << " ms | UDP Send latency: " << send_dt << " ms\n";

            // std::cout << std::fixed << std::setprecision(3);
            // std::cout << "  -> FOLLOWER JP:      [" << sendJpMsg.transpose() << "]\n";
            // std::cout << "  -> TX JV:      [" << sendJvMsg.transpose() << "]\n";
            // std::cout << "  -> TX ExtTrq:  [" << sendExtTorqueMsg.transpose() << "]\n";
            // std::cout << "  -> TX MeasTrq: [" << compute_policy_control(policyJp, policyJv, policyExtTorque, wamJP, wamJV, extTorque, wamGrav, wamDyn, loop_dt) << "]\n\n";
            // std::cout << "  -> TX MeasTrq: [" << control << "]\n";
            // std::cout << "  -> inf: [" << isInference() << "]\n";
            // std::cout << "  -> teleop: [" << isLinked() << "]\n\n";
            // std::cout << "  -> TX GrpTrq:  " << current_gripper_torque.load() << "\n\n";
            // std::cout << "  -> TX GrpPos:  " << current_gripper_pos.load() << "\n\n";
            // std::cout << "  -> ref:  " << theirExtTorque.transpose() << "\n\n";
            std::cout << "  -> LEADER JP:  " << theirJp.transpose() << "\n\n";
            // std::cout << "  -> P JP:      [" << policyJp.transpose() << "]\n";
            // std::cout << "  -> P JV:      [" << policyJv.transpose() << "]\n";
            std::cout << "  -> P T:      [" << policyJt.transpose() << "]\n\n";
            // std::cout << "  -> P G:      [" << policy_gripper_pos.load() << "]\n\n";
        }
    }

    jp_type theirJp;
    jv_type theirJv;
    jt_type theirExtTorque;
    cf_type policyToolForce;
    ct_type policyToolTorque;
    jt_type control;
    jv_type prevError_;

    void pollGripper() {
        while (io_running.load()) {
            if (isLinked() && isInference()) { // shared control
                // TOOD: make this actually work
                gripper->setVelocity(0.0f);
            } else if (isLinked()) { // teleop only
                float local_gripper_vel = target_gripper_vel.load();
                gripper->setVelocity(local_gripper_vel);
                gripper->controlLoopCallback();
            } else if (isInference()) { // inference only
                // float local_gripper_pos = policy_gripper_pos.load();
                // gripper->setPosition(local_gripper_pos);
                // gripper->controlLoopCallback();
                gripper->setVelocity(0.0f);
            } else {
                gripper->setVelocity(0.0f);
            }

            GripperState gripper_state = gripper->getLatestState();
            current_gripper_pos.store(gripper_state.position);
            current_gripper_torque.store(gripper_state.torque);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        gripper->setVelocity(0.0f);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(Follower);
    std::mutex state_mutex;
    jp_type joint_positions;
    UDPHandler<DOF> udp_handler;
    const std::chrono::milliseconds TELEOP_TIMEOUT_DURATION = std::chrono::milliseconds(20);
    const std::chrono::milliseconds INFERENCE_TIMEOUT_DURATION = std::chrono::milliseconds(150); // allow for 10hz
    const Eigen::Matrix<double, DOF, 1> kp;
    const Eigen::Matrix<double, DOF, 1> kd;

    GeckoGripper* gripper;
    std::thread io_thread;
    std::atomic<bool> io_running;
    std::atomic<float> target_gripper_vel;
    std::atomic<float> policy_gripper_pos;
    std::atomic<float> current_gripper_pos;
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

        jt_type u = u1;

        for (size_t i = 4; i < 7; ++i) {
            u[i] = 0.0;
        }
        return u;
    };

    jt_type compute_policy_control(const jp_type& ref_pos, const jv_type& ref_vel, const jt_type& ref_extTorque,
                            const jp_type& cur_pos, const jv_type& cur_vel, const jt_type& cur_extTorque,
                            const jt_type& cur_grav, const jt_type& cur_dyn, double loop_dt) {
        double dt_s = loop_dt / 1000.0; // convert ms -> seconds

        jv_type error = ref_pos - cur_pos;
        jv_type derivative = (dt_s > 0.0 && dt_s < 0.01) ? ((error - prevError_) / dt_s) : jv_type(0.0);
        prevError_ = error;

        jt_type j_torque = kp.cwiseProduct(error) + kd.cwiseProduct(derivative);
        j_torque << 0, 0, 0, 0, 0, 0, 0;
        return j_torque;
    };
};

