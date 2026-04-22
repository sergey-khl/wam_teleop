#pragma once

#include <boost/asio.hpp>
#include <iostream>
#include <cmath>
#include <atomic>
#include <thread>
#include <chrono>
#include "gripper/gecko/gecko_gripper.h"


#include "udp_handler.h"
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

using namespace gripper::gecko;

template <size_t DOF>
class Follower : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Input<jt_type> extTorqueIn;
    Input<jt_type> wamGravIn;
    Input<jt_type> wamDynIn;
    // Output<jt_type> wamJPOutput;
    Output<jp_type> wamJPOutput;
    Output<jp_type> theirJPOutput;

    enum class State { INIT, LINKED, UNLINKED };

    explicit Follower(barrett::systems::ExecutionManager* em,  GeckoGripper* gripper, const std::string& remoteHost, int rec_port = 5554,
                      int send_port = 5555, const std::string& sysName = "Follower")
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
        // , wamJPOutput(this, &jtOutputValue)
        , wamJPOutput(this, &jpOutputValue)
        , theirJPOutput(this, &theirJPOutputValue)
        , udp_handler(remoteHost, send_port, rec_port)
        , gripper(gripper)
        , target_gripper_vel(0.0f)
        , current_gripper_torque(0.0f)
        , io_running(false)
        , state(State::INIT) {

        kp << 750, 1000, 400, 200, 10, 10, 2.5;
        kd << 8.3, 8, 3.3, 0.8, 0.5, 0.5, 0.05;
        cf << 0.375, 0.4, 0.2, 0.1, 0.01, 0.01, 0.01;

        last_op_time = std::chrono::steady_clock::now();

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

  protected:
    typename Output<jp_type>::Value* jpOutputValue;
    // typename Output<jt_type>::Value* jtOutputValue;
    typename Output<jp_type>::Value* theirJPOutputValue;
    jp_type wamJP;
    jv_type wamJV;
    jt_type extTorque;
    jt_type wamGrav;
    jt_type wamDyn;
    jp_type commandJp;
    Eigen::Matrix<double, DOF, 1> sendJpMsg;
    Eigen::Matrix<double, DOF, 1> sendJvMsg;
    Eigen::Matrix<double, DOF, 1> sendExtTorqueMsg;


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

        boost::optional<ReceivedData> received_data = udp_handler.getLatestReceived();
        auto now = std::chrono::steady_clock::now();
        double udp_rx_age = 0.0;
        if (received_data && (now - received_data->timestamp <= TIMEOUT_DURATION)) {
            udp_rx_age = std::chrono::duration<double, std::milli>(now - received_data->timestamp).count();

            theirJp = received_data->jp;
            theirJv = received_data->jv;
            theirExtTorque = received_data->extTorque;
            theirJPOutputValue->setData(&theirJp);
            target_gripper_vel.store(static_cast<double>(received_data->gripper));

        } else {
            if (state == State::LINKED) {
                std::cout << "lost link" << std::endl;
                state = State::UNLINKED;
            }
        }

        switch (state) {
            case State::INIT:
                commandJp = wamJP;
                resetJ7Hybrid();
                jpOutputValue->setData(&commandJp);
                control.setZero();
                // jtOutputValue->setData(&control);
                break;
            case State::LINKED:
                commandJp = theirJp;
                applyJoint7Hybrid();
                jpOutputValue->setData(&commandJp);
                // // Active teleop. Only the callee can transition to LINKED
                control = compute_control(theirJp, theirJv, theirExtTorque, wamJP, wamJV, extTorque, wamGrav, wamDyn);
                // jtOutputValue->setData(&control);
                break;
            case State::UNLINKED:
                commandJp = wamJP;
                resetJ7Hybrid();
                jpOutputValue->setData(&commandJp);
                // // Changed to unlinked with either timeout or callee.
                control.setZero();
                // jtOutputValue->setData(&control);
                break;
        }

        // sendExtTorqueMsg << control;

        auto send_start = std::chrono::steady_clock::now();
        udp_handler.send(wamJP, wamJV, sendExtTorqueMsg, control, static_cast<double>(current_gripper_torque.load()));
        auto send_end = std::chrono::steady_clock::now();
        double send_dt = std::chrono::duration<double, std::milli>(send_end - send_start).count();

        if (++loop_counter % 500 == 0) {
            std::cout << "[FOLLOWER] Loop dt: " << loop_dt 
                      << " ms | UDP Rx Age: " << udp_rx_age 
                      << " ms | UDP Send latency: " << send_dt << " ms\n";
        }
    }

    jp_type theirJp;
    jv_type theirJv;
    jt_type theirExtTorque;
    jt_type control;

    void pollGripper() {
        while (io_running.load()) {
            gripper->setVelocity(target_gripper_vel.load());
            gripper->controlLoopCallback();
            
            GripperState gripper_state = gripper->getLatestState();
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
    const std::chrono::milliseconds TIMEOUT_DURATION = std::chrono::milliseconds(20);
    State state;
    Eigen::Matrix<double, DOF, 1> kp;
    Eigen::Matrix<double, DOF, 1> kd;
    Eigen::Matrix<double, DOF, 1> cf;

    GeckoGripper* gripper;
    std::thread io_thread;
    std::atomic<bool> io_running;
    std::atomic<float> target_gripper_vel;
    std::atomic<float> current_gripper_torque;

    static constexpr size_t J7_INDEX = 6;
    const double j7_joy_deadband = 0.05;
    const double j7_max_velocity_rad_s = 1.0;
    bool j7_initialized = false;
    bool j7_joystick_active = false;
    double j7_command_pos = 0.0;
    std::chrono::steady_clock::time_point j7_last_update;


    jt_type compute_control(const jp_type& ref_pos, const jv_type& ref_vel, const jt_type& ref_extTorque,
                            const jp_type& cur_pos, const jv_type& cur_vel, const jt_type& cur_extTorque,
                            const jt_type& cur_grav, const jt_type& cur_dyn) {
        
        // cases where the follower and leader have the same control law

        jt_type u1 = 0.0 * cur_extTorque; // zero feedforward (equal to default P-P with gravity compensation)

        jt_type u2 = cur_dyn - cur_grav; // P-P with dynamic compensation

        jt_type u3 = -0.5 * ref_extTorque; // PF-PF with ref external torque feedback

        jt_type u4 = -0.5 * ref_extTorque + cur_dyn - cur_grav; // PF-PF with ref external torque feedback and dynamic compensation (Lawrence's perfect transparency architecture);


        jt_type u5 = -0.5 * ref_extTorque -0.15 * (ref_extTorque + cur_extTorque); // PF-PF with ref external torque and cur external torque feedback

        jt_type u6 = -0.5 * ref_extTorque -0.15 * (ref_extTorque + cur_extTorque) + cur_dyn - cur_grav; // it has the best performance


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

    void resetJ7Hybrid() {
        j7_initialized = false;
        j7_joystick_active = false;
    }

    void applyJoint7Hybrid() {
        if constexpr (DOF <= J7_INDEX) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (!j7_initialized) {
            j7_command_pos = wamJP(J7_INDEX);
            j7_last_update = now;
            j7_initialized = true;
        }

        double dt = std::chrono::duration<double>(now - j7_last_update).count();
        j7_last_update = now;
        if (dt < 0.0) {
            dt = 0.0;
        } else if (dt > 0.1) {
            dt = 0.1;
        }

        const double joy_cmd = theirJv(J7_INDEX);
        const bool active = std::abs(joy_cmd) > j7_joy_deadband;

        if (active) {
            const double desired_vel = j7_max_velocity_rad_s * joy_cmd;
            j7_command_pos += desired_vel * dt;
            j7_joystick_active = true;
        } else {
            if (j7_joystick_active) {
                // Latch at the current measured position when returning to center.
                j7_command_pos = wamJP(J7_INDEX);
            }
            j7_joystick_active = false;
        }

        commandJp(J7_INDEX) = j7_command_pos;
    }
};
