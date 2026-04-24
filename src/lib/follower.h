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
    Output<jt_type> wamJPOutput;
    // Output<jp_type> wamJPOutput;
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
        , wamJPOutput(this, &jtOutputValue)
        // , wamJPOutput(this, &jpOutputValue)
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
    // typename Output<jp_type>::Value* jpOutputValue;
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
            target_gripper_vel.store(static_cast<double>(received_data->gripper));

            // mirror and offset j1, j4, j5 and j6 
            theirJp(0) = -theirJp(0) - 1.57;
            // theirJp(3) = -theirJp(3);
            theirJp(4) = -theirJp(4) - 1.57;
            theirJp(5) *= -1;
            theirJv(0) *= -1;
            // theirJv(3) *= -1;
            theirJv(4) *= -1;
            theirJv(5) *= -1;
            theirExtTorque(0) *= -1;
            // theirExtTorque(3) *= -1;
            theirExtTorque(4) *= -1;
            theirExtTorque(5) *= -1;

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

        // jt_type u = u2;
        jt_type u = u1;

        for (size_t i = 4; i < 7; ++i) {
            u[i] = 0.0;
        }
        return u;
    };
};
