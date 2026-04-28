#include "ros/ros.h"

#include "sensor_msgs/JointState.h"
#include <barrett/systems.h>
#include <barrett/units.h>
#include <thread>
#include "wam_teleop/GravityTorque.h"

namespace haptic_wrist {
class HapticWrist;
};

// ExposedInput monitoring system
template <typename T> class ExposedInput : public barrett::systems::System, public barrett::systems::SingleInput<T> {
  public:
    explicit ExposedInput(barrett::systems::ExecutionManager *em, const std::string &sysName = "ExposedInput")
        : barrett::systems::System(sysName), barrett::systems::SingleInput<T>(this) {

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~ExposedInput() { this->mandatoryCleanUp(); }

    T getValue() { return value; }

  protected:
    virtual void operate() { value = this->input.getValue(); }

  public:
    T value;

  private:
    DISALLOW_COPY_AND_ASSIGN(ExposedInput);
};

#ifdef BUILD_LEADER
#include <haptic_wrist/haptic_wrist.h>
#endif

template <size_t WAM_DOF> class BackgroundStatePublisher {
    using jp_type = typename barrett::math::Vector<WAM_DOF>::type;
    using jt_type = typename barrett::units::JointTorques<WAM_DOF>::type;
    using jv_type = typename barrett::units::JointPositions<WAM_DOF>::type;

  public:
    BackgroundStatePublisher(barrett::systems::ExecutionManager* em, barrett::systems::Wam<WAM_DOF> &wam, haptic_wrist::HapticWrist *hw = nullptr)
        : wam(wam), hw(hw), stop_thread(false), nh((WAM_DOF == 4) ? "leader" : "follower"), exposedGravity(em) {

        barrett::systems::connect(wam.gravity.output, exposedGravity.input);

        joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
        grav_pub = nh.advertise<wam_teleop::GravityTorque>("gravity", 1);

        std::vector<std::string> joint_names;
        joint_names.push_back("wam_j1");
        joint_names.push_back("wam_j2");
        joint_names.push_back("wam_j3");
        joint_names.push_back("wam_j4");
        if (WAM_DOF == 7) {
            joint_names.push_back("wam_j5");
            joint_names.push_back("wam_j6");
            joint_names.push_back("wam_j7");
        } else {
            joint_names.push_back("hapticwrist_j1");
            joint_names.push_back("hapticwrist_j2");
            joint_names.push_back("hapticwrist_j3");
        }
        joint_state.name = joint_names;
        joint_state.position.resize(7);
        joint_state.velocity.resize(7);
        joint_state.effort.resize(7);

        gravTorque.torque.resize(7);

        pub_thread = std::thread(&BackgroundStatePublisher::run, this);
    }

    ~BackgroundStatePublisher() {
        stop_thread.store(true, std::memory_order_relaxed);
        if (pub_thread.joinable()) {
            pub_thread.join();
        }
    }

  private:
    ros::NodeHandle nh;
    barrett::systems::Wam<WAM_DOF> &wam;
    haptic_wrist::HapticWrist *hw;
    ros::Publisher joint_state_pub;
    ros::Publisher grav_pub;
    sensor_msgs::JointState joint_state;
    wam_teleop::GravityTorque gravTorque;
    ExposedInput<jt_type> exposedGravity;

    std::thread pub_thread;
    std::atomic<bool> stop_thread;

    void run() {
        ros::Rate pub_rate(500);
        while (ros::ok() && !stop_thread.load(std::memory_order_relaxed)) {


            jp_type wam_jp = wam.getJointPositions();
            jv_type wam_jv = wam.getJointVelocities();
            jt_type wam_jt = wam.getJointTorques();

            jt_type grav = exposedGravity.getValue();
            for (size_t i = 0; i < WAM_DOF; i++) {
                joint_state.position[i] = wam_jp[i];
                joint_state.velocity[i] = wam_jv[i];
                joint_state.effort[i] = wam_jt[i];

                gravTorque.torque[i] = grav[i];

            }
#ifdef BUILD_LEADER
            if (WAM_DOF == 4 && hw != nullptr) {
                haptic_wrist::jp_type hw_jp = hw->getPosition();
                haptic_wrist::jv_type hw_jv = hw->getVelocity();
                haptic_wrist::jt_type hw_jt = hw->getTorque();
                // NOTE: j7 is ignored here cus its a little weird with the joystick
                for (size_t i = 0; i < 2; i++) {
                    joint_state.position[WAM_DOF + i] = hw_jp[i];
                    joint_state.velocity[WAM_DOF + i] = hw_jv[i];
                    joint_state.effort[WAM_DOF + i] = hw_jt[i];
                }
            }
#endif
            joint_state.header.stamp = ros::Time::now();
            joint_state_pub.publish(joint_state);


            gravTorque.header.stamp = ros::Time::now();
            grav_pub.publish(gravTorque);
            pub_rate.sleep();
        }
    }
};
