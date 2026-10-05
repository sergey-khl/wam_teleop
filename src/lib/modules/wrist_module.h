#pragma once

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include <haptic_wrist/haptic_wrist.h>

#include "module.h"
#include "../utils/data_packets.h"

template <size_t DOF>
class WristModule : public Module<DOF> {
  public:
    WristModule(haptic_wrist::HapticWrist* wrist, TeleopState<DOF>* state)
        : wrist_(wrist), state_(state), name_(wrist->getName()) {}

    ~WristModule() override { stop(); }

    char key() const override { return 'w'; }
    const char* name() const override { return "wrist"; }
    const std::string& wristName() const { return name_; }

    void start() {
        if (running_.exchange(true)) return;
        thread_ = std::thread(&WristModule::loop, this);
    }

    void stop() {
        running_.store(false);
        if (thread_.joinable()) thread_.join();
    }

    // Move the wrist to the sync position. The target depends on the wrist.
    void sync(const jp_teleop_type& sync_pos) {
        if (TeleopData<DOF>::WRIST_DOF != 3) return;
        if (name_ == "ee_wrist") {
            wrist_->jointMoveTo(Eigen::Vector3d(sync_pos.tail(TeleopData<DOF>::WRIST_DOF)));
        }
    }

    // Track the peer wrist. Only called while teleop is linked.
    void follow(const jp_teleop_type& their_jp) {
        if (TeleopData<DOF>::WRIST_DOF != 3) return;
        if (name_ == "ee_wrist") {
            wrist_->setTarget(Eigen::Vector3d(their_jp.tail(TeleopData<DOF>::WRIST_DOF)));
        }
    }

  private:
    void loop() {
        while (running_.load()) {
            const haptic_wrist::jp_type wrist_jp = wrist_->getPosition();
            const haptic_wrist::jv_type wrist_jv = wrist_->getVelocity();

            state_->with_lock([&](TeleopData<DOF>& st) {
                if (TeleopData<DOF>::WRIST_DOF == 3) {
                    st.leader_jp.tail(3) = wrist_jp;
                    st.leader_jv.tail(3) = wrist_jv;
                }
            });

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    haptic_wrist::HapticWrist* wrist_;
    TeleopState<DOF>* state_;
    std::string name_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};
