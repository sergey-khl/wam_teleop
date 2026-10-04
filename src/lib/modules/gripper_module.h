#pragma once

#include <atomic>
#include <chrono>
#include <thread>

#include <gripper/gecko/gecko_gripper.h>

#include "module.h"
#include "../utils/data_packets.h"

// running on startup. move the gripper with a cmd
template <size_t DOF>
class GripperModule : public Module<DOF> {
  public:
    GripperModule(gripper::gecko::GeckoGripper* gripper, TeleopState<DOF>* state)
        : gripper_(gripper), state_(state) {}

    ~GripperModule() override { stop(); }

    char key() const override { return 'r'; }
    const char* name() const override { return "gripper"; }

    void start() {
        if (running_.exchange(true)) return;
        thread_ = std::thread(&GripperModule::loop, this);
    }

    void stop() {
        running_.store(false);
        if (thread_.joinable()) thread_.join();
    }

  private:
    void loop() {
        while (running_.load()) {
            float gripper_cmd = 0.0f;
            state_->with_lock([&](TeleopData<DOF>& st) {
                gripper_cmd = static_cast<float>(st.gripper_cmd);
            });
            gripper_->setPosition(gripper_cmd);
            gripper_->controlLoopCallback();

            gripper::gecko::GripperState gripper_state = gripper_->getLatestState();
            state_->with_lock([&](TeleopData<DOF>& st) {
                setLocalStateValue(gripper_state.position, st.gripper_pos);
                setLocalStateValue(gripper_state.velocity, st.gripper_vel);
                setLocalStateValue(gripper_state.torque, st.gripper_torque);
            });

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        gripper_->setVelocity(0.0f);
    }

    gripper::gecko::GeckoGripper* gripper_;
    TeleopState<DOF>* state_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};
