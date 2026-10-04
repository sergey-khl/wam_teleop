#pragma once

#include <atomic>
#include <chrono>
#include <thread>

#include <boost/optional.hpp>
#include <haptic_wrist/handle.h>

#include "module.h"
#include "../utils/data_packets.h"

// running on startup. read from handle user commands.
template <size_t DOF>
class HandleModule : public Module<DOF> {
  public:
    HandleModule(haptic_wrist::Handle* handle, TeleopState<DOF>* state)
        : handle_(handle), state_(state) {}

    ~HandleModule() override { stop(); }

    char key() const override { return 'h'; }
    const char* name() const override { return "handle"; }

    void start() {
        if (running_.exchange(true)) return;
        thread_ = std::thread(&HandleModule::loop, this);
    }

    void stop() {
        running_.store(false);
        if (thread_.joinable()) thread_.join();
    }

  private:
    void loop() {
        bool bumper = false;
        bool trigger = false;
        double cancel_policy = 0.0;
        float target_position = 0.0f;

        while (running_.load()) {
            handle_->poll(); // need to poll sony controller
            if (boost::optional<haptic_wrist::handle_type> opt_handle = handle_->getHandle()) {
                haptic_wrist::handle_type handle = *opt_handle;
                bumper = handle[0];
                trigger = handle[1];
                cancel_policy = handle[2]; // up button on controller
            }

            // still position controlled. just send to max and min gripper pos
            if (bumper && !trigger) {
                target_position = -1;
            } else if (trigger && !bumper) {
                target_position = 1;
            }

            state_->with_lock([&](TeleopData<DOF>& st) {
                setLocalStateValue(target_position, st.gripper_cmd);
                setLocalStateValue(cancel_policy, st.cancel_policy);
            });

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    haptic_wrist::Handle* handle_;
    TeleopState<DOF>* state_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};
