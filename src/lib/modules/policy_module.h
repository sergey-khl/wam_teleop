#pragma once

#include <iostream>
#include <memory>

#include <boost/optional.hpp>

#include "module.h"
#include "../utils/config_loader.h"
#include "../udp/udp_policy.h"

// apply torque from outside policy
template <size_t DOF>
class PolicyModule : public Module<DOF> {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    PolicyModule(const Config& config, std::unique_ptr<PolicyUDPHandler<DOF>> handler)
        : config_(config), handler_(std::move(handler)) {}

    char key() const override { return 'p'; }
    const char* name() const override { return "policy"; }
    bool producesTorque() const override { return true; }

    // Last gripper command received from the policy (follower uses it to
    // override the operator's command).
    bool hasGripperCmd() const { return have_gripper_cmd_; }
    double lastGripperCmd() const { return last_gripper_cmd_; }

    void onLoad() override { std::cout << "policy module loaded" << std::endl; }
    void onUnload() override { std::cout << "policy module unloaded" << std::endl; }

    void receive(ControlContext<DOF>& ctx, TeleopData<DOF>& st) override {
        have_gripper_cmd_ = false;

        if (ctx.cancel_policy) {
            handler_->clearQueueAndPause();
        }

        boost::optional<PolicyReceivedData> data = handler_->getLatestPolicyReceived();
        if (data) {
            st.policyJp << data->base_policy_jp;
            st.resPolicyJp << data->res_policy_jp;
            st.refPolicyTorque << data->ref_torque;
            last_gripper_cmd_ = data->gripper_cmd;
            have_gripper_cmd_ = true;
        }
    }

    void send(TeleopData<DOF>& st) override { handler_->send(st); }

    jt_type torque(const ControlContext<DOF>& ctx) override {
        const TeleopData<DOF>& st = *ctx.st;
        const jt_type base = st.policyTorqueScale.asDiagonal() * st.policyJt;

        if (config_.policy.type == "dg") {
            // policy torque applied as feed forward
            return base + st.refPolicyTorque;
        }
        if (config_.policy.type == "cr") {
            // policy torque treated as a desired torque, PID tracks it
            return base + st.resPolicyJt + st.refTorquePolicyJt;
        }
        // "base"
        return base;
    }

  private:
    Config config_;
    std::unique_ptr<PolicyUDPHandler<DOF>> handler_;

    double last_gripper_cmd_ = 0.0;
    bool have_gripper_cmd_ = false;
};
