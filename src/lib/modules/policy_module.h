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
        : config_(config), handler_(std::move(handler)) {
        for (size_t i = 0; i < DOF; ++i) {
            clip_val_[i] = i < config_.policy.clip_val.size() ? config_.policy.clip_val[i] : 0.0;
            clip_ref_torque_[i] =
                i < config_.policy.clip_ref_torque.size() ? config_.policy.clip_ref_torque[i] : 0.0;
        }
    }

    char key() const override { return 'p'; }
    const char* name() const override { return "policy"; }
    bool producesTorque() const override { return true; }

    void onLoad() override { std::cout << "policy module loaded" << std::endl; }
    void onUnload() override { std::cout << "policy module unloaded" << std::endl; }

    void receive(ControlContext<DOF>& ctx, TeleopData<DOF>& st) override {
        if (ctx.cancel_policy) {
            handler_->clearQueueAndPause();
        }

        boost::optional<PolicyReceivedData> data = handler_->getLatestPolicyReceived();
        if (!data) {
            // policy defaults
            setLocalStateValue(*ctx.cur_pos, st.policyJp);
            setLocalStateValue(jp_type::Zero(), st.resPolicyJp);
            setLocalStateValue(jt_type::Zero(), st.refPolicyTorque);
            setLocalStateValue(jt_type::Zero(), st.policyTorqueScale);
            return;
        }

        // base jp is a position command, clipped around where the leader is now
        jp_type base_jp;
        for (size_t i = 0; i < DOF; ++i) base_jp[i] = data->base.jp[i];
        setLocalStateValue(clipToRange(base_jp, st.leader_jp, clip_val_), st.policyJp);

        // The operator keeps priority: the policy only drives the gripper when
        // the operator is not commanding it.
        if (st.gripper_cmd == 0.0) {
            setLocalStateValue(data->base.gripper_cmd, st.gripper_cmd);
        }

        if (config_.policy.type == "dg") {
            jt_type ref_torque;
            for (size_t i = 0; i < DOF; ++i) ref_torque[i] = data->base.ext_torque[i];
            setLocalStateValue(clipToRange(ref_torque, st.filtered_human_torque, clip_ref_torque_),
                               st.refPolicyTorque);
        }

        if (config_.policy.type == "cr") {
            jp_type res_jp;
            for (size_t i = 0; i < DOF; ++i) res_jp[i] = data->res.delta_jp[i];
            const jp_type zero_jp = jp_type::Zero();
            setLocalStateValue(clipToRange(res_jp, zero_jp, clip_val_), st.resPolicyJp);

            jt_type ref_torque;
            for (size_t i = 0; i < DOF; ++i) ref_torque[i] = data->res.ext_torque[i];
            setLocalStateValue(
                clipToRange(ref_torque, st.filtered_environment_torque, clip_ref_torque_),
                st.refPolicyTorque);
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
    template <typename Vec>
    static Vec clipToRange(const Vec& value, const Vec& center, const Vec& clip_val) {
        Vec clipped = value;
        for (size_t i = 0; i < DOF; ++i) {
            const double delta = value[i] - center[i];
            if (delta > clip_val[i]) {
                clipped[i] = center[i] + clip_val[i];
            } else if (delta < -clip_val[i]) {
                clipped[i] = center[i] - clip_val[i];
            }
        }
        return clipped;
    }

    Config config_;
    std::unique_ptr<PolicyUDPHandler<DOF>> handler_;

    jp_type clip_val_;
    jt_type clip_ref_torque_;
};
