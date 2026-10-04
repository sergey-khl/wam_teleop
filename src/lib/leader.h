#pragma once

#include <boost/optional.hpp>
#include <iostream>
#include <cmath>
#include <cstdint>
#include <atomic>
#include <memory>
#include <thread>
#include <chrono>
#include <iomanip>

#include "udp/leader_udp_handler.h"
#include "udp/udp_policy.h"
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>
#include "utils/config_loader.h"
#include "utils/utils.h"
#include "utils/data_packets.h"
#include "modules/module.h"
#include "modules/teleop_module.h"
#include "modules/policy_module.h"
#include "modules/dynamics_module.h"
#include "modules/logging_module.h"

template <size_t DOF>
class Leader : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Input<boost::tuple<cp_type, Eigen::Quaterniond>> wamTPIn;
    Input<jt_type> dyngravcompTorqueIn;   // may be undefined
    Input<jt_type> wamGravIn;
    Input<jt_type> wamDynIn;
    Input<jt_type> basePolicyJtIn;
    Input<jt_type> resPolicyJtIn;
    Input<jt_type> refTorquePolicyJtIn;
    Input<jt_type> policyTorqueScaleIn;
    Input<jt_type> humanTorqueIn;
    Input<jt_type> filteredHumanTorqueIn;

    Output<jt_type> wamJTOutput;      // control torque command for the WAM arm (DOF)
    Output<jp_type> theirJPOutput;    // their arm JP (DOF) for logging/monitoring
    Output<jp_type> basePolicyJpOutput;
    Output<jp_type> resPolicyJpOutput;
    Output<jt_type> refPolicyJtOutput;
    Output<jt_type> filteredEnvironmentTorqueOutput;

    explicit Leader(barrett::systems::ExecutionManager* em, haptic_wrist::Handle* handle,
                const Config& config,
                const std::string& sysName = "Leader")
        : System(sysName)
        , config(config)
        , control(0.0)
        , applied_control(0.0)
        , wamJPIn(this)
        , wamJVIn(this)
        , wamTPIn(this)
        , dyngravcompTorqueIn(this)
        , wamGravIn(this)
        , wamDynIn(this)
        , basePolicyJtIn(this)
        , resPolicyJtIn(this)
        , refTorquePolicyJtIn(this)
        , policyTorqueScaleIn(this)
        , humanTorqueIn(this)
        , filteredHumanTorqueIn(this)
        , wamJTOutput(this, &jtOutputValue)
        , theirJPOutput(this, &theirJPOutputValue)
        , basePolicyJpOutput(this, &basePolicyJpOutputValue)
        , resPolicyJpOutput(this, &resPolicyJpOutputValue)
        , refPolicyJtOutput(this, &refPolicyJtOutputValue)
        , filteredEnvironmentTorqueOutput(this, &filteredEnvironmentTorqueOutputValue)
        , handle(handle)
        , io_running(false) {

        torque_scaling   = config.handle.torque_scaling;
        minStiffness     = config.handle.minStiffness;
        maxStiffness     = config.handle.maxStiffness;
        alpha            = config.handle.alpha;

        makeModules();

        if (em != NULL) {
            em->startManaging(*this);
        }
        io_running.store(true);
        io_thread = std::thread(&Leader::pollHandle, this);
    }

    virtual ~Leader() {
        io_running.store(false);
        if (io_thread.joinable()) {
            io_thread.join();
        }
        this->mandatoryCleanUp();
    }

    virtual bool inputsValid() { return true; }

    bool isLinked() const { return teleop_module_ && teleop_module_->isLinked(); }
    void tryLink()  { BARRETT_SCOPED_LOCK(this->getEmMutex()); teleop_module_->tryLink(); }
    void unlink()   { BARRETT_SCOPED_LOCK(this->getEmMutex()); teleop_module_->unlink(); }

    // Module access (determined by the hotkey loop in leader.cpp).
    ModuleManager<DOF>& modules() { return modules_; }
    TeleopModule<DOF, LeaderUDPHandler<DOF>>& teleop() { return *teleop_module_; }
    bool policyLoaded() const { return policy_module_ && policy_module_->isLoaded(); }
    bool dynamicsLoaded() const { return dynamics_module_ && dynamics_module_->isLoaded(); }
    bool loggingLoaded() const { return logging_module_ && logging_module_->isLoaded(); }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    typename Output<jp_type>::Value* theirJPOutputValue;
    typename Output<jp_type>::Value* basePolicyJpOutputValue;
    typename Output<jp_type>::Value* resPolicyJpOutputValue;
    typename Output<jt_type>::Value* refPolicyJtOutputValue;
    typename Output<jt_type>::Value* filteredEnvironmentTorqueOutputValue;

    Config config;

    // All cross-thread / packet-bound state lives here and is passed around by
    // reference. See utils/data_packets.h.
    TeleopState<DOF> state;

    jt_type humanTorque;

    const float gripper_speed = 0.1f;

    float torque_scaling;
    float minStiffness;
    float maxStiffness;
    float alpha;

    virtual void operate() {
        const uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now().time_since_epoch())
                                    .count();

        // One lock for the whole cycle; the shared state is handed to the UDP
        // handlers at the end.
        auto state_lock = state.lock();
        TeleopData<DOF>& st = *state_lock;

        // always known vals
        setLocalStateValue(wamJPIn, st.leader_jp);
        setLocalStateValue(wamJVIn, st.leader_jv);
        auto wam_tp = wamTPIn.getValue();
        setLocalStateValue(boost::get<0>(wam_tp), st.leader_cart_pos);
        setLocalStateValue(boost::get<1>(wam_tp), st.leader_quat);
        setLocalStateValue(wamDynIn, st.wam_dyn);
        setLocalStateValue(wamGravIn, st.wam_grav);

        // Everything the modules need for this cycle.
        ControlContext<DOF> ctx;
        ctx.st = &st;
        ctx.ref_ext_torque = &st.environment_torque;
        ctx.cur_ext_torque = &humanTorque;
        ctx.cur_dyn = &st.wam_dyn;
        ctx.cur_grav = &st.wam_grav;
        ctx.cur_pos = &st.leader_jp;
        ctx.cancel_policy = (st.cancel_policy == 1.0);

        teleop_module_->receive(ctx, st);
        theirJPOutputValue->setData(&st.follower_jp);

        policy_module_->receive(ctx, st);

        basePolicyJpOutputValue->setData(&st.policyJp);
        resPolicyJpOutputValue->setData(&st.resPolicyJp);
        filteredEnvironmentTorqueOutputValue->setData(&st.filtered_environment_torque);
        refPolicyJtOutputValue->setData(&st.refPolicyTorque);

        // extTorqueIn.valueDefined() before setting a reference signal can cause bad feeling teleop
        // also cant put this before the policy read. i have no idea why
        setLocalStateValue(dyngravcompTorqueIn, st.leader_dyngravcomp_torque);
        setLocalStateValue(filteredHumanTorqueIn, st.filtered_human_torque);
        setLocalStateValue(humanTorqueIn, humanTorque);
        setLocalStateValue(humanTorque, st.human_torque);

        // impedance results
        setLocalStateValue(basePolicyJtIn, st.policyJt);
        setLocalStateValue(resPolicyJtIn, st.resPolicyJt);
        setLocalStateValue(refTorquePolicyJtIn, st.refTorquePolicyJt);

        // scale only applied to base policy
        setLocalStateValue(policyTorqueScaleIn, st.policyTorqueScale);

        // Always compute the control torque and log it, so the logging module
        // shows what would be applied.
        control = compute_control(ctx);
        setLocalStateValue(control, st.control_torque);

        applied_control = control;
        jtOutputValue->setData(&applied_control);
        setLocalStateValue(now_ns, st.timestamp);

        // Packet contents are driven by `data_routing` in the config.
        teleop_module_->send(st);
        // see how on_leader is used for the magic
        policy_module_->send(st);

        // Config-driven logging (no-op unless the logging module is loaded).
        logging_module_->update(ctx);
    }

    // Internal
    jt_type control;
    jt_type applied_control;
    std::thread io_thread;
    std::atomic<bool> io_running;

    void pollHandle() {
        bool bumper = false;
        bool trigger = false;
        double cancel_policy = 0.0;
        float target_position = 0.0f;

        while (io_running.load()) {
            handle->poll(); // need to poll sony controller
            if (boost::optional<haptic_wrist::handle_type> opt_handle = handle->getHandle()) {
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

            state.with_lock([&](TeleopData<DOF>& st) {
                setLocalStateValue(target_position, st.gripper_cmd);
                setLocalStateValue(cancel_policy, st.cancel_policy);
            });

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(Leader);

    haptic_wrist::Handle* handle;

    std::unique_ptr<TeleopModule<DOF, LeaderUDPHandler<DOF>>> teleop_module_;
    std::unique_ptr<PolicyModule<DOF>> policy_module_;
    std::unique_ptr<DynamicsModule<DOF>> dynamics_module_;
    std::unique_ptr<LoggingModule<DOF>> logging_module_;
    ModuleManager<DOF> modules_;

    void makeModules() {
        teleop_module_.reset(new TeleopModule<DOF, LeaderUDPHandler<DOF>>(
            ModuleRole::Leader, config, &state,
            std::unique_ptr<LeaderUDPHandler<DOF>>(new LeaderUDPHandler<DOF>(
                config.network.follower_host, config.network.teleop_send, config.network.teleop_recv,
                config.data_routing.teleop_send_leader, config.data_routing.teleop_send_follower))));

        policy_module_.reset(new PolicyModule<DOF>(
            config,
            std::unique_ptr<PolicyUDPHandler<DOF>>(new PolicyUDPHandler<DOF>(
                config, config.policy.on_leader, config.policy_network.policy_leader_recv,
                config.policy_data_routing.policy_send_wam))));

        dynamics_module_.reset(new DynamicsModule<DOF>(config));
        logging_module_.reset(new LoggingModule<DOF>(ModuleRole::Leader, config));

        modules_.add(teleop_module_.get());
        modules_.add(policy_module_.get());
        modules_.add(dynamics_module_.get());
        modules_.add(logging_module_.get());
    }

    // only loaded modules will add to the torque (policy, dynamics)
    jt_type compute_control(const ControlContext<DOF>& ctx) {
        return modules_.sumTorque(ctx);
    }
};
