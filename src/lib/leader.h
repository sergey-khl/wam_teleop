#pragma once

#include <boost/optional.hpp>
#include <iostream>
#include <cmath>
#include <cstdint>
#include <memory>
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
#include "modules/handle_module.h"
#include "modules/wrist_module.h"

template <size_t DOF>
class Leader : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Input<boost::tuple<cp_type, Eigen::Quaterniond>> wamTPIn;
    Input<jt_teleop_type> dyngravcompTorqueIn; // may be undefined
    Input<jt_type> wamGravIn;
    Input<jt_type> wamDynIn;
    Input<jt_teleop_type> basePolicyJtIn;
    Input<jt_teleop_type> resPolicyJtIn;
    Input<jt_teleop_type> refTorquePolicyJtIn;
    Input<jt_teleop_type> policyTorqueScaleIn;
    Input<jt_teleop_type> humanTorqueIn;
    Input<jt_teleop_type> filteredHumanTorqueIn;

    Output<jt_type> wamJTOutput;     // control torque command for the WAM arm (DOF)
    Output<jp_type> theirJPOutput;   // their arm JP (DOF) for logging/monitoring
    Output<jp_teleop_type> currentJpOutput;
    Output<jp_teleop_type> basePolicyJpOutput;
    Output<jp_teleop_type> resPolicyJpOutput;
    Output<jt_teleop_type> refPolicyJtOutput;
    Output<jt_teleop_type> filteredEnvironmentTorqueOutput;

    explicit Leader(barrett::systems::ExecutionManager* em, haptic_wrist::Handle* handle,
                haptic_wrist::HapticWrist* wrist,
                const Config& config,
                const std::string& sysName = "Leader")
        : System(sysName)
        , config(config)
        , control(jt_teleop_type::Zero())
        , wam_torque_(jt_type::Zero())
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
        , currentJpOutput(this, &currentJpOutputValue)
        , basePolicyJpOutput(this, &basePolicyJpOutputValue)
        , resPolicyJpOutput(this, &resPolicyJpOutputValue)
        , refPolicyJtOutput(this, &refPolicyJtOutputValue)
        , filteredEnvironmentTorqueOutput(this, &filteredEnvironmentTorqueOutputValue)
        , handle(handle)
        , wrist(wrist) {

        makeModules();

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~Leader() {
        handle_module_->stop();
        if (wrist_module_) wrist_module_->stop();
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

    bool hasWrist() const { return wrist_module_ != nullptr; }
    void syncWrist(const jp_teleop_type& sync_pos) {
        if (wrist_module_) wrist_module_->sync(sync_pos);
    }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    typename Output<jp_type>::Value* theirJPOutputValue;
    typename Output<jp_teleop_type>::Value* currentJpOutputValue;
    typename Output<jp_teleop_type>::Value* basePolicyJpOutputValue;
    typename Output<jp_teleop_type>::Value* resPolicyJpOutputValue;
    typename Output<jt_teleop_type>::Value* refPolicyJtOutputValue;
    typename Output<jt_teleop_type>::Value* filteredEnvironmentTorqueOutputValue;

    Config config;

    // All cross-thread / packet-bound state lives here and is passed around by
    // reference. See utils/data_packets.h.
    TeleopState<DOF> state;

    // The WAM works in DOF; this holds the arm slice of the control torque.
    jp_type their_arm_jp_;
    jt_type wam_torque_;

    virtual void operate() {
        const uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now().time_since_epoch())
                                    .count();

        // One lock for the whole cycle; the shared state is handed to the UDP
        // handlers at the end.
        auto state_lock = state.lock();
        TeleopData<DOF>& st = *state_lock;

        // WAM values are arm-wide; they land in the first DOF entries of the
        // full 7-wide state. The wrist fills the rest (see WristModule).
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
        ctx.cur_ext_torque = &st.human_torque;
        ctx.cur_dyn = &st.wam_dyn;
        ctx.cur_grav = &st.wam_grav;
        ctx.cur_pos = &st.leader_jp;
        ctx.cancel_policy = (st.cancel_policy == 1.0);

        teleop_module_->receive(ctx, st);
        their_arm_jp_ = st.follower_jp.head(DOF);
        theirJPOutputValue->setData(&their_arm_jp_);

        // the wrist only tracks the peer while teleop is linked
        if (wrist_module_ && teleop_module_->isLinked()) {
            wrist_module_->follow(st.follower_jp);
        }

        // set policy if p toggled
        policy_module_->receive(ctx, st);

        currentJpOutputValue->setData(&st.leader_jp);
        basePolicyJpOutputValue->setData(&st.policyJp);
        resPolicyJpOutputValue->setData(&st.resPolicyJp);
        filteredEnvironmentTorqueOutputValue->setData(&st.filtered_environment_torque);
        refPolicyJtOutputValue->setData(&st.refPolicyTorque);

        // extTorqueIn.valueDefined() before setting a reference signal can cause bad feeling teleop
        // also cant put this before the policy read. TODO: see why
        setLocalStateValue(dyngravcompTorqueIn, st.leader_dyngravcomp_torque);
        setLocalStateValue(filteredHumanTorqueIn, st.filtered_human_torque);
        setLocalStateValue(humanTorqueIn, st.human_torque);

        // impedance results
        setLocalStateValue(basePolicyJtIn, st.policyJt);
        setLocalStateValue(resPolicyJtIn, st.resPolicyJt);
        setLocalStateValue(refTorquePolicyJtIn, st.refTorquePolicyJt);

        // scale only applied to base policy
        setLocalStateValue(policyTorqueScaleIn, st.policyTorqueScale);

        // this control will only be applied if the necessary module is loaded
        control = compute_control(ctx);
        st.control_torque = control;

        wam_torque_ = control.head(DOF);
        jtOutputValue->setData(&wam_torque_);
        setLocalStateValue(now_ns, st.timestamp);

        // Packet contents are driven by `data_routing` in the config.
        teleop_module_->send(st);
        // see how on_leader is used for the magic
        policy_module_->send(st);

        logging_module_->update(ctx);
    }

    // Internal
    jt_teleop_type control;

  private:
    DISALLOW_COPY_AND_ASSIGN(Leader);

    std::unique_ptr<HandleModule<DOF>> handle_module_;
    std::unique_ptr<WristModule<DOF>> wrist_module_;

    haptic_wrist::Handle* handle;
    haptic_wrist::HapticWrist* wrist;

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
        handle_module_.reset(new HandleModule<DOF>(handle, &state));

        modules_.add(teleop_module_.get());
        modules_.add(policy_module_.get());
        modules_.add(dynamics_module_.get());
        modules_.add(logging_module_.get());

        // Modules flagged in their config are already on when the node starts.
        if (config.policy.auto_load) policy_module_->load();
        if (config.dynamics.auto_load) dynamics_module_->load();
        if (config.logging.auto_load) logging_module_->load();

        // always start
        handle_module_->start();
        if (wrist != nullptr) {
            wrist_module_.reset(new WristModule<DOF>(wrist, &state));
            wrist_module_->start();
        }
    }

    // only loaded modules will add to the torque (policy, dynamics)
    jt_teleop_type compute_control(const ControlContext<DOF>& ctx) {
        return modules_.sumTorque(ctx);
    }
};
