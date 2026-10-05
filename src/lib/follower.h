#pragma once

#include <iostream>
#include <cmath>
#include <memory>
#include <chrono>
#include <iomanip>


#include "udp/follower_udp_handler.h"
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
#include "modules/gripper_module.h"

template <size_t DOF>
class Follower : public barrett::systems::System {
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
    Input<jt_teleop_type> environmentTorqueIn;
    Input<jt_teleop_type> filteredEnvironmentTorqueIn;

    Output<jt_type> wamJTOutput;
    Output<jp_type> theirJPOutput;
    Output<jp_teleop_type> currentJpOutput;
    Output<jp_teleop_type> basePolicyJpOutput;
    Output<jp_teleop_type> resPolicyJpOutput;
    Output<jt_teleop_type> refPolicyJtOutput;

    explicit Follower(barrett::systems::ExecutionManager* em, gripper::gecko::GeckoGripper* gripper,
                  const Config& config,
                  const std::string& sysName = "Follower")
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
        , environmentTorqueIn(this)
        , filteredEnvironmentTorqueIn(this)
        , wamJTOutput(this, &jtOutputValue)
        , theirJPOutput(this, &theirJPOutputValue)
        , currentJpOutput(this, &currentJpOutputValue)
        , basePolicyJpOutput(this, &basePolicyJpOutputValue)
        , resPolicyJpOutput(this, &resPolicyJpOutputValue)
        , refPolicyJtOutput(this, &refPolicyJtOutputValue)
        , gripper(gripper) {

        makeModules();

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~Follower() {
        gripper_module_->stop();
        this->mandatoryCleanUp();
    }

    virtual bool inputsValid() {return true;}

    bool isLinked() const { return teleop_module_ && teleop_module_->isLinked(); }
    void tryLink()  { BARRETT_SCOPED_LOCK(this->getEmMutex()); teleop_module_->tryLink(); }
    void unlink()   { BARRETT_SCOPED_LOCK(this->getEmMutex()); teleop_module_->unlink(); }

    // Module access (determined by the hotkey loop in follower.cpp).
    ModuleManager<DOF>& modules() { return modules_; }
    TeleopModule<DOF, FollowerUDPHandler<DOF>>& teleop() { return *teleop_module_; }
    bool policyLoaded() const { return policy_module_ && policy_module_->isLoaded(); }
    bool dynamicsLoaded() const { return dynamics_module_ && dynamics_module_->isLoaded(); }
    bool loggingLoaded() const { return logging_module_ && logging_module_->isLoaded(); }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    typename Output<jp_type>::Value* theirJPOutputValue;
    typename Output<jp_teleop_type>::Value* currentJpOutputValue;
    typename Output<jp_teleop_type>::Value* basePolicyJpOutputValue;
    typename Output<jp_teleop_type>::Value* resPolicyJpOutputValue;
    typename Output<jt_teleop_type>::Value* refPolicyJtOutputValue;

    // All cross-thread / packet-bound state lives here and is passed around by
    // reference. See utils/data_packets.h.
    TeleopState<DOF> state;

    Config config;

    // The WAM works in DOF; this holds the arm slice of the control torque.
    jp_type their_arm_jp_;
    jt_type wam_torque_;

    virtual void operate() {
        const uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now().time_since_epoch())
                                    .count();

        // One lock for the whole cycle
        auto state_lock = state.lock();
        TeleopData<DOF>& st = *state_lock;

        // WAM values are arm-wide; they land in the first DOF entries of the
        // full 7-wide state. The follower has no wrist today, so the rest is zero.
        setLocalStateValue(wamJPIn, st.follower_jp);
        setLocalStateValue(wamJVIn, st.follower_jv);
        auto wam_tp = wamTPIn.getValue();
        setLocalStateValue(boost::get<0>(wam_tp), st.follower_cart_pos);
        setLocalStateValue(boost::get<1>(wam_tp), st.follower_quat);
        setLocalStateValue(wamDynIn, st.wam_dyn);
        setLocalStateValue(wamGravIn, st.wam_grav);

        // Everything the modules need for this cycle.
        ControlContext<DOF> ctx;
        ctx.st = &st;
        ctx.ref_ext_torque = &st.human_torque;
        ctx.cur_ext_torque = &st.environment_torque;
        ctx.cur_dyn = &st.wam_dyn;
        ctx.cur_grav = &st.wam_grav;
        ctx.cur_pos = &st.follower_jp;

        teleop_module_->receive(ctx, st);
        their_arm_jp_ = st.leader_jp.head(DOF);
        theirJPOutputValue->setData(&their_arm_jp_);
        ctx.cancel_policy = (st.cancel_policy == 1.0);

        // set policy if p toggled
        policy_module_->receive(ctx, st);

        currentJpOutputValue->setData(&st.follower_jp);
        basePolicyJpOutputValue->setData(&st.policyJp);
        resPolicyJpOutputValue->setData(&st.resPolicyJp);
        refPolicyJtOutputValue->setData(&st.refPolicyTorque);

        // extTorqueIn.valueDefined() before setting a reference signal can cause bad feeling teleop
        // also cant put this before the policy read. TODO: see why
        setLocalStateValue(dyngravcompTorqueIn, st.follower_dyngravcomp_torque);
        setLocalStateValue(filteredEnvironmentTorqueIn, st.filtered_environment_torque);
        setLocalStateValue(environmentTorqueIn, st.environment_torque);

        // impedance results
        setLocalStateValue(basePolicyJtIn, st.policyJt);
        setLocalStateValue(resPolicyJtIn, st.resPolicyJt);
        setLocalStateValue(refTorquePolicyJtIn, st.refTorquePolicyJt);

        // this control will only be applied if the necessary module is loaded
        control = compute_control(ctx);
        st.control_torque = control;

        wam_torque_ = control.head(DOF);
        jtOutputValue->setData(&wam_torque_);
        setLocalStateValue(now_ns, st.timestamp);

        // send to leader then send to policy
        teleop_module_->send(st);
        // see how on_follower is used for the magic
        policy_module_->send(st);

        logging_module_->update(ctx);
    }

    jt_teleop_type control;

  private:
    DISALLOW_COPY_AND_ASSIGN(Follower);

    std::unique_ptr<GripperModule<DOF>> gripper_module_;

    gripper::gecko::GeckoGripper* gripper;

    std::unique_ptr<TeleopModule<DOF, FollowerUDPHandler<DOF>>> teleop_module_;
    std::unique_ptr<PolicyModule<DOF>> policy_module_;
    std::unique_ptr<DynamicsModule<DOF>> dynamics_module_;
    std::unique_ptr<LoggingModule<DOF>> logging_module_;
    ModuleManager<DOF> modules_;

    void makeModules() {
        teleop_module_.reset(new TeleopModule<DOF, FollowerUDPHandler<DOF>>(
            ModuleRole::Follower, config, &state,
            std::unique_ptr<FollowerUDPHandler<DOF>>(new FollowerUDPHandler<DOF>(
                config.network.leader_host, config.network.teleop_recv, config.network.teleop_send,
                config.data_routing.teleop_send_follower, config.data_routing.teleop_send_leader))));

        policy_module_.reset(new PolicyModule<DOF>(
            config,
            std::unique_ptr<PolicyUDPHandler<DOF>>(new PolicyUDPHandler<DOF>(
                config, config.policy.on_follower, config.policy_network.policy_follower_recv,
                config.policy_data_routing.policy_send_wam))));

        dynamics_module_.reset(new DynamicsModule<DOF>(config));
        logging_module_.reset(new LoggingModule<DOF>(ModuleRole::Follower, config));
        gripper_module_.reset(new GripperModule<DOF>(gripper, &state));

        modules_.add(teleop_module_.get());
        modules_.add(policy_module_.get());
        modules_.add(dynamics_module_.get());
        modules_.add(logging_module_.get());

        // Modules flagged in their config are already on when the node starts.
        if (config.policy.auto_load) policy_module_->load();
        if (config.dynamics.auto_load) dynamics_module_->load();
        if (config.logging.auto_load) logging_module_->load();

        // always start
        gripper_module_->start();
    }

    // A module produces the control torque it wants; here we simply sum the
    // torques of every loaded module.
    jt_teleop_type compute_control(const ControlContext<DOF>& ctx) {
        return modules_.sumTorque(ctx);
    }
};
