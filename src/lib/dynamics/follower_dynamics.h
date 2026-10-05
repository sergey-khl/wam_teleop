#pragma once
#ifndef FOLLOWER_DYNAMICS_H_
#define FOLLOWER_DYNAMICS_H_

#include <algorithm>
#include <iostream>
#include <string>

#include <eigen3/Eigen/Dense>
#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/math/kinematics.h>

#include "../utils/dynamics_config_loader.h"
#include "regressor_W_4dof.h"
#include "regressor_W_7dof.h"
#include "follower_beta/slax_skid_hand_4dof.h"
#include "follower_beta/slax_skid_hand_7dof.h"

template <size_t DOF>
class FollowerDynamics : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jp_type> jpInputDynamics;
    Input<jv_type> jvInputDynamics;
    Input<ja_type> jaInputDynamics;
    Output<jt_type> dynamicsFeedFWD;

    explicit FollowerDynamics(barrett::systems::ExecutionManager* em, const DynamicsConfig& config)
        : jpInputDynamics(this)
        , jvInputDynamics(this)
        , jaInputDynamics(this)
        , dynamicsFeedFWD(this, &dynamicsFeedFWDValue)
        , model_dof_(config.dof)
        , beta_name_(config.follower_beta) {
        (void)em;
        if (config.dof != 4 && config.dof != 7) {
            std::cerr << "WARNING: dynamics.dof must be 4 or 7, using 4" << std::endl;
        }
    }

    virtual ~FollowerDynamics() {
        this->mandatoryCleanUp();
    }

  protected:
    typename Output<jt_type>::Value* dynamicsFeedFWDValue;
    size_t model_dof_;
    std::string beta_name_;
    jt_type dynFeedFWD;

    virtual void operate() {
        const Eigen::VectorXd q = this->jpInputDynamics.getValue().head(model_dof_);
        const Eigen::VectorXd dq = this->jvInputDynamics.getValue().head(model_dof_);
        Eigen::VectorXd ddq = this->jaInputDynamics.getValue().head(model_dof_);

        Eigen::VectorXd feed_fwd;
        if (model_dof_ == 7) {
            ddq *= 0.14;
            feed_fwd = dynamics7::calculate_W_eigen(q, dq, ddq) * beta7(beta_name_);
        } else {
            ddq *= 0.25;
            feed_fwd = dynamics4::calculate_W_eigen(q, dq, ddq) * beta4(beta_name_);
        }

        dynFeedFWD.setZero();
        dynFeedFWD.head(model_dof_) = feed_fwd;
        this->dynamicsFeedFWDValue->setData(&dynFeedFWD);
    }

  private:
    static Eigen::VectorXd beta4(const std::string& name) {
        return dynamics4::initialize_follower_beta();
    }

    static Eigen::VectorXd beta7(const std::string& name) {
        return dynamics7::initialize_follower_beta();
    }

    DISALLOW_COPY_AND_ASSIGN(FollowerDynamics);
};

#endif /* FOLLOWER_DYNAMICS_H_ */
