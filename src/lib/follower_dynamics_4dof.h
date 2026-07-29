// This file calculates the inverse dynamics of only the first four dofs of the WAM.

#ifndef FOLLOWER_DYNAMICS_H_
#define FOLLOWER_DYNAMICS_H_

#pragma once
#include <eigen3/Eigen/Dense>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/math/kinematics.h> 
#include "follower_beta_slax_skid_hand_4dof.h"
#include "regressor_W_4dof.h"

using namespace barrett;

template<size_t DOF>
class FollowerDynamics: public systems::System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	/* Torque*/
public:
	Input<jp_type> jpInputDynamics;
	Input<jv_type> jvInputDynamics;
    Input<ja_type> jaInputDynamics;

public:
	Output<jt_type> dynamicsFeedFWD;
protected:
	typename Output<jt_type>::Value* dynamicsFeedFWDValue;

public:
	explicit FollowerDynamics(systems::ExecutionManager* em) :
			jpInputDynamics(this), jvInputDynamics(this), jaInputDynamics(this), dynamicsFeedFWD(this,
					&dynamicsFeedFWDValue) {
	//		      em->startManaging(*this);
//		    }
	}

	virtual ~FollowerDynamics() {
		this->mandatoryCleanUp();
	}

protected:
	jp_type tmp_theta_pos;
	jv_type tmp_theta_vel;
	ja_type tmp_theta_acc;
	Eigen::Matrix<double, 4, 30> W;
	Eigen::Matrix<double, 30, 1> beta;
	Eigen::Matrix<double, 4, 1> ThetaInput;
	Eigen::Matrix<double, 4, 1> ThetadotInput;
	Eigen::Matrix<double, 4, 1> ThetaddotInput;
	Eigen::Matrix<double, 4, 1> FeedFwd;
	jt_type dynFeedFWD;

	virtual void operate() {
		tmp_theta_pos = this->jpInputDynamics.getValue();
		ThetaInput << tmp_theta_pos[0], tmp_theta_pos[1], tmp_theta_pos[2], tmp_theta_pos[3];
        tmp_theta_vel = this->jvInputDynamics.getValue();
		ThetadotInput << tmp_theta_vel[0], tmp_theta_vel[1], tmp_theta_vel[2], tmp_theta_vel[3];
        tmp_theta_acc = this->jaInputDynamics.getValue();
		ThetaddotInput << 0.25 * tmp_theta_acc[0], 0.25 * tmp_theta_acc[1],  0.25 * tmp_theta_acc[2], 0.25 * tmp_theta_acc[3];
		W = calculate_W_eigen(ThetaInput, ThetadotInput, ThetaddotInput);

		beta = initialize_follower_beta();
		
		FeedFwd = W * beta;

		dynFeedFWD << FeedFwd[0], FeedFwd[1], FeedFwd[2], FeedFwd[3], 0, 0, 0;

		this->dynamicsFeedFWDValue->setData(&dynFeedFWD);

	}
private:
	DISALLOW_COPY_AND_ASSIGN(FollowerDynamics);
};
#endif /* FOLLOWER_DYNAMICS_H_ */
