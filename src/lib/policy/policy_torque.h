#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

#include "../utils/data_packets.h"

// Scales the policy torque by how hard the operator/environment is pushing.
template <size_t DOF>
class PolicyTorque : public barrett::systems::System {
  public:
    Input<jt_teleop_type> wamExtTorqueIn;
    Input<jt_teleop_type> policyExtTorqueIn;
    Output<jt_teleop_type> extTorqueOutput; // human if on leader and environment if on follower
    Output<jt_teleop_type> policyTorqueScaleOutput;

    explicit PolicyTorque(barrett::systems::ExecutionManager* em, const std::string& sysName = "PolicyTorque")
        : System(sysName)
        , currPolicyTorqueScale(0.0)
        , wamExtTorqueIn(this)
        , policyExtTorqueIn(this)
        , extTorqueOutput(this, &extTorqueOutputValue)
        , policyTorqueScaleOutput(this, &policyTorqueScaleOutputValue) {

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~PolicyTorque() {
        this->mandatoryCleanUp();
    }

  protected:
    typename Output<jt_teleop_type>::Value* extTorqueOutputValue;
    typename Output<jt_teleop_type>::Value* policyTorqueScaleOutputValue;
    jt_teleop_type currPolicyTorqueScale;
    jt_teleop_type wamExtTorque;
    jt_teleop_type policyExtTorque;
    jt_teleop_type extTorque;
    jt_teleop_type normalized_ext_torque;
    jt_teleop_type nextPolicyTorqueScale;

    // rate limit the scale
    static constexpr double maxDelta = 0.001;

    virtual void operate() {
        wamExtTorque = wamExtTorqueIn.getValue();
        policyExtTorque = policyExtTorqueIn.getValue();

        extTorque = wamExtTorque - currPolicyTorqueScale.asDiagonal() * policyExtTorque;

        // Normalize at most the first four joints; remaining joints keep full gain.
        const double max_torques[4] = {3.5, 3.0, 3.5, 2.0};

        for (size_t i = 0; i < TELEOP_DOF; ++i) {
            nextPolicyTorqueScale[i] = 1.0;
        }
        for (size_t i = 0; i < TELEOP_DOF && i < 4; ++i) {
            normalized_ext_torque[i] = std::abs(extTorque[i]) / max_torques[i]; // 1 means a lot of human, 0 is not
            // flipped sigmoid. The higher the user input the lower the policy gains
            nextPolicyTorqueScale[i] = 1.0 / (1.0 + std::exp(8 * (normalized_ext_torque[i] - 0.7)));
        }

        // rate limit the torque scales
        for (size_t i = 0; i < TELEOP_DOF && i < 4; ++i) {
            double delta = nextPolicyTorqueScale[i] - currPolicyTorqueScale[i];

            if (delta > maxDelta) {
                delta = maxDelta;
            } else if (delta < -maxDelta) {
                delta = -maxDelta;
            }

            nextPolicyTorqueScale[i] = currPolicyTorqueScale[i] + delta;
        }
        currPolicyTorqueScale = nextPolicyTorqueScale;

        policyTorqueScaleOutputValue->setData(&nextPolicyTorqueScale);
        extTorqueOutputValue->setData(&extTorque);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(PolicyTorque);
};
