#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

template <size_t DOF>
class PolicyTorque : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jt_type> wamExtTorqueIn;
    Input<jt_type> policyExtTorqueIn;
    Output<jt_type> extTorqueOutput; // human if on leader and environment if on follower
    Output<jt_type> policyTorqueScaleOutput;

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
    typename Output<jt_type>::Value* extTorqueOutputValue;
    typename Output<jt_type>::Value* policyTorqueScaleOutputValue;
    jt_type currPolicyTorqueScale;
    jt_type wamExtTorque;
    jt_type policyExtTorque;
    jt_type extTorque;
    jt_type normalized_ext_torque;
    jt_type nextPolicyTorqueScale;

    jt_type max_torques;

    // rate limit the scale
    static constexpr double maxDelta = 0.01;

    virtual void operate() {
        wamExtTorque = wamExtTorqueIn.getValue();
        policyExtTorque = policyExtTorqueIn.getValue();

        extTorque = wamExtTorque - currPolicyTorqueScale.asDiagonal() * policyExtTorque;

        max_torques << 3.5, 3, 3.5, 2;
        for (size_t i = 0; i < 4; ++i) {
            normalized_ext_torque[i] = std::abs(extTorque[i]) / max_torques[i]; // 1 means a lot of human, 0 is not
        }
        // flipped sigmoid. The higher the user input the lower the policy gains
        for (size_t i = 0; i < 4; ++i) {
            nextPolicyTorqueScale[i] = 1.0 / (1.0 + std::exp(8 * (normalized_ext_torque[i] - 0.7)));
        }
        // rate limit the torque scales
        for (size_t i = 0; i < 4; ++i) {
            double delta = nextPolicyTorqueScale[i] - currPolicyTorqueScale[i];

            if (delta > maxDelta) {
                delta = maxDelta;
            } else if (delta < -maxDelta) {
                delta = -maxDelta;
            }

            nextPolicyTorqueScale[i] = currPolicyTorqueScale[i] + delta;
        }
        currPolicyTorqueScale << nextPolicyTorqueScale;

        policyTorqueScaleOutputValue->setData(&nextPolicyTorqueScale);
        extTorqueOutputValue->setData(&extTorque);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(PolicyTorque);
};
