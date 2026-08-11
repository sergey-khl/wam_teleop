#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

template <size_t DOF>
class PolicyTorqueScale : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jt_type> humanExtTorqueIn;
    Output<jt_type> output;

    explicit PolicyTorqueScale(barrett::systems::ExecutionManager* em, const std::string& sysName = "PolicyTorqueScale")
        : System(sysName)
        , humanExtTorqueIn(this)
        , output(this, &jtOutputValue) {

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~PolicyTorqueScale() {
        this->mandatoryCleanUp();
    }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    jt_type humanExtTorque;
    jt_type normalized_ext_torque;
    jt_type policyTorqueScale;

    jt_type max_torques;

    virtual void operate() {
        // because of cyclic dependency we ensure the ext torque exists first 
        if (humanExtTorqueIn.valueDefined()) {
            humanExtTorque = humanExtTorqueIn.getValue();
        } else {
            humanExtTorque << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        max_torques << 7, 5, 3, 2;
        for (size_t i = 0; i < 4; ++i) {
            normalized_ext_torque[i] = std::abs(humanExtTorque[i]) / max_torques[i]; // 1 means a lot of human, 0 is not
        }
        // flipped sigmoid. The higher the user input the lower the policy gains
        for (size_t i = 0; i < 4; ++i) {
            policyTorqueScale[i] = 1.0 / (1.0 + std::exp(6 * normalized_ext_torque[i] - 3));
        }
        jtOutputValue->setData(&policyTorqueScale);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(PolicyTorqueScale);
};
