// External torque is estimated using estimated dynamics.

#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

template <size_t DOF>
class PolicyExternalTorque : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<ja_type> policyJaIn;
    Input<jt_type> policyTorqueIn;
    // Input<jt_type> policyTorqueScaleIn;
    Output<ja_type> output;

    explicit PolicyExternalTorque(barrett::systems::ExecutionManager* em, const std::string& sysName = "PolicyExternalTorque")
        : System(sysName)
        , policyJaIn(this)
        , policyTorqueIn(this)
        // , policyTorqueScaleIn(this)
        , output(this, &outputValue) {

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~PolicyExternalTorque() {
        this->mandatoryCleanUp();
    }

  protected:
    typename Output<ja_type>::Value* outputValue;
    ja_type policyJa;
    ja_type policyTorque;
    // jt_type policyTorqueScale;
    ja_type externalTorque;

    virtual void operate() {
        policyJa = policyJaIn.getValue();
        policyTorque << policyTorqueIn.getValue();
        // policyTorqueScale = policyTorqueScaleIn.getValue();
        // externalTorque = compTorque - policyTorque + policyTorqueScale.asDiagonal() * policyTorque;
        externalTorque = policyTorque + policyJa;
        // externalTorque = compTorque - policyTorqueScale.asDiagonal() * policyTorque;
        outputValue->setData(&externalTorque);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(PolicyExternalTorque);
};
