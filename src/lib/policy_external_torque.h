// External torque is estimated using estimated dynamics.

#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

template <size_t DOF>
class PolicyExternalTorque : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jt_type> wamCompTorqIn;
    Input<jt_type> policyTorqueIn;
    Input<jt_type> policyTorqueScaleIn;
    Output<jt_type> output;

    explicit PolicyExternalTorque(barrett::systems::ExecutionManager* em, const std::string& sysName = "PolicyExternalTorque")
        : System(sysName)
        , wamCompTorqIn(this)
        , policyTorqueIn(this)
        , policyTorqueScaleIn(this)
        , output(this, &jtOutputValue) {

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~PolicyExternalTorque() {
        this->mandatoryCleanUp();
    }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    jt_type compTorque;
    jt_type policyTorque;
    jt_type policyTorqueScale;
    jt_type externalTorque;

    virtual void operate() {
        compTorque = wamCompTorqIn.getValue();
        policyTorque = policyTorqueIn.getValue();
        policyTorqueScale = policyTorqueScaleIn.getValue();
        // externalTorque = compTorque - policyTorque + policyTorqueScale.asDiagonal() * policyTorque;
        externalTorque = policyTorqueScale.asDiagonal() * policyTorque;
        jtOutputValue->setData(&externalTorque);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(PolicyExternalTorque);
};
