#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

template <size_t DOF>
class PolicyExternalTorque : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jt_type> wamExtTorqueIn;
    Input<jt_type> policyExtTorqueIn;
    Output<jt_type> output;

    explicit PolicyExternalTorque(barrett::systems::ExecutionManager* em, const std::string& sysName = "PolicyExternalTorque")
        : System(sysName)
        , wamExtTorqueIn(this)
        , policyExtTorqueIn(this)
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
    jt_type wamExtTorque;
    jt_type policyExtTorque;
    jt_type externalTorque;

    virtual void operate() {
        wamExtTorque = wamExtTorqueIn.getValue();
        policyExtTorque = policyExtTorqueIn.getValue();
        externalTorque = wamExtTorque - policyExtTorque;
        jtOutputValue->setData(&externalTorque);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(PolicyExternalTorque);
};
