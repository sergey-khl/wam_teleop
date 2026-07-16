// External torque is estimated using only gravity rather than dynamics.

#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

template <size_t DOF>
class ExternalTorque : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jt_type> wamTorqueSumIn;
    Input<jt_type> wamGravityIn;
    Output<jt_type> wamExternalTorqueOut;

    explicit ExternalTorque(barrett::systems::ExecutionManager* em, const std::string& sysName = "ExternalTorque")
        : System(sysName)
        , wamTorqueSumIn(this)
        , wamGravityIn(this)
        , wamExternalTorqueOut(this, &jtOutputValue) {

        if (em != NULL) {
            em->startManaging(*this);
        }
    }
    
    virtual ~ExternalTorque() {
        this->mandatoryCleanUp();
    }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    jt_type jtSum;
    jt_type gravity;
    jt_type externalTorque;

    virtual void operate() {

        jtSum = wamTorqueSumIn.getValue();
        gravity = wamGravityIn.getValue();
        externalTorque = jtSum - gravity;
        jtOutputValue->setData(&externalTorque);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(ExternalTorque);
};
