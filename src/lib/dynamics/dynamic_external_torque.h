// External torque is estimated using estimated dynamics.

#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

#include "../utils/data_packets.h"

template <size_t DOF>
class DynamicExternalTorque : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jt_type> wamTorqueSumIn;
    Input<jt_type> wamDynamicsIn;
    Output<jt_teleop_type> wamExternalTorqueOut;

    explicit DynamicExternalTorque(barrett::systems::ExecutionManager* em, const std::string& sysName = "DynamicExternalTorque")
        : System(sysName)
        , wamTorqueSumIn(this)
        , wamDynamicsIn(this)
        , wamExternalTorqueOut(this, &jtOutputValue) {

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~DynamicExternalTorque() {
        this->mandatoryCleanUp();
    }

  protected:
    typename Output<jt_teleop_type>::Value* jtOutputValue;
    jt_type jtSum;
    jt_type dynamics;
    jt_teleop_type externalTorque;

    virtual void operate() {
        jtSum = wamTorqueSumIn.getValue();
        dynamics = wamDynamicsIn.getValue();

        // only the arm has a measured torque; the wrist stays zero for now
        externalTorque.setZero();
        externalTorque.head(DOF) = jtSum - dynamics;
        jtOutputValue->setData(&externalTorque);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(DynamicExternalTorque);
};
