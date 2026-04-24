#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

template <size_t DOF>
class FollowerVerticalDynamics : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    // Inputs
    Input<jt_type> followerDynamicsIn;   // τ_follow
    Input<jt_type> horizontalGravityIn;  // τ_g_horizontal
    Input<jt_type> gravityIn;            // τ_g (full gravity to re-add)

    // Output
    Output<jt_type> followerVerticalDynamicsOut; // τ_vertical = τ_follow − τ_g_horizontal + τ_g

    explicit FollowerVerticalDynamics(barrett::systems::ExecutionManager* em,
                                      const std::string& sysName = "FollowerVerticalDynamics")
        : barrett::systems::System(sysName)
        , followerDynamicsIn(this)
        , horizontalGravityIn(this)
        , gravityIn(this)
        , followerVerticalDynamicsOut(this, &vdynOutputValue)
    {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~FollowerVerticalDynamics() {
        this->mandatoryCleanUp();
    }

  protected:
    // Output value holder
    typename Output<jt_type>::Value* vdynOutputValue;

    // Working buffers
    jt_type followerDynamics;   // τ_follow
    jt_type horizontalGravity;  // τ_g_horizontal
    jt_type gravity;            // τ_g
    jt_type verticalDynamics;   // τ_follow − τ_g_horizontal + τ_g

    virtual void operate() {
        followerDynamics = followerDynamicsIn.getValue();
        horizontalGravity = horizontalGravityIn.getValue();
        gravity = gravityIn.getValue();

        // Compute: verticalDynamics = followerDynamics - horizontalGravity + gravity
        verticalDynamics = followerDynamics - horizontalGravity + gravity;

        vdynOutputValue->setData(&verticalDynamics);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(FollowerVerticalDynamics);
};
