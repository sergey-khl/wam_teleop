#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

template <size_t DOF>
class LeaderVerticalDynamics : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    // Inputs
    Input<jt_type> leaderDynamicsIn;     // τ_leader
    Input<jt_type> horizontalGravityIn;  // τ_g_horizontal
    Input<jt_type> gravityIn;            // τ_g (full gravity to re-add)

    // Output
    Output<jt_type> leaderVerticalDynamicsOut; // τ_vertical = τ_leader − τ_g_horizontal + τ_g

    explicit LeaderVerticalDynamics(barrett::systems::ExecutionManager* em,
                                    const std::string& sysName = "LeaderVerticalDynamics")
        : barrett::systems::System(sysName)
        , leaderDynamicsIn(this)
        , horizontalGravityIn(this)
        , gravityIn(this)
        , leaderVerticalDynamicsOut(this, &vdynOutputValue)
    {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~LeaderVerticalDynamics() {
        this->mandatoryCleanUp();
    }

  protected:
    // Output value holder
    typename Output<jt_type>::Value* vdynOutputValue;

    // Working buffers
    jt_type leaderDynamics;     // τ_leader
    jt_type horizontalGravity;  // τ_g_horizontal
    jt_type gravity;            // τ_g
    jt_type verticalDynamics;   // τ_leader − τ_g_horizontal + τ_g

    virtual void operate() {
        leaderDynamics = leaderDynamicsIn.getValue();
        horizontalGravity = horizontalGravityIn.getValue();
        gravity = gravityIn.getValue();

        // Compute: verticalDynamics = leaderDynamics - horizontalGravity + gravity
        verticalDynamics = leaderDynamics - horizontalGravity + gravity;

        vdynOutputValue->setData(&verticalDynamics);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(LeaderVerticalDynamics);
};