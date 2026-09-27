#pragma once

#include <string>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/units.h>
#include <barrett/math/traits.h>
#include <cmath>
#include <algorithm>
#include <cassert>

template <size_t DOF>
class TrajectorySmoother
    : public barrett::systems::SingleIO<
          typename barrett::units::JointPositions<DOF>::type,
          typename barrett::units::JointPositions<DOF>::type> {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
  /**
   * @param f Natural frequency (Hz). Higher f = faster response, less smoothing.
   * @param zeta Damping ratio. 1.0 = critically damped (no oscillation).
   * @param r Initial response. Higher r = faster initial reaction to new targets.
   * @param sysName The name of the system.
   */
  explicit TrajectorySmoother(double f = 2.0, double zeta = 2.0, double r = 1.0,
                              const std::string &sysName = "TrajectorySmoother")
      : barrett::systems::SingleIO<jp_type, jp_type>(sysName), f(f), zeta(zeta),
        r(r), isInitialized(false)
    {
        // Initialize state vectors to zero. They will be properly set on the first operate() call.
        xp.setZero();
        y.setZero();
        yd.setZero();
    }
  virtual ~TrajectorySmoother() { this->mandatoryCleanUp(); }

  /**
   * Sets the filter parameters and recalculates internal coefficients.
   * Can be called on-the-fly to re-tune the filter.
   */
  void setParameters(double f, double zeta, double r) {
    this->f = f;
    this->zeta = zeta;
    this->r = r;
    recalculateCoefficients();
  }

protected:
  // Filter state variables
  jp_type xp; // Previous raw target from input
  jp_type y;  // Current smoothed position (the output)
  jp_type yd; // Current smoothed velocity
  
  double dt;

  // Tuning parameters
  double f, zeta, r;

  // Internal coefficients calculated from parameters
  double k1, k2, k3;

  bool isInitialized;

  virtual void operate() {
    // Get the raw target from the system's input port
    assert(dt > 0.0 && "TrajectorySmoother: dt is not set. This should not happen.");
    const jp_type &raw_target = this->input.getValue();

    if (!isInitialized) {
        // Use the first target received as the initial state for everything.
        xp = raw_target;
        y = raw_target;
        yd.setZero();
        
        isInitialized = true;
    } else {
        // Estimate velocity of the raw target signal
        jp_type xd = (raw_target - xp) / dt;
        xp = raw_target; // Update previous target state

        // Clamp k2 to ensure stability if sample rate changes slightly
        double k2_stable = std::max(k2, 1.1 * (dt * dt / 4.0 + dt * k1 / 2.0));

        // Integrate the filter equations to find the new smoothed position
        y = y + dt * yd;
        yd = yd + dt * (raw_target + k3 * xd - y - k1 * yd) / k2_stable;
    }
    
    // Set the system's output port to our new smoothed position
    this->outputValue->setData(&y);
  }

  /**
   * Called by the system when it's connected to an execution manager.
   * Sets the sample period
   */
  virtual void onExecutionManagerChanged() {
    barrett::systems::SingleIO<jp_type, jp_type>::onExecutionManagerChanged();

    if (this->hasExecutionManager()) {
        assert(this->getExecutionManager()->getPeriod() > 0.0);
        dt = this->getExecutionManager()->getPeriod();
    } else {
        dt = 0.0; // Should not happen in a running system
    }

    recalculateCoefficients();
  }

private:
  /**
   * Calculates the internal filter constants k1, k2, k3 from f, zeta, r.
   * This is done only when parameters change, not in the real-time loop.
   */
  void recalculateCoefficients() {
    // Avoid division by zero if f is not set yet or dt is unknown
    if (f <= 0.0 || dt <= 0.0) return;

    double omega = 2.0 * M_PI * f; // Convert Hz to rad/s
    k1 = zeta / omega;
    k2 = 1.0 / (omega * omega);
    k3 = r * zeta / omega;
  }

  DISALLOW_COPY_AND_ASSIGN(TrajectorySmoother);
};

