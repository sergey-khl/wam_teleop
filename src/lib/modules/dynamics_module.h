#pragma once

#include <cstdlib>
#include <iostream>
#include <string>

#include "module.h"
#include "../utils/config_loader.h"

template <size_t DOF>
class DynamicsModule : public Module<DOF> {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    explicit DynamicsModule(const Config& config) : config_(config) {
        law_ = config_.dynamics.law;
    }

    char key() const override { return 'd'; }
    const char* name() const override { return "dynamics"; }
    bool producesTorque() const override { return true; }

    const char* law() const { return law_; }

    void onLoad() override { std::cout << "dynamics module loaded (" << law_ << ")" << std::endl; }
    void onUnload() override { std::cout << "dynamics module unloaded" << std::endl; }

    jt_teleop_type torque(const ControlContext<DOF>& ctx) override {
        const jt_type ref = ctx.ref_ext_torque->head(DOF);
        const jt_type cur = ctx.cur_ext_torque->head(DOF);
        const jt_type dyn = ctx.cur_dyn->head(DOF);
        const jt_type grav = ctx.cur_grav->head(DOF);

        jt_type u = jt_type::Zero();

        if (law_ == "u1") { // zero feedforward
            u = 0.0 * cur;
        } else if (law_ == "u2") { // P-P with dynamic compensation
            u = dyn - grav;
        } else if (law_ == "u3") {  // PF-PF with ref external torque feedback
            u = -0.5 * ref;
        } else if (law_ == "u4") { // PF-PF with ref external torque feedback and dynamic compensation (Lawrence's perfect transparency architecture);
            u = -0.5 * ref + dyn - grav;
        } else if (law_ == "u5") { // PF-PF with ref external torque and cur external torque feedback
            u = -0.5 * ref - 0.15 * (ref + cur);
        } else if (law_ == "u6") { // best performance
            u = -0.1 * ref - 0.03 * (ref + cur) + dyn - grav;
        } else {
            // default / fallback: 0
            std::cerr << "WARNING: invalid control law." << std::endl;
            u.setZero();
        }

        // j5-7 does not give a usable ext torque
        for (size_t i = 4; i < DOF && i < 7; ++i) {
            u[i] = 0.0;
        }

        // arm torque in the head; wrist torque will fill the tail later
        jt_teleop_type full = jt_teleop_type::Zero();
        full.head(DOF) = u;
        return full;
    }

  private:
    Config config_;
    const char* law_ = "u1";
};
