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
        law_ = parseLaw(config_.dynamics.law);
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

        if (law_ == "u1") {
            u = 0.0 * cur;
        } else if (law_ == "u2") {
            u = dyn - grav;
        } else if (law_ == "u3") {
            u = -0.5 * ref;
        } else if (law_ == "u4") {
            u = -0.5 * ref + dyn - grav;
        } else if (law_ == "u5") {
            u = -0.5 * ref - 0.15 * (ref + cur);
        } else if (law_ == "u6") {
            u = -0.1 * ref - 0.03 * (ref + cur) + dyn - grav;
        } else if (law_ == "u7") {
            u = -0.0 * cur;
        } else if (law_ == "u8") {
            u = -0.0 * (ref + cur);
        } else if (law_ == "u9") {
            u = -0.5 * ref;
        } else if (law_ == "u10") {
            u = -0.5 * ref;
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
    static const char* parseLaw(const std::string& law) {
        static const char* kLaws[] = {"u1", "u2", "u3", "u4", "u5", "u6", "u7", "u8", "u9", "u10"};
        for (const char* candidate : kLaws) {
            if (law == candidate) return candidate;
        }
        // also accept the bare number ("2")
        for (int i = 1; i <= 10; ++i) {
            if (law == std::to_string(i)) return kLaws[i - 1];
        }
        std::cerr << "WARNING: unknown dynamics.law '" << law << "'; defaulting to 0 output" << std::endl;
        return kLaws[0];
    }

    Config config_;
    const char* law_ = "u1";
};
