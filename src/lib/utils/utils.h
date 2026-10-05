#pragma once
#include <boost/filesystem.hpp>
#include <cstdlib>
#include <iostream>
#include <libconfig.h++>
#include "config_loader.h"

void print_leader_banner(const Config& config);
void print_follower_banner(const Config& config);

std::string get_teleop_config_directory();

int read_can_port();

// create libbarret gains settings from our teleop_config.yaml
template <size_t DOF, typename Controller>
void apply_gains(Controller& controller, const PolicyGains& gains) {
    typename Controller::unitless_type kp, ki, kd, int_limit, cs_limit;

    for (size_t i = 0; i < DOF; ++i) {
        kp[i] = gains.kp[i];
        ki[i] = gains.ki[i];
        kd[i] = gains.kd[i];
        int_limit[i] = gains.integrator_limit[i];
        cs_limit[i] = gains.control_signal_limit[i];
    }

    controller.setKp(kp);
    controller.setKi(ki);
    controller.setKd(kd);
    controller.setIntegratorLimit(int_limit);
    controller.setControlSignalLimit(cs_limit);
}
