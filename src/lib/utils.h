#pragma once
#include <boost/filesystem.hpp>
#include <cstdlib>
#include <iostream>
#include <libconfig.h++>
#include "teleop_config_loader.h"

void print_leader_banner(const TeleopConfig& config);
void print_follower_banner(const TeleopConfig& config);

std::string get_teleop_config_directory();

int read_can_port();

// create libbarret gains settings from our teleop_config.yaml
template <size_t DOF, typename Controller>
void apply_gains(Controller& controller, const PolicyGains& gains);
