#pragma once
#include <boost/filesystem.hpp>
#include <cstdlib>
#include <iostream>
#include "teleop_config_loader.h"

void print_leader_banner(const TeleopConfig& config);
void print_follower_banner(const TeleopConfig& config);

std::string get_teleop_config_directory();

int read_can_port();
