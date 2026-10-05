#pragma once
#include <string>
#include <iostream>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

#include "teleop_config_loader.h"
#include "policy_config_loader.h"
#include "logging_config_loader.h"
#include "gripper_config_loader.h"
#include "handle_config_loader.h"
#include "dynamics_config_loader.h"

struct Config {
    // teleop_config.yaml
    TeleopNetworkConfig network;
    TeleopDataRoutingConfig data_routing;
    SyncMapping sync_mapping;
    RobotTeleopConfig leader, follower;
    double link_tolerance = 0.1;

    // policy_config.yaml
    PolicyNetworkConfig policy_network;
    PolicyDataRoutingConfig policy_data_routing;
    PolicyConfig policy;
    double interp_hz;
    int slow_down_factor;

    // logging_config.yaml
    LoggingConfig logging;

    // gripper_config.yaml, handle_config.yaml, dynamics_config.yaml
    GripperConfig gripper;
    HandleConfig handle;
    DynamicsConfig dynamics;
};

inline Config load_config(const std::string& config_dir) {
    try {
        const boost::filesystem::path dir(config_dir);

        TeleopFileConfig teleop = load_teleop_file_config((dir / "teleop_config.yaml").string());
        PolicyFileConfig policy = load_policy_file_config((dir / "policy_config.yaml").string());
        LoggingConfig logging = load_logging_file_config((dir / "logging_config.yaml").string());
        GripperConfig gripper = load_gripper_file_config((dir / "gripper_config.yaml").string());
        HandleConfig handle = load_handle_file_config((dir / "handle_config.yaml").string());
        DynamicsConfig dynamics = load_dynamics_file_config((dir / "dynamics_config.yaml").string());

        Config c;
        c.network = teleop.network;
        c.data_routing = teleop.data_routing;
        c.sync_mapping = teleop.sync_mapping;
        c.leader = teleop.leader;
        c.follower = teleop.follower;
        c.link_tolerance = teleop.link_tolerance;

        c.policy_network = policy.network;
        c.policy_data_routing = policy.data_routing;
        c.policy = policy.policy;
        c.interp_hz = policy.interp_hz;
        c.slow_down_factor = policy.slow_down_factor;

        c.logging = logging;
        c.gripper = gripper;
        c.handle = handle;
        c.dynamics = dynamics;
        return c;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading config from dir (" << config_dir << "): " << e.what() << std::endl;
        throw;
    }
}
