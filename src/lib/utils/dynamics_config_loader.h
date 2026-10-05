#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include <iostream>

struct DynamicsConfig {
    std::string law = "u2";
    bool auto_load = false;
    int dof = 4; // size of the regressor/beta, independent of the arm DOF
    std::string leader_beta = "zeus_bwrist";
    std::string follower_beta = "slax_skid_hand";
};

namespace YAML {

template<> struct convert<DynamicsConfig> {
    static bool decode(const Node& node, DynamicsConfig& c) {
        if (node["law"]) {
            c.law = node["law"].as<std::string>();
        }
        if (node["auto_load"]) {
            c.auto_load = node["auto_load"].as<bool>();
        }
        if (node["dof"]) {
            c.dof = node["dof"].as<int>();
        }
        if (node["leader_beta"]) {
            c.leader_beta = node["leader_beta"].as<std::string>();
        }
        if (node["follower_beta"]) {
            c.follower_beta = node["follower_beta"].as<std::string>();
        }
        return true;
    }
};

} // namespace YAML

inline DynamicsConfig load_dynamics_file_config(const std::string& file) {
    try {
        return YAML::LoadFile(file).as<DynamicsConfig>();
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading dynamics config (" << file << "): " << e.what() << std::endl;
        throw;
    }
}
