#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include <iostream>

struct HandleConfig {
    double torque_scaling;
    double minStiffness, maxStiffness, alpha;
};

namespace YAML {

template<> struct convert<HandleConfig> {
    static bool decode(const Node& node, HandleConfig& c) {
        c.torque_scaling = node["torque_scaling"].as<double>();
        c.minStiffness = node["minStiffness"].as<double>();
        c.maxStiffness = node["maxStiffness"].as<double>();
        c.alpha = node["alpha"].as<double>();
        return true;
    }
};

} // namespace YAML

inline HandleConfig load_handle_file_config(const std::string& file) {
    try {
        return YAML::LoadFile(file).as<HandleConfig>();
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading handle config (" << file << "): " << e.what() << std::endl;
        throw;
    }
}
