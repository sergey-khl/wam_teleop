#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include <iostream>

struct GripperConfig {
    bool usable;
};

namespace YAML {

template<> struct convert<GripperConfig> {
    static bool decode(const Node& node, GripperConfig& c) {
        c.usable = node["usable"].as<bool>();
        return true;
    }
};

} // namespace YAML

inline GripperConfig load_gripper_file_config(const std::string& file) {
    try {
        return YAML::LoadFile(file).as<GripperConfig>();
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading gripper config (" << file << "): " << e.what() << std::endl;
        throw;
    }
}
