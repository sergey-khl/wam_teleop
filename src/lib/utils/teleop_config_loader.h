#pragma once
#include <vector>
#include <string>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <iostream>

struct TeleopNetworkConfig {
    std::string leader_host;
    std::string follower_host;
    int teleop_send;
    int teleop_recv;
};

// Which fields of the shared TeleopData are packed into each outgoing teleop
// packet. Names must match a field in src/lib/utils/data_packets.h.
struct TeleopDataRoutingConfig {
    std::vector<std::string> teleop_send_leader;
    std::vector<std::string> teleop_send_follower;
};

struct SyncMapping {
    std::vector<double> scales, offsets;
};

struct RobotTeleopConfig {
    std::vector<double> sync_pos;
    bool vertical;
};

struct GripperConfig {
    bool usable;
};

struct HandleConfig {
    double torque_scaling;
    double minStiffness, maxStiffness, alpha;
};

// Everything decoded from teleop_config.yaml.
struct TeleopFileConfig {
    TeleopNetworkConfig network;
    TeleopDataRoutingConfig data_routing;
    SyncMapping sync_mapping;
    RobotTeleopConfig leader, follower;
    HandleConfig handle;
    GripperConfig gripper;
    double link_tolerance = 0.1;
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

template<> struct convert<GripperConfig> {
    static bool decode(const Node& node, GripperConfig& c) {
        c.usable = node["usable"].as<bool>();
        return true;
    }
};

template<> struct convert<SyncMapping> {
    static bool decode(const Node& node, SyncMapping& c) {
        c.scales = node["scales"].as<std::vector<double>>();
        c.offsets = node["offsets"].as<std::vector<double>>();
        return true;
    }
};

template<> struct convert<RobotTeleopConfig> {
    static bool decode(const Node& node, RobotTeleopConfig& c) {
        c.sync_pos = node["sync_pos"].as<std::vector<double>>();
        c.vertical = node["vertical"].as<bool>();
        return true;
    }
};

} // namespace YAML

inline TeleopFileConfig load_teleop_file_config(const std::string& file) {
    // A routing list is required and must not be empty.
    const auto require_list = [](const YAML::Node& node, const char* key) {
        std::vector<std::string> fields = node[key].as<std::vector<std::string>>();
        if (fields.empty()) {
            throw std::runtime_error(std::string("data_routing.") + key + " must not be empty");
        }
        return fields;
    };

    try {
        YAML::Node teleop = YAML::LoadFile(file);

        TeleopFileConfig c;
        c.network.leader_host = teleop["network"]["leader_host"].as<std::string>();
        c.network.follower_host = teleop["network"]["follower_host"].as<std::string>();
        c.network.teleop_send = teleop["network"]["teleop_send"].as<int>();
        c.network.teleop_recv = teleop["network"]["teleop_recv"].as<int>();
        c.sync_mapping = teleop["sync_mapping"].as<SyncMapping>();
        c.leader = teleop["leader"].as<RobotTeleopConfig>();
        c.follower = teleop["follower"].as<RobotTeleopConfig>();
        c.gripper = teleop["gripper"].as<GripperConfig>();
        c.handle = teleop["handle"].as<HandleConfig>();
        c.data_routing.teleop_send_leader = require_list(teleop["data_routing"], "teleop_send_leader");
        c.data_routing.teleop_send_follower = require_list(teleop["data_routing"], "teleop_send_follower");

        if (teleop["link_tolerance"]) {
            c.link_tolerance = teleop["link_tolerance"].as<double>();
        }

        return c;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading teleop config (" << file << "): " << e.what() << std::endl;
        throw;
    }
}
