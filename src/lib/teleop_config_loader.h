#pragma once
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <boost/filesystem.hpp>

struct NetworkConfig {
    std::string leader_host;
    std::string follower_host;
    int teleop_send;
    int teleop_recv;
    std::string policy_host;
    int policy_send;
    int policy_recv;
    bool policy_send_active;
};
struct PolicyConfig {
    std::vector<double> kp;
    std::vector<double> ki;
    std::vector<double> kd;
    std::vector<double> control_signal_limit;
    std::vector<double> integrator_limit;
};

struct HapticsConfig {
    double torque_scaling;
    double minStiffness, maxStiffness, alpha;
};


struct SyncMapping {
    std::vector<double> scales, offsets;
};

struct RobotTeleopConfig {
    std::vector<double> sync_pos;
    HapticsConfig haptics;
    bool vertical;
};

struct TeleopConfig {
    NetworkConfig network;
    PolicyConfig policy;
    SyncMapping sync_mapping;
    RobotTeleopConfig leader, follower;
};

namespace YAML {

template<> struct convert<NetworkConfig> {
    static bool decode(const Node& node, NetworkConfig& c) {
        c.leader_host = node["leader_host"].as<std::string>();
        c.follower_host = node["follower_host"].as<std::string>();
        c.teleop_send = node["teleop_send"].as<int>();
        c.teleop_recv = node["teleop_recv"].as<int>();
        c.policy_host = node["policy_host"].as<std::string>();
        c.policy_send = node["policy_send"].as<int>();
        c.policy_recv = node["policy_recv"].as<int>();
        c.policy_send_active = node["policy_send_active"].as<bool>();
        return true;
    }
};

template<> struct convert<HapticsConfig> {
    static bool decode(const Node& node, HapticsConfig& c) {
        if (!node) return true; // Follower might not have this
        c.torque_scaling = node["torque_scaling"].as<double>();
        c.minStiffness = node["minStiffness"].as<double>();
        c.maxStiffness = node["maxStiffness"].as<double>();
        c.alpha = node["alpha"].as<double>();
        return true;
    }
};

template<> struct convert<PolicyConfig> {
    static bool decode(const Node& node, PolicyConfig& c) {
        c.kp = node["kp"].as<std::vector<double>>();
        c.ki = node["ki"].as<std::vector<double>>();
        c.kd = node["kd"].as<std::vector<double>>();
        c.control_signal_limit = node["control_signal_limit"].as<std::vector<double>>();
        c.integrator_limit = node["integrator_limit"].as<std::vector<double>>();
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
        if (node["haptics"]) {
            c.haptics = node["haptics"].as<HapticsConfig>();
        }
        c.vertical = node["vertical"].as<bool>();
        return true;
    }
};

template<> struct convert<TeleopConfig> {
    static bool decode(const Node& node, TeleopConfig& c) {
        c.network = node["network"].as<NetworkConfig>();
        c.policy = node["policy"].as<PolicyConfig>();
        c.sync_mapping = node["sync_mapping"].as<SyncMapping>();
        c.leader = node["leader"].as<RobotTeleopConfig>();
        c.follower = node["follower"].as<RobotTeleopConfig>();
        return true;
    }
};

} // namespace YAML

inline TeleopConfig load_teleop_config(const std::string& config_dir) {
    try {
        boost::filesystem::path main_config_path = boost::filesystem::path(config_dir) / "teleop_config.yaml";
        YAML::Node main_yaml = YAML::LoadFile(main_config_path.string());

        return main_yaml.as<TeleopConfig>();
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading from config dir (" << config_dir << "): " << e.what() << std::endl;
        throw;
    }
}
