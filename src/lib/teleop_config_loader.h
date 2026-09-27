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
    int policy_leader_recv;
    int policy_follower_recv;
};

struct PolicyGains {
    std::vector<double> kp;
    std::vector<double> ki;
    std::vector<double> kd;
    std::vector<double> control_signal_limit;
    std::vector<double> integrator_limit;
};

struct PolicyConfig {
    PolicyGains base;
    PolicyGains res;
    PolicyGains torque;
    bool on_leader;
    bool on_follower;
    std::string type;
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

struct TeleopConfig {
    NetworkConfig network;
    PolicyConfig policy;
    SyncMapping sync_mapping;
    RobotTeleopConfig leader, follower;
    HandleConfig handle;
    GripperConfig gripper;
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
        c.policy_leader_recv = node["policy_leader_recv"].as<int>();
        c.policy_follower_recv = node["policy_follower_recv"].as<int>();
        return true;
    }
};

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

template<> struct convert<PolicyGains> {
    static bool decode(const Node& node, PolicyGains& c) {
        c.kp = node["kp"].as<std::vector<double>>();
        c.ki = node["ki"].as<std::vector<double>>();
        c.kd = node["kd"].as<std::vector<double>>();
        c.control_signal_limit = node["control_signal_limit"].as<std::vector<double>>();
        c.integrator_limit = node["integrator_limit"].as<std::vector<double>>();
        return true;
    }
};

template<> struct convert<PolicyConfig> {
    static bool decode(const Node& node, PolicyConfig& c) {
        c.base = node["base"].as<PolicyGains>();
        c.res = node["res"].as<PolicyGains>();
        c.torque = node["torque"].as<PolicyGains>();
        c.on_leader = node["on_leader"].as<bool>();
        c.on_follower = node["on_follower"].as<bool>();
        c.type = node["type"].as<std::string>();
        if (c.type != "base" && c.type != "cr" && c.type != "dg") {
            std::cerr << "Policy type must be one of base, cr or dg. Got: " << c.type << std::endl;
            return false;
        }
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

template<> struct convert<TeleopConfig> {
    static bool decode(const Node& node, TeleopConfig& c) {
        c.network = node["network"].as<NetworkConfig>();
        c.policy = node["policy"].as<PolicyConfig>();
        c.sync_mapping = node["sync_mapping"].as<SyncMapping>();
        c.leader = node["leader"].as<RobotTeleopConfig>();
        c.follower = node["follower"].as<RobotTeleopConfig>();
        c.gripper = node["gripper"].as<GripperConfig>();
        c.handle = node["handle"].as<HandleConfig>();
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
