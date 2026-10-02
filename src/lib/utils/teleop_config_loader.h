#pragma once
#include <vector>
#include <string>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <boost/filesystem.hpp>

struct NetworkConfig {
    // teleop_config.yaml
    std::string leader_host;
    std::string follower_host;
    int teleop_send;
    int teleop_recv;
    // policy_config.yaml
    std::string policy_host;
    int policy_send;
    int policy_leader_recv;
    int policy_follower_recv;
};

// Which fields of the shared TeleopData are packed into each outgoing packet.
// Names must match a field in src/lib/utils/data_packets.h.
struct DataRoutingConfig {
    std::vector<std::string> teleop_send_leader;
    std::vector<std::string> teleop_send_follower;
    std::vector<std::string> policy_send_wam;
};

struct PolicyGains {
    std::vector<double> kp;
    std::vector<double> ki;
    std::vector<double> kd;
    std::vector<double> control_signal_limit;
    std::vector<double> integrator_limit;
};

// policy action stream (base/res/dg):
struct ActionStreamConfig {
    std::vector<std::string> fields;
    int horizon;
    double uninterp_hz;
};

struct ActionConfig {
    ActionStreamConfig base;
    ActionStreamConfig res;
    ActionStreamConfig dg;
};

struct PolicyConfig {
    PolicyGains base;
    PolicyGains res;
    PolicyGains torque;
    bool on_leader;
    bool on_follower;
    std::string type;
    std::vector<double> clip_val;
    std::vector<double> clip_ref_torque;
    ActionConfig action;
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
    DataRoutingConfig data_routing;
    RobotTeleopConfig leader, follower;
    HandleConfig handle;
    GripperConfig gripper;
    double interp_hz;
    int slow_down_factor;
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

template<> struct convert<ActionStreamConfig> {
    static bool decode(const Node& node, ActionStreamConfig& c) {
        c.fields = node["fields"].as<std::vector<std::string>>();
        c.horizon = node["horizon"].as<int>();
        c.uninterp_hz = node["uninterp_hz"].as<double>();
        return true;
    }
};

template<> struct convert<ActionConfig> {
    static bool decode(const Node& node, ActionConfig& c) {
        c.base = node["base"].as<ActionStreamConfig>();
        c.res = node["res"].as<ActionStreamConfig>();
        c.dg = node["dg"].as<ActionStreamConfig>();
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
        c.clip_val = node["clip_val"].as<std::vector<double>>();
        c.clip_ref_torque = node["clip_ref_torque"].as<std::vector<double>>();
        c.action = node["action"].as<ActionConfig>();
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

} // namespace YAML

// A routing list is required and must not be empty.
inline std::vector<std::string> require_string_list(const YAML::Node& node, const char* key) {
    std::vector<std::string> fields = node[key].as<std::vector<std::string>>();
    if (fields.empty()) {
        throw std::runtime_error(std::string("data_routing.") + key + " must not be empty");
    }
    return fields;
}

inline TeleopConfig load_teleop_config(const std::string& config_dir) {
    try {
        const boost::filesystem::path dir(config_dir);
        YAML::Node teleop = YAML::LoadFile((dir / "teleop_config.yaml").string());
        YAML::Node policy = YAML::LoadFile((dir / "policy_config.yaml").string());

        TeleopConfig c;

        // teleop_config.yaml
        c.network.leader_host = teleop["network"]["leader_host"].as<std::string>();
        c.network.follower_host = teleop["network"]["follower_host"].as<std::string>();
        c.network.teleop_send = teleop["network"]["teleop_send"].as<int>();
        c.network.teleop_recv = teleop["network"]["teleop_recv"].as<int>();
        c.sync_mapping = teleop["sync_mapping"].as<SyncMapping>();
        c.leader = teleop["leader"].as<RobotTeleopConfig>();
        c.follower = teleop["follower"].as<RobotTeleopConfig>();
        c.gripper = teleop["gripper"].as<GripperConfig>();
        c.handle = teleop["handle"].as<HandleConfig>();
        c.data_routing.teleop_send_leader = require_string_list(teleop["data_routing"], "teleop_send_leader");
        c.data_routing.teleop_send_follower = require_string_list(teleop["data_routing"], "teleop_send_follower");

        // policy_config.yaml
        c.network.policy_host = policy["network"]["policy_host"].as<std::string>();
        c.network.policy_send = policy["network"]["policy_send"].as<int>();
        c.network.policy_leader_recv = policy["network"]["policy_leader_recv"].as<int>();
        c.network.policy_follower_recv = policy["network"]["policy_follower_recv"].as<int>();
        c.interp_hz = policy["interp_hz"].as<double>();
        c.slow_down_factor = policy["slow_down_factor"].as<int>();
        c.policy = policy["policy"].as<PolicyConfig>();
        c.data_routing.policy_send_wam = require_string_list(policy["data_routing"], "policy_send_wam");

        return c;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading config from dir (" << config_dir << "): " << e.what() << std::endl;
        throw;
    }
}
