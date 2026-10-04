#pragma once
#include <vector>
#include <string>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <iostream>

struct PolicyNetworkConfig {
    std::string policy_host;
    int policy_send;
    int policy_leader_recv;
    int policy_follower_recv;
};

// Which fields of the shared TeleopData are packed into the packet sent to the
// policy. Names must match a field in src/lib/utils/data_packets.h.
struct PolicyDataRoutingConfig {
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
    bool auto_load = false;
    std::string type;
    std::vector<double> clip_val;
    std::vector<double> clip_ref_torque;
    ActionConfig action;
};

// both leader and follower use same control law
struct DynamicsConfig {
    std::string law = "u2";
    bool auto_load = false;
};

struct PolicyFileConfig {
    PolicyNetworkConfig network;
    PolicyDataRoutingConfig data_routing;
    PolicyConfig policy;
    DynamicsConfig dynamics;
    double interp_hz;
    int slow_down_factor;
};

namespace YAML {

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
        if (node["auto_load"]) {
            c.auto_load = node["auto_load"].as<bool>();
        }
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

template<> struct convert<DynamicsConfig> {
    static bool decode(const Node& node, DynamicsConfig& c) {
        if (node["law"]) {
            c.law = node["law"].as<std::string>();
        }
        if (node["auto_load"]) {
            c.auto_load = node["auto_load"].as<bool>();
        }
        return true;
    }
};

} // namespace YAML

inline PolicyFileConfig load_policy_file_config(const std::string& file) {
    // A routing list is required and must not be empty.
    const auto require_list = [](const YAML::Node& node, const char* key) {
        std::vector<std::string> fields = node[key].as<std::vector<std::string>>();
        if (fields.empty()) {
            throw std::runtime_error(std::string("data_routing.") + key + " must not be empty");
        }
        return fields;
    };

    try {
        YAML::Node policy = YAML::LoadFile(file);

        PolicyFileConfig c;
        c.network.policy_host = policy["network"]["policy_host"].as<std::string>();
        c.network.policy_send = policy["network"]["policy_send"].as<int>();
        c.network.policy_leader_recv = policy["network"]["policy_leader_recv"].as<int>();
        c.network.policy_follower_recv = policy["network"]["policy_follower_recv"].as<int>();
        c.interp_hz = policy["interp_hz"].as<double>();
        c.slow_down_factor = policy["slow_down_factor"].as<int>();
        c.policy = policy["policy"].as<PolicyConfig>();
        c.data_routing.policy_send_wam = require_list(policy["data_routing"], "policy_send_wam");

        if (policy["dynamics"]) {
            c.dynamics = policy["dynamics"].as<DynamicsConfig>();
        }

        return c;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading policy config (" << file << "): " << e.what() << std::endl;
        throw;
    }
}
