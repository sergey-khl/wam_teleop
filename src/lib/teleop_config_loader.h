#pragma once
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include "udp_handler.h" 
#include <iostream>
#include <boost/filesystem.hpp>

struct NetworkConfig {
    std::string leader_host;
    std::string follower_host;
    int teleop_send;
    int teleop_recv;
    std::string inference_host;
    int leader_inference_send;
    int follower_inference_send;
    int inference_recv;
    OperationMode mode;
};

struct TeleopGains {
    std::vector<double> kp, kd, cf;
};

struct HapticsConfig {
    double trigger_rest_pos, target_velocity, torque_scaling;
    double minStiffness, maxStiffness, alpha;
    double j7_joy_deadband, j7_max_velocity_rad_s;
};

struct SyncMapping {
    std::vector<double> scales, offsets;
};

struct GripperConfig {
    std::string type = "gecko";
    std::string port = "can0";
    bool force_init = false;
    double finger_velocity_scale = 1.0;
    double spread_velocity = 0.0;
    double velocity_filter_alpha = 0.18;
    double velocity_deadband = 0.02;
    double velocity_slew_rate = 1.5;
    int bumper_double_click_ms = 350;
};

struct RobotTeleopConfig {
    std::vector<double> sync_pos;
    TeleopGains gains;
    HapticsConfig haptics;
};

struct TeleopConfig {
    NetworkConfig network;
    SyncMapping sync_mapping;
    GripperConfig gripper;
    RobotTeleopConfig leader, follower;
};

namespace YAML {

template<> struct convert<NetworkConfig> {
    static bool decode(const Node& node, NetworkConfig& c) {
        c.leader_host = node["leader_host"].as<std::string>();
        c.follower_host = node["follower_host"].as<std::string>();
        c.teleop_send = node["teleop_send"].as<int>();
        c.teleop_recv = node["teleop_recv"].as<int>();
        c.inference_host = node["inference_host"].as<std::string>();
        c.leader_inference_send = node["leader_inference_send"].as<int>();
        c.follower_inference_send = node["follower_inference_send"].as<int>();
        c.inference_recv = node["inference_recv"].as<int>();
        c.mode = static_cast<OperationMode>(node["mode"].as<int>());
        return true;
    }
};

template<> struct convert<TeleopGains> {
    static bool decode(const Node& node, TeleopGains& c) {
        c.kp = node["kp"].as<std::vector<double>>();
        c.kd = node["kd"].as<std::vector<double>>();
        c.cf = node["cf"].as<std::vector<double>>();
        return true;
    }
};

template<> struct convert<HapticsConfig> {
    static bool decode(const Node& node, HapticsConfig& c) {
        if (!node) return true; // Follower might not have this
        c.trigger_rest_pos = node["trigger_rest_pos"].as<double>();
        c.target_velocity = node["target_velocity"].as<double>();
        c.torque_scaling = node["torque_scaling"].as<double>();
        c.minStiffness = node["minStiffness"].as<double>();
        c.maxStiffness = node["maxStiffness"].as<double>();
        c.alpha = node["alpha"].as<double>();
        c.j7_joy_deadband = node["j7_joy_deadband"].as<double>();
        c.j7_max_velocity_rad_s = node["j7_max_velocity_rad_s"].as<double>();
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

template<> struct convert<GripperConfig> {
    static bool decode(const Node& node, GripperConfig& c) {
        if (!node) return true;
        if (node["type"]) c.type = node["type"].as<std::string>();
        if (node["port"]) c.port = node["port"].as<std::string>();
        if (node["force_init"]) c.force_init = node["force_init"].as<bool>();
        if (node["finger_velocity_scale"]) c.finger_velocity_scale = node["finger_velocity_scale"].as<double>();
        if (node["spread_velocity"]) c.spread_velocity = node["spread_velocity"].as<double>();
        if (node["velocity_filter_alpha"]) c.velocity_filter_alpha = node["velocity_filter_alpha"].as<double>();
        if (node["velocity_deadband"]) c.velocity_deadband = node["velocity_deadband"].as<double>();
        if (node["velocity_slew_rate"]) c.velocity_slew_rate = node["velocity_slew_rate"].as<double>();
        if (node["bumper_double_click_ms"]) c.bumper_double_click_ms = node["bumper_double_click_ms"].as<int>();
        return true;
    }
};

template<> struct convert<RobotTeleopConfig> {
    static bool decode(const Node& node, RobotTeleopConfig& c) {
        c.sync_pos = node["sync_pos"].as<std::vector<double>>();
        c.gains = node["gains"].as<TeleopGains>();
        if (node["haptics"]) {
            c.haptics = node["haptics"].as<HapticsConfig>();
        }
        return true;
    }
};

template<> struct convert<TeleopConfig> {
    static bool decode(const Node& node, TeleopConfig& c) {
        c.network = node["network"].as<NetworkConfig>();
        c.sync_mapping = node["sync_mapping"].as<SyncMapping>();
        if (node["gripper"]) {
            c.gripper = node["gripper"].as<GripperConfig>();
        }
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
