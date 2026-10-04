#pragma once
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <boost/filesystem.hpp>

struct LoggingConfig {
    int every_n_loops = 250;
    bool auto_load = false;
    std::vector<std::string> leader;
    std::vector<std::string> follower;
};

inline LoggingConfig load_logging_file_config(const std::string& file) {
    LoggingConfig c;

    try {
        YAML::Node root = YAML::LoadFile(file);
        if (root["every_n_loops"]) {
            c.every_n_loops = root["every_n_loops"].as<int>();
        }
        if (root["auto_load"]) {
            c.auto_load = root["auto_load"].as<bool>();
        }
        if (root["leader"]) {
            c.leader = root["leader"].as<std::vector<std::string>>();
        }
        if (root["follower"]) {
            c.follower = root["follower"].as<std::vector<std::string>>();
        }
        return c;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading logging config (" << file << "): " << e.what() << std::endl;
        throw;
    }
}
