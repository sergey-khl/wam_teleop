#pragma once
#include <cstdint>
#include <cstddef>
#include <Eigen/Dense>

// tightly packed layouts
#pragma pack(push, 1)

template <size_t DOF>
struct LeaderToFollowerPacket {
    double jp[DOF];
    double jv[DOF];
    double extTorque[DOF];
    double gripper_cmd;
    uint64_t timestamp;
};

template <size_t DOF>
struct FollowerToLeaderPacket {
    double jp[DOF];
    double jv[DOF];
    double extTorque[DOF];
    double gripper_torque;
    uint64_t timestamp;
};

// a bunch of state data sent from the follower
template <size_t DOF>
struct PolicyPacket {
    double follower_jp[DOF];
    double follower_jv[DOF];
    double follower_extTorque[DOF];
    double leader_jp[DOF];
    double leader_jv[DOF];
    double leader_extTorque[DOF];
    double follower_cart_pos[3];   // x, y, z
    double follower_cart_rot[4];   // w, x, y, z
    double gripper_pos;
    double gripper_vel;
    double gripper_torque;
    uint64_t timestamp;
};

// the action we perform on the follower
struct PolicyActionPacket {
    double cart_pos[3];   // x, y, z
    double cart_rot[3];   // roll, pitch, yaw
    double gripper_cmd;
    uint64_t timestamp;
};

#pragma pack(pop)


template <size_t DOF>
struct LeaderReceivedData {
    Eigen::Matrix<double, DOF, 1> jp;
    Eigen::Matrix<double, DOF, 1> jv;
    Eigen::Matrix<double, DOF, 1> extTorque;
    double gripper_torque;
    uint64_t timestamp;
};

template <size_t DOF>
struct FollowerReceivedData {
    Eigen::Matrix<double, DOF, 1> jp;
    Eigen::Matrix<double, DOF, 1> jv;
    Eigen::Matrix<double, DOF, 1> extTorque;
    double gripper_cmd;
    uint64_t timestamp;
};

struct PolicyReceivedData {
    Eigen::Vector3d cart_pos;   // x, y, z
    Eigen::Vector3d cart_rot;   // roll, pitch, yaw
    double gripper_cmd;
    uint64_t timestamp;
};
