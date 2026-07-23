#pragma once
#include <cstdint>
#include <cstddef>
#include <Eigen/Dense>

static constexpr size_t ACTION_HORIZON = 8;
static constexpr double CHUNK_DURATION_SEC = 1.0;
static constexpr double INTERP_HZ = 500.0;
static constexpr double UNINTERP_HZ = 10.0;

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

struct RawAction {
    double jp[7];   // x, y, z
    double gripper_cmd;
};

struct PolicyActionChunkPacket {
    uint64_t inference_timestamp_ns;   // when the policy produced this chunk (informational)
    RawAction actions[ACTION_HORIZON];
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
    Eigen::Matrix<double, 7, 1> jp;
    double gripper_cmd;
    uint64_t timestamp;
};
