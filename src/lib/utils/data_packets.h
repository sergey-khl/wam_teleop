#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <barrett/units.h>

template <size_t DOF>
struct TeleopData {
    typedef typename barrett::units::JointPositions<DOF>::type jp_type;
    typedef typename barrett::units::JointVelocities<DOF>::type jv_type;
    typedef typename barrett::units::JointTorques<DOF>::type jt_type;

    jp_type leader_jp = jp_type::Zero();
    jv_type leader_jv = jv_type::Zero();
    jt_type leader_dyngravcomp_torque = jt_type::Zero();
    jp_type follower_jp = jp_type::Zero();
    jv_type follower_jv = jv_type::Zero();
    jt_type follower_dyngravcomp_torque = jt_type::Zero();

    jt_type human_torque = jt_type::Zero();
    jt_type filtered_human_torque = jt_type::Zero();
    jt_type environment_torque = jt_type::Zero();
    jt_type filtered_environment_torque = jt_type::Zero();

    jp_type policyJp = jp_type::Zero();
    jt_type policyJt = jt_type::Zero();
    jt_type policyTorqueScale = jt_type::Zero();
    jp_type resPolicyJp = jp_type::Zero();
    jt_type resPolicyJt = jt_type::Zero();
    jt_type refPolicyTorque = jt_type::Zero();
    jt_type refTorquePolicyJt = jt_type::Zero();

    Eigen::Vector3d leader_cart_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond leader_quat = Eigen::Quaterniond::Identity();
    Eigen::Vector3d follower_cart_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond follower_quat = Eigen::Quaterniond::Identity();

    double gripper_pos = 0.0;
    double gripper_vel = 0.0;
    double gripper_torque = 0.0;
    double gripper_cmd = 0.0;
    double cancel_policy = 0.0;

    uint64_t timestamp = 0;
    uint64_t time_to_chunk_end = 0;
    uint64_t res_time_to_chunk_end = 0;

    void zero() { *this = TeleopData<DOF>(); }
};

struct ActionData {
    double jp[7] = {};
    double delta_jp[7] = {};
    double ext_torque[7] = {};
    double gripper_cmd = 0.0;

    void zero() { *this = ActionData(); }
};

// String -> field accessors
struct FieldRef {
    void* ptr;
    size_t bytes;
};
struct ConstFieldRef {
    const void* ptr;
    size_t bytes;
};

template <size_t DOF>
ConstFieldRef field(const TeleopData<DOF>& d, const std::string& name) {
    if (name == "leader_jp") return {&d.leader_jp, sizeof(d.leader_jp)};
    if (name == "leader_jv") return {&d.leader_jv, sizeof(d.leader_jv)};
    if (name == "leader_dyngravcomp_torque") return {&d.leader_dyngravcomp_torque, sizeof(d.leader_dyngravcomp_torque)};
    if (name == "follower_jp") return {&d.follower_jp, sizeof(d.follower_jp)};
    if (name == "follower_jv") return {&d.follower_jv, sizeof(d.follower_jv)};
    if (name == "follower_dyngravcomp_torque") return {&d.follower_dyngravcomp_torque, sizeof(d.follower_dyngravcomp_torque)};
    if (name == "human_torque") return {&d.human_torque, sizeof(d.human_torque)};
    if (name == "filtered_human_torque") return {&d.filtered_human_torque, sizeof(d.filtered_human_torque)};
    if (name == "environment_torque") return {&d.environment_torque, sizeof(d.environment_torque)};
    if (name == "filtered_environment_torque") return {&d.filtered_environment_torque, sizeof(d.filtered_environment_torque)};
    if (name == "policyJp") return {&d.policyJp, sizeof(d.policyJp)};
    if (name == "policyJt") return {&d.policyJt, sizeof(d.policyJt)};
    if (name == "policyTorqueScale") return {&d.policyTorqueScale, sizeof(d.policyTorqueScale)};
    if (name == "resPolicyJp") return {&d.resPolicyJp, sizeof(d.resPolicyJp)};
    if (name == "resPolicyJt") return {&d.resPolicyJt, sizeof(d.resPolicyJt)};
    if (name == "refPolicyTorque") return {&d.refPolicyTorque, sizeof(d.refPolicyTorque)};
    if (name == "refTorquePolicyJt") return {&d.refTorquePolicyJt, sizeof(d.refTorquePolicyJt)};
    if (name == "leader_cart_pos") return {&d.leader_cart_pos, sizeof(d.leader_cart_pos)};
    if (name == "leader_quat") return {&d.leader_quat, sizeof(d.leader_quat)};
    if (name == "follower_cart_pos") return {&d.follower_cart_pos, sizeof(d.follower_cart_pos)};
    if (name == "follower_quat") return {&d.follower_quat, sizeof(d.follower_quat)};
    if (name == "gripper_pos") return {&d.gripper_pos, sizeof(d.gripper_pos)};
    if (name == "gripper_vel") return {&d.gripper_vel, sizeof(d.gripper_vel)};
    if (name == "gripper_torque") return {&d.gripper_torque, sizeof(d.gripper_torque)};
    if (name == "gripper_cmd") return {&d.gripper_cmd, sizeof(d.gripper_cmd)};
    if (name == "cancel_policy") return {&d.cancel_policy, sizeof(d.cancel_policy)};
    if (name == "timestamp") return {&d.timestamp, sizeof(d.timestamp)};
    if (name == "time_to_chunk_end") return {&d.time_to_chunk_end, sizeof(d.time_to_chunk_end)};
    if (name == "res_time_to_chunk_end") return {&d.res_time_to_chunk_end, sizeof(d.res_time_to_chunk_end)};
    return {nullptr, 0};
}

template <size_t DOF>
FieldRef field(TeleopData<DOF>& d, const std::string& name) {
    const ConstFieldRef ref = field(static_cast<const TeleopData<DOF>&>(d), name);
    return {const_cast<void*>(ref.ptr), ref.bytes};
}

inline ConstFieldRef field(const ActionData& d, const std::string& name) {
    if (name == "jp") return {&d.jp, sizeof(d.jp)};
    if (name == "delta_jp") return {&d.delta_jp, sizeof(d.delta_jp)};
    if (name == "ext_torque") return {&d.ext_torque, sizeof(d.ext_torque)};
    if (name == "gripper_cmd") return {&d.gripper_cmd, sizeof(d.gripper_cmd)};
    return {nullptr, 0};
}

inline FieldRef field(ActionData& d, const std::string& name) {
    const ConstFieldRef ref = field(static_cast<const ActionData&>(d), name);
    return {const_cast<void*>(ref.ptr), ref.bytes};
}

// process sending
template <typename Data>
bool encode(const std::vector<std::string>& names, const Data& data, std::vector<uint8_t>& out) {
    out.clear();
    for (const std::string& name : names) {
        const ConstFieldRef ref = field(data, name);
        if (ref.ptr == nullptr) return false;
        const size_t offset = out.size();
        out.resize(offset + ref.bytes);
        std::memcpy(out.data() + offset, ref.ptr, ref.bytes);
    }
    return true;
}
// process receive
template <typename Data>
bool decode(const std::vector<std::string>& names, const uint8_t* bytes, size_t length, Data& data) {
    size_t offset = 0;
    for (const std::string& name : names) {
        const FieldRef ref = field(data, name);
        if (ref.ptr == nullptr) return false;
        if (offset + ref.bytes > length) return false;
        std::memcpy(ref.ptr, bytes + offset, ref.bytes);
        offset += ref.bytes;
    }
    return offset == length;
}

// total size of sent
template <typename Data>
size_t encodedSize(const std::vector<std::string>& names, const Data& data) {
    size_t total = 0;
    for (const std::string& name : names) {
        const ConstFieldRef ref = field(data, name);
        if (ref.ptr == nullptr) return 0;
        total += ref.bytes;
    }
    return total;
}

template <typename Data>
bool validateFields(const std::vector<std::string>& names, const Data& probe) {
    if (names.empty()) return false;
    for (const std::string& name : names) {
        if (field(probe, name).ptr == nullptr) return false;
    }
    return true;
}

template <size_t DOF>
class TeleopState {
public:
    class Locked {
    public:
        explicit Locked(TeleopState& owner) : lock_(owner.mutex_), data_(owner.data_) {}

        TeleopData<DOF>* operator->() { return &data_; }
        TeleopData<DOF>& operator*() { return data_; }

    private:
        std::unique_lock<std::mutex> lock_;
        TeleopData<DOF>& data_;
    };

    Locked lock() { return Locked(*this); }

    // mutex guard setting and getting
    template <typename Fn>
    auto with_lock(Fn&& fn) -> decltype(fn(std::declval<TeleopData<DOF>&>())) {
        std::lock_guard<std::mutex> guard(mutex_);
        return fn(data_);
    }

private:
    std::mutex mutex_;
    TeleopData<DOF> data_;
};
