#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <barrett/systems/abstract/system.h>
#include <barrett/units.h>

// The shared state always carries all 7 joints. The arm occupies the first DOF
// entries; whatever is left (TELEOP_DOF - DOF) belongs to the wrist.
constexpr size_t TELEOP_DOF = 7;

using jp_teleop_type = barrett::units::JointPositions<TELEOP_DOF>::type;
using jv_teleop_type = barrett::units::JointVelocities<TELEOP_DOF>::type;
using jt_teleop_type = barrett::units::JointTorques<TELEOP_DOF>::type;

template <size_t DOF>
struct TeleopData {
    static constexpr size_t WRIST_DOF = TELEOP_DOF - DOF;

    jp_teleop_type leader_jp = jp_teleop_type::Zero();
    jv_teleop_type leader_jv = jv_teleop_type::Zero();
    jt_teleop_type leader_dyngravcomp_torque = jt_teleop_type::Zero();
    jp_teleop_type follower_jp = jp_teleop_type::Zero();
    jv_teleop_type follower_jv = jv_teleop_type::Zero();
    jt_teleop_type follower_dyngravcomp_torque = jt_teleop_type::Zero();

    jt_teleop_type human_torque = jt_teleop_type::Zero();
    jt_teleop_type filtered_human_torque = jt_teleop_type::Zero();
    jt_teleop_type environment_torque = jt_teleop_type::Zero();
    jt_teleop_type filtered_environment_torque = jt_teleop_type::Zero();

    jp_teleop_type policyJp = jp_teleop_type::Zero();
    jt_teleop_type policyJt = jt_teleop_type::Zero();
    jt_teleop_type policyTorqueScale = jt_teleop_type::Zero();
    jp_teleop_type resPolicyJp = jp_teleop_type::Zero();
    jt_teleop_type resPolicyJt = jt_teleop_type::Zero();
    jt_teleop_type refPolicyTorque = jt_teleop_type::Zero();
    jt_teleop_type refTorquePolicyJt = jt_teleop_type::Zero();

    jt_teleop_type control_torque = jt_teleop_type::Zero();
    jt_teleop_type wam_dyn = jt_teleop_type::Zero();
    jt_teleop_type wam_grav = jt_teleop_type::Zero();

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
};

struct ActionData {
    double jp[7] = {};
    double delta_jp[7] = {};
    double ext_torque[7] = {};
    double gripper_cmd = 0.0;
};

// format a field without knowing its name.
enum class FieldType { None, Scalar, Vector3, Vector7, Quaternion, UInt64 };

struct FieldRef {
    void* ptr;
    size_t bytes;
    FieldType type;
};
struct ConstFieldRef {
    const void* ptr;
    size_t bytes;
    FieldType type;
};

template <size_t DOF>
ConstFieldRef field(const TeleopData<DOF>& d, const std::string& name) {
    if (name == "leader_jp") return {&d.leader_jp, sizeof(d.leader_jp), FieldType::Vector7};
    if (name == "leader_jv") return {&d.leader_jv, sizeof(d.leader_jv), FieldType::Vector7};
    if (name == "leader_dyngravcomp_torque") return {&d.leader_dyngravcomp_torque, sizeof(d.leader_dyngravcomp_torque), FieldType::Vector7};
    if (name == "follower_jp") return {&d.follower_jp, sizeof(d.follower_jp), FieldType::Vector7};
    if (name == "follower_jv") return {&d.follower_jv, sizeof(d.follower_jv), FieldType::Vector7};
    if (name == "follower_dyngravcomp_torque") return {&d.follower_dyngravcomp_torque, sizeof(d.follower_dyngravcomp_torque), FieldType::Vector7};
    if (name == "human_torque") return {&d.human_torque, sizeof(d.human_torque), FieldType::Vector7};
    if (name == "filtered_human_torque") return {&d.filtered_human_torque, sizeof(d.filtered_human_torque), FieldType::Vector7};
    if (name == "environment_torque") return {&d.environment_torque, sizeof(d.environment_torque), FieldType::Vector7};
    if (name == "filtered_environment_torque") return {&d.filtered_environment_torque, sizeof(d.filtered_environment_torque), FieldType::Vector7};
    if (name == "policyJp") return {&d.policyJp, sizeof(d.policyJp), FieldType::Vector7};
    if (name == "policyJt") return {&d.policyJt, sizeof(d.policyJt), FieldType::Vector7};
    if (name == "policyTorqueScale") return {&d.policyTorqueScale, sizeof(d.policyTorqueScale), FieldType::Vector7};
    if (name == "resPolicyJp") return {&d.resPolicyJp, sizeof(d.resPolicyJp), FieldType::Vector7};
    if (name == "resPolicyJt") return {&d.resPolicyJt, sizeof(d.resPolicyJt), FieldType::Vector7};
    if (name == "refPolicyTorque") return {&d.refPolicyTorque, sizeof(d.refPolicyTorque), FieldType::Vector7};
    if (name == "refTorquePolicyJt") return {&d.refTorquePolicyJt, sizeof(d.refTorquePolicyJt), FieldType::Vector7};
    if (name == "control_torque") return {&d.control_torque, sizeof(d.control_torque), FieldType::Vector7};
    if (name == "wam_dyn") return {&d.wam_dyn, sizeof(d.wam_dyn), FieldType::Vector7};
    if (name == "wam_grav") return {&d.wam_grav, sizeof(d.wam_grav), FieldType::Vector7};
    if (name == "leader_cart_pos") return {&d.leader_cart_pos, sizeof(d.leader_cart_pos), FieldType::Vector3};
    if (name == "leader_quat") return {&d.leader_quat, sizeof(d.leader_quat), FieldType::Quaternion};
    if (name == "follower_cart_pos") return {&d.follower_cart_pos, sizeof(d.follower_cart_pos), FieldType::Vector3};
    if (name == "follower_quat") return {&d.follower_quat, sizeof(d.follower_quat), FieldType::Quaternion};
    if (name == "gripper_pos") return {&d.gripper_pos, sizeof(d.gripper_pos), FieldType::Scalar};
    if (name == "gripper_vel") return {&d.gripper_vel, sizeof(d.gripper_vel), FieldType::Scalar};
    if (name == "gripper_torque") return {&d.gripper_torque, sizeof(d.gripper_torque), FieldType::Scalar};
    if (name == "gripper_cmd") return {&d.gripper_cmd, sizeof(d.gripper_cmd), FieldType::Scalar};
    if (name == "cancel_policy") return {&d.cancel_policy, sizeof(d.cancel_policy), FieldType::Scalar};
    if (name == "timestamp") return {&d.timestamp, sizeof(d.timestamp), FieldType::UInt64};
    if (name == "time_to_chunk_end") return {&d.time_to_chunk_end, sizeof(d.time_to_chunk_end), FieldType::UInt64};
    if (name == "res_time_to_chunk_end") return {&d.res_time_to_chunk_end, sizeof(d.res_time_to_chunk_end), FieldType::UInt64};
    return {nullptr, 0, FieldType::None};
}

template <size_t DOF>
FieldRef field(TeleopData<DOF>& d, const std::string& name) {
    const ConstFieldRef ref = field(static_cast<const TeleopData<DOF>&>(d), name);
    return {const_cast<void*>(ref.ptr), ref.bytes, ref.type};
}

inline ConstFieldRef field(const ActionData& d, const std::string& name) {
    if (name == "jp") return {&d.jp, sizeof(d.jp), FieldType::Vector7};
    if (name == "delta_jp") return {&d.delta_jp, sizeof(d.delta_jp), FieldType::Vector7};
    if (name == "ext_torque") return {&d.ext_torque, sizeof(d.ext_torque), FieldType::Vector7};
    if (name == "gripper_cmd") return {&d.gripper_cmd, sizeof(d.gripper_cmd), FieldType::Scalar};
    return {nullptr, 0, FieldType::None};
}

inline FieldRef field(ActionData& d, const std::string& name) {
    const ConstFieldRef ref = field(static_cast<const ActionData&>(d), name);
    return {const_cast<void*>(ref.ptr), ref.bytes, ref.type};
}

// Pretty-print a single TeleopData field (used by the logging module)
template <size_t DOF>
bool formatField(std::ostream& os, const TeleopData<DOF>& d, const std::string& name) {
    const ConstFieldRef ref = field(d, name);
    if (ref.ptr == nullptr) return false;

    switch (ref.type) {
    case FieldType::Vector7: {
        const double* v = static_cast<const double*>(ref.ptr);
        // barrett unit types carry a gsl struct after the coefficients, so the
        // joint count is TELEOP_DOF, not ref.bytes / sizeof(double).
        size_t n = ref.bytes / sizeof(double);
        if (n > TELEOP_DOF) n = TELEOP_DOF;
        os << "[";
        for (size_t i = 0; i < n; ++i) os << (i ? ", " : "") << v[i];
        os << "]";
        return true;
    }
    case FieldType::Vector3: {
        const double* v = static_cast<const double*>(ref.ptr);
        os << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
        return true;
    }
    case FieldType::Quaternion: {
        const Eigen::Quaterniond* q = static_cast<const Eigen::Quaterniond*>(ref.ptr);
        os << "[" << q->w() << " " << q->x() << " " << q->y() << " " << q->z() << "]";
        return true;
    }
    case FieldType::Scalar: {
        double v;
        std::memcpy(&v, ref.ptr, sizeof(v));
        os << v;
        return true;
    }
    case FieldType::UInt64: {
        uint64_t v;
        std::memcpy(&v, ref.ptr, sizeof(v));
        os << v;
        return true;
    }
    default:
        return false;
    }
}

// templated setters for the state
template <typename Source, typename Field>
inline void setLocalStateValue(const Source& source, Field& field) {
    field = source;
}

template <typename T, typename Field>
inline void setLocalStateValue(const barrett::systems::System::Input<T>& input, Field& field) {
    // The input is arm-wide (DOF); write it into the arm part of the field.
    constexpr int N = T::RowsAtCompileTime;
    if (input.valueDefined()) {
        field.head(N) = input.getValue();
    } else {
        field.head(N).setZero();
    }
}

// process sending
template <typename Data>
bool encode(const std::vector<std::string>& names, const Data& data, std::vector<uint8_t>& out) {
    out.clear();
    for (const std::string& name : names) {
        const ConstFieldRef ref = field(data, name);
        if (ref.ptr == nullptr) return false;
        // copy local state into output to be sent over udp
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
        // copy a local state field into data as output
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
