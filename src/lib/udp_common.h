#pragma once
#include <cstdint>
#include <cstddef>
#include <Eigen/Dense>
#include <Eigen/Geometry>
 
#include <boost/asio.hpp>
#include <boost/optional.hpp>
 
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
 

static constexpr int ACTION_HORIZON = 8;
static constexpr double SEGMENT_DURATION_SEC = 0.5;
static constexpr double INTERP_HZ = 500.0;
static constexpr double UNINTERP_HZ = 10.0;

// tightly packed layouts
#pragma pack(push, 1)

template <size_t DOF>
struct LeaderToFollowerPacket {
    double jp[DOF];
    double jv[DOF];
    double extTorque[DOF];
    double cart_pos[3];   // x, y, z
    double quat[4];   // w, x, y, z
    double policyTorqueScale[DOF];
    double gripper_cmd;
    double cancel_policy;
    uint64_t timestamp;
};

template <size_t DOF>
struct FollowerToLeaderPacket {
    double jp[DOF];
    double jv[DOF];
    double extTorque[DOF];
    double cart_pos[3];   // x, y, z
    double quat[4];   // w, x, y, z
    double gripper_torque;
    double gripper_pos;
    double gripper_vel;
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
    double policyTorqueScale[DOF];
    double policyJt[DOF];
    double follower_cart_pos[3];   // x, y, z
    double follower_quat[4];   // w, x, y, z
    double leader_cart_pos[3];   // x, y, z
    double leader_quat[4];   // w, x, y, z
    double gripper_pos;
    double gripper_vel;
    double gripper_torque;
    uint64_t time_to_chunk_end;
    uint64_t timestamp;
};

struct RawAction {
    double jp[7];   // x, y, z
    double gripper_cmd;
};

struct PolicyActionChunkPacket {
    uint64_t time_to_skip;   // ns of the previous chunk still unconsumed when inference began
    RawAction actions[ACTION_HORIZON];
};

#pragma pack(pop)


template <size_t DOF>
struct LeaderReceivedData {
    Eigen::Matrix<double, DOF, 1> jp;
    Eigen::Matrix<double, DOF, 1> jv;
    Eigen::Matrix<double, DOF, 1> extTorque;
    Eigen::Matrix<double, 3, 1> cart_pos;
    Eigen::Quaterniond quat;
    double gripper_torque;
    double gripper_pos;
    double gripper_vel;
    uint64_t timestamp;
};

template <size_t DOF>
struct FollowerReceivedData {
    Eigen::Matrix<double, DOF, 1> jp;
    Eigen::Matrix<double, DOF, 1> jv;
    Eigen::Matrix<double, DOF, 1> extTorque;
    Eigen::Matrix<double, 3, 1> cart_pos;
    Eigen::Quaterniond quat;
    Eigen::Matrix<double, DOF, 1> policyTorqueScale;
    double gripper_cmd;
    double cancel_policy;
    uint64_t timestamp;
};

struct PolicyReceivedData {
    Eigen::Matrix<double, 7, 1> jp;
    Eigen::Matrix<double, 7, 1> jv;
    Eigen::Matrix<double, 7, 1> ja;
    double gripper_cmd;
};

template <size_t DOF>
class PolicyUDPHandler {
public:
    using jp_type = Eigen::Matrix<double, DOF, 1>;
    using jv_type = Eigen::Matrix<double, DOF, 1>;
    using jt_type = Eigen::Matrix<double, DOF, 1>;
    using PolicyPacketType = PolicyPacket<DOF>;
 
    PolicyUDPHandler(bool send_active, const std::string& policy_host, int policy_send_port, int policy_recv_port);
    ~PolicyUDPHandler();
 
    void stop();
 
    // Latest interpolated action received from the policy (pops the front of the queue).
    boost::optional<PolicyReceivedData> getLatestPolicyReceived();

    void clearQueueAndPause(std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double, std::milli>(ACTION_HORIZON * SEGMENT_DURATION_SEC * 1e3)));
    void clearQueue();

 
    // Queue a PolicyPacket to be sent to the policy. No-op if inactive.
    void send(const jp_type& follower_jp, const jv_type& follower_jv, const jt_type& follower_extTorque,
                       const jp_type& leader_jp, const jv_type& leader_jv, const jt_type& leader_extTorque,
                       const jt_type& policyTorqueScale, const jt_type& policyJt,
                       const Eigen::Vector3d& follower_cart_pos, const Eigen::Quaterniond& follower_quat,
                       const Eigen::Vector3d& leader_cart_pos, const Eigen::Quaterniond& leader_quat,
                       double gripper_pos, double gripper_vel, double gripper_torque,
                       uint64_t timestamp);
 
private:
    static constexpr size_t SAMPLES_PER_SEGMENT = static_cast<size_t>(INTERP_HZ * SEGMENT_DURATION_SEC);
 
    bool send_active;
    std::atomic<bool> stop_threads;
 
    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket send_socket;
    boost::asio::ip::udp::socket recv_socket;
    boost::asio::ip::udp::endpoint policy_endpoint;
    int policy_recv_port;
 
    std::thread send_thread;
    std::thread recv_thread;
 
    std::mutex state_mutex;
    std::mutex send_mutex;
    std::condition_variable send_condition;
 
    PolicyPacketType pending_packet;
    bool new_data_available = false;
 
    std::deque<PolicyReceivedData> action_queue;
 
    void sendLoop();
    void receiveLoop();

    std::deque<RawAction> raw_waypoint_queue;
    std::mutex raw_mutex;
    std::condition_variable raw_condition;
    bool first_segment_ever = true;

    std::thread interp_thread;
    void interpLoop();

    std::atomic<uint64_t> samples_to_skip{0};
 
    std::atomic<int64_t> chunk_end_ns{0};
 
    // always use unique points for a0-3
    static std::deque<PolicyReceivedData> interpolateSegment(const RawAction& a0, const RawAction& a1, const RawAction& a2, const RawAction& a3);

    std::chrono::steady_clock::time_point pause_until{};
};

