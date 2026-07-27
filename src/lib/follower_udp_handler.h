#pragma once
#include "udp_common.h"
#include <boost/asio.hpp>
#include <boost/optional.hpp>
#include <atomic>
#include <deque>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

// follower handles all policy related stuff
template <size_t DOF>
class FollowerUDPHandler {
public:
    using jp_type = Eigen::Matrix<double, DOF, 1>;
    using jv_type = Eigen::Matrix<double, DOF, 1>;
    using jt_type = Eigen::Matrix<double, DOF, 1>;

    using TeleopPacket = FollowerToLeaderPacket<DOF>;
    using TeleopRecvPacket = LeaderToFollowerPacket<DOF>;
    using PolicyPacketType = PolicyPacket<DOF>;
    using TeleopReceivedData = FollowerReceivedData<DOF>;

    FollowerUDPHandler(const std::string& leader_host, int teleop_send, int teleop_recv,
                        bool policy_send_active,
                        const std::string& policy_host = "127.0.0.1", int policy_send = 6000, int policy_recv = 6001);
    ~FollowerUDPHandler();

    void stop();

    // Latest command received from the leader.
    boost::optional<TeleopReceivedData> getLatestTeleopReceived();

    // Latest action received from the policy
    boost::optional<PolicyReceivedData> getLatestPolicyReceived();

    // Queue a FollowerToLeaderPacket to be sent to the leader.
    void send(const jp_type& jp, const jv_type& jv, const jt_type& extTorque, const jt_type& policyTorque,
              double gripper_torque, uint64_t timestamp);

    // Queue a PolicyPacket to be sent to the policy.
    void sendToPolicy(const jp_type& follower_jp, const jv_type& follower_jv, const jt_type& follower_extTorque,
                       const jp_type& leader_jp, const jv_type& leader_jv, const jt_type& leader_extTorque,
                       const Eigen::Vector3d& follower_cart_pos, const Eigen::Quaterniond& follower_cart_rot,
                       double gripper_pos, double gripper_vel, double gripper_torque,
                       uint64_t timestamp);

private:
    static constexpr size_t NUM_INTERP_SAMPLES = static_cast<size_t>(INTERP_HZ * CHUNK_DURATION_SEC);

    std::atomic<bool> stop_threads;

    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket teleop_send_socket;
    boost::asio::ip::udp::socket policy_send_socket;
    boost::asio::ip::udp::socket teleop_recv_socket;
    boost::asio::ip::udp::socket policy_recv_socket;

    boost::asio::ip::udp::endpoint leader_endpoint;
    boost::asio::ip::udp::endpoint policy_endpoint;

    bool policy_send_active;
    int policy_recv_port;

    std::thread teleop_recv_thread;
    std::thread policy_recv_thread;
    std::thread teleop_send_thread;
    std::thread policy_send_thread;

    std::mutex state_mutex;
    std::mutex teleop_send_mutex, policy_send_mutex;
    std::condition_variable teleop_send_condition, policy_send_condition;

    TeleopPacket pending_teleop_packet;
    bool new_teleop_data = false;

    PolicyPacketType pending_policy_packet;
    bool new_policy_data = false;

    boost::optional<TeleopReceivedData> latest_teleop_received;
    boost::optional<PolicyReceivedData> latest_policy_received;

    std::deque<PolicyReceivedData> policy_action_queue;

    void teleopReceiveLoop();
    void policyReceiveLoop();
    void teleopSendLoop();
    void policySendLoop();

    static TeleopReceivedData unpackTeleopPacket(const TeleopRecvPacket& pkt);
    static std::deque<PolicyReceivedData> interpolateChunk( const RawAction (&actions)[ACTION_HORIZON], uint64_t inference_timestamp_ns);
};
