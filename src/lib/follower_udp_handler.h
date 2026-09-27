#pragma once
#include "udp_common.h"
#include <boost/asio.hpp>
#include <boost/optional.hpp>
#include <atomic>
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
    using cp_type = Eigen::Matrix<double, 3, 1>;

    using TeleopPacket = FollowerToLeaderPacket<DOF>;
    using TeleopRecvPacket = LeaderToFollowerPacket<DOF>;
    using PolicyPacketType = PolicyPacket<DOF>;
    using TeleopReceivedData = FollowerReceivedData<DOF>;

    FollowerUDPHandler(const std::string& leader_host, int teleop_send, int teleop_recv);
    ~FollowerUDPHandler();

    void stop();

    // Latest command received from the leader.
    boost::optional<TeleopReceivedData> getLatestTeleopReceived();

    // Queue a FollowerToLeaderPacket to be sent to the leader.
    void send(const jp_type& jp, const jv_type& jv,
              const jt_type& dyngravcompTorque, const jt_type& environmentTorque, const jt_type& filteredEnvironmentTorque,
              const cp_type& cart_pos, const Eigen::Quaterniond& quat,
              double gripper_torque, double gripper_pos, double gripper_vel, uint64_t timestamp);

private:
    std::atomic<bool> stop_threads;

    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket teleop_send_socket;
    boost::asio::ip::udp::socket teleop_recv_socket;

    boost::asio::ip::udp::endpoint leader_endpoint;

    std::thread teleop_recv_thread;
    std::thread teleop_send_thread;

    std::mutex state_mutex;
    std::mutex teleop_send_mutex;
    std::condition_variable teleop_send_condition;

    TeleopPacket pending_teleop_packet;
    bool new_teleop_data = false;

    boost::optional<TeleopReceivedData> latest_teleop_received;

    void teleopReceiveLoop();
    void teleopSendLoop();

    static TeleopReceivedData unpackTeleopPacket(const TeleopRecvPacket& pkt);
};
