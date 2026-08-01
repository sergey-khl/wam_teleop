#pragma once
#include "udp_common.h"
#include <boost/asio.hpp>
#include <boost/optional.hpp>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

// only deals with teleopp
template <size_t DOF>
class LeaderUDPHandler {
public:
    using jp_type = Eigen::Matrix<double, DOF, 1>;
    using jv_type = Eigen::Matrix<double, DOF, 1>;
    using jt_type = Eigen::Matrix<double, DOF, 1>;
    using cp_type = Eigen::Matrix<double, 3, 1>;

    using TeleopPacket = LeaderToFollowerPacket<DOF>;
    using TeleopRecvPacket = FollowerToLeaderPacket<DOF>;
    using TeleopReceivedData = LeaderReceivedData<DOF>;

    LeaderUDPHandler(const std::string& follower_host, int teleop_send, int teleop_recv);
    ~LeaderUDPHandler();

    void stop();

    // Force feedback most recently received from the follower.
    boost::optional<TeleopReceivedData> getLatestTeleopReceived();

    // Queue a LeaderToFollowerPacket to be sent to the follower.
    void send(const jp_type& jp, const jv_type& jv, const jt_type& extTorque,
              const cp_type& cart_pos, const Eigen::Quaterniond& quat,
              const jt_type& policyTorqueScale,
              double gripper_cmd, double cancel_policy, uint64_t timestamp);

private:
    std::atomic<bool> stop_threads;

    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket send_socket;
    boost::asio::ip::udp::socket recv_socket;
    boost::asio::ip::udp::endpoint follower_endpoint;

    std::thread recv_thread;
    std::thread send_thread;

    std::mutex state_mutex, send_mutex;
    std::condition_variable send_condition;

    TeleopPacket pending_send_packet;
    bool new_data_available = false;

    boost::optional<TeleopReceivedData> latest_received;

    void receiveLoop();
    void sendLoop();

    static TeleopReceivedData unpackPacket(const TeleopRecvPacket& pkt);
};
