#pragma once
#include <cstdint>
#include <stdexcept>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <boost/asio.hpp>
#include <boost/optional.hpp>
#include "../utils/data_packets.h"

// Follower side of the teleop link. Sends the configured subset of the shared
// state to the leader and decodes the leader's packet back into TeleopData.
template <size_t DOF>
class FollowerUDPHandler {
public:
    FollowerUDPHandler(const std::string& leader_host, int teleop_send, int teleop_recv,
                       std::vector<std::string> send_fields, std::vector<std::string> recv_fields);
    ~FollowerUDPHandler();

    void stop();

    // Most recently received peer state.
    boost::optional<TeleopData<DOF>> getLatestTeleopReceived();

    // Queue the shared state to be sent to the leader.
    void send(const TeleopData<DOF>& state);

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

    std::vector<std::string> send_fields;
    std::vector<std::string> recv_fields;
    size_t recv_packet_size;

    std::vector<uint8_t> pending_teleop_send;
    bool new_teleop_data = false;

    boost::optional<TeleopData<DOF>> latest_teleop_received;

    void teleopReceiveLoop();
    void teleopSendLoop();
};
