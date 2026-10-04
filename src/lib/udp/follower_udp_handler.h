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

template <size_t DOF>
class FollowerUDPHandler {
public:
    FollowerUDPHandler(const std::string& leader_host, int teleop_send, int teleop_recv,
                       std::vector<std::string> send_fields, std::vector<std::string> recv_fields);
    ~FollowerUDPHandler();

    void stop();

    // Most recently received packet, still encoded. Decode it with recvFields().
    boost::optional<std::vector<uint8_t>> getLatestTeleopReceived();

    // Queue the shared state to be sent to the leader.
    void send(const TeleopData<DOF>& state);

    // Fields this link was configured to receive (leader -> follower).
    const std::vector<std::string>& recvFields() const { return recv_fields; }

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

    boost::optional<std::vector<uint8_t>> latest_received;

    void teleopReceiveLoop();
    void teleopSendLoop();
};
