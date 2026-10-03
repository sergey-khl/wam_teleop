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

// Leader side of the teleop link. Sends the configured subset of the shared
// state to the follower and decodes the follower's packet back into TeleopData.
template <size_t DOF>
class LeaderUDPHandler {
public:
    LeaderUDPHandler(const std::string& follower_host, int teleop_send, int teleop_recv,
                     std::vector<std::string> send_fields, std::vector<std::string> recv_fields);
    ~LeaderUDPHandler();

    void stop();

    // Most recently received packet, still encoded. Decode it with recvFields().
    boost::optional<std::vector<uint8_t>> getLatestTeleopReceived();

    // Queue the shared state to be sent to the follower.
    void send(const TeleopData<DOF>& state);

    // Fields this link was configured to receive (follower -> leader).
    const std::vector<std::string>& recvFields() const { return recv_fields; }

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

    std::vector<std::string> send_fields;
    std::vector<std::string> recv_fields;
    size_t recv_packet_size;

    std::vector<uint8_t> pending_send;
    bool new_data_available = false;

    boost::optional<std::vector<uint8_t>> latest_received;

    void receiveLoop();
    void sendLoop();
};
