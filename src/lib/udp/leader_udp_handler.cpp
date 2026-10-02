#include "leader_udp_handler.h"
#include <boost/system/error_code.hpp>
#include <cstring>

template <size_t DOF>
LeaderUDPHandler<DOF>::LeaderUDPHandler(const std::string& follower_host, int teleop_send, int teleop_recv,
                                        std::vector<std::string> send_fields, std::vector<std::string> recv_fields)
    : stop_threads(false)
    , send_socket(io_context, boost::asio::ip::udp::v4())
    , recv_socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), teleop_recv))
    , follower_endpoint(boost::asio::ip::make_address(follower_host), teleop_send)
    , send_fields(std::move(send_fields))
    , recv_fields(std::move(recv_fields)) {

    if (!validateFields(this->send_fields, TeleopData<DOF>()) ||
        !validateFields(this->recv_fields, TeleopData<DOF>())) {
        throw std::runtime_error("invalid data_routing for the leader teleop link");
    }
    recv_packet_size = encodedSize(this->recv_fields, TeleopData<DOF>());

    recv_thread = std::thread(&LeaderUDPHandler::receiveLoop, this);
    send_thread = std::thread(&LeaderUDPHandler::sendLoop, this);
}

template <size_t DOF>
LeaderUDPHandler<DOF>::~LeaderUDPHandler() {
    stop();
}

template <size_t DOF>
void LeaderUDPHandler<DOF>::stop() {
    stop_threads = true;
    io_context.stop();
    send_condition.notify_all();
    try {
        if (recv_socket.is_open()) {
            recv_socket.cancel();
            recv_socket.shutdown(boost::asio::ip::udp::socket::shutdown_both);
            recv_socket.close();
        }
    } catch (...) {
        // Ignore any exceptions during socket cleanup
    }

    if (recv_thread.joinable()) recv_thread.join();
    if (send_thread.joinable()) send_thread.join();
}

template <size_t DOF>
boost::optional<TeleopData<DOF>> LeaderUDPHandler<DOF>::getLatestTeleopReceived() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return latest_received;
}

template <size_t DOF>
void LeaderUDPHandler<DOF>::receiveLoop() {
    boost::asio::ip::udp::endpoint sender_endpoint;
    std::vector<uint8_t> buffer(recv_packet_size);

    while (!stop_threads) {
        boost::system::error_code ec;
        size_t len = recv_socket.receive_from(boost::asio::buffer(buffer), sender_endpoint, 0, ec);

        if (ec == boost::asio::error::operation_aborted || len != recv_packet_size)
            continue;

        TeleopData<DOF> received;
        if (!decode(recv_fields, buffer.data(), len, received)) continue;

        std::lock_guard<std::mutex> lock(state_mutex);
        latest_received = received;
    }
    recv_socket.close();
}

template <size_t DOF>
void LeaderUDPHandler<DOF>::send(const TeleopData<DOF>& state) {
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        if (!encode(send_fields, state, pending_send)) return;
        new_data_available = true;
    }
    send_condition.notify_one();
}

template <size_t DOF>
void LeaderUDPHandler<DOF>::sendLoop() {
    std::vector<uint8_t> packet;

    while (!stop_threads) {
        {
            std::unique_lock<std::mutex> lock(send_mutex);
            send_condition.wait(lock, [this] { return new_data_available || stop_threads; });
            if (stop_threads) break;

            packet = pending_send;
            new_data_available = false;
        }

        boost::system::error_code ec;
        send_socket.send_to(boost::asio::buffer(packet), follower_endpoint, 0, ec);
    }
    send_socket.close();
}

template class LeaderUDPHandler<7>; // For DOF=7
