#include "follower_udp_handler.h"
#include <boost/system/error_code.hpp>
#include <cstring>

template <size_t DOF>
FollowerUDPHandler<DOF>::FollowerUDPHandler(const std::string& leader_host, int teleop_send, int teleop_recv,
                                            std::vector<std::string> send_fields, std::vector<std::string> recv_fields)
    : stop_threads(false)
    , teleop_send_socket(io_context, boost::asio::ip::udp::v4())
    , teleop_recv_socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), teleop_recv))
    , leader_endpoint(boost::asio::ip::make_address(leader_host), teleop_send)
    , send_fields(std::move(send_fields))
    , recv_fields(std::move(recv_fields)) {

    if (!validateFields(this->send_fields, TeleopData<DOF>()) ||
        !validateFields(this->recv_fields, TeleopData<DOF>())) {
        throw std::runtime_error("invalid data_routing for the follower teleop link");
    }
    recv_packet_size = encodedSize(this->recv_fields, TeleopData<DOF>());

    teleop_recv_thread = std::thread(&FollowerUDPHandler::teleopReceiveLoop, this);
    teleop_send_thread = std::thread(&FollowerUDPHandler::teleopSendLoop, this);
}

template <size_t DOF>
FollowerUDPHandler<DOF>::~FollowerUDPHandler() {
    stop();
}

template <size_t DOF>
void FollowerUDPHandler<DOF>::stop() {
    stop_threads = true;
    io_context.stop();
    teleop_send_condition.notify_all();
    try {
        if (teleop_recv_socket.is_open()) {
            teleop_recv_socket.cancel();
            teleop_recv_socket.shutdown(boost::asio::ip::udp::socket::shutdown_both);
            teleop_recv_socket.close();
        }
    } catch (...) {
        // Ignore any exceptions during socket cleanup
    }

    if (teleop_recv_thread.joinable()) teleop_recv_thread.join();
    if (teleop_send_thread.joinable()) teleop_send_thread.join();
}

template <size_t DOF>
boost::optional<std::vector<uint8_t>> FollowerUDPHandler<DOF>::getLatestTeleopReceived() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return latest_received;
}

template <size_t DOF>
void FollowerUDPHandler<DOF>::teleopReceiveLoop() {
    boost::asio::ip::udp::endpoint sender_endpoint;
    std::vector<uint8_t> buffer(recv_packet_size);

    while (!stop_threads) {
        boost::system::error_code ec;
        size_t len = teleop_recv_socket.receive_from(boost::asio::buffer(buffer), sender_endpoint, 0, ec);

        if (ec == boost::asio::error::operation_aborted || len != recv_packet_size)
            continue;

        std::lock_guard<std::mutex> lock(state_mutex);
        latest_received = buffer;
    }
    teleop_recv_socket.close();
}

template <size_t DOF>
void FollowerUDPHandler<DOF>::send(const TeleopData<DOF>& state) {
    {
        std::lock_guard<std::mutex> lock(teleop_send_mutex);
        if (!encode(send_fields, state, pending_teleop_send)) return;
        new_teleop_data = true;
    }
    teleop_send_condition.notify_one();
}

template <size_t DOF>
void FollowerUDPHandler<DOF>::teleopSendLoop() {
    std::vector<uint8_t> packet;

    while (!stop_threads) {
        {
            std::unique_lock<std::mutex> lock(teleop_send_mutex);
            teleop_send_condition.wait(lock, [this] { return new_teleop_data || stop_threads; });
            if (stop_threads) break;

            packet = pending_teleop_send;
            new_teleop_data = false;
        }

        boost::system::error_code ec;
        teleop_send_socket.send_to(boost::asio::buffer(packet), leader_endpoint, 0, ec);
    }
    teleop_send_socket.close();
}

template class FollowerUDPHandler<4>;
template class FollowerUDPHandler<7>;
