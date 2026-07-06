#include "leader_udp_handler.h"
#include <boost/system/error_code.hpp>
#include <cstring>

template <size_t DOF>
LeaderUDPHandler<DOF>::LeaderUDPHandler(const std::string& follower_host, int teleop_send, int teleop_recv)
    : stop_threads(false)
    , send_socket(io_context, boost::asio::ip::udp::v4())
    , recv_socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), teleop_recv))
    , follower_endpoint(boost::asio::ip::make_address(follower_host), teleop_send) {

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
boost::optional<typename LeaderUDPHandler<DOF>::TeleopReceivedData> LeaderUDPHandler<DOF>::getLatestTeleopReceived() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return latest_received;
}

template <size_t DOF>
typename LeaderUDPHandler<DOF>::TeleopReceivedData LeaderUDPHandler<DOF>::unpackPacket(const TeleopRecvPacket& pkt) {
    TeleopReceivedData rd;
    std::memcpy(rd.jp.data(), pkt.jp, sizeof(double) * DOF);
    std::memcpy(rd.jv.data(), pkt.jv, sizeof(double) * DOF);
    std::memcpy(rd.extTorque.data(), pkt.extTorque, sizeof(double) * DOF);
    rd.gripper_torque = pkt.gripper_torque;
    rd.timestamp = pkt.timestamp;
    return rd;
}

template <size_t DOF>
void LeaderUDPHandler<DOF>::receiveLoop() {
    boost::asio::ip::udp::endpoint sender_endpoint;
    TeleopRecvPacket pkt;

    while (!stop_threads) {
        boost::system::error_code ec;
        size_t len = recv_socket.receive_from(
            boost::asio::buffer(&pkt, sizeof(TeleopRecvPacket)), sender_endpoint, 0, ec);

        if (ec == boost::asio::error::operation_aborted || len != sizeof(TeleopRecvPacket))
            continue;

        std::lock_guard<std::mutex> lock(state_mutex);
        latest_received = unpackPacket(pkt);
    }
    recv_socket.close();
}

template <size_t DOF>
void LeaderUDPHandler<DOF>::send(const jp_type& jp, const jv_type& jv, const jt_type& extTorque,
                                   double gripper_cmd, uint64_t timestamp) {
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        std::memcpy(pending_send_packet.jp, jp.data(), sizeof(double) * DOF);
        std::memcpy(pending_send_packet.jv, jv.data(), sizeof(double) * DOF);
        std::memcpy(pending_send_packet.extTorque, extTorque.data(), sizeof(double) * DOF);
        pending_send_packet.gripper_cmd = gripper_cmd;
        pending_send_packet.timestamp = timestamp;
        new_data_available = true;
    }
    send_condition.notify_one();
}

template <size_t DOF>
void LeaderUDPHandler<DOF>::sendLoop() {
    TeleopPacket pkt_to_send;

    while (!stop_threads) {
        {
            std::unique_lock<std::mutex> lock(send_mutex);
            send_condition.wait(lock, [this] { return new_data_available || stop_threads; });
            if (stop_threads) break;

            pkt_to_send = pending_send_packet;
            new_data_available = false;
        }

        boost::system::error_code ec;
        send_socket.send_to(boost::asio::buffer(&pkt_to_send, sizeof(TeleopPacket)), follower_endpoint, 0, ec);
    }
    send_socket.close();
}

template class LeaderUDPHandler<7>; // For DOF=7
