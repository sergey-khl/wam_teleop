#include "udp_handler.h"
#include <bits/stdint-uintn.h>
#include <boost/system/error_code.hpp>

// TODO: inference recv can just pass to current teleop_recv for now. change later
template <size_t DOF>
UDPHandler<DOF>::UDPHandler(const std::string& teleop_host, int teleop_send, int teleop_recv,
                             bool recording, const std::string& inference_host,
                             int inference_send, int inference_recv)
    : stop_threads(false)
    , recording(recording)
    , inference_active(false)
    , send_socket(io_context, boost::asio::ip::udp::v4())
    , teleop_recv_socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), teleop_recv))
    , inference_recv_socket(io_context)
    , inference_host(inference_host)
    , inference_send_port(inference_send)
    , inference_recv_port(inference_recv) {

    // Always add teleop to our broadcast list
    send_endpoints.push_back(boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(teleop_host), teleop_send));

    teleop_recv_thread = std::thread(&UDPHandler::receiveLoop, this, std::ref(teleop_recv_socket), std::ref(latest_teleop_received));

    if (recording) {
        send_endpoints.push_back(boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(inference_host), inference_send));
    }

    send_thread = std::thread(&UDPHandler::sendLoop, this);
}

template <size_t DOF>
UDPHandler<DOF>::~UDPHandler() {
    stop();
}

template <size_t DOF>
void UDPHandler<DOF>::stop() {
    stop_threads = true;
    io_context.stop();
    send_condition.notify_all();
    try {
        if (teleop_recv_socket.is_open()) {
            teleop_recv_socket.cancel();
            teleop_recv_socket.shutdown(boost::asio::ip::udp::socket::shutdown_both);
            teleop_recv_socket.close();
        }
        if (inference_recv_socket.is_open()) {
            inference_recv_socket.cancel();
            inference_recv_socket.shutdown(boost::asio::ip::udp::socket::shutdown_both);
            inference_recv_socket.close();
        }
    } catch (...) {
        // Ignore any exceptions during socket cleanup
    }

    if (teleop_recv_thread.joinable()) teleop_recv_thread.join();
    if (inference_recv_thread.joinable()) inference_recv_thread.join();
    if (send_thread.joinable()) send_thread.join();
}

template <size_t DOF>
void UDPHandler<DOF>::enableInference() {
    std::lock_guard<std::mutex> lock(inference_socket_mutex);
    if (inference_active) return;

    inference_recv_socket = boost::asio::ip::udp::socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), inference_recv_port));
    inference_recv_thread = std::thread(&UDPHandler::receiveLoop, this, std::ref(inference_recv_socket), std::ref(latest_inference_received));

    if (!recording) {
        send_endpoints.push_back(boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(inference_host), inference_send_port));
    }

    inference_active = true;
}

template <size_t DOF>
void UDPHandler<DOF>::disableInference() {
    {
        std::lock_guard<std::mutex> lock(inference_socket_mutex);
        if (!inference_active) return;

        if (inference_recv_socket.is_open()) {
            inference_recv_socket.cancel();
            inference_recv_socket.close();
        }

        // recording still communicates over inference socket
        if (!recording) {
            std::lock_guard<std::mutex> send_lock(send_mutex);
            auto it = std::remove_if(send_endpoints.begin(), send_endpoints.end(),
                [&](const auto& ep) { return ep.port() == inference_send_port; });
            send_endpoints.erase(it, send_endpoints.end());
        }

        latest_inference_received = boost::none;
        inference_active = false;
    }
    if (inference_recv_thread.joinable()) {
        inference_recv_thread.join();
    }
}

template <size_t DOF>
boost::optional<typename UDPHandler<DOF>::ReceivedData> UDPHandler<DOF>::getLatestTeleopReceived() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return latest_teleop_received;
}

template <size_t DOF>
boost::optional<typename UDPHandler<DOF>::ReceivedData> UDPHandler<DOF>::getLatestInferenceReceived() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return latest_inference_received;
}

template <size_t DOF>
typename UDPHandler<DOF>::ReceivedData UDPHandler<DOF>::unpackPacket(const Packet& pkt) {
    ReceivedData rd;
    std::memcpy(rd.jp.data(),        pkt.jp,        sizeof(double) * DOF);
    std::memcpy(rd.jv.data(),        pkt.jv,        sizeof(double) * DOF);
    std::memcpy(rd.extTorque.data(), pkt.extTorque, sizeof(double) * DOF);
    std::memcpy(rd.measTorque.data(),pkt.measTorque,sizeof(double) * DOF);
    rd.cart_pos = Eigen::Vector3d(pkt.cart_pos[0], pkt.cart_pos[1], pkt.cart_pos[2]);
    rd.cart_rot = Eigen::Quaterniond(pkt.cart_rot[0], pkt.cart_rot[1], pkt.cart_rot[2], pkt.cart_rot[3]);
    rd.gripper   = pkt.gripper;
    rd.is_clutching   = pkt.is_clutching;
    std::memcpy(rd.offset.data(),    pkt.offset,    sizeof(double) * DOF);
    rd.timestamp = pkt.timestamp;
    return rd;
}

template <size_t DOF>
void UDPHandler<DOF>::receiveLoop(boost::asio::ip::udp::socket& recv_socket, boost::optional<ReceivedData>& latest_received) {
    boost::asio::ip::udp::endpoint sender_endpoint;
    Packet pkt;

    while (!stop_threads) {
        boost::system::error_code ec;
        size_t len = recv_socket.receive_from(
            boost::asio::buffer(&pkt, sizeof(Packet)), sender_endpoint, 0, ec);

        if (ec == boost::asio::error::operation_aborted || len != sizeof(Packet))
            continue;

        std::lock_guard<std::mutex> lock(state_mutex);
        latest_received = unpackPacket(pkt);
    }
    recv_socket.close();
}

template <size_t DOF>
void UDPHandler<DOF>::send(const jp_type& jp, const jv_type& jv,
                           const jt_type& extTorque, const jt_type& measTorque,
                           const Eigen::Vector3d& cart_pos, const Eigen::Quaterniond& cart_rot,
                           const double gripper, const bool is_clutching, const jp_type& offset, uint64_t timestamp) {
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        std::memcpy(pending_send_packet.jp, jp.data(), sizeof(double) * DOF);
        std::memcpy(pending_send_packet.jv, jv.data(), sizeof(double) * DOF);
        std::memcpy(pending_send_packet.extTorque, extTorque.data(), sizeof(double) * DOF);
        std::memcpy(pending_send_packet.measTorque, measTorque.data(), sizeof(double) * DOF);
        pending_send_packet.cart_pos[0] = cart_pos.x();
        pending_send_packet.cart_pos[1] = cart_pos.y();
        pending_send_packet.cart_pos[2] = cart_pos.z();
        pending_send_packet.cart_rot[0] = cart_rot.w();
        pending_send_packet.cart_rot[1] = cart_rot.x();
        pending_send_packet.cart_rot[2] = cart_rot.y();
        pending_send_packet.cart_rot[3] = cart_rot.z();
        pending_send_packet.gripper = gripper;
        pending_send_packet.is_clutching = is_clutching;
        std::memcpy(pending_send_packet.offset, offset.data(), sizeof(double) * DOF);
        pending_send_packet.timestamp = timestamp;
        new_data_available = true;
    }
    send_condition.notify_one();
}

template <size_t DOF>
void UDPHandler<DOF>::sendLoop() {
    Packet pkt_to_send;
    std::vector<boost::asio::ip::udp::endpoint> endpoints_snapshot;

    while (!stop_threads) {
        {
            std::unique_lock<std::mutex> lock(send_mutex);
            send_condition.wait(lock, [this] { return new_data_available || stop_threads; });
            if (stop_threads) break;

            pkt_to_send        = pending_send_packet;
            endpoints_snapshot = send_endpoints;
            new_data_available = false;
        }

        boost::system::error_code ec;
        for (const auto& ep : endpoints_snapshot)
            send_socket.send_to(boost::asio::buffer(&pkt_to_send, sizeof(Packet)), ep, 0, ec);
    }
    send_socket.close();
}


template class UDPHandler<7>; // For DOF=7
