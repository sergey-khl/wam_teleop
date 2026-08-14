#include "follower_udp_handler.h"
#include <boost/system/error_code.hpp>
#include <cstdint>
#include <cstring>

template <size_t DOF>
FollowerUDPHandler<DOF>::FollowerUDPHandler(const std::string& leader_host, int teleop_send, int teleop_recv)
    : stop_threads(false)
    , teleop_send_socket(io_context, boost::asio::ip::udp::v4())
    , teleop_recv_socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), teleop_recv))
    , leader_endpoint(boost::asio::ip::make_address(leader_host), teleop_send) {

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
boost::optional<typename FollowerUDPHandler<DOF>::TeleopReceivedData> FollowerUDPHandler<DOF>::getLatestTeleopReceived() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return latest_teleop_received;
}

template <size_t DOF>
typename FollowerUDPHandler<DOF>::TeleopReceivedData FollowerUDPHandler<DOF>::unpackTeleopPacket(const TeleopRecvPacket& pkt) {
    TeleopReceivedData rd;
    std::memcpy(rd.jp.data(), pkt.jp, sizeof(double) * DOF);
    std::memcpy(rd.jv.data(), pkt.jv, sizeof(double) * DOF);
    std::memcpy(rd.dyngravcompTorque.data(), pkt.dyngravcompTorque, sizeof(double) * DOF);
    std::memcpy(rd.humanTorque.data(), pkt.humanTorque, sizeof(double) * DOF);
    std::memcpy(rd.cart_pos.data(), pkt.cart_pos, sizeof(double) * 3);
    rd.quat = Eigen::Quaterniond(pkt.quat[0], pkt.quat[1], pkt.quat[2], pkt.quat[3]); // w, x, y, z
    std::memcpy(rd.policyTorqueScale.data(), pkt.policyTorqueScale, sizeof(double) * DOF);
    rd.gripper_cmd = pkt.gripper_cmd;
    rd.cancel_policy = pkt.cancel_policy;
    rd.timestamp = pkt.timestamp;
    return rd;
}

template <size_t DOF>
void FollowerUDPHandler<DOF>::teleopReceiveLoop() {
    boost::asio::ip::udp::endpoint sender_endpoint;
    TeleopRecvPacket pkt;

    while (!stop_threads) {
        boost::system::error_code ec;
        size_t len = teleop_recv_socket.receive_from(
            boost::asio::buffer(&pkt, sizeof(TeleopRecvPacket)), sender_endpoint, 0, ec);

        if (ec == boost::asio::error::operation_aborted || len != sizeof(TeleopRecvPacket))
            continue;

        std::lock_guard<std::mutex> lock(state_mutex);
        latest_teleop_received = unpackTeleopPacket(pkt);
    }
    teleop_recv_socket.close();
}

template <size_t DOF>
void FollowerUDPHandler<DOF>::send(const jp_type& jp, const jv_type& jv,
                                   const jt_type& dyngravcompTorque, const jt_type& humanTorque,
                                   const cp_type& cart_pos, const Eigen::Quaterniond& quat,
                                   double gripper_torque, double gripper_pos, double gripper_vel, uint64_t timestamp) {
    {
        std::lock_guard<std::mutex> lock(teleop_send_mutex);
        std::memcpy(pending_teleop_packet.jp, jp.data(), sizeof(double) * DOF);
        std::memcpy(pending_teleop_packet.jv, jv.data(), sizeof(double) * DOF);
        std::memcpy(pending_teleop_packet.dyngravcompTorque, dyngravcompTorque.data(), sizeof(double) * DOF);
        std::memcpy(pending_teleop_packet.humanTorque, humanTorque.data(), sizeof(double) * DOF);
        std::memcpy(pending_teleop_packet.cart_pos, cart_pos.data(), sizeof(double) * 3);
        pending_teleop_packet.quat[0] = quat.w();
        pending_teleop_packet.quat[1] = quat.x();
        pending_teleop_packet.quat[2] = quat.y();
        pending_teleop_packet.quat[3] = quat.z();
        pending_teleop_packet.gripper_torque = gripper_torque;
        pending_teleop_packet.gripper_pos = gripper_pos;
        pending_teleop_packet.gripper_vel = gripper_vel;
        pending_teleop_packet.timestamp = timestamp;
        new_teleop_data = true;
    }
    teleop_send_condition.notify_one();
}

template <size_t DOF>
void FollowerUDPHandler<DOF>::teleopSendLoop() {
    TeleopPacket pkt_to_send;

    while (!stop_threads) {
        {
            std::unique_lock<std::mutex> lock(teleop_send_mutex);
            teleop_send_condition.wait(lock, [this] { return new_teleop_data || stop_threads; });
            if (stop_threads) break;

            pkt_to_send = pending_teleop_packet;
            new_teleop_data = false;
        }

        boost::system::error_code ec;
        teleop_send_socket.send_to(boost::asio::buffer(&pkt_to_send, sizeof(TeleopPacket)), leader_endpoint, 0, ec);
    }
    teleop_send_socket.close();
}

template class FollowerUDPHandler<7>; // For DOF=7
