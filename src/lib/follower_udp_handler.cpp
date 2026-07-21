#include "follower_udp_handler.h"
#include <boost/system/error_code.hpp>
#include <cstdint>
#include <cstring>
#include <iostream>

template <size_t DOF>
FollowerUDPHandler<DOF>::FollowerUDPHandler(const std::string& leader_host, int teleop_send, int teleop_recv,
                                              bool policy_send_active,
                                              const std::string& policy_host, int policy_send, int policy_recv)
    : stop_threads(false)
    , teleop_send_socket(io_context, boost::asio::ip::udp::v4())
    , policy_send_socket(io_context, boost::asio::ip::udp::v4())
    , teleop_recv_socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), teleop_recv))
    , policy_recv_socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), policy_recv))
    , leader_endpoint(boost::asio::ip::make_address(leader_host), teleop_send)
    , policy_endpoint(boost::asio::ip::make_address(policy_host), policy_send)
    , policy_send_active(policy_send_active)
    , policy_recv_port(policy_recv) {

    teleop_recv_thread = std::thread(&FollowerUDPHandler::teleopReceiveLoop, this);
    teleop_send_thread = std::thread(&FollowerUDPHandler::teleopSendLoop, this);
    policy_send_thread = std::thread(&FollowerUDPHandler::policySendLoop, this);
    policy_recv_thread = std::thread(&FollowerUDPHandler::policyReceiveLoop, this);
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
    policy_send_condition.notify_all();
    try {
        if (teleop_recv_socket.is_open()) {
            teleop_recv_socket.cancel();
            teleop_recv_socket.shutdown(boost::asio::ip::udp::socket::shutdown_both);
            teleop_recv_socket.close();
        }
        if (policy_recv_socket.is_open()) {
            policy_recv_socket.cancel();
            policy_recv_socket.shutdown(boost::asio::ip::udp::socket::shutdown_both);
            policy_recv_socket.close();
        }
    } catch (...) {
        // Ignore any exceptions during socket cleanup
    }

    if (teleop_recv_thread.joinable()) teleop_recv_thread.join();
    if (policy_recv_thread.joinable()) policy_recv_thread.join();
    if (teleop_send_thread.joinable()) teleop_send_thread.join();
    if (policy_send_thread.joinable()) policy_send_thread.join();
}

template <size_t DOF>
boost::optional<typename FollowerUDPHandler<DOF>::TeleopReceivedData> FollowerUDPHandler<DOF>::getLatestTeleopReceived() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return latest_teleop_received;
}


template <size_t DOF>
boost::optional<PolicyReceivedData> FollowerUDPHandler<DOF>::getLatestPolicyReceived() {
    std::lock_guard<std::mutex> lock(state_mutex);
    if (policy_action_queue.empty()) {
        return boost::none;
    }
    PolicyReceivedData rd = policy_action_queue.front();
    policy_action_queue.pop_front();
    return rd;
}

template <size_t DOF>
typename FollowerUDPHandler<DOF>::TeleopReceivedData FollowerUDPHandler<DOF>::unpackTeleopPacket(const TeleopRecvPacket& pkt) {
    TeleopReceivedData rd;
    std::memcpy(rd.jp.data(), pkt.jp, sizeof(double) * DOF);
    std::memcpy(rd.jv.data(), pkt.jv, sizeof(double) * DOF);
    std::memcpy(rd.extTorque.data(), pkt.extTorque, sizeof(double) * DOF);
    rd.gripper_cmd = pkt.gripper_cmd;
    rd.timestamp = pkt.timestamp;
    return rd;
}

template <size_t DOF>
std::deque<PolicyReceivedData> FollowerUDPHandler<DOF>::interpolateChunk(
    const RawAction (&actions)[ACTION_HORIZON], uint64_t inference_timestamp_ns) {
    std::deque<PolicyReceivedData> queue;

    const double policy_dt_ns = 1e9 / UNINTERP_HZ;

    const uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

    for (size_t j = 0; j < NUM_INTERP_SAMPLES; ++j) {
        // map sample j -> fractional position across the horizon waypoints
        double frac = static_cast<double>(j) / static_cast<double>(NUM_INTERP_SAMPLES - 1);
        double pos = frac * static_cast<double>(ACTION_HORIZON - 1);
        // the idx are the floor and ceiling of the continuous pos
        size_t idx0 = static_cast<size_t>(pos);
        size_t idx1 = std::min(idx0 + 1, ACTION_HORIZON - 1);
        double alpha = pos - static_cast<double>(idx0);

        const RawAction& a = actions[idx0];
        const RawAction& b = actions[idx1];

        PolicyReceivedData rd;
        for (size_t k = 0; k < DOF; ++k) {
            rd.jp[k] = a.jp[k] + alpha * (b.jp[k] - a.jp[k]);
        }
        rd.gripper_cmd = a.gripper_cmd + alpha * (b.gripper_cmd - a.gripper_cmd);

        rd.timestamp = inference_timestamp_ns + static_cast<uint64_t>(pos * policy_dt_ns) - static_cast<uint64_t>(2 * policy_dt_ns);

        queue.push_back(rd);
    }
    return queue;
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
void FollowerUDPHandler<DOF>::policyReceiveLoop() {
    boost::asio::ip::udp::endpoint sender_endpoint;
    PolicyActionChunkPacket pkt;

    while (!stop_threads) {
        boost::system::error_code ec;
        size_t len = policy_recv_socket.receive_from(
            boost::asio::buffer(&pkt, sizeof(PolicyActionChunkPacket)), sender_endpoint, 0, ec);

        if (ec == boost::asio::error::operation_aborted || len != sizeof(PolicyActionChunkPacket))
            continue;

        std::deque<PolicyReceivedData> new_queue = interpolateChunk(pkt.actions, pkt.inference_timestamp_ns);

        std::lock_guard<std::mutex> lock(state_mutex);
        // a new chunk supersedes whatever's left of the old one
        policy_action_queue.insert(policy_action_queue.end(),
                            std::make_move_iterator(new_queue.begin()),
                            std::make_move_iterator(new_queue.end()));
    }
    policy_recv_socket.close();
}

template <size_t DOF>
void FollowerUDPHandler<DOF>::send(const jp_type& jp, const jv_type& jv, const jt_type& extTorque,
                                     double gripper_torque, uint64_t timestamp) {
    {
        std::lock_guard<std::mutex> lock(teleop_send_mutex);
        std::memcpy(pending_teleop_packet.jp, jp.data(), sizeof(double) * DOF);
        std::memcpy(pending_teleop_packet.jv, jv.data(), sizeof(double) * DOF);
        std::memcpy(pending_teleop_packet.extTorque, extTorque.data(), sizeof(double) * DOF);
        pending_teleop_packet.gripper_torque = gripper_torque;
        pending_teleop_packet.timestamp = timestamp;
        new_teleop_data = true;
    }
    teleop_send_condition.notify_one();
}

template <size_t DOF>
void FollowerUDPHandler<DOF>::sendToPolicy(const jp_type& follower_jp, const jv_type& follower_jv, const jt_type& follower_extTorque,
                                             const jp_type& leader_jp, const jv_type& leader_jv, const jt_type& leader_extTorque,
                                             const Eigen::Vector3d& follower_cart_pos, const Eigen::Quaterniond& follower_cart_rot,
                                             double gripper_pos, double gripper_vel, double gripper_torque,
                                             uint64_t timestamp) {
    {
        std::lock_guard<std::mutex> lock(policy_send_mutex);
        std::memcpy(pending_policy_packet.follower_jp, follower_jp.data(), sizeof(double) * DOF);
        std::memcpy(pending_policy_packet.follower_jv, follower_jv.data(), sizeof(double) * DOF);
        std::memcpy(pending_policy_packet.follower_extTorque, follower_extTorque.data(), sizeof(double) * DOF);
        std::memcpy(pending_policy_packet.leader_jp, leader_jp.data(), sizeof(double) * DOF);
        std::memcpy(pending_policy_packet.leader_jv, leader_jv.data(), sizeof(double) * DOF);
        std::memcpy(pending_policy_packet.leader_extTorque, leader_extTorque.data(), sizeof(double) * DOF);
        pending_policy_packet.follower_cart_pos[0] = follower_cart_pos.x();
        pending_policy_packet.follower_cart_pos[1] = follower_cart_pos.y();
        pending_policy_packet.follower_cart_pos[2] = follower_cart_pos.z();
        pending_policy_packet.follower_cart_rot[0] = follower_cart_rot.w();
        pending_policy_packet.follower_cart_rot[1] = follower_cart_rot.x();
        pending_policy_packet.follower_cart_rot[2] = follower_cart_rot.y();
        pending_policy_packet.follower_cart_rot[3] = follower_cart_rot.z();
        pending_policy_packet.gripper_pos = gripper_pos;
        pending_policy_packet.gripper_vel = gripper_vel;
        pending_policy_packet.gripper_torque = gripper_torque;
        pending_policy_packet.timestamp = timestamp;
        new_policy_data = true;
    }
    policy_send_condition.notify_one();
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

template <size_t DOF>
void FollowerUDPHandler<DOF>::policySendLoop() {
    PolicyPacketType pkt_to_send;
    bool should_send;

    while (!stop_threads) {
        {
            std::unique_lock<std::mutex> lock(policy_send_mutex);
            policy_send_condition.wait(lock, [this] { return new_policy_data || stop_threads; });
            if (stop_threads) break;

            pkt_to_send = pending_policy_packet;
            // for creating a seamless policy loop, we start inference when x% of our actions are left
            should_send = policy_send_active && static_cast<double>(policy_action_queue.size()) / static_cast<double>(NUM_INTERP_SAMPLES) <= 0.0;
            new_policy_data = false;
        }

        if (should_send) {
            boost::system::error_code ec;
            policy_send_socket.send_to(boost::asio::buffer(&pkt_to_send, sizeof(PolicyPacketType)), policy_endpoint, 0, ec);
        }
    }
    policy_send_socket.close();
}

template class FollowerUDPHandler<7>; // For DOF=7
