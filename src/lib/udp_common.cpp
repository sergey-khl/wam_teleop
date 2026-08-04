#include "udp_common.h"
#include <boost/system/error_code.hpp>
#include <algorithm>
#include <cstring>
#include <iostream>

template <size_t DOF>
PolicyUDPHandler<DOF>::PolicyUDPHandler(bool send_active, const std::string& policy_host, int policy_send_port, int policy_recv_port)
    : send_active(send_active)
    , stop_threads(false)
    , send_socket(io_context)
    , recv_socket(io_context)
    , policy_endpoint(boost::asio::ip::make_address(policy_host), policy_send_port)
    , policy_recv_port(policy_recv_port) {

    recv_socket.open(boost::asio::ip::udp::v4());
    recv_socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), policy_recv_port));
    recv_thread = std::thread(&PolicyUDPHandler::receiveLoop, this);

    interp_thread = std::thread(&PolicyUDPHandler::interpLoop, this);

    if (send_active) {
        send_socket.open(boost::asio::ip::udp::v4());

        send_thread = std::thread(&PolicyUDPHandler::sendLoop, this);
    }
}

template <size_t DOF>
PolicyUDPHandler<DOF>::~PolicyUDPHandler() {
    stop();
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::stop() {
    stop_threads = true;
    io_context.stop();
    send_condition.notify_all();
    raw_condition.notify_all();
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
    if (interp_thread.joinable()) interp_thread.join();
}

template <size_t DOF>
boost::optional<PolicyReceivedData> PolicyUDPHandler<DOF>::getLatestPolicyReceived() {
    std::lock_guard<std::mutex> lock(state_mutex);
    if (action_queue.empty()) {
        return boost::none;
    }

    PolicyReceivedData rd = action_queue.front();
    if (action_queue.size() == 1) {
        return rd;
    }
    action_queue.pop_front();
    return rd;
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::send(const jp_type& follower_jp, const jv_type& follower_jv,
                                          const jt_type& follower_extTorque, const jp_type& leader_jp,
                                          const jv_type& leader_jv, const jt_type& leader_extTorque,
                                          const jt_type& policyTorqueScale,
                                          const Eigen::Vector3d& follower_cart_pos, const Eigen::Quaterniond& follower_quat,
                                          const Eigen::Vector3d& leader_cart_pos, const Eigen::Quaterniond& leader_quat,
                                          double gripper_pos, double gripper_vel, double gripper_torque,
                                          uint64_t timestamp) {
    if (!send_active) return;

    {
        std::lock_guard<std::mutex> lock(send_mutex);

        uint64_t time_to_chunk_end = static_cast<double>();

        std::memcpy(pending_packet.follower_jp, follower_jp.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.follower_jv, follower_jv.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.follower_extTorque, follower_extTorque.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.leader_jp, leader_jp.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.leader_jv, leader_jv.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.leader_extTorque, leader_extTorque.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.policyTorqueScale, policyTorqueScale.data(), sizeof(double) * DOF);
        pending_packet.follower_cart_pos[0] = follower_cart_pos.x();
        pending_packet.follower_cart_pos[1] = follower_cart_pos.y();
        pending_packet.follower_cart_pos[2] = follower_cart_pos.z();
        pending_packet.follower_quat[0] = follower_quat.w();
        pending_packet.follower_quat[1] = follower_quat.x();
        pending_packet.follower_quat[2] = follower_quat.y();
        pending_packet.follower_quat[3] = follower_quat.z();
        pending_packet.leader_cart_pos[0] = leader_cart_pos.x();
        pending_packet.leader_cart_pos[1] = leader_cart_pos.y();
        pending_packet.leader_cart_pos[2] = leader_cart_pos.z();
        pending_packet.leader_quat[0] = leader_quat.w();
        pending_packet.leader_quat[1] = leader_quat.x();
        pending_packet.leader_quat[2] = leader_quat.y();
        pending_packet.leader_quat[3] = leader_quat.z();
        pending_packet.gripper_pos = gripper_pos;
        pending_packet.gripper_vel = gripper_vel;
        pending_packet.gripper_torque = gripper_torque;
        pending_packet.time_to_chunk_end = time_to_chunk_end;
        pending_packet.timestamp = timestamp;
        new_data_available = true;
    }
    send_condition.notify_one();
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::clearQueueAndPause(std::chrono::milliseconds duration) {
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        action_queue.clear();
        pause_until = std::chrono::steady_clock::now() + duration;
    }
    {
        std::lock_guard<std::mutex> lock(raw_mutex);
        raw_waypoint_queue.clear();
    }
}

template <size_t DOF>
std::deque<PolicyReceivedData> PolicyUDPHandler<DOF>::interpolateSegment(
        const RawAction& a0, const RawAction& a1, const RawAction& a2, const RawAction& a3) {
    std::deque<PolicyReceivedData> queue;
    const double policy_dt_ns = 1e9 / UNINTERP_HZ;
    const double dt_s = policy_dt_ns * 1e-9;

    struct CRResult { double pos, dpos_dt, d2pos_dt2; };
    auto catmullRom = [](double p0, double p1, double p2, double p3, double t) -> CRResult {
        const double t2 = t * t;
        const double c1 = (-p0 + p2);
        const double c2 = (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3);
        const double c3 = (-p0 + 3.0 * p1 - 3.0 * p2 + p3);
        CRResult r;
        r.pos       = 0.5 * (2.0 * p1 + c1 * t + c2 * t2 + c3 * t2 * t);
        r.dpos_dt   = 0.5 * (c1 + 2.0 * c2 * t + 3.0 * c3 * t2);
        r.d2pos_dt2 = 0.5 * (2.0 * c2 + 6.0 * c3 * t);
        return r;
    };

    for (size_t j = 0; j < SAMPLES_PER_SEGMENT; ++j) {
        // alpha goes between a1 and a2
        double alpha = static_cast<double>(j) / static_cast<double>(SAMPLES_PER_SEGMENT);

        PolicyReceivedData rd;
        for (size_t k = 0; k < 7; ++k) {
            CRResult r = catmullRom(a0.jp[k], a1.jp[k], a2.jp[k], a3.jp[k], alpha);
            rd.jp[k] = r.pos;
            rd.jv[k] = r.dpos_dt / dt_s;
            rd.ja[k] = r.d2pos_dt2 / (dt_s * dt_s);
        }

        CRResult rg = catmullRom(a0.gripper_cmd, a1.gripper_cmd, a2.gripper_cmd, a3.gripper_cmd, alpha);
        rd.gripper_cmd = rg.pos;

        queue.push_back(rd);
    }
    return queue;
}


template <size_t DOF>
void PolicyUDPHandler<DOF>::receiveLoop() {
    boost::asio::ip::udp::endpoint sender_endpoint;
    PolicyActionChunkPacket pkt;

    while (!stop_threads) {
        boost::system::error_code ec;
        size_t len = recv_socket.receive_from(
            boost::asio::buffer(&pkt, sizeof(PolicyActionChunkPacket)), sender_endpoint, 0, ec);

        if (ec == boost::asio::error::operation_aborted || len != sizeof(PolicyActionChunkPacket)) {
            // std::cout << "got " << len << " expected " << sizeof(PolicyActionChunkPacket) << std::endl;
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(raw_mutex);
            raw_waypoint_queue.insert(raw_waypoint_queue.end(),
                                       std::begin(pkt.actions), std::end(pkt.actions));
        }
        raw_condition.notify_one();
    }
    recv_socket.close();
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::interpLoop() {
    while (!stop_threads) {
        RawAction a0, a1, a2, a3;
        {
            std::unique_lock<std::mutex> lock(raw_mutex);
            raw_condition.wait(lock, [this] {
                return stop_threads || raw_waypoint_queue.size() >= 4;
            });
            if (stop_threads) break;

            if (first_segment_ever) {
                a0 = raw_waypoint_queue[0];
                a1 = raw_waypoint_queue[0];
                a2 = raw_waypoint_queue[1];
                a3 = raw_waypoint_queue[2];

                first_segment_ever = false;
            } else {
                a0 = raw_waypoint_queue[0];
                a1 = raw_waypoint_queue[1];
                a2 = raw_waypoint_queue[2];
                a3 = raw_waypoint_queue[3];
                
                raw_waypoint_queue.pop_front(); // slide the window by one support point
            }
        }

        std::deque<PolicyReceivedData> new_samples = interpolateSegment(a0, a1, a2, a3);

        std::lock_guard<std::mutex> lock(state_mutex);
        if (std::chrono::steady_clock::now() < pause_until) {
            continue;
        }
        action_queue.insert(action_queue.end(),
                             std::make_move_iterator(new_samples.begin()),
                             std::make_move_iterator(new_samples.end()));
    }
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::sendLoop() {
    PolicyPacketType pkt_to_send;
    bool should_send;

    while (!stop_threads) {
        {
            std::unique_lock<std::mutex> lock(send_mutex);
            send_condition.wait(lock, [this] { return new_data_available || stop_threads; });
            if (stop_threads) break;

            pkt_to_send = pending_packet;
            new_data_available = false;
        }

        {
            // for creating a seamless policy loop, we start inference when x% of our actions are left
            std::lock_guard<std::mutex> lock(state_mutex);
        }

        boost::system::error_code ec;
        send_socket.send_to(boost::asio::buffer(&pkt_to_send, sizeof(PolicyPacketType)), policy_endpoint, 0, ec);
    }
    send_socket.close();
}

template class PolicyUDPHandler<7>; // For DOF=7
