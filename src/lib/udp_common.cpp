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
                                          const Eigen::Vector3d& follower_cart_pos, const Eigen::Quaterniond& follower_quat,
                                          const Eigen::Vector3d& leader_cart_pos, const Eigen::Quaterniond& leader_quat,
                                          double gripper_pos, double gripper_vel, double gripper_torque,
                                          uint64_t timestamp) {
    if (!send_active) return;

    {
        std::lock_guard<std::mutex> lock(send_mutex);
        std::memcpy(pending_packet.follower_jp, follower_jp.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.follower_jv, follower_jv.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.follower_extTorque, follower_extTorque.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.leader_jp, leader_jp.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.leader_jv, leader_jv.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.leader_extTorque, leader_extTorque.data(), sizeof(double) * DOF);
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
        pending_packet.timestamp = timestamp;
        new_data_available = true;
    }
    send_condition.notify_one();
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::clearQueueAndPause(std::chrono::milliseconds duration) {
    std::lock_guard<std::mutex> lock(state_mutex);
    action_queue.clear();
    pause_until = std::chrono::steady_clock::now() + duration;
}


template <size_t DOF>
std::deque<PolicyReceivedData> PolicyUDPHandler<DOF>::interpolateChunk(const RawAction (&actions)[ACTION_HORIZON],
                                                                        uint64_t inference_timestamp_ns,
                                                                        const RawAction* last_action) {
    std::deque<PolicyReceivedData> queue;
    const double policy_dt_ns = 1e9 / UNINTERP_HZ;
    const double dt_s = policy_dt_ns * 1e-9; // seconds between consecutive raw waypoints

    // Catmull-Rom cubic position/velocity/acceleration between p1 and p2 (p0/p3 = outer support points).
    struct CRResult { double pos, dpos_dt, d2pos_dt2; };
    auto catmullRom = [](double p0, double p1, double p2, double p3, double t) -> CRResult {
        const double t2 = t * t;
        // coefficients of the cubic in t
        const double c1 = (-p0 + p2);
        const double c2 = (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3);
        const double c3 = (-p0 + 3.0 * p1 - 3.0 * p2 + p3);
 
        CRResult r;
        r.pos       = 0.5 * (2.0 * p1 + c1 * t + c2 * t2 + c3 * t2 * t);
        r.dpos_dt   = 0.5 * (c1 + 2.0 * c2 * t + 3.0 * c3 * t2);
        r.d2pos_dt2 = 0.5 * (2.0 * c2 + 6.0 * c3 * t);
        return r;
    };

    for (size_t j = 0; j < NUM_INTERP_SAMPLES; ++j) {
        // map sample j -> fractional position across the horizon waypoints
        double frac = static_cast<double>(j) / static_cast<double>(NUM_INTERP_SAMPLES - 1);
        double pos = frac * static_cast<double>(ACTION_HORIZON - 1);

        // idx1/idx2 bracket pos (equivalent to old idx0/idx1);
        // idx0/idx3 are outer support points used to estimate local tangents/curvature.
        size_t idx1 = static_cast<size_t>(pos);
        size_t idx2 = std::min(idx1 + 1, ACTION_HORIZON - 1);
        size_t idx3 = std::min(idx2 + 1, ACTION_HORIZON - 1);
        double alpha = pos - static_cast<double>(idx1);

        // For the very first segment (idx1 == 0) the "outer support" point normally has to be
        // clamped (duplicated) because there's nothing before actions[0] within this chunk.
        // If we have the previous chunk's real last waypoint, use that instead so the spline's
        // tangent/velocity is continuous across the chunk seam rather than artificially flattened.
        const RawAction& a0 = (idx1 == 0 && last_action != nullptr) ? *last_action : actions[(idx1 == 0) ? idx1 : idx1 - 1];
        const RawAction& a1 = actions[idx1];
        const RawAction& a2 = actions[idx2];
        const RawAction& a3 = actions[idx3];

        PolicyReceivedData rd;
        for (size_t k = 0; k < 7; ++k) {
            CRResult r = catmullRom(a0.jp[k], a1.jp[k], a2.jp[k], a3.jp[k], alpha);
            rd.jp[k] = r.pos;
            rd.jv[k] = r.dpos_dt / dt_s;             // chain rule: d/dtime = d/dalpha * dalpha/dtime
            rd.ja[k] = r.d2pos_dt2 / (dt_s * dt_s);  // d^2/dtime^2 = d^2/dalpha^2 * (dalpha/dtime)^2
        }

        {
            CRResult rg = catmullRom(a0.gripper_cmd, a1.gripper_cmd, a2.gripper_cmd, a3.gripper_cmd, alpha);
            rd.gripper_cmd = rg.pos;
        }

        rd.timestamp = inference_timestamp_ns + static_cast<uint64_t>(pos * policy_dt_ns) - static_cast<uint64_t>(2 * policy_dt_ns);
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

        RawAction local_last_action;
        bool local_have_last_action = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            if (have_last_action) {
                local_last_action = last_action;
                local_have_last_action = true;
            }

        }

        std::deque<PolicyReceivedData> new_queue = interpolateChunk(pkt.actions, pkt.inference_timestamp_ns, local_have_last_action ? &local_last_action : nullptr);

        PolicyActionChunkPacket record;
        std::memcpy(record.actions, pkt.actions, sizeof(pkt.actions));
        record.inference_timestamp_ns = pkt.inference_timestamp_ns;


        std::lock_guard<std::mutex> lock(state_mutex);
        if (std::chrono::steady_clock::now() < pause_until) {
            // do not add any new actions to the queue for a little bit if the user canceled an episode
            continue;
        }

        // a new chunk extends what is left in the queue
        action_queue.insert(action_queue.end(),
                             std::make_move_iterator(new_queue.begin()),
                             std::make_move_iterator(new_queue.end()));

        last_action = pkt.actions[ACTION_HORIZON - 1];
        have_last_action = true;
    }
    recv_socket.close();
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
            should_send = static_cast<double>(action_queue.size()) / static_cast<double>(NUM_INTERP_SAMPLES) <= 0.1;
        }

        if (should_send) {
            boost::system::error_code ec;
            send_socket.send_to(boost::asio::buffer(&pkt_to_send, sizeof(PolicyPacketType)), policy_endpoint, 0, ec);
        }
    }
    send_socket.close();
}

template class PolicyUDPHandler<7>; // For DOF=7
