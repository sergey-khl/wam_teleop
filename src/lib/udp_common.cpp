#include "udp_common.h"
#include <boost/system/error_code.hpp>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <iostream>

template <size_t DOF>
GenericAction PolicyUDPHandler<DOF>::toGeneric(const BaseRawAction& a) {
    GenericAction g{};
    std::memcpy(g.pos, a.jp, sizeof(g.pos));
    g.gripper_cmd = a.gripper_cmd;
    // as is base has no torque
    std::memset(g.torque, 0, sizeof(g.torque));
    return g;
}
 
template <size_t DOF>
GenericAction PolicyUDPHandler<DOF>::toGeneric(const DgRawAction& a) {
    GenericAction g{};
    std::memcpy(g.pos, a.jp, sizeof(g.pos));
    g.gripper_cmd = a.gripper_cmd;
    std::memcpy(g.torque, a.ext_torque, sizeof(g.torque));
    return g;
}
 
template <size_t DOF>
GenericAction PolicyUDPHandler<DOF>::toGeneric(const CrRawAction& a) {
    GenericAction g{};
    std::memcpy(g.pos, a.delta_jp, sizeof(g.pos));
    g.gripper_cmd = a.gripper_cmd;
    std::memcpy(g.torque, a.ext_torque, sizeof(g.torque));
    return g;
}


template <size_t DOF>
PolicyUDPHandler<DOF>::PolicyUDPHandler(const std::string& type, bool send_active, const std::string& policy_host, int policy_send_port, int policy_recv_port)
    : type(type)
    , send_active(send_active)
    , stop_threads(false)
    , send_socket(io_context)
    , recv_socket(io_context)
    , cr_recv_socket(io_context)
    , policy_endpoint(boost::asio::ip::make_address(policy_host), policy_send_port)
    , policy_recv_port(policy_recv_port) {

    recv_socket.open(boost::asio::ip::udp::v4());
    recv_socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), policy_recv_port));

    if (type == "dg") {
        recv_thread = std::thread([this] {
            genericReceiveLoop<DgPolicyActionChunkPacket, DgRawAction>(
                recv_socket, dg_raw_waypoint_queue, raw_mutex, raw_condition,
                samples_to_skip, chunk_end_ns, DG_ACTION_HORIZON, DG_SEGMENT_DURATION_SEC);
        });
        interp_thread = std::thread([this] {
            genericInterpLoop<DgRawAction>(
                dg_raw_waypoint_queue, raw_mutex, raw_condition, first_segment_ever,
                samples_to_skip, DG_SAMPLES_PER_SEGMENT, 1.0 / DG_UNINTERP_HZ, action_queue);
        });
    } else {
        // base or cr base part
        recv_thread = std::thread([this] {
            genericReceiveLoop<BasePolicyActionChunkPacket, BaseRawAction>(
                recv_socket, base_raw_waypoint_queue, raw_mutex, raw_condition,
                samples_to_skip, chunk_end_ns, BASE_ACTION_HORIZON, BASE_SEGMENT_DURATION_SEC);
        });
        interp_thread = std::thread([this] {
            genericInterpLoop<BaseRawAction>(
                base_raw_waypoint_queue, raw_mutex, raw_condition, first_segment_ever,
                samples_to_skip, BASE_SAMPLES_PER_SEGMENT, 1.0 / BASE_UNINTERP_HZ, action_queue);
        });
    }
 
    if (type == "cr") {
        cr_recv_socket.open(boost::asio::ip::udp::v4());
        cr_recv_socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), policy_recv_port + 1));
        cr_recv_thread = std::thread([this] {
            genericReceiveLoop<CrPolicyActionChunkPacket, CrRawAction>(
                cr_recv_socket, cr_raw_waypoint_queue, cr_raw_mutex, cr_raw_condition,
                cr_samples_to_skip, cr_chunk_end_ns, CR_ACTION_HORIZON, CR_SEGMENT_DURATION_SEC);
        });
        cr_interp_thread = std::thread([this] {
            genericInterpLoop<CrRawAction>(
                cr_raw_waypoint_queue, cr_raw_mutex, cr_raw_condition, cr_first_segment_ever,
                cr_samples_to_skip, CR_SAMPLES_PER_SEGMENT, 1.0 / CR_UNINTERP_HZ, cr_action_queue);
        });
    }
 
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
    cr_raw_condition.notify_all();
    try {
        if (recv_socket.is_open()) {
            recv_socket.cancel();
            recv_socket.shutdown(boost::asio::ip::udp::socket::shutdown_both);
            recv_socket.close();
        }
    } catch (...) {}
    try {
        if (cr_recv_socket.is_open()) {
            cr_recv_socket.cancel();
            cr_recv_socket.shutdown(boost::asio::ip::udp::socket::shutdown_both);
            cr_recv_socket.close();
        }
    } catch (...) {}


    if (recv_thread.joinable()) recv_thread.join();
    if (send_thread.joinable()) send_thread.join();
    if (interp_thread.joinable()) interp_thread.join();
    if (cr_recv_thread.joinable()) cr_recv_thread.join();
    if (cr_interp_thread.joinable()) cr_interp_thread.join();
}

template <size_t DOF>
typename PolicyUDPHandler<DOF>::jp_type PolicyUDPHandler<DOF>::clipToRange(
        const jp_type& value, const jp_type& center, const jp_type& clip_val, std::string& joints_str_out) {
    jp_type clipped = value;
    joints_str_out.clear();
    for (size_t i = 0; i < DOF; ++i) {
        double delta = value[i] - center[i];
        bool joint_clipped = false;
        if (delta > clip_val[i]) {
            clipped[i] = center[i] + clip_val[i];
            joint_clipped = true;
        } else if (delta < -clip_val[i]) {
            clipped[i] = center[i] - clip_val[i];
            joint_clipped = true;
        }
        // debug
        if (joint_clipped) {
            if (!joints_str_out.empty()) joints_str_out += ", ";
            joints_str_out += std::to_string(i);
        }
    }
    return clipped;
}

template <size_t DOF>
boost::optional<PolicyReceivedData> PolicyUDPHandler<DOF>::getLatestPolicyReceived(const jp_type& wamJP) {
    std::lock_guard<std::mutex> lock(state_mutex);
    bool queues_have_data = !action_queue.empty() || !cr_action_queue.empty();
    {
        std::lock_guard<std::mutex> raw_lock(raw_mutex);
        queues_have_data = queues_have_data || !base_raw_waypoint_queue.empty() || !dg_raw_waypoint_queue.empty();
    }
    {
        std::lock_guard<std::mutex> cr_raw_lock(cr_raw_mutex);
        queues_have_data = queues_have_data || !cr_raw_waypoint_queue.empty();
    }
    if (queues_have_data && std::chrono::steady_clock::now() < pause_until) {
        clearQueue();
    }

    jp_type clip_val;
    clip_val << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    jt_type clip_ff_torque;
    clip_ff_torque << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

    if (action_queue.empty()) {
        return boost::none;
    }

    PolicyReceivedData out;

    PolicyReceivedData base_sample = action_queue.front();
    if (action_queue.size() > 1) action_queue.pop_front();

    jp_type clipped_policy_jp = clipToRange(base_sample.jp, wamJP, clip_val, out.clipped_jp_joints_str);
    out.base_policy_jp = clipped_policy_jp;
 
    if (type == "cr") {
        if (cr_action_queue.empty()) {
            return boost::none;
        }
        PolicyReceivedData residual_sample = cr_action_queue.front();
        if (cr_action_queue.size() > 1) cr_action_queue.pop_front();


        jp_type clipped_delta = clipToRange(residual_sample.jp, jp_type::Zero(), clip_val, out.clipped_jp_joints_str);
        jt_type clipped_ff_torque = clipToRange(residual_sample.ff_torque, jt_type::Zero(), clip_ff_torque, out.clipped_ff_torques_str);
 
        out.jv = residual_sample.jv;
        out.ja = residual_sample.ja;
        out.gripper_cmd = residual_sample.gripper_cmd;
 
        out.jp = clipped_policy_jp + clipped_delta;
 
        out.ff_torque = clipped_ff_torque;
        return out;
    }

    out.jp = clipped_policy_jp;
 
    // technically this can be done without an if but adds clarity
    if (type == "dg") {
        out.ff_torque = clipToRange(base_sample.ff_torque, jt_type::Zero(), clip_ff_torque, out.clipped_ff_torques_str);
    }
 
    return out;

}

template <size_t DOF>
void PolicyUDPHandler<DOF>::send(const jp_type& follower_jp, const jv_type& follower_jv,
                                          const jt_type& follower_dyngravcomp_torque, const jt_type& environment_torque, const jt_type& filtered_environment_torque,
                                          const jp_type& leader_jp, const jv_type& leader_jv,
                                          const jt_type& leader_dyngravcomp_torque, const jt_type& human_torque, const jt_type& filtered_human_torque,
                                          const jp_type& policyJp, const jt_type& policyJt, const jt_type& policyTorqueScale,
                                          const Eigen::Vector3d& follower_cart_pos, const Eigen::Quaterniond& follower_quat,
                                          const Eigen::Vector3d& leader_cart_pos, const Eigen::Quaterniond& leader_quat,
                                          double gripper_pos, double gripper_vel, double gripper_torque) {
    if (!send_active) return;

    {
        std::lock_guard<std::mutex> lock(send_mutex);

        int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        int64_t remaining_ns = chunk_end_ns.load() - now_ns;
        int64_t res_remaining_ns = cr_chunk_end_ns.load() - now_ns;

        std::memcpy(pending_packet.follower_jp, follower_jp.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.follower_jv, follower_jv.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.follower_dyngravcomp_torque, follower_dyngravcomp_torque.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.environment_torque, environment_torque.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.filtered_environment_torque, filtered_environment_torque.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.leader_jp, leader_jp.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.leader_jv, leader_jv.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.leader_dyngravcomp_torque, leader_dyngravcomp_torque.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.human_torque, human_torque.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.filtered_human_torque, filtered_human_torque.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.policyJp, policyJp.data(), sizeof(double) * DOF);
        std::memcpy(pending_packet.policyJt, policyJt.data(), sizeof(double) * DOF);
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
        pending_packet.time_to_chunk_end = remaining_ns > 0 ? static_cast<uint64_t>(remaining_ns) : 0;
        pending_packet.res_time_to_chunk_end = res_remaining_ns > 0 ? static_cast<uint64_t>(res_remaining_ns) : 0;
        new_data_available = true;
    }
    send_condition.notify_one();
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::clearQueueAndPause(std::chrono::milliseconds duration) {
    clearQueue();
    std::lock_guard<std::mutex> lock(state_mutex);
    pause_until = std::chrono::steady_clock::now() + duration;
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::clearQueue() {
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        action_queue.clear();
        cr_action_queue.clear();
    }
    {
        std::lock_guard<std::mutex> lock(raw_mutex);
        base_raw_waypoint_queue.clear();
        dg_raw_waypoint_queue.clear();
        first_segment_ever = true;
    }
    {
        std::lock_guard<std::mutex> lock(cr_raw_mutex);
        cr_raw_waypoint_queue.clear();
        cr_first_segment_ever = true;
    }
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        samples_to_skip.store(0);
        cr_samples_to_skip.store(0);
        int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        chunk_end_ns = now_ns;
        cr_chunk_end_ns = now_ns;
    }
}

template <size_t DOF>
std::deque<PolicyReceivedData> PolicyUDPHandler<DOF>::interpolateSegment(
        const GenericAction& a0, const GenericAction& a1, const GenericAction& a2, const GenericAction& a3,
        size_t samples_per_segment, double dt_s) {
    std::deque<PolicyReceivedData> queue;
 
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
 
    for (size_t j = 0; j < samples_per_segment; ++j) {
        double alpha = static_cast<double>(j) / static_cast<double>(samples_per_segment);
 
        PolicyReceivedData rd;
        for (size_t k = 0; k < 7; ++k) {
            CRResult r = catmullRom(a0.pos[k], a1.pos[k], a2.pos[k], a3.pos[k], alpha);
            rd.jp[k] = r.pos;
            rd.jv[k] = r.dpos_dt / dt_s;
            rd.ja[k] = r.d2pos_dt2 / (dt_s * dt_s);
 
            CRResult rt = catmullRom(a0.torque[k], a1.torque[k], a2.torque[k], a3.torque[k], alpha);
            rd.ff_torque[k] = rt.pos;
        }
        CRResult rg = catmullRom(a0.gripper_cmd, a1.gripper_cmd, a2.gripper_cmd, a3.gripper_cmd, alpha);
        rd.gripper_cmd = rg.pos;
 
        queue.push_back(rd);
    }
    return queue;
}

template <size_t DOF>
template <typename PacketT, typename RawT>
void PolicyUDPHandler<DOF>::genericReceiveLoop(boost::asio::ip::udp::socket& sock,
                                                std::deque<RawT>& raw_queue,
                                                std::mutex& raw_mtx,
                                                std::condition_variable& raw_cv,
                                                std::atomic<uint64_t>& skip_counter,
                                                std::atomic<int64_t>& chunk_end_ns_out,
                                                int action_horizon,
                                                double segment_duration_sec) {
    boost::asio::ip::udp::endpoint sender_endpoint;
    PacketT pkt;
 
    while (!stop_threads) {
        boost::system::error_code ec;
        size_t len = sock.receive_from(boost::asio::buffer(&pkt, sizeof(PacketT)), sender_endpoint, 0, ec);
 
        if (ec == boost::asio::error::operation_aborted || len != sizeof(PacketT)) {
            continue;
        }
 
        std::lock_guard<std::mutex> lock(state_mutex);
        if (std::chrono::steady_clock::now() < pause_until) {
            continue;
        }
 
        skip_counter = static_cast<uint64_t>(pkt.time_to_skip / (1e9 / INTERP_HZ));
 
        int64_t nominal_duration_ns = static_cast<int64_t>(action_horizon * segment_duration_sec * 1e9);
        int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        chunk_end_ns_out = now_ns + nominal_duration_ns - static_cast<int64_t>(pkt.time_to_skip);
        // TODO: sub one segment duration and add the duration of the segments in the raw_waypoints_queue. 
        // TODO: samples should be skipped of the incoming chunk only. right now we are skipping starting from the last action of the last chunk
 
        {
            std::lock_guard<std::mutex> raw_lock(raw_mtx);
            raw_queue.insert(raw_queue.end(), std::begin(pkt.actions), std::end(pkt.actions));
        }
        raw_cv.notify_one();
    }
    sock.close();
}

template <size_t DOF>
template <typename RawT>
void PolicyUDPHandler<DOF>::genericInterpLoop(std::deque<RawT>& raw_queue,
                                               std::mutex& raw_mtx,
                                               std::condition_variable& raw_cv,
                                               bool& first_segment_flag,
                                               std::atomic<uint64_t>& skip_counter,
                                               size_t samples_per_segment,
                                               double dt_s,
                                               std::deque<PolicyReceivedData>& out_queue) {
    while (!stop_threads) {
        RawT a0, a1, a2, a3;
        {
            std::unique_lock<std::mutex> lock(raw_mtx);
            raw_cv.wait(lock, [this, &raw_queue] {
                return stop_threads || raw_queue.size() >= 4;
            });
            if (stop_threads) break;
 
            if (first_segment_flag) {
                a0 = raw_queue[0];
                a1 = raw_queue[0];
                a2 = raw_queue[1];
                a3 = raw_queue[2];
                first_segment_flag = false;
            } else {
                a0 = raw_queue[0];
                a1 = raw_queue[1];
                a2 = raw_queue[2];
                a3 = raw_queue[3];
                raw_queue.pop_front(); // slide the window by one support point
            }
        }
 
        uint64_t skip_now = skip_counter.load();
        if (skip_now >= samples_per_segment) {
            // The whole segment would be discarded, so don't bother interpolating it at all.
            skip_counter.fetch_sub(static_cast<uint64_t>(samples_per_segment));
            continue;
        }
 
        std::deque<PolicyReceivedData> new_samples =
            interpolateSegment(toGeneric(a0), toGeneric(a1), toGeneric(a2), toGeneric(a3), samples_per_segment, dt_s);
 
        if (skip_now > 0) {
            // Interpolation can start partway through a segment
            new_samples.erase(new_samples.begin(), new_samples.begin() + static_cast<std::ptrdiff_t>(skip_now));
            skip_counter.fetch_sub(skip_now);
        }
 
        std::lock_guard<std::mutex> lock(state_mutex);
        out_queue.insert(out_queue.end(),
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
