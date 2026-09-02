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
    // g.gripper_cmd = 0;
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
                recv_socket, dg_raw_waypoint_queue, valid_sample_queue, raw_mutex, raw_condition,
                chunk_end_ns, DG_ACTION_HORIZON, DG_SEGMENT_DURATION_SEC);
        });
        interp_thread = std::thread([this] {
            genericInterpLoop<DgRawAction>(
                dg_raw_waypoint_queue, valid_sample_queue, raw_mutex, raw_condition, first_segment_ever,
                DG_SAMPLES_PER_SEGMENT, 1.0 / DG_UNINTERP_HZ, action_queue);
        });
    } else {
        // base or cr base part
        recv_thread = std::thread([this] {
            genericReceiveLoop<BasePolicyActionChunkPacket, BaseRawAction>(
                recv_socket, base_raw_waypoint_queue, valid_sample_queue, raw_mutex, raw_condition,
                chunk_end_ns, BASE_ACTION_HORIZON, BASE_SEGMENT_DURATION_SEC);
        });
        interp_thread = std::thread([this] {
            genericInterpLoop<BaseRawAction>(
                base_raw_waypoint_queue, valid_sample_queue, raw_mutex, raw_condition, first_segment_ever,
                BASE_SAMPLES_PER_SEGMENT, 1.0 / BASE_UNINTERP_HZ, action_queue);
        });
    }
 
    if (type == "cr") {
        cr_recv_socket.open(boost::asio::ip::udp::v4());
        cr_recv_socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), policy_recv_port + 1));
        cr_recv_thread = std::thread([this] {
            genericReceiveLoop<CrPolicyActionChunkPacket, CrRawAction>(
                cr_recv_socket, cr_raw_waypoint_queue, cr_valid_sample_queue, cr_raw_mutex, cr_raw_condition,
                cr_chunk_end_ns, CR_ACTION_HORIZON, CR_SEGMENT_DURATION_SEC);
        });
        cr_interp_thread = std::thread([this] {
            genericInterpLoop<CrRawAction>(
                cr_raw_waypoint_queue, cr_valid_sample_queue, cr_raw_mutex, cr_raw_condition, cr_first_segment_ever,
                CR_SAMPLES_PER_SEGMENT, 1.0 / CR_UNINTERP_HZ, cr_action_queue);
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
boost::optional<PolicyReceivedData> PolicyUDPHandler<DOF>::getLatestPolicyReceived() {
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
    clip_val << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05;
    // clip_val << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    jt_type clip_ref_torque;
    clip_ref_torque << 2.5, 2.5, 2.5, 2.5, 0.0, 0.0, 0.0;

    if (action_queue.empty()) {
        return boost::none;
    }

    PolicyReceivedData out;

    GenericAction base_sample = action_queue.front();
    if (action_queue.size() > 1) action_queue.pop_front();

    jp_type base_jp;
    for (size_t i = 0; i < DOF; ++i) base_jp[i] = base_sample.pos[i];

    jp_type clipped_policy_jp = clipToRange(base_jp, latest_leader_state.jp, clip_val, out.clipped_base_jp_joints_str);
    out.base_policy_jp = clipped_policy_jp;
    // cr dagger paper combines base and res for gripper action but they also dont use a gripper
    out.gripper_cmd = base_sample.gripper_cmd;
 
    if (type == "cr") {
        if (cr_action_queue.empty()) {
            return boost::none;
        }
        GenericAction residual_sample = cr_action_queue.front();
        if (cr_action_queue.size() > 1) cr_action_queue.pop_front();

        jp_type res_jp;
        for (size_t i = 0; i < DOF; ++i) res_jp[i] = residual_sample.pos[i];
        jp_type clipped_delta = clipToRange(res_jp, jp_type::Zero(), clip_val, out.clipped_res_jp_joints_str);
        
        jt_type ref_torque;
        for (size_t i = 0; i < DOF; ++i) ref_torque[i] = residual_sample.torque[i];
        jt_type clipped_ref_torque = clipToRange(ref_torque, latest_leader_state.filtered_human_torque, clip_ref_torque, out.clipped_ref_torques_str);
 
        out.res_policy_jp = clipped_delta;
        out.ref_torque = clipped_ref_torque;
        return out;
    }
 
    // technically this can be done without an if but adds clarity
    if (type == "dg") {
        jt_type ref_torque;
        for (size_t i = 0; i < DOF; ++i) ref_torque[i] = base_sample.torque[i];
        out.ref_torque = clipToRange(ref_torque, latest_leader_state.filtered_human_torque, clip_ref_torque, out.clipped_ref_torques_str);
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
    // always captured no matter if on_leader or on_follower
    {
        std::lock_guard<std::mutex> lock(leader_state_mutex);
        latest_leader_state.jp = leader_jp;
        latest_leader_state.filtered_human_torque = filtered_human_torque;
        latest_leader_state.gripper_pos = gripper_pos;
    }

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
        valid_sample_queue.clear();
        first_segment_ever = true;
    }
    {
        std::lock_guard<std::mutex> lock(cr_raw_mutex);
        cr_raw_waypoint_queue.clear();
        cr_valid_sample_queue.clear();
        cr_first_segment_ever = true;
    }
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        chunk_end_ns = now_ns;
        cr_chunk_end_ns = now_ns;
    }
}

template <size_t DOF>
std::deque<GenericAction> PolicyUDPHandler<DOF>::interpolateSegment(
        const GenericAction& a0, const GenericAction& a1, const GenericAction& a2, const GenericAction& a3,
        size_t samples_per_segment, double dt_s) {
    std::deque<GenericAction> queue;
 
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
 
        GenericAction ga;
        for (size_t k = 0; k < 7; ++k) {
            CRResult r = catmullRom(a0.pos[k], a1.pos[k], a2.pos[k], a3.pos[k], alpha);
            ga.pos[k] = r.pos;
            // rd.jv[k] = r.dpos_dt / dt_s;
            // rd.ja[k] = r.d2pos_dt2 / (dt_s * dt_s);
 
            CRResult rt = catmullRom(a0.torque[k], a1.torque[k], a2.torque[k], a3.torque[k], alpha);
            ga.torque[k] = rt.pos;
        }
        CRResult rg = catmullRom(a0.gripper_cmd, a1.gripper_cmd, a2.gripper_cmd, a3.gripper_cmd, alpha);
        ga.gripper_cmd = rg.pos;
 
        queue.push_back(ga);
    }
    return queue;
}

template <size_t DOF>
template <typename PacketT, typename RawT>
void PolicyUDPHandler<DOF>::genericReceiveLoop(boost::asio::ip::udp::socket& sock,
                                                std::deque<RawT>& raw_queue,
                                                std::deque<uint8_t>& valid_sample_queue,
                                                std::mutex& raw_mtx,
                                                std::condition_variable& raw_cv,
                                                std::atomic<int64_t>& chunk_end_ns_out,
                                                int action_horizon,
                                                double segment_duration_sec) {
    boost::asio::ip::udp::endpoint sender_endpoint;
    PacketT pkt;
    const size_t samples_per_segment = static_cast<size_t>(INTERP_HZ * segment_duration_sec);
    const int64_t segment_duration_ns = static_cast<int64_t>(segment_duration_sec * 1e9);
 
    while (!stop_threads) {
        boost::system::error_code ec;
        size_t len = sock.receive_from(boost::asio::buffer(&pkt, sizeof(PacketT)), sender_endpoint, 0, ec);
 
        if (ec == boost::asio::error::operation_aborted || len != sizeof(PacketT)) {
            std::cout << "size mismatch. expected " << sizeof(PacketT) << " got " << len << std::endl;
            continue;
        }
 
        std::lock_guard<std::mutex> lock(state_mutex);
        if (std::chrono::steady_clock::now() < pause_until) {
            continue;
        }
 
        uint64_t total_new_samples = static_cast<uint64_t>(action_horizon) * samples_per_segment;
        uint64_t skip_samples = static_cast<uint64_t>(pkt.time_to_skip / (1e9 / INTERP_HZ));
        skip_samples = std::min(skip_samples, total_new_samples);
 
        int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        {
            std::lock_guard<std::mutex> raw_lock(raw_mtx);

            for (uint64_t i = 0; i < total_new_samples; ++i) {
                valid_sample_queue.push_back(i < skip_samples ? 0 : 1);
            }
            raw_queue.insert(raw_queue.end(), std::begin(pkt.actions), std::end(pkt.actions));

            // we can only interpolate up to the N-1'th action for a chunk until we need a new one
            int64_t playable_segments = static_cast<int64_t>(raw_queue.size()) >= 2 ? static_cast<int64_t>(raw_queue.size()) - 2 : 0;
            int64_t nominal_duration_ns = playable_segments * segment_duration_ns;
            chunk_end_ns_out = now_ns + nominal_duration_ns - static_cast<int64_t>(pkt.time_to_skip);
        }

        raw_cv.notify_one();
    }
    sock.close();
}

template <size_t DOF>
template <typename RawT>
void PolicyUDPHandler<DOF>::genericInterpLoop(std::deque<RawT>& raw_queue,
                                               std::deque<uint8_t>& valid_sample_queue,
                                               std::mutex& raw_mtx,
                                               std::condition_variable& raw_cv,
                                               bool& first_segment_flag,
                                               size_t samples_per_segment,
                                               double dt_s,
                                               std::deque<GenericAction>& out_queue) {
    while (!stop_threads) {
        // size of the segment. 1 
        std::deque<uint8_t> seg_valid;
        GenericAction a0, a1, a2, a3;
        bool leader_jp_uninitialized = false;
        {
            std::unique_lock<std::mutex> lock(raw_mtx);
            raw_cv.wait(lock, [this, &raw_queue] {
                return stop_threads || raw_queue.size() >= 4;
            });
            if (stop_threads) break;
 
            if (first_segment_flag) {
                LeaderState leader_state;
                {
                    std::lock_guard<std::mutex> jp_lock(leader_state_mutex);
                    leader_state = latest_leader_state;
                }
                // should always have a jp
                if (leader_state.jp == jp_type::Zero()) {
                    leader_jp_uninitialized = true;
                } else {
                    std::memcpy(a0.pos, leader_state.jp.data(), sizeof(a0.pos));
                    std::memcpy(a0.torque, leader_state.filtered_human_torque.data(), sizeof(a0.torque));
                    a0.gripper_cmd = leader_state.gripper_pos;

                    a1 = toGeneric(raw_queue[0]);
                    a2 = toGeneric(raw_queue[1]);
                    a3 = toGeneric(raw_queue[2]);
                    first_segment_flag = false;
                }
            } else {
                a0 = toGeneric(raw_queue[0]);
                a1 = toGeneric(raw_queue[1]);
                a2 = toGeneric(raw_queue[2]);
                a3 = toGeneric(raw_queue[3]);
                raw_queue.pop_front(); // slide the window by one support point
            }

            size_t n = std::min(samples_per_segment, valid_sample_queue.size());
            seg_valid.assign(valid_sample_queue.begin(), valid_sample_queue.begin() + static_cast<std::ptrdiff_t>(n));
            valid_sample_queue.erase(valid_sample_queue.begin(), valid_sample_queue.begin() + static_cast<std::ptrdiff_t>(n));
        }

        if (leader_jp_uninitialized) {
            std::cerr << "no intialization point -- pausing policy queue" << std::endl;
            clearQueueAndPause(std::chrono::seconds(30));
            continue;
        }
 
        // skip the whole segment
        bool any_valid = std::any_of(seg_valid.begin(), seg_valid.end(), [](uint8_t v) { return v != 0; });
        if (!any_valid) {
            continue;
        }
 
        std::deque<GenericAction> new_samples = interpolateSegment(a0, a1, a2, a3, samples_per_segment, dt_s);

        // seg_valid and new_samples should always be same size
        if (seg_valid.size() != new_samples.size()) {
            std::cerr << "seg_valid size (" << seg_valid.size()
                      << ") != new_samples size (" << new_samples.size()
                      << ") -- pausing policy queue" << std::endl;
            clearQueueAndPause(std::chrono::seconds(30));
            continue;
        }
 
        size_t idx = 0;
        new_samples.erase(
            std::remove_if(new_samples.begin(), new_samples.end(),
                            [&](const GenericAction&) { return seg_valid[idx++] == 0; }),
            new_samples.end());

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
