#include "udp_policy.h"
#include <boost/system/error_code.hpp>
#include <cstring>

template <size_t DOF>
PolicyUDPHandler<DOF>::PolicyUDPHandler(const Config& config, bool send_active, int policy_recv_port,
                                        std::vector<std::string> send_fields)
    : type(config.policy.type)
    , send_active(send_active)
    , stop_threads(false)
    , interp_hz(config.interp_hz)
    , send_socket(io_context)
    , policy_endpoint(boost::asio::ip::make_address(config.policy_network.policy_host), config.policy_network.policy_send)
    , send_fields(std::move(send_fields)) {

    if (!validateFields(this->send_fields, TeleopData<DOF>())) {
        throw std::runtime_error("invalid policy data_routing");
    }

    const ActionConfig& action = config.policy.action;
    const auto make_stream = [&](const ActionStreamConfig& cfg, int port) {
        if (!validateFields(cfg.fields, ActionData())) {
            throw std::runtime_error("invalid fields for a policy action stream");
        }
        return std::unique_ptr<Stream>(new Stream(io_context, cfg, port, config.interp_hz, config.slow_down_factor));
    };

    if (type == "dg") {
        dg_stream = make_stream(action.dg, policy_recv_port);
        primary = dg_stream.get();
    } else {
        base_stream = make_stream(action.base, policy_recv_port);
        primary = base_stream.get();
    }
    if (type == "cr") {
        res_stream = make_stream(action.res, policy_recv_port + 1);
    }

    startStream(*primary);
    if (res_stream) startStream(*res_stream);

    if (send_active) {
        send_socket.open(boost::asio::ip::udp::v4());
        send_thread = std::thread(&PolicyUDPHandler::sendLoop, this);
    }

    const double base_segment_sec = static_cast<double>(config.slow_down_factor) / action.base.uninterp_hz;
    default_pause = std::chrono::milliseconds(static_cast<int64_t>(action.base.horizon * base_segment_sec * 1e3));
}

template <size_t DOF>
PolicyUDPHandler<DOF>::~PolicyUDPHandler() {
    stop();
}

template <size_t DOF>
std::vector<typename PolicyUDPHandler<DOF>::Stream*> PolicyUDPHandler<DOF>::activeStreams() {
    std::vector<Stream*> streams;
    if (base_stream) streams.push_back(base_stream.get());
    if (res_stream) streams.push_back(res_stream.get());
    if (dg_stream) streams.push_back(dg_stream.get());
    return streams;
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::startStream(Stream& s) {
    s.recv_thread = std::thread([this, &s] { receiveLoop(s); });
    s.interp_thread = std::thread([this, &s] { interpLoop(s); });
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::stop() {
    stop_threads = true;
    io_context.stop();
    send_condition.notify_all();

    for (Stream* s : activeStreams()) {
        s->condition.notify_all();
        try {
            if (s->socket.is_open()) {
                s->socket.cancel();
                s->socket.shutdown(boost::asio::ip::udp::socket::shutdown_both);
                s->socket.close();
            }
        } catch (...) {
        }
    }
    try {
        if (send_socket.is_open()) {
            send_socket.cancel();
            send_socket.shutdown(boost::asio::ip::udp::socket::shutdown_both);
            send_socket.close();
        }
    } catch (...) {
    }

    for (Stream* s : activeStreams()) {
        if (s->recv_thread.joinable()) s->recv_thread.join();
        if (s->interp_thread.joinable()) s->interp_thread.join();
    }
    if (send_thread.joinable()) send_thread.join();
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::send(TeleopData<DOF>& state) {
    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const int64_t remaining_ns = primary ? primary->chunk_end_ns.load() - now_ns : 0;
    const int64_t res_remaining_ns = res_stream ? res_stream->chunk_end_ns.load() - now_ns : 0;
    const uint64_t time_to_chunk_end = remaining_ns > 0 ? static_cast<uint64_t>(remaining_ns) : 0;
    const uint64_t res_time_to_chunk_end =
        res_remaining_ns > 0 ? static_cast<uint64_t>(res_remaining_ns) : 0;
    setLocalStateValue(time_to_chunk_end, state.time_to_chunk_end);
    setLocalStateValue(res_time_to_chunk_end, state.res_time_to_chunk_end);

    {
        std::lock_guard<std::mutex> lock(latest_state_mutex);
        latest_state = state;
    }

    if (!send_active) return;

    {
        std::lock_guard<std::mutex> lock(send_mutex);
        if (!encode(send_fields, state, pending_send)) return;
        new_data_available = true;
    }
    send_condition.notify_one();
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::receiveLoop(Stream& s) {
    boost::asio::ip::udp::endpoint sender_endpoint;
    std::vector<uint8_t> buffer(s.packet_size);

    while (!stop_threads) {
        boost::system::error_code ec;
        const size_t len = s.socket.receive_from(boost::asio::buffer(buffer), sender_endpoint, 0, ec);

        if (ec == boost::asio::error::operation_aborted || len != s.packet_size) {
            std::cout << "policy packet size mismatch. expected " << s.packet_size << " got " << len << std::endl;
            continue;
        }

        uint64_t time_to_skip = 0;
        std::memcpy(&time_to_skip, buffer.data(), sizeof(time_to_skip));

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            if (std::chrono::steady_clock::now() < pause_until) continue;
        }

        const uint64_t total_new_samples = static_cast<uint64_t>(s.config.horizon) * s.samples_per_segment;
        uint64_t skip_samples = static_cast<uint64_t>(time_to_skip / (1e9 / interp_hz));
        skip_samples = std::min(skip_samples, total_new_samples);

        const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        {
            std::lock_guard<std::mutex> lock(s.mutex);
            for (uint64_t i = 0; i < total_new_samples; ++i) {
                s.valid_queue.push_back(i < skip_samples ? 0 : 1);
            }
            for (int h = 0; h < s.config.horizon; ++h) {
                ActionData action;
                const size_t offset = sizeof(uint64_t) + static_cast<size_t>(h) * s.action_bytes;
                if (!decode(s.config.fields, buffer.data() + offset, s.action_bytes, action)) break;
                s.raw_queue.push_back(action);
            }
            const int64_t playable_segments = s.raw_queue.size() >= 2 ? static_cast<int64_t>(s.raw_queue.size()) - 2 : 0;
            const int64_t nominal_duration_ns = playable_segments * s.segment_duration_ns;
            s.chunk_end_ns = now_ns + nominal_duration_ns - static_cast<int64_t>(time_to_skip);
        }
        s.condition.notify_one();
    }
    s.socket.close();
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::interpLoop(Stream& s) {
    while (!stop_threads) {
        std::deque<uint8_t> seg_valid;
        ActionData a0, a1, a2, a3;
        bool leader_uninitialized = false;
        {
            std::unique_lock<std::mutex> lock(s.mutex);
            s.condition.wait(lock, [this, &s] { return stop_threads.load() || s.raw_queue.size() >= 4; });
            if (stop_threads) break;

            if (s.first_segment) {
                TeleopData<DOF> leader_state;
                {
                    std::lock_guard<std::mutex> ls_lock(latest_state_mutex);
                    leader_state = latest_state;
                }
                if (leader_state.leader_jp == jp_teleop_type::Zero()) {
                    leader_uninitialized = true;
                } else {
                    seedAction(a0, leader_state, s.config.fields);
                    a1 = s.raw_queue[0];
                    a2 = s.raw_queue[1];
                    a3 = s.raw_queue[2];
                    s.first_segment = false;
                }
            } else {
                a0 = s.raw_queue[0];
                a1 = s.raw_queue[1];
                a2 = s.raw_queue[2];
                a3 = s.raw_queue[3];
                s.raw_queue.pop_front();
            }

            const size_t n = std::min(s.samples_per_segment, s.valid_queue.size());
            seg_valid.assign(s.valid_queue.begin(), s.valid_queue.begin() + static_cast<std::ptrdiff_t>(n));
            s.valid_queue.erase(s.valid_queue.begin(), s.valid_queue.begin() + static_cast<std::ptrdiff_t>(n));
        }

        if (leader_uninitialized) {
            std::cerr << "no initialization point -- pausing policy queue" << std::endl;
            clearQueueAndPause(std::chrono::seconds(30));
            continue;
        }

        const bool any_valid = std::any_of(seg_valid.begin(), seg_valid.end(), [](uint8_t v) { return v != 0; });
        if (!any_valid) continue;

        std::deque<ActionData> new_samples = interpolateSegment(a0, a1, a2, a3, s.config.fields, s.samples_per_segment);

        if (seg_valid.size() != new_samples.size()) {
            std::cerr << "seg_valid size (" << seg_valid.size() << ") != new_samples size ("
                      << new_samples.size() << ") -- pausing policy queue" << std::endl;
            clearQueueAndPause(std::chrono::seconds(30));
            continue;
        }

        size_t idx = 0;
        new_samples.erase(
            std::remove_if(new_samples.begin(), new_samples.end(),
                           [&](const ActionData&) { return seg_valid[idx++] == 0; }),
            new_samples.end());

        std::lock_guard<std::mutex> lock(s.mutex);
        s.action_queue.insert(s.action_queue.end(),
                              std::make_move_iterator(new_samples.begin()),
                              std::make_move_iterator(new_samples.end()));
    }
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::sendLoop() {
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
        send_socket.send_to(boost::asio::buffer(packet), policy_endpoint, 0, ec);
    }
    send_socket.close();
}

template <size_t DOF>
boost::optional<PolicyReceivedData> PolicyUDPHandler<DOF>::getLatestPolicyReceived() {
    bool paused;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        paused = std::chrono::steady_clock::now() < pause_until;
    }
    if (paused) {
        bool queues_have_data = false;
        for (Stream* s : activeStreams()) {
            std::lock_guard<std::mutex> lock(s->mutex);
            queues_have_data = queues_have_data || !s->action_queue.empty() || !s->raw_queue.empty();
        }
        if (queues_have_data) clearQueue();
    }

    if (!primary) return boost::none;

    PolicyReceivedData out;
    {
        std::lock_guard<std::mutex> lock(primary->mutex);
        if (primary->action_queue.empty()) return boost::none;
        out.base = primary->action_queue.front();
        if (primary->action_queue.size() > 1) primary->action_queue.pop_front();
    }

    if (type == "cr") {
        if (!res_stream) return boost::none;

        std::lock_guard<std::mutex> lock(res_stream->mutex);
        if (res_stream->action_queue.empty()) return boost::none;
        out.res = res_stream->action_queue.front();
        if (res_stream->action_queue.size() > 1) res_stream->action_queue.pop_front();
    }

    return out;
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::clearQueueAndPause(std::chrono::milliseconds duration) {
    clearQueue();
    std::lock_guard<std::mutex> lock(state_mutex);
    pause_until = std::chrono::steady_clock::now() + duration;
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::clearQueueAndPause() {
    clearQueueAndPause(default_pause);
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::clearQueue() {
    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    for (Stream* s : activeStreams()) {
        std::lock_guard<std::mutex> lock(s->mutex);
        s->action_queue.clear();
        s->raw_queue.clear();
        s->valid_queue.clear();
        s->first_segment = true;
        s->chunk_end_ns = now_ns;
    }
}

template <size_t DOF>
void PolicyUDPHandler<DOF>::seedAction(ActionData& a, const TeleopData<DOF>& leader_state,
                                       const std::vector<std::string>& fields) {
    for (const std::string& name : fields) {
        if (name == "jp") {
            for (size_t i = 0; i < TELEOP_DOF; ++i) a.jp[i] = leader_state.leader_jp[i];
        } else if (name == "delta_jp") {
            for (size_t i = 0; i < TELEOP_DOF; ++i) a.jp[i] = 0;
        } else if (name == "ext_torque") {
            for (size_t i = 0; i < TELEOP_DOF; ++i) a.ext_torque[i] = leader_state.filtered_human_torque[i];
        } else if (name == "gripper_cmd") {
            a.gripper_cmd = leader_state.gripper_pos;
        }
    }
}

// interpolate with 2 supporting points
template <size_t DOF>
std::deque<ActionData> PolicyUDPHandler<DOF>::interpolateSegment(const ActionData& a0, const ActionData& a1,
                                                                 const ActionData& a2, const ActionData& a3,
                                                                 const std::vector<std::string>& fields,
                                                                 size_t samples_per_segment) {
    std::deque<ActionData> queue;

    const auto catmullRom = [](double p0, double p1, double p2, double p3, double t) -> double {
        const double t2 = t * t;
        const double c1 = (-p0 + p2);
        const double c2 = (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3);
        const double c3 = (-p0 + 3.0 * p1 - 3.0 * p2 + p3);
        return 0.5 * (2.0 * p1 + c1 * t + c2 * t2 + c3 * t2 * t);
    };

    for (size_t j = 0; j < samples_per_segment; ++j) {
        const double alpha = static_cast<double>(j) / static_cast<double>(samples_per_segment);
        ActionData sample;

        for (const std::string& name : fields) {
            const size_t count = field(a0, name).bytes / sizeof(double);
            const double* p0 = static_cast<const double*>(field(a0, name).ptr);
            const double* p1 = static_cast<const double*>(field(a1, name).ptr);
            const double* p2 = static_cast<const double*>(field(a2, name).ptr);
            const double* p3 = static_cast<const double*>(field(a3, name).ptr);
            double* out = static_cast<double*>(field(sample, name).ptr);

            for (size_t k = 0; k < count; ++k) {
                out[k] = catmullRom(p0[k], p1[k], p2[k], p3[k], alpha);
            }
        }
        queue.push_back(sample);
    }
    return queue;
}

template class PolicyUDPHandler<4>;
template class PolicyUDPHandler<7>;
