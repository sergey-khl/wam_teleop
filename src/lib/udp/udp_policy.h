#pragma once
#include <cstdint>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <boost/asio.hpp>
#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <barrett/units.h>
#include "../utils/data_packets.h"
#include "../utils/config_loader.h"

// All policies reuse this for simplicity.
struct PolicyReceivedData {
    double gripper_cmd = 0.0;
    Eigen::Matrix<double, 7, 1> base_policy_jp = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> res_policy_jp = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> ref_torque = Eigen::Matrix<double, 7, 1>::Zero();
    std::string clipped_base_jp_joints_str;
    std::string clipped_res_jp_joints_str;
    std::string clipped_ref_torques_str;
};

template <size_t DOF>
class PolicyUDPHandler {
public:
    typedef typename barrett::units::JointPositions<DOF>::type jp_type;
    typedef typename barrett::units::JointTorques<DOF>::type jt_type;

    PolicyUDPHandler(const Config& config, bool send_active, int policy_recv_port,
                     std::vector<std::string> send_fields);
    ~PolicyUDPHandler();

    void stop();

    // Latest interpolated action received from the policy
    boost::optional<PolicyReceivedData> getLatestPolicyReceived();

    void clearQueueAndPause(std::chrono::milliseconds duration);
    void clearQueueAndPause();
    void clearQueue();

    // Queue the shared state to be sent to the policy. Stamps the chunk-end times.
    void send(TeleopData<DOF>& state);

private:
    // One action stream. base/dg use policy_recv_port, res uses policy_recv_port+1.
    struct Stream {
        ActionStreamConfig config;
        size_t action_bytes;
        size_t packet_size;
        int64_t segment_duration_ns;
        size_t samples_per_segment;

        boost::asio::ip::udp::socket socket;
        std::thread recv_thread;
        std::thread interp_thread;

        std::deque<ActionData> raw_queue;
        std::deque<uint8_t> valid_queue;
        std::mutex mutex;
        std::condition_variable condition;
        bool first_segment = true;
        std::atomic<int64_t> chunk_end_ns{0};
        std::deque<ActionData> action_queue;

        Stream(boost::asio::io_context& io, const ActionStreamConfig& cfg, int port,
               double interp_hz, int slow_down_factor)
            : config(cfg), socket(io) {
            action_bytes = encodedSize(config.fields, ActionData());
            packet_size = sizeof(uint64_t) + static_cast<size_t>(config.horizon) * action_bytes;
            const double segment_duration_sec = static_cast<double>(slow_down_factor) / config.uninterp_hz;
            segment_duration_ns = static_cast<int64_t>(segment_duration_sec * 1e9);
            samples_per_segment = static_cast<size_t>(interp_hz * segment_duration_sec);
            socket.open(boost::asio::ip::udp::v4());
            socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port));
        }
    };

    const std::string type;
    bool send_active;
    std::atomic<bool> stop_threads;
    double interp_hz;

    boost::asio::io_context io_context;
    std::unique_ptr<Stream> base_stream; // base or cr primary
    std::unique_ptr<Stream> res_stream;  // cr residual
    std::unique_ptr<Stream> dg_stream;   // dg
    Stream* primary = nullptr;

    boost::asio::ip::udp::socket send_socket;
    boost::asio::ip::udp::endpoint policy_endpoint;
    std::thread send_thread;

    std::mutex send_mutex;
    std::condition_variable send_condition;
    std::vector<uint8_t> pending_send;
    bool new_data_available = false;

    std::vector<std::string> send_fields;

    jp_type clip_val;
    jt_type clip_ref_torque;

    std::mutex state_mutex;
    std::chrono::steady_clock::time_point pause_until{};
    std::chrono::milliseconds default_pause{0};

    TeleopData<DOF> latest_state; // guarded by state_mutex, for first-segment seeding
    std::mutex latest_state_mutex;

    std::vector<Stream*> activeStreams();
    void startStream(Stream& s);
    void receiveLoop(Stream& s);
    void interpLoop(Stream& s);
    void sendLoop();

    void seedAction(ActionData& a, const TeleopData<DOF>& leader_state, const std::vector<std::string>& fields);

    static std::deque<ActionData> interpolateSegment(const ActionData& a0, const ActionData& a1,
                                                     const ActionData& a2, const ActionData& a3,
                                                     const std::vector<std::string>& fields,
                                                     size_t samples_per_segment);

    template <typename Vec>
    static Vec clipToRange(const Vec& value, const Vec& center, const Vec& clip_val,
                           std::string& joints_str_out);
};
