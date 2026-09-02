#pragma once
#include <cstdint>
#include <cstddef>
#include <Eigen/Dense>
#include <Eigen/Geometry>
 
#include <boost/asio.hpp>
#include <boost/optional.hpp>
 
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
 

static constexpr double INTERP_HZ = 500.0;

// 1 is real time exec. the larger the number the slower the execution
static constexpr int SLOW_DOWN_FACTOR = 2;
 
// used in base and cr
static constexpr int BASE_ACTION_HORIZON = 8;
static constexpr double BASE_UNINTERP_HZ = 10.0;
static constexpr double BASE_SEGMENT_DURATION_SEC = SLOW_DOWN_FACTOR / BASE_UNINTERP_HZ;
 
// the residual of cr-dagger
static constexpr int CR_ACTION_HORIZON = 5;
static constexpr double CR_UNINTERP_HZ = 50.0;
static constexpr double CR_SEGMENT_DURATION_SEC = SLOW_DOWN_FACTOR / CR_UNINTERP_HZ;
 
// dg-dagger
static constexpr int DG_ACTION_HORIZON = 64;
static constexpr double DG_UNINTERP_HZ = 50.0;
static constexpr double DG_SEGMENT_DURATION_SEC = SLOW_DOWN_FACTOR / DG_UNINTERP_HZ;


// tightly packed layouts
#pragma pack(push, 1)

template <size_t DOF>
struct LeaderToFollowerPacket {
    double jp[DOF];
    double jv[DOF];
    double dyngravcompTorque[DOF];
    double humanTorque[DOF];
    double filteredHumanTorque[DOF];
    double cart_pos[3];   // x, y, z
    double quat[4];   // w, x, y, z
    double policyTorqueScale[DOF];
    double gripper_cmd;
    double cancel_policy;
    uint64_t timestamp;
};

template <size_t DOF>
struct FollowerToLeaderPacket {
    double jp[DOF];
    double jv[DOF];
    double dyngravcompTorque[DOF];
    double environmentTorque[DOF];
    double filteredEnvironmentTorque[DOF];
    double cart_pos[3];   // x, y, z
    double quat[4];   // w, x, y, z
    double gripper_torque;
    double gripper_pos;
    double gripper_vel;
    uint64_t timestamp;
};

// a bunch of state data sent from the follower
template <size_t DOF>
struct PolicyPacket {
    double follower_jp[DOF];
    double follower_jv[DOF];
    double follower_dyngravcomp_torque[DOF];
    double environment_torque[DOF]; // compensating dynamics, gravity and policy
    double filtered_environment_torque[DOF]; // compensating dynamics, gravity and policy
    double leader_jp[DOF];
    double leader_jv[DOF];
    double leader_dyngravcomp_torque[DOF];
    double human_torque[DOF]; // compensating dynamics, gravity and policy
    double filtered_human_torque[DOF]; // compensating dynamics, gravity and policy
    double policyJp[DOF]; // latest recived policy command. Clipped
    double policyJt[DOF]; // policy torque to be executed
    double policyTorqueScale[DOF]; // (0, 1)
    double follower_cart_pos[3];   // x, y, z
    double follower_quat[4];   // w, x, y, z
    double leader_cart_pos[3];   // x, y, z
    double leader_quat[4];   // w, x, y, z
    double gripper_pos;
    double gripper_vel;
    double gripper_torque;
    uint64_t time_to_chunk_end;
    uint64_t res_time_to_chunk_end; // only cr-dagger cares about this
};

struct BaseRawAction {
    double jp[7];
    double gripper_cmd;
};
 
struct BasePolicyActionChunkPacket {
    uint64_t time_to_skip;
    BaseRawAction actions[BASE_ACTION_HORIZON];
};
 
struct CrRawAction {
    double delta_jp[7];
    double gripper_cmd;
    double ext_torque[7];
};
 
struct CrPolicyActionChunkPacket {
    uint64_t time_to_skip;
    CrRawAction actions[CR_ACTION_HORIZON];
};
 
struct DgRawAction {
    double jp[7];
    double gripper_cmd;
    double ext_torque[7];
};
 
struct DgPolicyActionChunkPacket {
    uint64_t time_to_skip;
    DgRawAction actions[DG_ACTION_HORIZON];
};


#pragma pack(pop)


template <size_t DOF>
struct LeaderReceivedData {
    Eigen::Matrix<double, DOF, 1> jp;
    Eigen::Matrix<double, DOF, 1> jv;
    Eigen::Matrix<double, DOF, 1> dyngravcompTorque;
    Eigen::Matrix<double, DOF, 1> environmentTorque;
    Eigen::Matrix<double, DOF, 1> filteredEnvironmentTorque;
    Eigen::Matrix<double, 3, 1> cart_pos;
    Eigen::Quaterniond quat;
    double gripper_torque;
    double gripper_pos;
    double gripper_vel;
    uint64_t timestamp;
};

template <size_t DOF>
struct FollowerReceivedData {
    Eigen::Matrix<double, DOF, 1> jp;
    Eigen::Matrix<double, DOF, 1> jv;
    Eigen::Matrix<double, DOF, 1> dyngravcompTorque;
    Eigen::Matrix<double, DOF, 1> humanTorque;
    Eigen::Matrix<double, DOF, 1> filteredHumanTorque;
    Eigen::Matrix<double, 3, 1> cart_pos;
    Eigen::Quaterniond quat;
    Eigen::Matrix<double, DOF, 1> policyTorqueScale;
    double gripper_cmd;
    double cancel_policy;
    uint64_t timestamp;
};

struct GenericAction {
    double pos[7];
    double gripper_cmd;
    double torque[7];
};


// all policies reuse this for simplicity. see getLatestPolicyReceived for how
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
    using jp_type = Eigen::Matrix<double, DOF, 1>;
    using jv_type = Eigen::Matrix<double, DOF, 1>;
    using jt_type = Eigen::Matrix<double, DOF, 1>;
    using PolicyPacketType = PolicyPacket<DOF>;
 
    PolicyUDPHandler(const std::string& type, bool send_active, const std::string& policy_host, int policy_send_port, int policy_recv_port);
    ~PolicyUDPHandler();
 
    void stop();
 
    // Latest interpolated action received from the policy (pops the front of the queue).
    boost::optional<PolicyReceivedData> getLatestPolicyReceived();

    void clearQueueAndPause(std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double, std::milli>(BASE_ACTION_HORIZON * BASE_SEGMENT_DURATION_SEC * 1e3)));
    void clearQueue();

 
    // Queue a PolicyPacket to be sent to the policy. No-op if inactive.
    void send(const jp_type& follower_jp, const jv_type& follower_jv,
                       const jt_type& follower_dyngravcomp_torque, const jt_type& environment_torque, const jt_type& filtered_environment_torque,
                       const jp_type& leader_jp, const jv_type& leader_jv,
                       const jt_type& leader_dyngravcomp_torque, const jt_type& human_torque, const jt_type& filtered_human_torque,
                       const jp_type& policyjp, const jt_type& policyJt, const jt_type& policyTorqueScale,
                       const Eigen::Vector3d& follower_cart_pos, const Eigen::Quaterniond& follower_quat,
                       const Eigen::Vector3d& leader_cart_pos, const Eigen::Quaterniond& leader_quat,
                       double gripper_pos, double gripper_vel, double gripper_torque);

private:
    static constexpr size_t BASE_SAMPLES_PER_SEGMENT = static_cast<size_t>(INTERP_HZ * BASE_SEGMENT_DURATION_SEC);
    static constexpr size_t CR_SAMPLES_PER_SEGMENT = static_cast<size_t>(INTERP_HZ * CR_SEGMENT_DURATION_SEC);
    static constexpr size_t DG_SAMPLES_PER_SEGMENT = static_cast<size_t>(INTERP_HZ * DG_SEGMENT_DURATION_SEC);
 
    const std::string type;
    bool send_active;
    std::atomic<bool> stop_threads;
 
    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket send_socket;
    boost::asio::ip::udp::socket recv_socket;
    boost::asio::ip::udp::endpoint policy_endpoint;
    int policy_recv_port;
 
    std::thread send_thread;
    std::thread recv_thread;
 
    std::mutex state_mutex;
    std::mutex send_mutex;
    std::condition_variable send_condition;
 
    PolicyPacketType pending_packet;
    bool new_data_available = false;
 
    std::deque<GenericAction> action_queue;
 
    void sendLoop();
    template <typename PacketT, typename RawT>
    void genericReceiveLoop(boost::asio::ip::udp::socket& sock,
                             std::deque<RawT>& raw_queue,
                             std::deque<uint8_t>& valid_sample_queue,
                             std::mutex& raw_mtx,
                             std::condition_variable& raw_cv,
                             std::atomic<int64_t>& chunk_end_ns_out,
                             int action_horizon,
                             double segment_duration_sec);


    std::thread interp_thread;
    template <typename RawT>
    void genericInterpLoop(std::deque<RawT>& raw_queue,
                            std::deque<uint8_t>& valid_sample_queue,
                            std::mutex& raw_mtx,
                            std::condition_variable& raw_cv,
                            bool& first_segment_flag,
                            size_t samples_per_segment,
                            double dt_s,
                            std::deque<GenericAction>& out_queue);

    static GenericAction toGeneric(const BaseRawAction& a);
    static GenericAction toGeneric(const DgRawAction& a);
    static GenericAction toGeneric(const CrRawAction& a);

    // always use unique points for a0-3
    static std::deque<GenericAction> interpolateSegment(const GenericAction& a0, const GenericAction& a1, const GenericAction& a2, const GenericAction& a3, size_t samples_per_segment, double dt_s);
 
    static jp_type clipToRange(const jp_type& value, const jp_type& center, const jp_type& clip_val, std::string& joints_str_out);
 
    std::deque<BaseRawAction> base_raw_waypoint_queue; // for base or the base part of cr
    std::deque<DgRawAction> dg_raw_waypoint_queue;
    std::deque<uint8_t> valid_sample_queue;
    std::mutex raw_mutex;
    std::condition_variable raw_condition;
    bool first_segment_ever = true;

    std::atomic<int64_t> chunk_end_ns{0};

    boost::asio::ip::udp::socket cr_recv_socket;
    std::thread cr_recv_thread;
    std::thread cr_interp_thread;
 
    std::deque<CrRawAction> cr_raw_waypoint_queue;
    std::deque<uint8_t> cr_valid_sample_queue;
    std::mutex cr_raw_mutex;
    std::condition_variable cr_raw_condition;
    bool cr_first_segment_ever = true;
 
    std::atomic<int64_t> cr_chunk_end_ns{0};
 
    std::deque<GenericAction> cr_action_queue;

    // used in first time interpolation and clipping to reasonable values
    struct LeaderState {
        jp_type jp = jp_type::Zero();
        jt_type filtered_human_torque = jt_type::Zero();
        double gripper_pos = 0.0;
    };
    std::mutex leader_state_mutex;
    LeaderState latest_leader_state;
 
    std::chrono::steady_clock::time_point pause_until{};
};

