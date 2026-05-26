#pragma once
#include <boost/asio.hpp>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <Eigen/Dense>
#include <boost/optional.hpp>
#include <chrono>

template <size_t DOF>
class UDPHandler {
public:
    using jp_type = Eigen::Matrix<double, DOF, 1>;
    using jv_type = Eigen::Matrix<double, DOF, 1>;
    using jt_type = Eigen::Matrix<double, DOF, 1>;

    struct ReceivedData {
        jp_type jp;
        jv_type jv;
        jt_type extTorque;
        jt_type measTorque;
        double gripper;
        uint64_t timestamp;
    };

    UDPHandler(const std::string& teleop_host, int teleop_send, int teleop_recv,
               bool recording,
               const std::string& inference_host = "127.0.0.1", int inference_send = 6000, int inference_recv = 6001);
    ~UDPHandler();

    void stop();
    boost::optional<ReceivedData> getLatestTeleopReceived();
    boost::optional<ReceivedData> getLatestInferenceReceived();
    void send(const jp_type& jp, const jv_type& jv, const jt_type& extTorque, const jt_type& measTorque, const double& gripper);

    void enableInference();
    void disableInference();

private:
    std::atomic<bool> stop_threads;
    std::vector<boost::asio::ip::udp::endpoint> send_endpoints;

    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket send_socket;
    boost::asio::ip::udp::socket teleop_recv_socket;
    boost::asio::ip::udp::socket inference_recv_socket;

    bool recording;
    bool inference_active;
    std::string inference_host;
    int inference_send_port;
    int inference_recv_port;
    std::mutex inference_socket_mutex;

    std::thread teleop_recv_thread;
    std::thread inference_recv_thread;
    std::thread send_thread;

    std::mutex state_mutex, send_mutex;
    std::condition_variable send_condition;

    jp_type pending_send_jp;
    jv_type pending_send_jv;
    jt_type pending_send_extTorque;
    jt_type pending_send_measTorque;
    double pending_send_gripper;

    boost::optional<ReceivedData> latest_teleop_received;
    boost::optional<ReceivedData> latest_inference_received;
    bool new_data_available = false;

    void receiveLoop(boost::asio::ip::udp::socket& recv_socket, boost::optional<ReceivedData>& latest_received);
    void sendLoop();
};
