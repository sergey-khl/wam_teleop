#include "udp_handler.h"
#include <boost/system/error_code.hpp>
#include <cstring>

// TODO: inference recv can just pass to current teleop_recv for now. change later
template <size_t DOF>
UDPHandler<DOF>::UDPHandler(const std::string& teleop_host, int teleop_send, int teleop_recv, bool recording, const std::string& inference_host, int inference_send, int inference_recv)
    : stop_threads(false)
    , recording(recording)
    , inference_active(false)
    , send_socket(io_context, boost::asio::ip::udp::v4())
    , teleop_recv_socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), teleop_recv))
    , inference_recv_socket(io_context)
    , inference_host(inference_host)
    , inference_send_port(inference_send)
    , inference_recv_port(inference_recv) {

    // Always add teleop to our broadcast list
    send_endpoints.push_back(boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(teleop_host), teleop_send));

    teleop_recv_thread = std::thread(&UDPHandler::receiveLoop, this, std::ref(teleop_recv_socket), std::ref(latest_teleop_received));

    if (recording) {
        send_endpoints.push_back(boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(inference_host), inference_send));
    }

    send_thread = std::thread(&UDPHandler::sendLoop, this);
}

template <size_t DOF>
UDPHandler<DOF>::~UDPHandler() {
    stop();
}

template <size_t DOF>
void UDPHandler<DOF>::stop() {
    stop_threads = true;
    io_context.stop();
    send_condition.notify_all();
    try {
        if (teleop_recv_socket.is_open()) {
            teleop_recv_socket.cancel();
            teleop_recv_socket.shutdown(boost::asio::ip::udp::socket::shutdown_both);
            teleop_recv_socket.close();
        }
        if (inference_recv_socket.is_open()) {
            inference_recv_socket.cancel();
            inference_recv_socket.shutdown(boost::asio::ip::udp::socket::shutdown_both);
            inference_recv_socket.close();
        }
    } catch (...) {
        // Ignore any exceptions during socket cleanup
    }

    if (teleop_recv_thread.joinable()) teleop_recv_thread.join();
    if (inference_recv_thread.joinable()) inference_recv_thread.join();
    if (send_thread.joinable()) send_thread.join();
}

template <size_t DOF>
void UDPHandler<DOF>::enableInference() {
    std::lock_guard<std::mutex> lock(inference_socket_mutex);
    if (inference_active) return;

    inference_recv_socket = boost::asio::ip::udp::socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), inference_recv_port));
    inference_recv_thread = std::thread(&UDPHandler::receiveLoop, this, std::ref(inference_recv_socket), std::ref(latest_inference_received));

    if (!recording) {
        send_endpoints.push_back(boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(inference_host), inference_send_port));
    }

    inference_active = true;
}

template <size_t DOF>
void UDPHandler<DOF>::disableInference() {
    {
        std::lock_guard<std::mutex> lock(inference_socket_mutex);
        if (!inference_active) return;

        if (inference_recv_socket.is_open()) {
            inference_recv_socket.cancel();
            inference_recv_socket.close();
        }

        if (!recording) {
            std::lock_guard<std::mutex> send_lock(send_mutex);
            auto it = std::remove_if(send_endpoints.begin(), send_endpoints.end(),
                [&](const auto& ep) { return ep.port() == inference_send_port; });
            send_endpoints.erase(it, send_endpoints.end());
        }

        latest_inference_received = boost::none;
        inference_active = false;
    }
    if (inference_recv_thread.joinable()) {
        inference_recv_thread.join();
    }
}

template <size_t DOF>
boost::optional<typename UDPHandler<DOF>::ReceivedData> UDPHandler<DOF>::getLatestTeleopReceived() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return latest_teleop_received;
}

template <size_t DOF>
boost::optional<typename UDPHandler<DOF>::ReceivedData> UDPHandler<DOF>::getLatestInferenceReceived() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return latest_inference_received;
}

template <size_t DOF>
void UDPHandler<DOF>::receiveLoop(boost::asio::ip::udp::socket& recv_socket, boost::optional<ReceivedData>& latest_received) {
    boost::asio::ip::udp::endpoint sender_endpoint;
    jp_type received_jp;
    jv_type received_jv;
    jt_type received_extTorque;
    jt_type received_measTorque;
    double received_gripper;
    uint64_t received_timestamp;

    char buffer[sizeof(double) * DOF * 4 + sizeof(double) + sizeof(uint64_t)];

    while (!stop_threads) {
        boost::system::error_code ec;
        size_t len = recv_socket.receive_from(boost::asio::buffer(buffer, sizeof(buffer)), sender_endpoint, 0, ec);

        if (ec == boost::asio::error::operation_aborted || len != sizeof(buffer))
            continue;


        std::memcpy(received_jp.data(), buffer, sizeof(double) * DOF);
        std::memcpy(received_jv.data(), buffer + sizeof(double) * DOF, sizeof(double) * DOF);
        std::memcpy(received_extTorque.data(), buffer + 2*(sizeof(double) * DOF), sizeof(double) * DOF);
        std::memcpy(received_measTorque.data(), buffer + 3*(sizeof(double) * DOF), sizeof(double) * DOF);
        std::memcpy(&received_gripper, buffer + 4*(sizeof(double) * DOF), sizeof(double));
        std::memcpy(&received_timestamp, buffer + 4*(sizeof(double) * DOF) + sizeof(double), sizeof(uint64_t));

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            latest_received = ReceivedData{received_jp, received_jv, received_extTorque, received_measTorque, received_gripper, received_timestamp};
        }
    }
    recv_socket.close();
}

template <size_t DOF>
void UDPHandler<DOF>::send(const jp_type& jp, const jv_type& jv, const jt_type& extTorque, const jt_type& measTorque, const double& gripper) {
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        pending_send_jp = jp;
        pending_send_jv = jv;
        pending_send_extTorque = extTorque;
        pending_send_measTorque = measTorque;
        pending_send_gripper = gripper;
        new_data_available = true;
    }
    send_condition.notify_one();
}

template <size_t DOF>
void UDPHandler<DOF>::sendLoop() {
    while (!stop_threads) {
        jp_type data_to_send_jp;
        jv_type data_to_send_jv;
        jt_type data_to_send_extTorque;
        jt_type data_to_send_measTorque;
        double data_to_send_gripper;
        std::vector<boost::asio::ip::udp::endpoint> endpoints_snapshot;

        {
            std::unique_lock<std::mutex> lock(send_mutex);
            send_condition.wait(lock, [this] { return new_data_available || stop_threads; });
            if (stop_threads) break;

            new_data_available = false;
            data_to_send_jp = pending_send_jp;
            data_to_send_jv = pending_send_jv;
            data_to_send_extTorque = pending_send_extTorque;
            data_to_send_measTorque = pending_send_measTorque;
            data_to_send_gripper = pending_send_gripper;
            endpoints_snapshot = send_endpoints;
        }

        auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
        uint64_t current_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

        char buffer[sizeof(double) * DOF * 4 + sizeof(double) + sizeof(uint64_t)];
        std::memcpy(buffer, data_to_send_jp.data(), sizeof(double) * DOF);
        std::memcpy(buffer + sizeof(double) * DOF, data_to_send_jv.data(), sizeof(double) * DOF);
        std::memcpy(buffer + 2*(sizeof(double) * DOF), data_to_send_extTorque.data(), sizeof(double) * DOF);
        std::memcpy(buffer + 3*(sizeof(double) * DOF), data_to_send_measTorque.data(), sizeof(double) * DOF);
        std::memcpy(buffer + 4*(sizeof(double) * DOF), &data_to_send_gripper, sizeof(double));
        std::memcpy(buffer + 4*(sizeof(double) * DOF) + sizeof(double), &current_time_ns, sizeof(uint64_t));

        boost::system::error_code ec;
        for (const auto& endpoint : endpoints_snapshot) {
            send_socket.send_to(boost::asio::buffer(buffer, sizeof(buffer)), endpoint, 0, ec);
        }
    }
    send_socket.close();
}

template class UDPHandler<7>; // For DOF=7
