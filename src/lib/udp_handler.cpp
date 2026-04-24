
#include "udp_handler.h"
#include <boost/system/error_code.hpp>
#include <cstring>

// TODO: inference recv can just pass to current teleop_recv for now. change later
template <size_t DOF>
UDPHandler<DOF>::UDPHandler(const std::string& teleop_host, int teleop_send, int teleop_recv, OperationMode mode, const std::string& inference_host, int inference_send, int inference_recv)
    : stop_threads(false)
    , send_socket(io_context, boost::asio::ip::udp::v4())
    , recv_socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), teleop_recv)) {

    send_endpoints.push_back(boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(teleop_host), teleop_send));

    if (mode == OperationMode::RECORD || mode == OperationMode::INFERENCE) {
        send_endpoints.push_back(boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(inference_host), inference_send));
    }

    recv_thread = std::thread(&UDPHandler::receiveLoop, this);
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
        recv_socket.cancel();
        recv_socket.shutdown(boost::asio::ip::udp::socket::shutdown_both);
        recv_socket.close();
    } catch (...) {
        // Ignore any exceptions during socket cleanup
    }

    if (recv_thread.joinable())
        recv_thread.join();
    if (send_thread.joinable())
        send_thread.join();
}

template <size_t DOF>
boost::optional<typename UDPHandler<DOF>::ReceivedData> UDPHandler<DOF>::getLatestReceived() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return latest_received;
}

template <size_t DOF>
void UDPHandler<DOF>::receiveLoop() {
    boost::asio::ip::udp::endpoint sender_endpoint;
    jp_type received_jp;
    jv_type received_jv;
    jt_type received_extTorque;
    jt_type received_measTorque;
    double received_gripper;
    char buffer[sizeof(double) * DOF * 4 + sizeof(double)];

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

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            latest_received = ReceivedData{received_jp, received_jv, received_extTorque, received_measTorque, received_gripper, std::chrono::steady_clock::now()};
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
    boost::asio::ip::udp::endpoint remote_endpoint(boost::asio::ip::make_address(remote_host), send_port);

    while (!stop_threads) {
        std::unique_lock<std::mutex> lock(send_mutex);
        send_condition.wait(lock, [this] { return new_data_available || stop_threads; });

        if (stop_threads)
            break;

        new_data_available = false;
        jp_type data_to_send_jp = pending_send_jp;
        jp_type data_to_send_jv = pending_send_jv;
        jt_type data_to_send_extTorque = pending_send_extTorque;
        jt_type data_to_send_measTorque = pending_send_measTorque;
        double data_to_send_gripper = pending_send_gripper;
        lock.unlock();

        char buffer[sizeof(double) * DOF * 4 + sizeof(double)];
        std::memcpy(buffer, data_to_send_jp.data(), sizeof(double) * DOF);
        std::memcpy(buffer + sizeof(double) * DOF, data_to_send_jv.data(), sizeof(double) * DOF);
        std::memcpy(buffer + 2*(sizeof(double) * DOF), data_to_send_extTorque.data(), sizeof(double) * DOF);
        std::memcpy(buffer + 3*(sizeof(double) * DOF), data_to_send_measTorque.data(), sizeof(double) * DOF);
        std::memcpy(buffer + 4*(sizeof(double) * DOF), &data_to_send_gripper, sizeof(double));

        boost::system::error_code ec;
        send_socket.send_to(boost::asio::buffer(buffer, sizeof(buffer)), remote_endpoint, 0, ec);
    }
    send_socket.close();
}

template class UDPHandler<7>; // For DOF=7
