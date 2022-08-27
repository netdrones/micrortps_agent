// Copyright 2022 NetDrones Inc. All rights reserved.

#ifndef NETDRONES_MICRO_RTPS_AGENT_H
#define NETDRONES_MICRO_RTPS_AGENT_H

#include <cstdlib>
#include <string>
#include <cstdint>
#include <memory>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#ifdef ROS_BRIDGE
#include <rclcpp/rclcpp.hpp>
#endif // ROS_BRIDGE
#include "microRTPS_transport.h"
#include "RtpsTopics.h"
#include "../usb_serial/include/USBSerial.hpp"

namespace netdrones {
namespace micrortps_agent {

class MicroRTPSAgent {
public:
    /**
     * Initialize with UART transport.
     *
     * @param fd    UART file descriptor
     * @param baudrate
     * @param flow_ctrl 0: Disabled
     * @param verbose
     */
    MicroRTPSAgent(int fd,
                   int baudrate,
                   int flow_ctrl=0,
                   bool verbose=false);

    ~MicroRTPSAgent();

    MicroRTPSAgent(const MicroRTPSAgent &) = delete;
    MicroRTPSAgent(MicroRTPSAgent &&) = delete;
    MicroRTPSAgent& operator=(const MicroRTPSAgent&) = delete;

     void set_ros_namespace(const std::string& ns) {
         ns_ = ns;
     }

      void set_ros_localhost_only(bool only) {
         setenv("ROS_LOCALHOST_ONLY", only ? "1" : "0", 1);
      }

    /**
     * Start the microRTPS agent server in background thread.
     *
     * @return true if successful.
     */
    bool Start();

    /**
     * Stop the microRTPS agent server.
     *
     * @return
     */
    bool Stop();

private:
    bool verbose_;
    std::atomic_flag running_ = ATOMIC_FLAG_INIT;
    std::atomic_bool exit_sender_thread_ = false;
    std::string ns_;
    std::queue<uint8_t> send_queue_;
    std::unique_ptr<Transport_node> transport_;
#ifdef ROS_BRIDGE
    std::shared_ptr<RtpsTopics> topics_;
#else
    std::unique_ptr<RtpsTopics> topics_;
#endif // ROS_BRIDGE
    std::condition_variable send_queue_cv_;
    std::mutex send_queue_mutex_;
    std::thread poll_serial_thread_;
    std::thread sender_thread_;
#ifdef ROS_BRIDGE
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread executor_thread_;
#endif // ROS_BRIDGE
    mutable std::mutex mtx_;

    void PollSerial();
};

} // namespace micrortps_agent
} // namespace netdrones

#endif //NETDRONES_MICRO_RTPS_AGENT_H