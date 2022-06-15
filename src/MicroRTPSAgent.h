#ifndef MICRORTPSAGENT_H
#define MICRORTPSAGENT_H

#include <string>
#include <cstdint>
#include <memory>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include "microRTPS_transport.h"
#include "RtpsTopics.h"

namespace netdrones { namespace axon {

/**
 * MicroRTPSAgent
 */
class MicroRTPSAgent {
public:
    /**
     * Initialize with UDP transport.
     */
    MicroRTPSAgent();

    /**
     * Initialize with UART transport.
     *
     * @param device UART device
     */
    MicroRTPSAgent(
        const std::string& device
    );

    ~MicroRTPSAgent();
    MicroRTPSAgent(const MicroRTPSAgent &) = delete;

    /**
     * Start the microRTPS agent server in background thread.
     *
     * @return true if successful.
     */
    bool start();

    /**
     * Stop the microRTPS agent server.
     *
     * @return
     */
    bool stop();

    inline void setVerbose(bool verbose) noexcept {
        verbose_ = verbose;
    }
    inline void setRecvPort(int port) noexcept {
        recv_port_ = static_cast<uint16_t>(port);
    }
    inline void setSendPort(int port) noexcept {
        send_port_ = static_cast<uint16_t>(port);
    }
    inline void setBaudrate(int baudrate) noexcept {
        baudrate_ = baudrate;
    }
    inline void setPollInterval(int interval) noexcept {
        poll_interval_ = interval;
    }
//    inline void
    inline void setUARTDevice(const std::string& device) noexcept {
        device_ = device;
    }
    inline void setNamespace(const std::string& ns) noexcept {
        namespace_ = ns;
    }

private:
    std::atomic<bool> running_;
    bool verbose_;
    bool sw_flow_control_;
    bool hw_flow_control_;
    uint16_t recv_port_;
    uint16_t send_port_;
    uint32_t baudrate_;
    int poll_interval_;
    std::string device_;
    std::string namespace_;
    std::unique_ptr<RtpsTopics> topics_;
    std::condition_variable send_queue_cond_;
    std::mutex send_queue_mutex_;

    std::thread server_thread_;
    std::thread sender_thread_;
};

} } // namespace netdrones:axon

#endif // MICRORTPSAGENT_H
