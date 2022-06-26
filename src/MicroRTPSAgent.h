#ifndef MICRO_RTPS_AGENT_H
#define MICRO_RTPS_AGENT_H

#include <string>
#include <cstdint>
#include <memory>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include "microRTPS_transport.h"
#include "RtpsTopics.h"

namespace netdrones { namespace pilot {

/**
 * MicroRTPSAgent
 */
class MicroRTPSAgent {
public:
    /**
     * Initialize with UART transport.
     *
     * @param fd    UART file descriptor
     * @param baudrate
     * @param pollIntervalMillis
     * @param swFlowControl
     * @param hwFlowControl
     * @param verbose
     */
    MicroRTPSAgent(int fd,
                   int baudrate,
                   int pollIntervalMillis,
                   bool swFlowControl,
                   bool hwFlowControl,
                   bool verbose);

    ~MicroRTPSAgent();

    MicroRTPSAgent(const MicroRTPSAgent &) = delete;
    MicroRTPSAgent(MicroRTPSAgent &&) = delete;
    MicroRTPSAgent& operator=(const MicroRTPSAgent&) = delete;

    /**
     * Start the microRTPS g_agent server in background thread.
     *
     * @return true if successful.
     */
    bool Start();

    /**
     * Stop the microRTPS g_agent server.
     *
     * @return
     */
    bool Stop();

private:
    std::atomic<bool> running_;
    std::unique_ptr<Transport_node> transport_;
    std::unique_ptr<RtpsTopics> topics_;
    std::condition_variable send_queue_cond_;
    std::mutex send_queue_mutex_;
    std::thread server_thread_;
    std::thread sender_thread_;
};

} } // namespace netdrones:pilot

#endif // MICRO_RTPS_AGENT_H
