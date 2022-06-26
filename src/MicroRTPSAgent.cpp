#include <unistd.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <queue>
#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>
#include <fastcdr/exceptions/Exception.h>
#include <fastrtps/Domain.h>
#include "MicroRTPSAgent.h"
#include "RtpsTopics.h"
#include "microRTPS_timesync.h"
#include "microRTPS_transport.h"
#include "logging-android.h"

using netdrones::pilot::MicroRTPSAgent;

MicroRTPSAgent::MicroRTPSAgent(int fd,
                               int baudrate,
                               int pollIntervalMillis,
                               bool swFlowControl,
                               bool hwFlowControl,
                               bool verbose)
: running_(false), topics_(std::make_unique<RtpsTopics>()) {
    auto sys_id = static_cast<uint8_t>(MicroRtps::System::MISSION_COMPUTER);
    transport_ = std::make_unique<UART_node>(
        fd,
        baudrate,
        pollIntervalMillis,
        swFlowControl,
        hwFlowControl,
        sys_id,
        verbose
    );
    if (transport_->init() < 0) {
        LOGE("unable to initialize UART transport");
    }
    LOGD("Created with UART transport: fd: %d, baudrate: %d, poll: %dms", fd, baudrate, pollIntervalMillis);
}

MicroRTPSAgent::~MicroRTPSAgent() {
    Stop();
}

bool MicroRTPSAgent::Start() {
    if (running_.load()) {
        LOGI("microRTPS g_agent is already running");
        return false;
    }

    LOGI("starting micrortps_agent");

    running_ = true;
    sender_thread_ = std::thread([this] {
        char buffer[BUFFER_SIZE];
        uint32_t length = 0;
        uint8_t topic_id = 255;
        std::queue<uint8_t> send_queue;

        topics_->set_timesync(std::make_shared<TimeSync>(true));
        topics_->init(&send_queue_cond_, &send_queue_mutex_, &send_queue, "");

        while (running_) {
            std::unique_lock <std::mutex> lk(send_queue_mutex_);

            while (send_queue.empty() && running_) {
                send_queue_cond_.wait(lk);
            }

            topic_id = send_queue.front();
            send_queue.pop();
            lk.unlock();

            auto header_length = transport_->get_header_length();
            eprosima::fastcdr::FastBuffer cdrBuffer(&buffer[header_length],
                                                    sizeof(buffer) - header_length);
            eprosima::fastcdr::Cdr scdr(cdrBuffer);

            if (!running_) {
                if (topics_->getMsg(topic_id, scdr)) {
                    length = scdr.getSerializedDataLength();

                    length = transport_->write(topic_id, buffer, length);
                    if (length < 0) {
                        LOGE("Transport failed to write a topic: %d", topic_id);
                    }
                }
            }
        }
    });

    server_thread_ = std::thread([this] {
        uint8_t topic_id = 255;
        int length;
        char buffer[BUFFER_SIZE];

        while (running_) {
            // Publishing messages received from UART
            length = transport_->read(&topic_id, reinterpret_cast<char *>(&buffer), BUFFER_SIZE);
            LOGD("read %d bytes", length);
            if (length > 0) {
                topics_->publish(topic_id, buffer, sizeof(buffer));
            }
        }
        topics_.reset();
    });

    return true;
}

bool MicroRTPSAgent::Stop() {
    auto expected = true;
    LOGI("stopping micrortps_agent");
    if (running_.compare_exchange_strong(expected, false)) {
        server_thread_.join();
        sender_thread_.join();

        LOGD("stopped");
    }

    return true;
}
