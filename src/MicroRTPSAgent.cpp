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
#include "USBSerial_node.h"
#include "logging-android.h"

using netdrones::micrortps_agent::MicroRTPSAgent;

MicroRTPSAgent::MicroRTPSAgent(int uart_fd,
                               int baudrate,
                               int flow_ctrl,
                               bool verbose)
: verbose_(verbose),
  running_(false),
  topics_(std::make_unique<RtpsTopics>()) {
    auto sys_id = static_cast<uint8_t>(MicroRtps::System::MISSION_COMPUTER);
    transport_ = std::make_unique<USBSerial_node>(
        uart_fd,
        baudrate,
        flow_ctrl,
        sys_id,
        verbose
    );

}

MicroRTPSAgent::~MicroRTPSAgent() {
    Stop();
}

bool MicroRTPSAgent::Start() {
    if (running_.load()) {
        return false;
    }

    if (transport_->init() < 0) {
        LOGE("USB serial transport initialization failed");
        return false;
    }

    LOGD("--- MicroRTPS Agent ---");
    LOGD("ROS namespace: %s", ns_.c_str());
    LOGD("ROS_LOCALHOST_ONLY: %s", std::getenv("ROS_LOCALHSOT_ONLY"));

    running_.store(true, std::memory_order_release);
    sender_thread_ = std::thread([this] {
        char buffer[BUFFER_SIZE];
        uint32_t length = 0;
        uint8_t topic_id = 255;
        std::queue<uint8_t> send_queue;

        topics_->set_timesync(std::make_shared<TimeSync>(verbose_));
        topics_->init(&send_queue_cv_, &send_queue_mutex_, &send_queue, ns_);

        while (running_) {
            std::unique_lock <std::mutex> lk(send_queue_mutex_);

            while (send_queue.empty() && running_) {
                send_queue_cv_.wait(lk);
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
        ssize_t length;
        char buffer[BUFFER_SIZE];

        while (running_) {
            // Publishing messages received from UART
            length = transport_->read(&topic_id, reinterpret_cast<char *>(&buffer), BUFFER_SIZE);
            if (length > 0) {
                LOGD("read %lu bytes", length);
                topics_->publish(topic_id, buffer, sizeof(buffer));
            }
        }
        topics_.reset();
    });

    return true;
}

bool MicroRTPSAgent::Stop() {
    auto expected = true;
    LOGD("stopping micrortps_agent");

    if (running_.compare_exchange_strong(expected, false, std::memory_order_acquire)) {
        server_thread_.join();
        sender_thread_.join();
    }

    return true;
}
