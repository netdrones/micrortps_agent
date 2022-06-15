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

static const char* DEFAULT_IP = "127.0.0.1";
static const int DEFAULT_SEND_PORT = 2019;
static const int DEFAULT_RECV_PORT = 2020;
static const int DEFAULT_BAUDRATE = 460800;
static const int DEFAULT_POLL_INTERVAL_IN_MSEC = 1;

namespace netdrones { namespace axon {

MicroRTPSAgent::MicroRTPSAgent()
: running_(false),
  verbose_(false),
  sw_flow_control_(false),
  hw_flow_control_(false),
  recv_port_(DEFAULT_RECV_PORT),
  send_port_(DEFAULT_SEND_PORT),
  baudrate_(0),
  poll_interval_(0) {
}

MicroRTPSAgent::MicroRTPSAgent(const std::string& device)
: running_(false),
  verbose_(false),
  sw_flow_control_(false),
  hw_flow_control_(false),
  recv_port_(0),
  send_port_(0),
  baudrate_(DEFAULT_BAUDRATE),
  poll_interval_(DEFAULT_POLL_INTERVAL_IN_MSEC),
  device_(device) {
}

MicroRTPSAgent::~MicroRTPSAgent() {
    stop();
}

bool MicroRTPSAgent::start() {
    if (running_.load()) {
        LOGI("microRTPS agent is already running");
        return false;
    }

    LOGI("starting microRTPS agent");

    auto sys_id = static_cast<uint8_t>(MicroRtps::System::MISSION_COMPUTER);
    auto topics = std::make_unique<RtpsTopics>();
    std::unique_ptr<Transport_node> transport_node;
    if (device_.empty()) {
        transport_node = std::make_unique<UDP_node>(
                DEFAULT_IP,
                recv_port_,
                send_port_,
                sys_id,
                verbose_);
    } else {
        transport_node = std::make_unique<UART_node>(
                device_.c_str(),
                baudrate_,
                poll_interval_,
                sw_flow_control_,
                hw_flow_control_,
                sys_id,
                verbose_);
    }

    if (transport_node->init() < 0) {
        LOGE("failed to initialize transport");
        return false;
    }

    sleep(1);

    std::queue<uint8_t> send_queue;
    topics->set_timesync(std::make_shared<TimeSync>(verbose_));
    topics->init(&send_queue_cond_, &send_queue_mutex_, &send_queue, namespace_);

    running_.store(true);
    sender_thread_ = std::thread([&] {
        char buffer[BUFFER_SIZE];
        uint32_t length = 0;
        uint8_t topic_id = 255;

        while (running_.load()) {
            std::unique_lock <std::mutex> lk(send_queue_mutex_);

            while (send_queue.empty() && running_.load()) {
                send_queue_cond_.wait(lk);
            }

            topic_id = send_queue.front();
            send_queue.pop();
            lk.unlock();

            auto header_length = transport_node->get_header_length();
            eprosima::fastcdr::FastBuffer cdrBuffer(&buffer[header_length],
                                                    sizeof(buffer) - header_length);
            eprosima::fastcdr::Cdr scdr(cdrBuffer);

            if (!running_.load()) {
                if (topics->getMsg(topic_id, scdr)) {
                    length = scdr.getSerializedDataLength();

                    length = transport_node->write(topic_id, buffer, length);
                    if (length < 0) {
                        LOGE("Transport failed to write a topic: %d", topic_id);
                    }
                }
            }
        }
    });

    server_thread_ = std::thread([&] {
        uint8_t topic_id = 255;
        int length;
        char buffer[BUFFER_SIZE];

        while (running_.load()) {
            // Publishing messages received from UART
            length = transport_node->read(&topic_id, reinterpret_cast<char *>(&buffer), BUFFER_SIZE);
            if (length > 0) {
                topics->publish(topic_id, buffer, sizeof(buffer));
            }
        }

        transport_node.reset();
        topics.reset();
    });

    return true;
}

bool MicroRTPSAgent::stop() {
    auto expected = true;
    if (running_.compare_exchange_strong(expected, false)) {
        LOGD("Stopping microRTPS agent thread");

        server_thread_.join();
        sender_thread_.join();

        LOGD("stopped");
    }

    return true;
}

} } // namespace:axon
