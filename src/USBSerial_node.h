#pragma once

#include <cstring>
#include <cstdint>
#include "USBSerial.hpp"
#include "microRTPS_transport.h"

class USBSerial_node: public Transport_node {
public:
    USBSerial_node(int uart_fd,
                   int baudrate,
                   int flow_ctrl,
                   uint8_t sys_id,
                   bool debug=false);
    ~USBSerial_node();

    int init();
    uint8_t close();

private:
    netdrones::usb_serial::USBSerial serial_;

    bool fds_OK();
    ssize_t node_read(void* buffer, size_t len);
    ssize_t node_write(void* buffer, size_t len);
};