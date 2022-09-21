// Copyright 2022 NetDrones Inc. All rights reserved.

#include "errors.h"
#include "logging-android.h"
#include "USBSerial_node.h"

using namespace netdrones::errors;
using netdrones::usb_serial::USBSerial;

USBSerial_node::USBSerial_node(
    int uart_fd,
    int baudrate,
    int flow_ctrl,
    uint8_t sys_id,
    bool debug
)
: Transport_node(sys_id, debug),
//  serial_(uart_fd, baudrate, static_cast<USBSerial::FlowControl>()) {
  serial_(uart_fd, baudrate, USBSerial::RTS_CTS_HS) {
}

int USBSerial_node::init() {
    if (serial_.Open() != ERR_OK) {
        return -1;
    }
    return 0;
}

bool USBSerial_node::fds_OK() {
    return serial_.IsOpen();
}

uint8_t USBSerial_node::close() {
    return serial_.Close();
}

ssize_t USBSerial_node::node_read(void* buffer, size_t len) {
    return serial_.Read(buffer, len);
}

ssize_t USBSerial_node::node_write(void* buffer, size_t len) {
    return serial_.Write(buffer, len);
}