#pragma once

#include "channel_transport.h"
#include <poll.h>
#include <termios.h>
#include <string>

class SerialTransport: public Transport
{
public:
    SerialTransport(const char *uart_name, uint32_t _baudrate, bool hardware_control_flow);
    virtual ~SerialTransport();

    void close();
    bool is_open();

    static bool baudrate_to_speed(uint32_t bauds, speed_t *speed);
protected:
    int enable_flow_control(bool enabled);
    int open_impl();
    ssize_t read(void *buffer, size_t length);
    ssize_t write(void *buffer, size_t length);

    std::string _uart_name;
    uint32_t _baudrate;
    int _uart_fd;
    bool _control_flow_requested;
    bool _flow_control_enabled;
    struct pollfd _poll_fd[1];
};

