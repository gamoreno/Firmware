#pragma once

#include "channel_transport.h"
#include <string>
#include <arpa/inet.h>

class UDPTransport: public Transport
{
public:
    UDPTransport(uint16_t _local_port, const char *_remote_host, uint16_t _remote_port);
    virtual ~UDPTransport();

    int open();
    void close();
    bool is_open();

protected:
    ssize_t read(void *buffer, size_t length);
    ssize_t write(void *buffer, size_t length);

    uint16_t _local_port;
    std::string _remote_host;
    uint16_t _remote_port;
    int _socket_fd;
    struct sockaddr_in _remoteaddr;
    struct sockaddr_in _myaddr;
};
