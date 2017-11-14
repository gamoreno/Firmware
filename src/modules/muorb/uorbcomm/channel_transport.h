#pragma once

#ifdef __PX4_POSIX

#include <sys/types.h>
#include <stdint.h>

class Transport
{
public:
    Transport();
    virtual ~Transport();

    virtual int open() = 0;
    virtual void close() = 0;
    virtual bool is_open() = 0;
    ssize_t read_msg(char *buffer, size_t length, uint8_t *msg_type);
    ssize_t write_msg(const uint8_t msg_type, char *buffer, size_t length);

    static const size_t BUFFER_SIZE = 1024;

protected:
    virtual ssize_t read(void *buffer, size_t length) = 0;
    virtual ssize_t write(void *buffer, size_t length) = 0;

    char _recv_buffer[BUFFER_SIZE];
    uint32_t _recv_pos;

private:
    struct __attribute__((packed)) PacketHeader {
        char magic[2];
        uint8_t seq;
        uint16_t crc;
        uint8_t msg_type;
        uint16_t payload_length;
    };
    struct __attribute__((packed)) Packet {
        PacketHeader header;
        char payload[BUFFER_SIZE];
	};
};

#endif
