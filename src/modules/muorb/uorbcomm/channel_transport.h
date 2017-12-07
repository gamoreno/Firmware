#pragma once

#ifdef __PX4_POSIX

#include <sys/types.h>
#include <stdint.h>
#include <queue>
#include <semaphore.h>
#include <pthread.h>
#include <drivers/drv_hrt.h>
#include <mutex>

class Transport
{
public:
    Transport();
    virtual ~Transport();

    int open();
    virtual void close() = 0;
    virtual bool is_open() = 0;
    ssize_t read_msg(char *buffer, size_t length, uint8_t *msg_type);
    ssize_t write_msg(const uint8_t msg_type, char *buffer, size_t length);

    void print_stats();

    static const size_t BUFFER_SIZE = 1024;

protected:
    virtual int open_impl() = 0;
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

    void writer_thread();
    static void *writer_thread_start(void *);
    void stop_thread();

    std::queue<Packet*> _write_queue;
    sem_t _write_queue_sem;
    pthread_mutex_t mutex;
    pthread_t _write_thread;
    bool _writer_thread_started;
    volatile bool _should_exit;

    uint32_t _bytes_recv;
    uint32_t _bytes_sent;
    uint32_t _msg_recv;
    uint32_t _msg_sent;
    std::mutex _bytes_recv_mutex;
    std::mutex _bytes_sent_mutex;
    hrt_abstime _last_stats_start;
};

#endif
