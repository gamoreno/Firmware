#ifdef __PX4_POSIX

#include "channel_transport.h"
#include <arpa/inet.h>
#include <pthread.h>
#include <mavlink/v2.0/checksum.h>
#include <px4_log.h>
#include <time.h>
#include "uorb_comm_channel.h"
#include <px4_tasks.h>

Transport::Transport(): _recv_pos(0), _writer_thread_started(false)
{
    sem_init(&_write_queue_sem, 0, 0);
    mutex = PTHREAD_MUTEX_INITIALIZER;
}

Transport::~Transport()
{
}

void Transport::stop_thread() {
    if (_writer_thread_started) {
        _should_exit = true;
        pthread_join(_write_thread, nullptr);
    }
}

int Transport::open() {
    if (!uORB::UORBCommChannel::thread_start_helper(&_write_thread, "uorbcomm.wrt", SCHED_PRIORITY_SLOW_DRIVER, writer_thread_start, this)) {
        PX4_ERR("cannot start writer thread");
        return -1;
    }
    _writer_thread_started = true;

    int ret = open_impl();
    if (ret != 0) {
        stop_thread();
    }

    _last_stats_start = hrt_absolute_time();
    return ret;
}

ssize_t Transport::read_msg(char *buffer, size_t length, uint8_t *msg_type)
{
    if (buffer == nullptr || msg_type == nullptr || !is_open()) {
		return -1;
	}

    ssize_t len = read((void *)(_recv_buffer + _recv_pos), sizeof(_recv_buffer) - _recv_pos);

    if (len <= 0) {
        if (len < 0) {
            PX4_ERR("read error %s", strerror(-len));
        }
		return len;
	}

    {
        std::lock_guard<std::mutex> lock(_bytes_recv_mutex);
        _bytes_recv += len;
    }

    _recv_pos += len;

    if (_recv_pos < sizeof(PacketHeader)) {

        // don't have a complete header yet
		return 0;
	}


    // find packet header start
    uint16_t header_start = 0;
    bool header_magic_found = false;
    while (header_start <= _recv_pos - sizeof(PacketHeader::magic)) {
        if (_recv_buffer[header_start] == '=' && _recv_buffer[header_start + 1] == '>') {
            header_magic_found = true;
			break;
		}
        header_start++;
	}

	// Start not found
    if (!header_magic_found) {

        PX4_WARN("dropping %u unrecognized bytes", header_start);

        // at most the last byte read so far could be useful, keep it
        _recv_buffer[0] = _recv_buffer[header_start];
        _recv_pos = 1;

		return -1;
	}

    if (_recv_pos - header_start < sizeof(PacketHeader)) {

        // don't have a complete header yet
        return 0;
    }

    PacketHeader *header = (PacketHeader *)&_recv_buffer[header_start];
    uint16_t payload_len = ntohs(header->payload_length);

    if (header_start + sizeof(PacketHeader) + payload_len > _recv_pos) {

        // don't have a complete packet yet

        if (header_start > 0) {
            PX4_WARN("dropping %u unrecognized bytes", header_start);
            memmove(_recv_buffer, _recv_buffer + header_start, _recv_pos - header_start);
            _recv_pos -= header_start;
        }

        return 0;
    }


    // check if message fits in provided buffer
    if (payload_len > length) {
		return -EMSGSIZE;
	}

    char* payload = _recv_buffer + header_start + sizeof(PacketHeader);

    uint16_t crc = crc_calculate((uint8_t*) payload, payload_len);

    if (ntohs(header->crc) != crc) {
        PX4_WARN("bad CRC");
		len = -1;
	} else {
		// copy message to outbuffer and set other return values
        memcpy(buffer, payload, payload_len);
        *msg_type = header->msg_type;
        len = payload_len;
	}

    {
        std::lock_guard<std::mutex> lock(_bytes_recv_mutex);
        _msg_recv++;
    }


    // remove msg from buffer
    _recv_pos -= header_start + sizeof(PacketHeader) + payload_len;
    memmove(_recv_buffer, payload + payload_len, _recv_pos);

	return len;
}


ssize_t Transport::write_msg(const uint8_t msg_type, char buffer[], size_t length)
{
    if (!is_open() || length > BUFFER_SIZE) {
        return -1;
    }

    static uint8_t seq = 0;

    size_t packetLength = sizeof(PacketHeader) + length;

    auto packetPtr = new Packet;
    Packet& packet = *packetPtr;
    packet.header.magic[0] = '=';
    packet.header.magic[1] = '>';
    packet.header.crc = htons(crc_calculate((uint8_t *)buffer, length));
    packet.header.msg_type = msg_type;
    packet.header.payload_length = htons(length);
    memcpy(packet.payload, buffer, length);

    pthread_mutex_lock(&mutex);

    packet.header.seq = seq++;

    //ssize_t len = write(&packet, packetLength);
    _write_queue.push(packetPtr);
    pthread_mutex_unlock(&mutex);

    sem_post(&_write_queue_sem);
    ssize_t len = packetLength;

    {
        std::lock_guard<std::mutex> lock(_bytes_sent_mutex);
        _bytes_sent += packetLength;
        _msg_sent++;
    }

    return (len != (ssize_t) packetLength) ? -1 : length;
}


void *Transport::writer_thread_start(void *instance)
{
    ((Transport*) instance)->writer_thread();

    return 0;
}

void Transport::writer_thread() {
    _should_exit = false;

    while (!_should_exit)
    {
        struct timespec timeout;
        clock_gettime(CLOCK_REALTIME, &timeout);
        timeout.tv_sec += 1;
        if (sem_timedwait(&_write_queue_sem, &timeout) == 0) {
            pthread_mutex_lock(&mutex);
            if (!_write_queue.empty()) {
                Packet* packet = _write_queue.front();
                _write_queue.pop();
                pthread_mutex_unlock(&mutex);
                uint16_t packetLength = ntohs(packet->header.payload_length) + sizeof(PacketHeader);
                write(packet, packetLength);
                delete packet;
            } else {
                pthread_mutex_unlock(&mutex);
            }
        }
    }
}

void Transport::print_stats() {
    double avg_bytes_sent;
    double avg_bytes_recv;
    double avg_msg_sent;
    double avg_msg_recv;

    {
        std::lock_guard<std::mutex> locks(_bytes_sent_mutex);
        std::lock_guard<std::mutex> lockr(_bytes_recv_mutex);
        hrt_abstime now = hrt_absolute_time();
        avg_bytes_sent = (_bytes_sent * 1000000.0 / (now - _last_stats_start));
        avg_bytes_recv = (_bytes_recv * 1000000.0 / (now - _last_stats_start));
        avg_msg_sent = (_msg_sent * 1000000.0 / (now - _last_stats_start));
        avg_msg_recv = (_msg_recv * 1000000.0 / (now - _last_stats_start));
        _last_stats_start = now;
        _bytes_recv = 0;
        _bytes_sent = 0;
        _msg_recv = 0;
        _msg_sent = 0;
    }
    PX4_INFO("bytes/sec: sent=%f  recv=%f", avg_bytes_sent, avg_bytes_recv);
    PX4_INFO("msg/sec: sent=%f  recv=%f", avg_msg_sent, avg_msg_recv);
    PX4_INFO("send queue size=%d", _write_queue.size());
}

#endif
