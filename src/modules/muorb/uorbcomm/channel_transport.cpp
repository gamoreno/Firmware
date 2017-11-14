#ifdef __PX4_POSIX

#include "channel_transport.h"
#include <arpa/inet.h>
#include <pthread.h>
#include <mavlink/v2.0/checksum.h>
#include <px4_log.h>


Transport::Transport(): _recv_pos(0)
{
}

Transport::~Transport()
{
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

    static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    static uint8_t seq = 0;

    size_t packetLength = sizeof(PacketHeader) + length;

    Packet packet;
    packet.header.magic[0] = '=';
    packet.header.magic[1] = '>';
    packet.header.crc = htons(crc_calculate((uint8_t *)buffer, length));
    packet.header.msg_type = msg_type;
    packet.header.payload_length = htons(length);
    memcpy(packet.payload, buffer, length);

    pthread_mutex_lock(&mutex);

    packet.header.seq = seq++;
    ssize_t len = write(&packet, packetLength);

    pthread_mutex_unlock(&mutex);

    return (len != (ssize_t) packetLength) ? -1 : length;
}

#endif
