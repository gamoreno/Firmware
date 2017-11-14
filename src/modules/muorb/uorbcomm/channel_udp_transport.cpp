#include "channel_udp_transport.h"
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <mavlink/v2.0/checksum.h>
#include <px4_log.h>


UDPTransport::UDPTransport(uint16_t local_port, const char *remote_host, uint16_t remote_port):
    _local_port(local_port),
    _remote_host(remote_host),
    _remote_port(remote_port),
    _socket_fd(-1)
{
}

UDPTransport::~UDPTransport()
{
	close();
}

int UDPTransport::open()
{
    /* set remote address */
    memset((char *) &_remoteaddr, 0, sizeof(_remoteaddr));
    _remoteaddr.sin_family = AF_INET;
    if (inet_aton(_remote_host.c_str(), &_remoteaddr.sin_addr) == 0) {
        PX4_ERR("Invalid remote host %s", _remote_host.c_str());
        return -1;
    }
    _remoteaddr.sin_port = htons(_remote_port);

    PX4_DEBUG("Setting up UDP with port %u", _local_port);

    memset((char *)&_myaddr, 0, sizeof(_myaddr));
    _myaddr.sin_family = AF_INET;
    _myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    _myaddr.sin_port = htons(_local_port);

    if ((_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        PX4_ERR("create socket failed: %s", strerror(errno));
        return -1;
    }

    if (bind(_socket_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
        PX4_WARN("bind failed: %s", strerror(errno));
        close();
        return -1;
    }

	return 0;
}


void UDPTransport::close()
{
    if (_socket_fd != -1) {
        shutdown(_socket_fd, SHUT_RDWR);
        ::close(_socket_fd);
        _socket_fd = -1;
	}
}

bool UDPTransport::is_open()
{
    return (_socket_fd != -1);
}

ssize_t UDPTransport::read(void *buffer, size_t length)
{
    if (!is_open() || buffer == nullptr) {
		return -1;
	}

    return recvfrom(_socket_fd, buffer, length, 0, nullptr, nullptr);
}

ssize_t UDPTransport::write(void *buffer, size_t length)
{
    if (!is_open() || buffer == nullptr) {
        return -1;
	}

    return sendto(_socket_fd, buffer, length, 0, (struct sockaddr *)&_remoteaddr, sizeof(_remoteaddr));
}
