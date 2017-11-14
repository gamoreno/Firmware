#include "channel_serial_transport.h"
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <mavlink/v2.0/checksum.h>
#include <px4_log.h>


SerialTransport::SerialTransport(const char *uart_name, uint32_t baudrate, bool hardware_control_flow):
    _uart_name(uart_name),
    _baudrate(baudrate),
    _uart_fd(-1),
    _control_flow_requested(hardware_control_flow),
    _flow_control_enabled(false)
{
}

SerialTransport::~SerialTransport()
{
	close();
}

int SerialTransport::open()
{
    /* open uart */
    _uart_fd = ::open(_uart_name.c_str(), O_RDWR | O_NOCTTY); //  | O_NONBLOCK ?

    if (_uart_fd < 0) {
        PX4_ERR("failed to open device: %s (%s)\n", _uart_name.c_str(), strerror(errno));
        return -1;
	}

    /* Try to set baud rate */
    struct termios uart_config;
    int termios_state;

    /* Initialize the uart config */
    if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0) {
        PX4_ERR("ERR GET CONF %s: %d\n", _uart_name.c_str(), termios_state);
        close();
        return -1;
    }

    /* Clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;

    /* Set baud rate */
    speed_t speed;

    if (!baudrate_to_speed(_baudrate, &speed)) {
        printf("ERR SET BAUD %s: Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 921600, 1000000\n",
               _uart_name.c_str(), _baudrate);
        close();
        return -1;
    }

    if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
        PX4_ERR("ERR SET BAUD %s: %d\n", _uart_name.c_str(), termios_state);
        close();
        return -1;
    }

#if defined (__PX4_LINUX) || defined (__PX4_DARWIN)
    /* Put in raw mode */
    cfmakeraw(&uart_config);
#endif

    if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
        PX4_WARN("ERR SET CONF %s\n", _uart_name.c_str());
        ::close(_uart_fd);
        return -1;
    }

    /*
     * Setup hardware flow control. If the port has no RTS pin this call will fail,
     * which is not an issue, but requires a separate call so we can fail silently.
     */

    /* setup output flow control */
    if (enable_flow_control(_control_flow_requested)) {
        if (_control_flow_requested) {
            PX4_WARN("hardware flow control not supported");
        }
    }

    _poll_fd[0].fd = _uart_fd;
    _poll_fd[0].events = POLLIN;

    return 0;
}


void SerialTransport::close()
{
    if (_uart_fd != -1) {
        ::close(_uart_fd);
        _uart_fd = -1;
	}
}


bool SerialTransport::is_open()
{
    return (_uart_fd != -1);
}

ssize_t SerialTransport::read(void *buffer, size_t length)
{
    if (!is_open()) {
        return -EPERM;
    }

    if (buffer == nullptr) {
        return -EINVAL;
	}

	ssize_t ret = 0;
    int r = poll(_poll_fd, 1, 1000); // TODO check timeout

    if (r == 1 && (_poll_fd[0].revents & POLLIN)) {
        ret = ::read(_uart_fd, buffer, length);
        if (ret < 0) {
            return -errno;
        }
    }

	return ret;
}

ssize_t SerialTransport::write(void *buffer, size_t length)
{
    if (!is_open() || buffer == nullptr) {
        return -1;
	}

//    PX4_INFO("before write");
   int ret = ::write(_uart_fd, buffer, length);
//   PX4_INFO("after write %d", ret);
   if (ret < 0) {
       PX4_ERR("write error %s", strerror((errno)));
   }
   return ret;
}

bool SerialTransport::baudrate_to_speed(uint32_t bauds, speed_t *speed)
{
#ifndef B460800
#define B460800 460800
#endif

#ifndef B500000
#define B500000 500000
#endif

#ifndef B921600
#define B921600 921600
#endif

#ifndef B1000000
#define B1000000 1000000
#endif

    switch (bauds) {
    case 0:      *speed = B0;      break;

    case 50:     *speed = B50;     break;

    case 75:     *speed = B75;     break;

    case 110:    *speed = B110;    break;

    case 134:    *speed = B134;    break;

    case 150:    *speed = B150;    break;

    case 200:    *speed = B200;    break;

    case 300:    *speed = B300;    break;

    case 600:    *speed = B600;    break;

    case 1200:   *speed = B1200;   break;

    case 1800:   *speed = B1800;   break;

    case 2400:   *speed = B2400;   break;

    case 4800:   *speed = B4800;   break;

    case 9600:   *speed = B9600;   break;

    case 19200:  *speed = B19200;  break;

    case 38400:  *speed = B38400;  break;

    case 57600:  *speed = B57600;  break;

    case 115200: *speed = B115200; break;

    case 230400: *speed = B230400; break;

    case 460800: *speed = B460800; break;

    case 500000: *speed = B500000; break;

    case 921600: *speed = B921600; break;

    case 1000000: *speed = B1000000; break;

#ifdef B1500000

    case 1500000: *speed = B1500000; break;
#endif

#ifdef B3000000

    case 3000000: *speed = B3000000; break;
#endif

    default:
        return false;
    }

    return true;
}

int
SerialTransport::enable_flow_control(bool enabled)
{
    struct termios uart_config;

    int ret = tcgetattr(_uart_fd, &uart_config);

    if (enabled) {
        uart_config.c_cflag |= CRTSCTS;

    } else {
        uart_config.c_cflag &= ~CRTSCTS;

    }

    ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

    if (!ret) {
        _flow_control_enabled = enabled;
    }

    return ret;
}
