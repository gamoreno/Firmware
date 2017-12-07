#include "uorb_comm_channel.h"
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_getopt.h>
#include <algorithm>
#include <px4_module.h>
#include "channel_serial_transport.h"
#include "channel_udp_transport.h"
#include <string>

using namespace std;

namespace uORB
{

static void usage()
{
    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
uorbcomm is a communication channel uORB, allowing to have communication between
modules distributed across different computers (e.g., a Pixhawk and a
posix-based companion computer).

It must be started after uorb is started.
For time synchronization, the uorbcomm on the posix computer must synchronize
its time with the other computer (main).
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("uorbcomm", "communication");
    PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyAMA0", "<file:dev>", "Select Serial Device", true);
    PRINT_MODULE_USAGE_PARAM_INT('b', 57600, 9600, 3000000, "Baudrate", true);
#ifdef __PX4_POSIX
    PRINT_MODULE_USAGE_PARAM_INT('u', 3030, 0, 65536, "Local UDP Network Port", true);
    PRINT_MODULE_USAGE_PARAM_INT('o', 3030, 0, 65536, "Remote UDP Network Port", true);
    PRINT_MODULE_USAGE_PARAM_STRING('t', "127.0.0.1", nullptr,
                    "Remote UDP host", true);
#endif
    PRINT_MODULE_USAGE_PARAM_FLAG('s', "Set as time synchronization master", true);
    PRINT_MODULE_USAGE_PARAM_FLAG('h', "Print this help", true);
}

UORBCommChannel* UORBCommChannel::_instance = nullptr;

UORBCommChannel::UORBCommChannel()
    : _channel_rx_handler(nullptr)
{
    pthread_rwlock_init(&_subscribers_rw_lock, nullptr);
    pthread_rwlock_init(&_rate_limiting_rw_lock, nullptr);
}

UORBCommChannel::~UORBCommChannel() {
    pthread_rwlock_destroy(&_subscribers_rw_lock);
    pthread_rwlock_destroy(&_rate_limiting_rw_lock);
}

UORBCommChannel *UORBCommChannel::get_instance()
{
    if (_instance == nullptr) {
        _instance = new UORBCommChannel;
    }
    return _instance;
}

int16_t UORBCommChannel::topic_advertised(const char *message_name)
{
    char buffer[BUFFER_SIZE];
    strncpy(buffer, message_name, BUFFER_SIZE - 1);
    buffer[BUFFER_SIZE - 1] = 0;
    ssize_t msg_len = strlen(buffer) + 1;
    ssize_t written = _transport->write_msg(MSG_TYPE_ADVERTISE, buffer, msg_len);
    return (written == msg_len) ? 0 : -1;
}

int16_t UORBCommChannel::topic_unadvertised(const char *message_name)
{
    char buffer[BUFFER_SIZE];
    strncpy(buffer, message_name, BUFFER_SIZE - 1);
    buffer[BUFFER_SIZE - 1] = 0;
    ssize_t msg_len = strlen(buffer) + 1;
    ssize_t written = _transport->write_msg(MSG_TYPE_UNADVERTISE, buffer, msg_len);
    return (written == msg_len) ? 0 : -1;
}

int16_t UORBCommChannel::add_subscription(const char *message_name, int32_t msgRateInHz)
{
    char buffer[BUFFER_SIZE];
    strncpy(buffer, message_name, BUFFER_SIZE - 1);
    buffer[BUFFER_SIZE - 1] = 0;
    ssize_t msg_len = strlen(buffer) + 1;
    *((int32_t*) (buffer + msg_len)) = msgRateInHz;
    msg_len += sizeof(int32_t);
//    if (std::string(message_name).find("vehicle_control_mode") != string::npos) {
//        PX4_INFO("sending add_subscription(%s) @ %" PRIu64, message_name, hrt_absolute_time());
//    }
    ssize_t written = _transport->write_msg(MSG_TYPE_ADD_SUBSCRIBER, buffer, msg_len);
    return (written == msg_len) ? 0 : -1;
}

int16_t UORBCommChannel::remove_subscription(const char *message_name)
{
//    if (std::string(message_name).find("vehicle_control_mode") != string::npos) {
//        PX4_INFO("sending REMOVE_subscription(%s)", message_name);
//    }
    char buffer[BUFFER_SIZE];
    strncpy(buffer, message_name, BUFFER_SIZE - 1);
    buffer[BUFFER_SIZE - 1] = 0;
    ssize_t msg_len = strlen(buffer) + 1;
    ssize_t written = _transport->write_msg(MSG_TYPE_REMOVE_SUBSCRIBER, buffer, msg_len);
    return (written == msg_len) ? 0 : -1;
}

int16_t UORBCommChannel::register_handler(uORBCommunicator::IChannelRxHandler *handler)
{
    _channel_rx_handler = handler;
    return 0;
}

int16_t UORBCommChannel::send_message(const char *message_name, int32_t length, uint8_t *data)
{
    pthread_rwlock_rdlock(&_subscribers_rw_lock);
    bool has_subscribers = (_subscribers.find(message_name) != _subscribers.end());
    pthread_rwlock_unlock(&_subscribers_rw_lock);

//    if (std::string(message_name).find("vehicle_control_mode") != string::npos) {
//        PX4_INFO("send message (%s) has subscribers %d", message_name, has_subscribers);
//    }

    if (!_send_with_no_subscriber && !has_subscribers) {
            return 0;
    }

    pthread_rwlock_rdlock(&_rate_limiting_rw_lock);
    auto entry = _rate_limiting.find(message_name);
    pthread_rwlock_unlock(&_rate_limiting_rw_lock);

    if (entry == _rate_limiting.end()) {
        pthread_rwlock_wrlock(&_rate_limiting_rw_lock);
        auto new_entry = _rate_limiting.insert(std::pair<std::string, MavlinkRateLimiter>(message_name, MavlinkRateLimiter(250 * 1000)));
        pthread_rwlock_unlock(&_rate_limiting_rw_lock);
        new_entry.first->second.check(hrt_absolute_time());
    } else if (!entry->second.check(hrt_absolute_time())) {
        return 0;
    }

    char buffer[BUFFER_SIZE];
    strncpy(buffer, message_name, BUFFER_SIZE - 1);
    buffer[BUFFER_SIZE - 1] = 0;
    ssize_t msg_len = strlen(buffer) + 1;
    ssize_t data_length = std::min((size_t) length, BUFFER_SIZE - msg_len);
    memcpy(buffer + msg_len, data, data_length);
    msg_len += data_length;
    //PX4_INFO("send_message(%s, %d, data)", message_name, length);
    ssize_t written = _transport->write_msg(MSG_TYPE_MSG, buffer, msg_len);
    //PX4_INFO("wrote %d of %d", written, msg_len);
    return (written == msg_len) ? 0 : -1;
}

bool UORBCommChannel::thread_start_helper(pthread_t *new_thread, const char *name, int priority, void *(*start_routine) (void *), void *arg) {
    pthread_attr_t thread_attr;
    pthread_attr_init(&thread_attr);

    struct sched_param param;
    (void)pthread_attr_getschedparam(&thread_attr, &param);
    param.sched_priority = priority;
    (void)pthread_attr_setschedparam(&thread_attr, &param);

    pthread_attr_setstacksize(&thread_attr, PX4_STACK_ADJUSTED(2840));
    int error = pthread_create(new_thread, &thread_attr, start_routine, arg);

    pthread_attr_destroy(&thread_attr);

    if (error) {
        PX4_ERR("starting thread %s: %s", name, strerror(error));
    } else {
        pthread_setname_np(*new_thread, name);
        PX4_INFO("thread %s started", name);
    }

    return !error;
}

bool UORBCommChannel::start(int argc, char *argv[]) {
    int ch;
    const char *device_name = "/dev/ttyAMA0";
    uint32_t baudrate = 57600;
    bool serial_transport = true;
    bool time_sync = false;
    bool hardware_control_flow = false;

    _should_exit = false;
    _timesync_should_exit = false;
    _send_with_no_subscriber = false;

#ifdef __PX4_POSIX
    uint16_t local_port = 3030;
    const char *remote_host = "127.0.0.1";
    uint16_t remote_port = 3030;
#endif

    /* don't exit from getopt loop to leave getopt global variables in consistent state,
     * set error flag instead */
    bool err_flag = false;
    int myoptind = 1;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "b:d:u:o:t:shfn", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
        case 'b':
            baudrate = strtoul(myoptarg, nullptr, 10);
            serial_transport = true;
            break;

        case 'd':
            device_name = myoptarg;
            serial_transport = true;
            break;

        case 'f':
            hardware_control_flow = true;
            break;

        case 'h':
            err_flag = true;
            break;

        case 'n':
            _send_with_no_subscriber = true;
            break;

        case 's':
            time_sync = true;
            break;

#ifdef __PX4_POSIX

        case 'u':
            local_port = strtoul(myoptarg, nullptr, 10);
            serial_transport = false;
            break;

        case 'o':
            remote_port = strtoul(myoptarg, nullptr, 10);
            serial_transport = false;
            break;

        case 't':
            remote_host = myoptarg;
            serial_transport = false;
            break;
#else

        case 'u':
        case 'o':
        case 't':
            PX4_ERR("UDP options not supported on this platform");
            err_flag = true;
            break;
#endif

        default:
            err_flag = true;
            break;
        }
    }

    if (err_flag) {
        usage();
        return false;
    }

    if (serial_transport) {
        _transport = new SerialTransport(device_name, baudrate, hardware_control_flow);
    }
#ifdef __PX4_POSIX
    else {
        _transport = new UDPTransport(local_port, remote_host, remote_port);
    }
#endif

    if (_transport->open() < 0) {
        PX4_ERR("channel start(): could not open transport port");
        delete _transport;
        return false;
    }

    sem_init(&_timesync_sem, 0, 0);
    if (!thread_start_helper(&_recv_thread, "uorbcomm.rcv", SCHED_PRIORITY_SLOW_DRIVER, receiver_thread_start, this)) {
        delete _transport;
        return false;
    }

    if (time_sync) {
        if (!thread_start_helper(&_timesync_thread, "uorbcomm.ts", SCHED_PRIORITY_DEFAULT, timesync_thread_start, this)) {
            _should_exit = true;
            pthread_join(_recv_thread, nullptr);
            delete _transport;
            return false;
        }
    } else {
        send_end_ready();
    }

    _send_time_sync = time_sync;

    // wait for the other end to be ready
    PX4_INFO("waiting for other end to be ready");
    sem_wait(&_timesync_sem);
    PX4_INFO("other end is ready");

    return true;
}


void UORBCommChannel::process_timesync(TimeSyncMsg* timeSyncMsg) {

    // catch up with the master time, but never go back in time

    hrt_abstime now = hrt_absolute_time();
    if (timeSyncMsg->master_time > now) {
        hrt_set_absolute_time(timeSyncMsg->master_time);
        PX4_INFO("timesync %" PRIu64 "->%" PRIu64 " (%" PRIu64 ")", now, hrt_absolute_time(), timeSyncMsg->master_time);
    }
}

void *UORBCommChannel::receiver_thread_start(void *)
{
    get_instance()->receiver_thread();

    return 0;
}

void UORBCommChannel::receiver_thread() {
    ssize_t read;
    char buffer[BUFFER_SIZE];
    uint8_t msg_type;

    while (!_should_exit)
    {
        while (0 < (read = _transport->read_msg(buffer, BUFFER_SIZE, &msg_type)))
        {
            //PX4_INFO("transport received %d bytes", read);

            if (msg_type == MSG_TYPE_TIMESYNC) {
                process_timesync((TimeSyncMsg*) buffer);
                continue;
            } else if (msg_type == MSG_TYPE_CHANNEL_END_READY) {
               sem_post(&_timesync_sem);
               //_timesync_should_exit = true;
               continue;
            }

            if (_channel_rx_handler == nullptr) {
                continue; // cannot process this yet
            }

            char *message_name = buffer;
            size_t message_name_length = strlen(message_name);
            uint8_t *msg_data = ((uint8_t *) buffer) + message_name_length + 1;
            int32_t data_length = read - message_name_length - 1;


            switch (msg_type)
            {
                case MSG_TYPE_ADD_SUBSCRIBER:
                {
                    int32_t msg_rate_hz = *((int32_t*) msg_data);
//                    static bool done = false;
//                    //if (std::string(message_name).find("vehicle_control_mode") != string::npos) {
//                    if (!done) {
//                        done = true;
//                        PX4_INFO("received add_subscription(%s) @ %" PRIu64, message_name, hrt_absolute_time());
//                    }
                    pthread_rwlock_wrlock(&_subscribers_rw_lock);
                    _subscribers.insert(message_name);
                    pthread_rwlock_unlock(&_subscribers_rw_lock);
                    _channel_rx_handler->process_add_subscription(message_name, msg_rate_hz);
                }
                    break;
                case MSG_TYPE_REMOVE_SUBSCRIBER:
//                if (std::string(message_name).find("vehicle_control_mode") != string::npos) {
//                    PX4_INFO("received REMOVE_subscription(%s)", message_name);
//                }
                    pthread_rwlock_wrlock(&_subscribers_rw_lock);
                    _subscribers.erase(message_name);
                    pthread_rwlock_unlock(&_subscribers_rw_lock);
                    _channel_rx_handler->process_remove_subscription(message_name);
                    break;
            case MSG_TYPE_ADVERTISE:
                _channel_rx_handler->process_remote_topic(message_name, true);
                break;
            case MSG_TYPE_UNADVERTISE:
                _channel_rx_handler->process_remote_topic(message_name, false);
                break;
                case MSG_TYPE_MSG:
//                    if (std::string(message_name).find("vehicle_control_mode") != string::npos) {
//                        PX4_INFO("invoking  RxHandler->process_received_message(%s, %d, msgData)", message_name, data_length);
//                    }
                    _channel_rx_handler->process_received_message(message_name,
                                                             data_length, msg_data);
                    break;

                default:
                    PX4_WARN("Unexpected message type\n");
                break;
            }
        }

        //usleep(_options.sleep_ms * 1000);
    }

}

void *UORBCommChannel::timesync_thread_start(void *)
{
    get_instance()->timesync_thread();

    return 0;
}

void UORBCommChannel::timesync_thread() {

    static int count = 0;
    // send the current time every 1 second
    while (!_timesync_should_exit) {
        send_end_ready();

        if (count > 0) {
            count--;
        } else if (_send_time_sync) {
            TimeSyncMsg time_sync_msg;
            time_sync_msg.master_time = hrt_absolute_time();
            _transport->write_msg(MSG_TYPE_TIMESYNC, (char*) &time_sync_msg, sizeof(time_sync_msg));
        }
        usleep(1000000);
        //_transport->print_stats();
    }
}

void UORBCommChannel::send_end_ready() {
    char d = 0; // need to send at least one byte
    _transport->write_msg(MSG_TYPE_CHANNEL_END_READY, &d, sizeof(d));
}

} // namespace
