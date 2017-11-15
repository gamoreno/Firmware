#pragma once

#include <modules/uORB/uORBCommunicator.hpp>
#include "channel_transport.h"
#include <pthread.h>
#include <semaphore.h>
#include <set>
#include <string>
#include <drivers/drv_hrt.h>

namespace uORB
{

class UORBCommChannel : public uORBCommunicator::IChannel
{
public:

    static UORBCommChannel* get_instance();
    virtual ~UORBCommChannel();
    virtual int16_t topic_advertised(const char *messageName) override;
    virtual int16_t topic_unadvertised(const char *messageName); // commented out in uORBCommunicator::IChannel
    virtual int16_t add_subscription(const char *messageName, int32_t msgRateInHz) override;
    virtual int16_t remove_subscription(const char *messageName) override;
    virtual int16_t register_handler(uORBCommunicator::IChannelRxHandler *handler) override;

    virtual int16_t send_message(const char *messageName, int32_t length, uint8_t *data) override;

    bool start(int argc, char *argv[]);
    void stop();

protected:
    static const size_t BUFFER_SIZE = 1024;

    static UORBCommChannel* _instance;
    Transport* _transport;
    uORBCommunicator::IChannelRxHandler *_channel_rx_handler;
    pthread_t _recv_thread;
    pthread_t _timesync_thread;
    bool _timesync_done;
    sem_t _timesync_sem;
    bool _should_exit;
    bool _timesync_should_exit;
    std::set<std::string> _subscribers;
    pthread_rwlock_t _subscribers_rw_lock;


    static const uint8_t MSG_TYPE_ADD_SUBSCRIBER = 1;
    static const uint8_t MSG_TYPE_REMOVE_SUBSCRIBER = 2;
    static const uint8_t MSG_TYPE_ADVERTISE = 3;
    static const uint8_t MSG_TYPE_UNADVERTISE = 4;
    static const uint8_t MSG_TYPE_MSG = 5;
    static const uint8_t MSG_TYPE_TIMESYNC = 6;
    static const uint8_t MSG_TYPE_TIMESYNC_ACK = 7;

    struct TimeSyncMsg {
        hrt_abstime master_time;
    };

    UORBCommChannel(); // enforce singleton
    void process_timesync(TimeSyncMsg* timeSyncMsg);
    static bool thread_start_helper(pthread_t *new_thread, const char *name, int priority, void *(*start_routine) (void *));

    void receiver_thread();
    static void *receiver_thread_start(void *);
    void timesync_thread();
    static void *timesync_thread_start(void *);
};

} // namespace

