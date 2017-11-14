#include <px4_module.h>
#include <uORB/uORBManager.hpp>
#include "uorb_comm_channel.h"
#include <px4_log.h>

extern "C" { __EXPORT int uorbcomm_main(int argc, char *argv[]); }

int
uorbcomm_main(int argc, char *argv[])
{
    uORB::UORBCommChannel *channel = uORB::UORBCommChannel::get_instance();
    if (channel == uORB::Manager::get_instance()->get_uorb_communicator()) {
        PX4_INFO("already started");
    } else if (channel->start(argc, argv)) {
        PX4_INFO("registering communicator channel");
        uORB::Manager::get_instance()->set_uorb_communicator(channel);
    } else {
        return PX4_ERROR;
    }

    return PX4_OK;
}
