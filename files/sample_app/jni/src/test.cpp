#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <vector>
#include <android/log.h>
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <boost/foreach.hpp>

#include <android_native_app_glue.h>

void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "RBA", msg, args);
    va_end(args);
}

#define LASTERR strerror(errno)
#define foreach BOOST_FOREACH

void readbag() {
    rosbag::Bag bag;
    try {
    bag.open("/sdcard/test.bag", rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        log("could not open bag file for reading: %s, %s", e.what(), LASTERR);
        return;
    }

    std::vector<std::string> topics;
    topics.push_back(std::string("chatter"));
    topics.push_back(std::string("numbers"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
        if (s != NULL)
            log("this should read 'foo': %s", s->data.c_str());

        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        if (i != NULL)
            log("this should read 42: %d", i->data);
    }

    bag.close();
}

void testbag() {
    rosbag::Bag bag;

    log("initializing time");
    ros::Time::init();

    log("opening bag");
    try {
        bag.open("/sdcard/test.bag", rosbag::bagmode::Write);
    } catch (rosbag::BagException e) {
        log("could not open bag file for writing: %s, %s", e.what(), LASTERR);
        return;
    }

    std_msgs::String str;
    str.data = std::string("foo");

    std_msgs::Int32 i;
    i.data = 42;

    log("writing stuff into bag");
    try {
        bag.write("chatter", ros::Time::now(), str);
        bag.write("numbers", ros::Time::now(), i);
    } catch (const std::exception &e) {
        log("Oops! could not write to bag: %s, %s", e.what(), strerror(errno));
        return;
    }

    log("closing bag");
    bag.close();
}

void ev_loop(android_app *papp) {
    int32_t lr;
    int32_t le;
    bool first = true;
    bool second = false;
    android_poll_source *ps;

    app_dummy();

    log("starting event loop");

    while (true) {
        lr = ALooper_pollAll(-1, NULL, &le, (void **) &ps);
        if (lr < 0) {
            break;
        }
        if (ps) {
            log("event received");
            if (first) {
                log("ready? launching rosbag write!");
                testbag();
                first = false;
                second = true;
            }
            if (second) {
                log("ready? launching rosbag read!");
                readbag();
                second = false;
            }
            ps->process(papp, ps);
        }
        if (papp->destroyRequested) {
            log("quitting event loop");
            return;
        }
    }
}

void android_main(android_app *papp) {
    ev_loop(papp);
}
