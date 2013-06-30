#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <vector>
#include <fstream>
#include <android/log.h>
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/CameraInfo.h"


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
    topics.push_back(std::string("images"));
    topics.push_back(std::string("camera_info"));
    topics.push_back(std::string("compressed_images"));
    topics.push_back(std::string("imu"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
        if (s != NULL)
            log("String this should read 'foo': %s", s->data.c_str());

        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        if (i != NULL)
            log("Int32 this should read 42: %d", i->data);

        sensor_msgs::Image::ConstPtr im = m.instantiate<sensor_msgs::Image>();
        if (im != NULL)
            log("Image this should read 640x480: %dx%d", im->width, im->height);

        sensor_msgs::CameraInfo::ConstPtr ci = m.instantiate<sensor_msgs::CameraInfo>();
        if (ci != NULL)
            log("CameraInfo this should read 640x480: %dx%d", ci->width, ci->height);

        sensor_msgs::CompressedImage::ConstPtr cim = m.instantiate<sensor_msgs::CompressedImage>();
        if (cim != NULL)
            log("CompressedImage this should read png: %s", cim->format.c_str());

        sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
        if (imu != NULL)
          log("Imu this should read 1 2 3 4 5 6 7 8 9: %g %g %g %g %g %g %g %g %g", 
              imu->orientation_covariance[0],
              imu->orientation_covariance[1],
              imu->orientation_covariance[2],
              imu->orientation_covariance[3],
              imu->orientation_covariance[4],
              imu->orientation_covariance[5],
              imu->orientation_covariance[6],
              imu->orientation_covariance[7],
              imu->orientation_covariance[8]
              
              );
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

    sensor_msgs::Image image;
    image.height = 480;
    image.width = 640;

    sensor_msgs::CameraInfo camera_info;
    image.height = 480;
    image.width = 640;
    
    sensor_msgs::CompressedImage cimage;
    cimage.format = "foobar";

    std::ifstream image_file;
    image_file.open("/sdcard/ROS.png", std::ios::binary);
    int file_length;
    image_file.seekg(0, std::ios::end);
    file_length = image_file.tellg();
    image_file.seekg(0, std::ios::beg);
    
    if (file_length > 0)
    {
      log("Using real compressed image from /sdcard/ROS.png");
      cimage.data.reserve(file_length);
      cimage.data.assign(std::istreambuf_iterator<char>(image_file),
                         std::istreambuf_iterator<char>());
    }
    else
      log("Didn't find file /sdcard/ROS.png");


    sensor_msgs::Imu imu;
    for (uint i = 0; i < 9; i++)
    {
      imu.orientation_covariance[i] = (double)i+1.0;
    }

    log("writing stuff into bag");
    try {
        bag.write("chatter", ros::Time::now(), str);
        bag.write("numbers", ros::Time::now(), i);
        bag.write("images", ros::Time::now(), image);
        bag.write("images/compressed", ros::Time::now(), cimage);
        bag.write("camera_info", ros::Time::now(), camera_info);
        bag.write("imu", ros::Time::now(), imu);

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
