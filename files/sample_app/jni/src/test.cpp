#include <stdarg.h>
#include <stdio.h>
#include <sstream>
#include <map>
#include <string.h>
#include <errno.h>
#include <vector>
#include <set>
#include <fstream>
#include <android/log.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <android_native_app_glue.h>

void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "ROSCPP_NDK_EXAMPLE", msg, args);
    va_end(args);
}


// from android samples
/* return current time in seconds */
static double now(void) {

  struct timespec res;
  clock_gettime(CLOCK_REALTIME, &res);
  return res.tv_sec + (double) res.tv_nsec / 1e9;

}


#define LASTERR strerror(errno)

void chatterCallback(const std_msgs::StringConstPtr& msg){
    log(msg->data.c_str());
}

void android_main(android_app *papp) {
    // Make sure glue isn't stripped
    app_dummy();

    int argc = 3;
    char *argv[] = {"nothing_important" , "__master:=http://10.52.90.103:11311", "__ip:=10.52.90.246"};
    //strcpy(argv[0], 'nothing_important');
    //argv[1] = '__master:=http://10.52.90.103:11311';
    //argv[2] = '__ip:=10.52.90.246';
    //argv[3] = '__hostname:=10.52.90.246';
    log("GOING TO ROS INIT~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    for(int i = 0; i < argc; i++){
        log(argv[i]);
    }
    ros::init(argc, &argv[0], "android_ndk_native_cpp", ros::init_options::NoRosout);
    log("GOING TO NODEHANDLE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    std::string master_uri = ros::master::getURI();
    if(ros::master::check()){
        log("ROS MASTER IS UP!!!!!!!!!>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    } else {
        log("NO ROS MASTER.______________________________________________________________________________________________");
    }
    log(master_uri.c_str());
    ros::NodeHandle n;
    log("GOING TO PUBLISHER");
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Subscriber sub = n.subscribe("a_chatter", 1000, chatterCallback);
    ros::Rate loop_rate(5);

    while(ros::ok() && !papp->destroyRequested){
            std_msgs::String msg;
            msg.data = "hello world from android ndk";
            chatter_pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
    }


}
