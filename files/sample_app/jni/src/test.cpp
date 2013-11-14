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
#include <amcl/amcl_node.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <android_native_app_glue.h>

int loop_count_ = 0;
ros::Publisher chatter_pub;

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
            loop_count_++;
            std_msgs::String msgo;
                std::stringstream ss;
               ss << "hello world from android ndk " << loop_count_;
               msgo.data = ss.str();
            chatter_pub.publish(msgo);
    //log(msg->data.c_str());
}

void android_main(android_app *papp) {
    // Make sure glue isn't stripped
    app_dummy();

    int argc = 3;
    char *argv[] = {"nothing_important" , "__master:=http://192.168.1.238:11311", "__ip:=192.168.1.221"};
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
    chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Subscriber sub = n.subscribe("a_chatter", 1000, chatterCallback);
    log("ADDING AMCL");
    AmclNode node;
    ros::WallRate loop_rate(100);

    

    while(ros::ok() && !papp->destroyRequested){

            ros::spinOnce();
            loop_rate.sleep();
    }


}
