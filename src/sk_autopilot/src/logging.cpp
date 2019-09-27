#include <ros/ros.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include "sk_autopilot/state.h"
#include <iomanip>

class Logging {
protected:
    ros::NodeHandle nh;
    ros::Subscriber state_sub;
    ros::Subscriber wp_sub;
    ros::Subscriber time_sub;
    ros::Subscriber gps_sub;
    ros::Publisher state_pub;

    sk_autopilot::state state;

public:
    Logging() {
        state_sub = nh.subscribe
                ("mavros/state", 10, &Logging::state_cb,this);
        wp_sub = nh.subscribe
                ("mavros/mission/reached",100,&Logging::reached_cb,this);
        time_sub = nh.subscribe
            ("mavros/time_reference",10,&Logging::time_cb,this);
        gps_sub = nh.subscribe
            ("mavros/global_position/global",10,&Logging::gps_cb,this);
        state_pub = nh.advertise<sk_autopilot::state>("sk/state",10);
    }
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
      if(msg->mode == "AUTO.MISSION") {
        state.autoflight_mode = true;
      }else {
        state.autoflight_mode = false;
      }
    }
    void reached_cb(const mavros_msgs::WaypointReached::ConstPtr& msg) {
      switch(msg->wp_seq) {
      case 0: case 1: case 2:
        state.waypoint=1;
        break;
      case 3:
        state.waypoint=2;
        break;
      case 4: case 5:
        state.waypoint=3;
        break;
      case 6: case 7:
        state.waypoint=0;
        break;
      }
    }
    void time_cb(const sensor_msgs::TimeReference::ConstPtr& msg) {
      state.gps_time = msg->time_ref;
    }
    void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
//      state.latitude = int(msg->latitude / 0.000001) * 1000000;
//      state.longitude = int(msg->longitude / 0.000001) * 1000000;
//      state.altitude = int(msg->altitude / 0.1) * 10;
      state.latitude = msg->latitude;
      state.longitude = msg->longitude;
      state.altitude = msg->altitude;
    }
    void pub() {
      state_pub.publish(state);
    }
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "logging");
    ros::Time::init();
    ros::Rate rate(1);
    Logging logging;
    while(ros::ok()) {
        logging.pub();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
