#include <ros/ros.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include "wiringPi.h"
#include "softPwm.h"

#define SERVO 1

class Mission {
protected:
    ros::NodeHandle nh;
    ros::Subscriber perform_sub;
    bool mission_start;

public:
    Mission() {

      perform_sub = nh.subscribe
              ("sk/mission_start",10,&Mission::start_cb,this);
      mission_start=false;
    }
    void start_cb(const std_msgs::Bool::ConstPtr& msg) {
      if(msg->data) {
        mission_start=true;
      }
    }
    bool missionStarted() {
      return mission_start;
    }
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "mission");
    ros::Time::init();
    ros::Rate rate(20.0);
    ros::NodeHandle nh;
    ros::Time mission_start_time;
    float unroll_time;
    float roll_time;
    int stage=0;
    Mission mission;

    nh.param("mission/unroll_time", unroll_time, (float)0.0);
    nh.param("mission/roll_time", roll_time, (float)0.0);

//    if(wiringPiSetup()==-1) {
//      ROS_INFO("Wiring connect fail");
//    }
//    softPwmCreate(SERVO,0,200);

    ROS_INFO("Waiting for mission start...");

//    while(ros::ok() && !mission.missionStarted()) {
//      ros::spinOnce();
//      rate.sleep();
//    }
    stage++;

    mission_start_time=ros::Time::now();
    ROS_INFO("Mission stage 1 : Unroll");

    while(ros::ok()) {
      switch(stage) {
      case 1:
        //softPwmWrite(SERVO,24);
        if(ros::Time::now().toSec()-mission_start_time.toSec()>unroll_time) {
          stage++;
          mission_start_time=ros::Time::now();
          ROS_INFO("Mission stage 2 : Roll");
        }
        break;
      case 2:
        //softPwmWrite(SERVO,5);
        if(ros::Time::now().toSec()-mission_start_time.toSec()>roll_time) {
          stage++;
          ROS_INFO("Mission complete");
        }
        break;
      case 3:
        //softPwmStop(SERVO);
        stage++;
        break;
      }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
