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
#define MISSION_POINT 4

class Mission {
protected:
    ros::NodeHandle nh;
    ros::Subscriber reached_sub;
    ros::Subscriber state_sub;
    ros::Publisher mission_perform_pub;
    bool mission_start;
    std_msgs::Bool mission_perform;
    int wp_seq;


public:
    mavros_msgs::State current_state;
    Mission() {
      reached_sub = nh.subscribe
              ("mavros/mission/reached",100,&Mission::reached_cb,this);
      mission_perform_pub = nh.advertise<std_msgs::Bool>
          ("sk/mission_perform",10);
      state_sub = nh.subscribe<mavros_msgs::State>
                  ("mavros/state", 10, &Mission::state_cb,this);
      mission_start=false;
      mission_perform.data=false;
    }
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }
    void reached_cb(const mavros_msgs::WaypointReached::ConstPtr& msg) {
        wp_seq=msg->wp_seq;
        if(wp_seq==MISSION_POINT) {
            mission_start=true;
        }else mission_start=false;
    }
    bool missionStarted() {
      return mission_start;
    }
    void setMissionPerform() {
      mission_perform.data=true;
    }
    void pubMissionPerform() {
      mission_perform_pub.publish(mission_perform);
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
    float delay;
    int stage=0;
    Mission mission;
    ros::Time perform_time;

    nh.param("mission/unroll_time", unroll_time, (float)0.0);
    nh.param("mission/roll_time", roll_time, (float)0.0);
    nh.param("mission/delay", delay, (float)0.0);

    ROS_INFO("Connecting WiringPi");

    if(wiringPiSetup()==-1) {
      ROS_INFO("Wiring connect fail");
    }


    ROS_INFO("Waiting for mission start");

    while(ros::ok() && !mission.missionStarted()) {
      ros::spinOnce();
      rate.sleep();
      ROS_INFO("connection : %d",mission.current_state.connected);
    }
    stage++;

    softPwmCreate(SERVO,0,200);
    mission_start_time=ros::Time::now();
    ROS_INFO("Mission stage 1 : Unroll");

    while(ros::ok()) {
      switch(stage) {
      case 1:
        softPwmWrite(SERVO,9);
        if(ros::Time::now().toSec()-mission_start_time.toSec()>unroll_time) {
          mission.setMissionPerform();
          perform_time=ros::Time::now();
          stage++;
        }
        break;
    case 2:
        softPwmWrite(SERVO,14);
        if(ros::Time::now().toSec()-perform_time.toSec()>delay) {
            mission_start_time=ros::Time::now();
            ROS_INFO("Mission stage 2 : Roll");
            stage++;
        }
    break;
      case 3:
        
        softPwmWrite(SERVO,21);
        if(ros::Time::now().toSec()-mission_start_time.toSec()>roll_time) {
          stage++;
          ROS_INFO("Mission complete");
        }else if(ros::Time::now().toSec()-mission_start_time.toSec()>delay) {
            
        }
        break;
      case 4:
        softPwmWrite(SERVO,14);
        break;
      default:
        softPwmWrite(SERVO,14);
      }

        mission.pubMissionPerform();
        ros::spinOnce();
        rate.sleep();
    }

    softPwmWrite(SERVO,14);
    softPwmStop(SERVO);

    return 0;
}
