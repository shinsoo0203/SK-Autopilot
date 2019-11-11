#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>
#include "math.h"
#include "sk_msgs/Waypoint.h"
#include "sk_msgs/WaypointArray.h"

class SetpointCommander {
private:
  ros::NodeHandle nh;

  ros::Subscriber state_sub;
  ros::Subscriber target_wp_sub;
  ros::Subscriber local_wp_sub;

  ros::Publisher local_pos_pub;

  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;

  double wn;
  bool received_local_wp;

  mavros_msgs::State current_state;
  ros::Time last_request;
  mavros_msgs::SetMode offb_set_mode;
  mavros_msgs::CommandBool arm_cmd;
  geometry_msgs::PoseStamped pose;
  sk_msgs::WaypointArray local_wp;
  int target_wp;

public:
  SetpointCommander() {
    state_sub = nh.subscribe<mavros_msgs::State>
          ("mavros/state", 10, &SetpointCommander::stateCb, this);
    target_wp_sub = nh.subscribe<std_msgs::Int32>
        ("/target_waypoint", 10, &SetpointCommander::targetWpCb, this);
    local_wp_sub = nh.subscribe(
          "/local_waypoints", 10, &SetpointCommander::localWpCb, this);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
          ("mavros/setpoint_position/local", 10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
          ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
          ("mavros/set_mode");

    last_request = ros::Time::now();
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
    target_wp = 0;
    received_local_wp = false;
  }
  bool localWpExist() {
    return received_local_wp;
  }
  bool connected() {
    return current_state.connected;
  }
  void pubInitialPose() {
    ros::Rate rate(20.0);
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
  }
  void stateCb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
  }
  void targetWpCb(const std_msgs::Int32::ConstPtr& msg) {
    target_wp = msg->data;
  }
  void localWpCb(const sk_msgs::WaypointArray::ConstPtr& msg) {
    if(received_local_wp) return;
    local_wp=*msg;
    local_wp.frame_id = "local_origin";
    received_local_wp = true;
  }
  void prevSetting() {
    if( current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0))){
      if( set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent){
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if( !current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(5.0))){
        if( arming_client.call(arm_cmd) &&
            arm_cmd.response.success){
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
  }
  void setpoint() {
    pose.pose.position.x = local_wp.wp[target_wp].point.x;
    pose.pose.position.y = local_wp.wp[target_wp].point.y;
    pose.pose.position.z = local_wp.wp[target_wp].point.z;

    local_pos_pub.publish(pose);
  }
};





int main(int argc, char **argv)
{
  ros::init(argc, argv, "setpoint_commander");
  SetpointCommander sc;

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // wait for FCU connection
  while(ros::ok() && (sc.connected() || !sc.localWpExist())){
    ros::spinOnce();
    rate.sleep();
  }

  for(int i = 100; ros::ok() && i > 0; --i){
    sc.setpoint();
    ros::spinOnce();
    rate.sleep();
  }

  while(ros::ok()){
    sc.prevSetting();
    sc.setpoint();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
