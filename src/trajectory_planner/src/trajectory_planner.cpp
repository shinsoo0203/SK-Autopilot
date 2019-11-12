#include <ros/ros.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include "sk_msgs/Waypoint.h"
#include "sk_msgs/WaypointArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;

class TrajectoryPlanner {
private:
  ros::NodeHandle nh;

  ros::Subscriber curr_wp_sub;
  ros::Subscriber local_wp_sub;

  ros::Publisher target_wp_pub;

  sk_msgs::WaypointArray local_wp;
  int curr_wp;

  bool received_local_wp;
  bool received_curr_wp;
  std_msgs::Int32 target_wp;
  int look_ahead;

public:
  TrajectoryPlanner() {
    curr_wp_sub = nh.subscribe(
          "/current_waypoint", 10, &TrajectoryPlanner::currWpCb, this);
    local_wp_sub = nh.subscribe(
          "/local_waypoints", 10, &TrajectoryPlanner::localWpCb, this);

    target_wp_pub = nh.advertise<std_msgs::Int32>(
          "/target_waypoint", 10);

    received_local_wp = false;
    received_curr_wp = false;
    target_wp.data = 0;
    look_ahead = 30;
  }
  bool localWpExist() {
    return received_local_wp;
  }
  bool currWpExist() {
    return received_curr_wp;
  }
  void currWpCb(const std_msgs::Int32::ConstPtr& msg) {
    curr_wp = msg->data;
    received_curr_wp = true;
  }
  void localWpCb(const sk_msgs::WaypointArray::ConstPtr& msg) {
    if(received_local_wp) return;
    local_wp=*msg;
    local_wp.frame_id = "local_origin";
    received_local_wp = true;
  }
  void setTargetWp() {
    target_wp.data = curr_wp + look_ahead;
    target_wp_pub.publish(target_wp);
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "trajectory_planner");
  TrajectoryPlanner tp;
  ros::Rate loop_rate(50);

  cout<<"Waiting for waypoints..."<<endl;

  while(ros::ok() && (!tp.localWpExist() || !tp.currWpExist()))
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  cout<<"Received waypoints."<<endl;

  while (ros::ok()) {
    tp.setTargetWp();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
