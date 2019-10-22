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

  ros::Subscriber sub_curr_wp;
  ros::Subscriber sub_local_wp;

  ros::Publisher pub_target_wp;

  sk_msgs::WaypointArray local_wp;
  int curr_wp;

  bool received_local_wp;
  bool received_curr_wp;

public:
  TrajectoryPlanner() {
    sub_curr_wp = nh.subscribe(
          "/current_waypoint", 10, &TrajectoryPlanner::currWpCb, this);
    sub_local_wp = nh.subscribe(
          "/local_waypoints", 10, &TrajectoryPlanner::localWpCb, this);

    pub_target_wp = nh.advertise<visualization_msgs::MarkerArray>(
          "/target_waypoint", 10);

    received_local_wp = false;
    received_curr_wp = false;
  }
  bool localWpExist() {
    return received_local_wp;
  }
  bool currWpExist() {
    return received_curr_wp;
  }
  void currWpCb(const std_msgs::Int32::ConstPtr& msg) {
    curr_wp = msg->data;
  }
  void globalWpCb(const sk_msgs::WaypointArray::ConstPtr& msg) {
    if(received_local_wp) return;
    local_wp=*msg;
    local_wp.frame_id = "local_origin";
    received_local_wp = true;
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "global_planner");
  TrajectoryPlanner sc;
  ros::Rate loop_rate(50);

  cout<<"Waiting for waypoints..."<<endl;

  while(ros::ok() && (!sc.localWpExist() || !sc.currWpExist()))
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
