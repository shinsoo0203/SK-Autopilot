#include <ros/ros.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include "sk_msgs/Waypoint.h"
#include "sk_msgs/WaypointArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "enu_conversion.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;

class GlobalPlanner {
private:
  ros::NodeHandle nh;

  ros::Subscriber sub_global_waypoints;
  ros::Subscriber sub_global_pose;
  ros::Subscriber sub_local_pose;

  ros::Publisher pub_marker_array;
  ros::Publisher pub_pose_marker;
  ros::Publisher pub_curr_wp;
  ros::Publisher pub_local_wp;

  sk_msgs::WaypointArray global_wp;
  sk_msgs::WaypointArray local_wp;
  std_msgs::Int32 curr_wp;
  geometry_msgs::Point global_pose;
  geometry_msgs::PoseStamped local_pose;
  geometry_msgs::Point tmp_local_pose;
  visualization_msgs::Marker pose_marker;
  tf::TransformBroadcaster tf_broadcaster;

  bool received_global_wp;
  bool received_global_pose;
  bool received_local_pose;

public:
  GlobalPlanner() {
    sub_global_waypoints = nh.subscribe(
          "/global_waypoints_latlon", 10, &GlobalPlanner::globalWpCb, this);
    sub_global_pose = nh.subscribe(
          "/mavros/global_position/global", 10, &GlobalPlanner::globalPoseCb, this);
    sub_local_pose = nh.subscribe(
          "/mavros/local_position/pose", 10, &GlobalPlanner::localPoseCb, this);

    pub_marker_array = nh.advertise<visualization_msgs::MarkerArray>(
          "/local_waypoints_marker", 10);
    pub_pose_marker = nh.advertise<visualization_msgs::Marker>(
          "/pose_marker", 10);
    pub_curr_wp = nh.advertise<std_msgs::Int32>(
          "/current_waypoint", 10);
    pub_local_wp = nh.advertise<sk_msgs::WaypointArray>(
          "/local_waypoints", 10);

    received_global_wp = false;
    received_global_pose = false;
    received_local_pose = false;
  }
  bool wpExist() {
    return received_global_wp;
  }
  bool poseExist() {
    return (received_global_pose && received_local_pose);
  }
  void initCurrWp() {
    int min_idx=0;
    double min_dist=500.0;
    double dist;
    for(int i=0;i<local_wp.wp.size();i++) {
      dist=sqrt(pow(local_wp.wp[i].point.x-local_pose.pose.position.x,2)+pow(local_wp.wp[i].point.y-local_pose.pose.position.y,2));
      if(dist<min_dist) {
        min_dist=dist;
        min_idx=i;
      }
    }
    curr_wp.data=min_idx;
    if(min_dist>=100.0) {
      cout<<"Too far from the point!!"<<endl;
    }else {
      cout<<"curr_wp : "<<curr_wp.data<<endl;
      pub_curr_wp.publish(curr_wp);
    }
  }
  void findCurrWp() {
      int min_idx=0;
      double min_dist=100.0;
      double dist;
      for(int i=0;i<local_wp.wp.size();i++) {
          dist=sqrt(pow(local_wp.wp[i].point.x-local_pose.pose.position.x,2)
                    +pow(local_wp.wp[i].point.y-local_pose.pose.position.y,2));
          if(dist<min_dist) {
              min_dist=dist;
              min_idx=i;
          }
      }
      if(min_dist>=20.0) {
          initCurrWp();
          return;
      }
      curr_wp.data=min_idx;
      cout<<"curr_wp : "<<curr_wp.data<<endl;
      pub_curr_wp.publish(curr_wp);
      pub_local_wp.publish(global_wp);

  }
  void globalWpCb(const sk_msgs::WaypointArray::ConstPtr& msg) {
    if(received_global_wp || !received_local_pose) return;
    global_wp=*msg;
    ENU_Conversion::enuConversion(global_wp, &local_wp);
    local_wp.frame_id = "local_origin";
    moveLocalWp();
    received_global_wp = true;
  }
  void globalPoseCb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    if(received_global_wp) return;
    global_pose.x = msg->longitude;
    global_pose.y = msg->latitude;
    ENU_Conversion::enuConversion(global_pose, &tmp_local_pose);
    received_global_pose = true;
  }
  void moveLocalWp() {
    float x_diff = local_pose.pose.position.x - tmp_local_pose.x;
    float y_diff = local_pose.pose.position.y - tmp_local_pose.y;

    for(int i=0;i<local_wp.wp.size();i++) {
      local_wp.wp[i].point.x += x_diff;
      local_wp.wp[i].point.y += y_diff;
      cout<<local_wp.wp[i].point.x<<", "<<local_wp.wp[i].point.y<<endl;
    }

    cout<<"Successfully moved local waypoints."<<endl;
  }
  void localPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    local_pose = *msg;
    tf::Quaternion q;
    q.setRPY(local_pose.pose.orientation.x, local_pose.pose.orientation.y, local_pose.pose.orientation.z);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z));
    transform.setRotation(q);
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_origin", "fcu"));

    received_local_pose = true;
  }
  void markPose() {

    int id = 0;
    pose_marker.header.frame_id = "local_origin";
    pose_marker.header.stamp = ros::Time::now();

    pose_marker.type = visualization_msgs::Marker::POINTS;
    pose_marker.action = visualization_msgs::Marker::ADD;

    pose_marker.id = 0;
    pose_marker.ns = "0";
    pose_marker.color.r = 0.5f;
    pose_marker.color.g = 0.0f;
    pose_marker.color.b = 0.0f;
    pose_marker.color.a = 0.7;
    pose_marker.scale.x = 0.2;
    pose_marker.lifetime = ros::Duration();

    geometry_msgs::Point currPoint;
    currPoint.x = local_pose.pose.position.x;
    currPoint.y = local_pose.pose.position.y;
    currPoint.z = local_pose.pose.position.z;
    pose_marker.points.push_back(currPoint);

    pub_pose_marker.publish(pose_marker);
  }
  void markOsmWp() {

    visualization_msgs::MarkerArray markerArray;

    int id = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "local_origin";
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.id = 0;
    marker.ns = "0";
    marker.color.r = 0.9f;
    marker.color.g = 0.9f;
    marker.color.b = 0.9f;
    marker.color.a = 0.7;
    marker.scale.x = 0.5;
    marker.lifetime = ros::Duration(0.1);

    geometry_msgs::Point prevPoint;
    bool first = true;

    for (int i_point = 0; i_point < local_wp.wp.size(); i_point+=2) {
      geometry_msgs::Point currPoint;
      currPoint.x = local_wp.wp[i_point].point.x;
      currPoint.y = local_wp.wp[i_point].point.y;
      currPoint.z = 10.0;

      if (first == true) {
        first = false;
      } else {
        marker.points.push_back(prevPoint);
        marker.points.push_back(currPoint);
      }
      prevPoint = currPoint;
    }
    markerArray.markers.push_back(marker);
    marker.points.clear();
    marker.id = id++;

    pub_marker_array.publish(markerArray);
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "global_planner");
  GlobalPlanner gp;
  ros::Rate loop_rate(50);

  cout<<"Waiting for global waypoints and pose..."<<endl;

  while(ros::ok() && (!gp.wpExist() || !gp.poseExist()))
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  gp.initCurrWp();

  while (ros::ok()) {
    gp.markOsmWp();
    gp.markPose();
    gp.findCurrWp();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
