#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include <std_msgs/Float64.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <ros/ros.h>
#include <string>

#include <cstdlib>
#include "enu_conversion.h"

class VirtualObstacle {
private:
    ros::NodeHandle nh;

    ros::Subscriber sub_x_diff;
    ros::Subscriber sub_y_diff;

    ros::Publisher pub_pointcloud;

    geometry_msgs::Point obstacle_point_global;
    geometry_msgs::Point obstacle_point_local;
    geometry_msgs::Point obstacle_point;

    float x_diff;
    float y_diff;



public:
    VirtualObstacle() {

        sub_x_diff = nh.subscribe(
                    "/x_diff", 1, &VirtualObstacle::xDiffCb, this);
        sub_y_diff = nh.subscribe(
                    "/y_diff", 1, &VirtualObstacle::xDiffCb, this);

        global_obstacle_point.x = 127.07576122518;
        global_obstacle_point.y = 37.54351992928;
        global_obstacle_point.z = 10.0;

        ENU_Conversion::enuConversion(obstacle_point_global, &obstacle_point_local);
    }
    void xDiffCb(const std_msgs::Float64::ConstPtr& msg) {
        x_diff = msg->data;
    }
    void yDiffCb(const std_msgs::Float64::ConstPtr& msg) {
        y_diff = msg->data;
    }
    void generateObstacle() {

    }
    void moveDiff() {
        obstacle_point.x = obstacle_point_local.x + x_diff;
        obstacle_point.y = obstacle_point_local.y + y_diff;
        ROS_INFO("%f, %f",obstacle_point.x,obstacle_point.y);
    }
    void markObstacle() {

    }
    void publish() {

    }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "virtual_obstacle");

  VirtualObstacle vo;

  ros::Rate loop_rate(50);

  while (ros::ok()) {

    vo.moveDiff();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
