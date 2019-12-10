#include "lidar_detector.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "laserScan_to_pointcloud");
    lidar_detector lidar_detector;

    lidar_detector.mainLoop();

    return 0;
}
