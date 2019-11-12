#ifndef ENU_CONVERSION_H
#define ENU_CONVERSION_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sk_msgs/WaypointArray.h>

#define PI 3.14159265358979323846

class ENU_Conversion {
  // ENU conversion
private:
  const static double m_origin_lat_deg;
  const static double m_origin_lat_rad;

  const static double m_origin_lon_deg;
  const static double m_origin_lon_rad;

  const static double m_a;       // semi-major axis [m]
  const static double m_b;  // semi-minor axis [m]

public:
  static void enuConversion(sk_msgs::WaypointArray globalArr, sk_msgs::WaypointArray* localArr) {

    for (int i_point = 0; i_point < globalArr.wp.size(); i_point++)
    {
      geometry_msgs::Point local_point;
      sk_msgs::Waypoint local_wp;

      enuConversion(globalArr.wp[i_point].point, &local_point);

      local_wp.point = local_point;
      localArr->wp.push_back(local_wp);
      ROS_INFO("east : %f, north : %f", local_point.x, local_point.y);
      //ROS_INFO("longitude: %f, latitude: %f", globalArr.point[i_point].x, globalArr.point[i_point].y);
    }
  }

  static void enuConversion(geometry_msgs::Point global_pose, geometry_msgs::Point* local_pose) {

    double temp_lon;
    double temp_lat;

    temp_lon = global_pose.x * PI / 180.0;
    temp_lat = global_pose.y * PI / 180.0;

    local_pose->x = (temp_lon - m_origin_lon_rad)*(NormalRadius(m_a, m_b, temp_lat)*cos(temp_lat));
    local_pose->y = (temp_lat - m_origin_lat_rad)*MeridionalRadius(m_a, m_b, temp_lat);
    local_pose->z = global_pose.z;

    ROS_INFO("east : %f, north : %f", local_pose->x, local_pose->y);
    //ROS_INFO("longitude: %f, latitude: %f", globalArr.point[i_point].x, globalArr.point[i_point].y);
  }

  static double MeridionalRadius(double a, double b, double lat){
    return pow(a*b, 2) / sqrt( pow((pow( a*cos(lat), 2) + pow( b*sin(lat), 2 )), 3));
  }

  static double NormalRadius(double a, double b, double lat){
    return (a*a) / sqrt(pow( a*cos(lat), 2 ) + pow( b*sin(lat), 2));
  }
};

const double ENU_Conversion::m_origin_lat_deg = 37.539873;
const double ENU_Conversion::m_origin_lat_rad = ENU_Conversion::m_origin_lat_deg * PI/180.0;
const double ENU_Conversion::m_origin_lon_deg = 127.070689;
const double ENU_Conversion::m_origin_lon_rad = ENU_Conversion::m_origin_lon_deg * PI/180.0;
const double ENU_Conversion::m_a = 6378137.0;
const double ENU_Conversion::m_b = 6356752.314245;

#endif // ENU_CONVERSION_H
