#include <ros/ros.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>

class WaypointGenerator {
protected:
    ros::NodeHandle m_rosNodeHandler;
    ros::Publisher m_waypointsPub;
    ros::Publisher m_rosPubLocalPose;
    ros::Subscriber global_pos_sub;
    mavros_msgs::WaypointList global_waypoints;
    mavros_msgs::WaypointList local_waypoints;

    double m_pi = 3.14159265358979323846;

    double m_origin_lat_deg = 37.544156;
    double m_origin_lat_rad = m_origin_lat_deg * m_pi/180.0;

    double m_origin_lon_deg = 127.078417;
    double m_origin_lon_rad = m_origin_lon_deg * m_pi/180.0;

    double m_a = 6378137.0;       // semi-major axis [m]
    double m_b = 6356752.314245;  // semi-minor axis [m]

    geometry_msgs::Point checkedPoint; //rviz
    ros::Publisher visual_pub; //visual point

    

public:
    WaypointGenerator()
    {
        m_waypointsPub=m_rosNodeHandler.advertise<mavros_msgs::WaypointList>("sk/local_waypoints",1000);
        visual_pub=m_rosNodeHandler.advertise<visualization_msgs::Marker>("sk/check_point", 1);
        global_pos_sub = m_rosNodeHandler.subscribe
                ("/mavros/global_position/raw/fix",1000,&WaypointGenerator::pos_cb,this);
        m_rosPubLocalPose = m_rosNodeHandler.advertise<geometry_msgs::PoseStamped>("sk/local_pose",1000);
        
        mavros_msgs::Waypoint wp;

        m_rosNodeHandler.param("waypoint_generator/wp0_lat", wp.x_lat, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp0_lon", wp.y_long, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp0_alt", wp.z_alt, 0.0);
        //m_rosNodeHandler.param("waypoint_generator/wp1_velocity", wp.velocity, 0.0);
        global_waypoints.waypoints.push_back(wp);

        m_rosNodeHandler.param("waypoint_generator/wp1_lat", wp.x_lat, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp1_lon", wp.y_long, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp1_alt", wp.z_alt, 0.0);
        //m_rosNodeHandler.param("waypoint_generator/wp1_velocity", wp.velocity, 0.0);
        global_waypoints.waypoints.push_back(wp);

        m_rosNodeHandler.param("waypoint_generator/wp2_lat", wp.x_lat, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp2_lon", wp.y_long, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp2_alt", wp.z_alt, 0.0);
        //m_rosNodeHandler.param("waypoint_generator/wp2_velocity", wp.velocity, 0.0);
        global_waypoints.waypoints.push_back(wp);

        m_rosNodeHandler.param("waypoint_generator/wp3_lat", wp.x_lat, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp3_lon", wp.y_long, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp3_alt", wp.z_alt, 0.0);
        //m_rosNodeHandler.param("waypoint_generator/wp3_velocity", wp.velocity, 0.0);
        global_waypoints.waypoints.push_back(wp);

        m_rosNodeHandler.param("waypoint_generator/wp4_lat", wp.x_lat, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp4_lon", wp.y_long, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp4_alt", wp.z_alt, 0.0);
        //m_rosNodeHandler.param("waypoint_generator/wp4_velocity", wp.velocity, 0.0);
        global_waypoints.waypoints.push_back(wp);

        m_rosNodeHandler.param("waypoint_generator/wp0_lat", wp.x_lat, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp0_lon", wp.y_long, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp0_alt", wp.z_alt, 0.0);
        //m_rosNodeHandler.param("waypoint_generator/wp1_velocity", wp.velocity, 0.0);
        global_waypoints.waypoints.push_back(wp);

        m_rosNodeHandler.param("waypoint_generator/wp0_lat", wp.x_lat, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp0_lon", wp.y_long, 0.0);
        wp.z_alt=0.0;
        //m_rosNodeHandler.param("waypoint_generator/wp1_velocity", wp.velocity, 0.0);
        global_waypoints.waypoints.push_back(wp);
    }
    void pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        double dLon_raw = 0.0;
        double dLat_raw = 0.0;

        double dLon_deg = 0.0;
        double dLat_deg = 0.0;
        double dLon_rad = 0.0;
        double dLat_rad = 0.0;

        double dEast_m = 0.0;
        double dNorth_m = 0.0;


        dLon_raw = msg->longitude;
        dLat_raw = msg->latitude;

        dLon_deg = ((double)dLon_raw)/10000000.0;
        dLon_rad = dLon_deg*m_pi/180.0;

        dLat_deg = ((double)dLat_raw)/10000000.0;
        dLat_rad = dLat_deg*m_pi/180.0;

        // LLH to ENU conversion
        dEast_m  = (dLon_rad - m_origin_lon_rad)*(NormalRadius(m_a, m_b, dLat_rad)*cos(dLat_rad));
        dNorth_m = (dLat_rad - m_origin_lat_rad)*MeridionalRadius(m_a, m_b, dLat_rad);
        //ROS_INFO("East : %f, North : %f", dEast_m, dNorth_m);

        // publish poseStamped msg
        geometry_msgs::PoseStamped local_pose;

        local_pose.header.stamp = ros::Time::now();
        local_pose.header.frame_id = "/world";
        local_pose.pose.position.x = dEast_m;
        local_pose.pose.position.y = dNorth_m;
        local_pose.pose.position.z = msg->altitude;

        m_rosPubLocalPose.publish(local_pose);
    }
    void enuConversion() {

      double temp_lon;
      double temp_lat;

        for (int i_point = 0; i_point < global_waypoints.waypoints.size(); i_point++)
        {
          mavros_msgs::Waypoint local_point;
          temp_lon = global_waypoints.waypoints.at(i_point).y_long * m_pi / 180.0;
          temp_lat = global_waypoints.waypoints.at(i_point).x_lat * m_pi / 180.0;

          local_point.x_lat = (temp_lon - m_origin_lon_rad)*(NormalRadius(m_a, m_b, temp_lat)*cos(temp_lat));
          local_point.y_long = (temp_lat - m_origin_lat_rad)*MeridionalRadius(m_a, m_b, temp_lat);
          local_point.z_alt = global_waypoints.waypoints.at(i_point).z_alt;
          //local_point.x = m_csvLanes.lane[i_lane].point[i_point].x * 1e-7;
          //local_point.y = m_csvLanes.lane[i_lane].point[i_point].y * 1e-7;
          local_waypoints.waypoints.push_back(local_point);
//          checkedPoint.x=local_point.x_lat;
//          checkedPoint.y=local_point.y_long;
//          checkedPoint.z =0;
//          pointVisualize(checkedPoint,i_point);
//          ROS_INFO("success visualize");
          // ROS_INFO("east : %f, north : %f", local_point.x, local_point.y);
          ROS_INFO("longitude: %f, latitude: %f", temp_lon = global_waypoints.waypoints.at(i_point).y_long, global_waypoints.waypoints.at(i_point).x_lat);
        }
        ROS_INFO("ENU conversion end");
    }
    double MeridionalRadius(double a, double b, double lat){
      return pow(a*b, 2) / sqrt( pow((pow( a*cos(lat), 2) + pow( b*sin(lat), 2 )), 3));
    }

    double NormalRadius(double a, double b, double lat){
      return (a*a) / sqrt(pow( a*cos(lat), 2 ) + pow( b*sin(lat), 2));
    }

    void waypointsPub() {
        m_waypointsPub.publish(local_waypoints);
    }
    void pointVisualize(const geometry_msgs::Point point, int i){ //rviz

           visualization_msgs::Marker checkpoint;
           checkpoint.header.stamp = ros::Time::now();
           checkpoint.header.frame_id= "local_origin";
           checkpoint.ns = "/checked_point";
           checkpoint.id = i;

           checkpoint.pose.position.x = point.x;
           checkpoint.pose.position.y = point.y;
           checkpoint.pose.position.z = point.z;

           checkpoint.type = visualization_msgs::Marker::SPHERE;
           checkpoint.action = visualization_msgs::Marker::ADD;
           checkpoint.scale.x = 1.15;
           checkpoint.scale.y = 1.15;
           checkpoint.scale.z = 1.15;

           checkpoint.color.a = 1.0;
           checkpoint.color.r = 0.0;
           checkpoint.color.g = 1.0;
           checkpoint.color.b = 0.0;

           checkpoint.lifetime = ros::Duration(10);

           visual_pub.publish(checkpoint);
    }
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "waypoint_generator");
    ros::Time::init();
    ros::Rate rate(20);
    WaypointGenerator wg;
    wg.enuConversion();

    while(ros::ok()) {
        wg.waypointsPub();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
