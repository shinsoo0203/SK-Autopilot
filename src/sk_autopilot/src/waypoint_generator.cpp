#include <ros/ros.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandCode.h>

class WaypointGenerator {
protected:
    ros::NodeHandle m_rosNodeHandler;
    ros::Publisher m_waypointsPub;
    mavros_msgs::WaypointList global_waypoints;
    float accept_rad;

public:
    WaypointGenerator()
    {
        m_waypointsPub=m_rosNodeHandler.advertise<mavros_msgs::WaypointList>("sk/global_map",1000);

        mavros_msgs::Waypoint wp;

        m_rosNodeHandler.param("waypoint_generator/accept_rad", accept_rad, (float)0.0);

        m_rosNodeHandler.param("waypoint_generator/wp0_lat", wp.x_lat, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp0_lon", wp.y_long, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp0_alt", wp.z_alt, 0.0);
        wp.frame=mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        wp.command=mavros_msgs::CommandCode::NAV_TAKEOFF;
        wp.param1=0.0; // minimum pitch (rad)
        wp.param3=1.0; // takeoff ascend rate (m/s)
        wp.is_current=true;
        wp.autocontinue=true;
        global_waypoints.waypoints.push_back(wp);

//        m_rosNodeHandler.param("waypoint_generator/restrict_zone_lat", wp.x_lat, 0.0);
//        m_rosNodeHandler.param("waypoint_generator/restrict_zone_lon", wp.y_long, 0.0);
//        //m_rosNodeHandler.param("waypoint_generator/restrict_zone_rad", wp.param1, 0.0); // Radius (m)
//        wp.frame=mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
//        wp.command=mavros_msgs::CommandCode::NAV_FENCE_CIRCLE_EXCLUSION;
//        wp.param1 = 30.0;
//        wp.param2 = wp.x_lat;
//        wp.param3 = wp.y_long;
//        wp.z_alt = 0.0;
//        wp.is_current=false;
//        wp.autocontinue=true;
//        global_waypoints.waypoints.push_back(wp);

        m_rosNodeHandler.param("waypoint_generator/wp1_lat", wp.x_lat, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp1_lon", wp.y_long, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp1_alt", wp.z_alt, 0.0);
        wp.frame=mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        wp.command=mavros_msgs::CommandCode::NAV_WAYPOINT;
        wp.param1=0.0; // hold time (s)
        wp.param2=accept_rad; // acceptance radius (m)
        wp.param3=0; // 0 to pass through
        wp.is_current=false;
        wp.autocontinue=true;
        global_waypoints.waypoints.push_back(wp);

//        m_rosNodeHandler.param("waypoint_generator/restrict_zone_lat", wp.x_lat, 0.0);
//        m_rosNodeHandler.param("waypoint_generator/restrict_zone_lon", wp.y_long, 0.0);
//        wp.z_alt=25.0;
//        wp.frame=mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
//        wp.command=mavros_msgs::CommandCode::NAV_WAYPOINT;
//        wp.param1=0.0; // hold time (s)
//        wp.param2=accept_rad; // acceptance radius (m)
//        m_rosNodeHandler.param("waypoint_generator/restrict_zone_rad",wp.param3,(float)0.0); // 0 to pass through
//        wp.is_current=false;
//        wp.autocontinue=true;
//        global_waypoints.waypoints.push_back(wp);



        m_rosNodeHandler.param("waypoint_generator/wp2_lat", wp.x_lat, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp2_lon", wp.y_long, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp2_alt", wp.z_alt, 0.0);
        wp.frame=mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        wp.command=mavros_msgs::CommandCode::NAV_LOITER_TIME;
        wp.param1=10.0; // loiter time (s)
        wp.is_current=false;
        wp.autocontinue=true;
        global_waypoints.waypoints.push_back(wp);

        m_rosNodeHandler.param("waypoint_generator/wp3_lat", wp.x_lat, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp3_lon", wp.y_long, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp3_alt", wp.z_alt, 0.0);
        wp.frame=mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        wp.command=mavros_msgs::CommandCode::NAV_WAYPOINT;
        wp.param1=0.0; // hold time (s)
        wp.param2=accept_rad; // acceptance radius (m)
        wp.param3=0; // 0 to pass through
        wp.is_current=false;
        wp.autocontinue=true;
        global_waypoints.waypoints.push_back(wp);

        m_rosNodeHandler.param("waypoint_generator/wp0_lat", wp.x_lat, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp0_lon", wp.y_long, 0.0);
        m_rosNodeHandler.param("waypoint_generator/wp0_alt", wp.z_alt, 0.0);
        wp.frame=mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        wp.command=mavros_msgs::CommandCode::NAV_LAND;
        wp.param1=0; // landing target number
        wp.param2=1.0; // accepted offset (m)
        wp.param3=1.0; // descend rate (m/s)
        wp.is_current=false;
        wp.autocontinue=true;
        global_waypoints.waypoints.push_back(wp);
    }

    void waypointsPub() {
        m_waypointsPub.publish(global_waypoints);
    }
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "waypoint_generator");
    ros::Time::init();
    ros::Rate rate(20);
    WaypointGenerator wg;

    while(ros::ok()) {
        wg.waypointsPub();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
