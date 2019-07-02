#include <ros/ros.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <vector>

#define MISSION_POINT 3

class WaypointFollower {
protected:
    ros::NodeHandle m_rosNodeHandler;
    ros::Subscriber m_waypointsSub;
    ros::Subscriber state_sub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Publisher local_pos_pub;
    ros::Subscriber local_pos_sub;

    mavros_msgs::WaypointList waypoints;
    mavros_msgs::State current_state;

    geometry_msgs::PoseStamped curr_local_pose;
    std::vector<geometry_msgs::PoseStamped> path;
    geometry_msgs::PoseStamped setpoint;
    
    int current_seq;
    int count;
    float dist_min;
    bool change_flag;
    bool mission_complete;
    ros::Time mission_start_time;
    int mission_cnt;

public:
    bool start;
    WaypointFollower() {
        m_waypointsSub = m_rosNodeHandler.subscribe
                ("sk/local_waypoints",1000,&WaypointFollower::waypointsCb,this);
        state_sub = m_rosNodeHandler.subscribe
                ("mavros/state", 10, &WaypointFollower::state_cb,this);
        local_pos_pub = m_rosNodeHandler.advertise<geometry_msgs::PoseStamped>
                ("mavros/setpoint_position/local", 1000);
        arming_client = m_rosNodeHandler.serviceClient<mavros_msgs::CommandBool>
                ("mavros/cmd/arming");
        set_mode_client = m_rosNodeHandler.serviceClient<mavros_msgs::SetMode>
                ("mavros/set_mode");
        local_pos_sub = m_rosNodeHandler.subscribe
                ("/mavros/local_position/pose",1000,&WaypointFollower::pose_cb,this);

        m_rosNodeHandler.param("waypoint_follower/dist_min", dist_min, (float)0.0);
        current_seq=0;
        mission_cnt=0;
        start=false;
        change_flag=false;
        mission_complete=false;
    }
    void nextWp() {
        ROS_INFO("pose : %f %f %f",curr_local_pose.pose.position.x,curr_local_pose.pose.position.y,curr_local_pose.pose.position.z);

        ROS_INFO("current sequence : %d",current_seq);
        ROS_INFO("dist_min : %f",dist_min);

        //std::cout<<"wp : "<<path.at(current_seq).pose.position.x<<" "<<path.at(current_seq).pose.position.y<<" "<<path.at(current_seq).pose.position.z<<std::endl;

        if(current_seq==MISSION_POINT && !mission_complete) {
            executeMission();
            return;
        }

        for(int i=0;i<path.size();i++) {
            if(i==current_seq)  {
                ROS_INFO("wp : %lf %lf %lf ",path.at(i).pose.position.x,path.at(i).pose.position.y,path.at(i).pose.position.z);
                double dist=sqrt(pow(waypoints.waypoints.at(current_seq).x_lat-curr_local_pose.pose.position.x,2)
                                +pow(waypoints.waypoints.at(current_seq).y_long-curr_local_pose.pose.position.y,2)
                                 +pow(waypoints.waypoints.at(current_seq).z_alt-curr_local_pose.pose.position.z,2));
                ROS_INFO("dist : %lf",dist);
                if(dist<dist_min && !change_flag) {
                    if(current_seq==waypoints.waypoints.size()-1) {
                        ROS_INFO("Landing..");
                    }
                    else current_seq++;
                    change_flag=true;
                    ROS_INFO("current sequence : %d",current_seq);
                }else if(dist>dist_min) change_flag=false;
            }
        }
    }
    void executeMission() {
        ros::Rate rate(1);
        while(mission_cnt<10) {
            mission_cnt++;
            rate.sleep();
        }
        mission_complete=true;
    }
    void waypointsCb(const mavros_msgs::WaypointList::ConstPtr& msg) {
        if(start) return;
        waypoints.waypoints=msg->waypoints;
        for(int i=0;i<waypoints.waypoints.size();i++) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x=waypoints.waypoints.at(i).x_lat;
            pose.pose.position.y=waypoints.waypoints.at(i).y_long;
            pose.pose.position.z=waypoints.waypoints.at(i).z_alt;
            path.push_back(pose);
            ROS_INFO("wp : %lf %lf %lf",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
        }
        start=true;
    }
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        curr_local_pose=*msg;
    }
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }
    bool isConnected() {
        return current_state.connected;
    }
    void setpointPub() {
        for(int i=0;i<waypoints.waypoints.size();i++) {
            if(i==current_seq) {
                setpoint.pose.position.x = waypoints.waypoints.at(i).x_lat;
                setpoint.pose.position.y = waypoints.waypoints.at(i).y_long;
                setpoint.pose.position.z = waypoints.waypoints.at(i).z_alt;
            }
        }
        local_pos_pub.publish(setpoint);
    }
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "waypoint_follower");
    ros::Time::init();
    ros::Rate rate(20.0);

    WaypointFollower wf;

    while(ros::ok() && wf.isConnected() && !wf.start){
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()) {
        wf.setpointPub();
        wf.nextWp();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
