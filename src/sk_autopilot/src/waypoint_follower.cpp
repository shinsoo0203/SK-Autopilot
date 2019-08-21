#include <ros/ros.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define MISSION_POINT 2

class WaypointFollower {
protected:
    ros::NodeHandle m_rosNodeHandler;
    ros::Subscriber m_waypointsSub;
    ros::Subscriber state_sub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient wp_client;
    ros::Subscriber reached_sub;


    mavros_msgs::WaypointList list;
    mavros_msgs::State current_state;

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;

    bool mission_complete;
    int mission_cnt;
    bool wp_pushed=false;
    int wp_seq;

public:
    bool start;
    bool finished;
    WaypointFollower() {
        m_waypointsSub = m_rosNodeHandler.subscribe
                ("sk/global_map",1000,&WaypointFollower::mapCb,this);
        state_sub = m_rosNodeHandler.subscribe
                ("mavros/state", 10, &WaypointFollower::state_cb,this);
        arming_client = m_rosNodeHandler.serviceClient<mavros_msgs::CommandBool>
                ("mavros/cmd/arming");
        set_mode_client = m_rosNodeHandler.serviceClient<mavros_msgs::SetMode>
                ("mavros/set_mode");
        wp_client = m_rosNodeHandler.serviceClient<mavros_msgs::WaypointPush>
                ("mavros/mission/push");
        reached_sub = m_rosNodeHandler.subscribe
                ("mavros/mission/reached",100,&WaypointFollower::reached_cb,this);

        offb_set_mode.request.custom_mode = "AUTO.MISSION";
        arm_cmd.request.value = true;

        start=false;
        finished=false;
        mission_complete=false;
    }
    void wpPush() {
        if(wp_pushed) return;
        mavros_msgs::WaypointPush srv;
        srv.request.start_index=0;
        srv.request.waypoints=list.waypoints;
        if(wp_client.call(srv) && srv.response.success) {
            ROS_INFO("Sended Waypoints state : %d, wp_transfered : %iu",srv.response.success, (uint32_t)srv.response.wp_transfered);
            wp_pushed=true;
        }else {
            //ROS_ERROR("Failed to call service wp_send");
        }
    }
    void reached_cb(const mavros_msgs::WaypointReached::ConstPtr& msg) {
        wp_seq=msg->wp_seq;
        if(wp_seq==list.waypoints.size()-1) {
            ROS_INFO("Flight Finished");
            finished=true;
        }else if(wp_seq==MISSION_POINT) {
            executeMission();
        }
    }
    void arm() {
        if( current_state.mode != "AUTO.MISSION" ){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                //ROS_INFO("Mission enabled");
            }
        } else {
            if( !current_state.armed ){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    //ROS_INFO("Vehicle armed");
                }
            }
        }
    }
    void executeMission() {

        mission_complete=true;
    }
    void mapCb(const mavros_msgs::WaypointList::ConstPtr& msg) {
        if(start) return;
        list=*msg;
        start=true;
    }
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
        bool connected = current_state.connected;
        bool armed = current_state.armed;
        ROS_INFO("%s", armed ? "" : "DisArmed");
    }
    bool isConnected() {
        return current_state.connected;
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

    while(ros::ok() && !wf.finished) {
        //wf.arm();
        wf.wpPush();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
