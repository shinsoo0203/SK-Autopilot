#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <iostream>
#include "math.h"
#include "sk_msgs/Waypoint.h"
#include "sk_msgs/WaypointArray.h"

#define PI 3.14159265359

using namespace std;

class SetpointCommander {
private:
    ros::NodeHandle nh;

    ros::Subscriber state_sub;
    ros::Subscriber target_wp_sub;
    ros::Subscriber current_wp_sub;
    ros::Subscriber local_wp_sub;

    ros::Publisher local_pos_pub;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    double wn;
    bool received_local_wp;

    mavros_msgs::State current_state;

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    geometry_msgs::PoseStamped pose;
    sk_msgs::WaypointArray local_wp;
    sk_msgs::Waypoint target_wp;
    int current_wp;

public:
    ros::Time last_request;
    SetpointCommander() {
        state_sub = nh.subscribe<mavros_msgs::State>
                ("mavros/state", 10, &SetpointCommander::stateCb, this);
        target_wp_sub = nh.subscribe<sk_msgs::Waypoint>
                ("/target_waypoint", 10, &SetpointCommander::targetWpCb, this);
        current_wp_sub = nh.subscribe<std_msgs::Int32>
                ("/current_waypoint", 1, &SetpointCommander::currentWpCb, this);
        local_wp_sub = nh.subscribe(
                    "/local_waypoints", 10, &SetpointCommander::localWpCb, this);

        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                ("mavros/setpoint_position/local", 10);

        arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                ("mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                ("mavros/set_mode");

        last_request = ros::Time::now();
        offb_set_mode.request.custom_mode = "OFFBOARD";
        arm_cmd.request.value = true;
        current_wp = 0;
        target_wp = sk_msgs::Waypoint();
        received_local_wp = false;
    }
    bool localWpExist() {
        return received_local_wp;
    }
    bool connected() {
        return current_state.connected;
    }
    void pubInitialPose() {
        ros::Rate rate(20.0);
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;

        //send a few setpoints before starting
        for(int i = 100; ros::ok() && i > 0; --i){
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
    }
    void stateCb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }
    void targetWpCb(const sk_msgs::Waypoint::ConstPtr& msg) {
        target_wp = *msg;
    }
    void currentWpCb(const std_msgs::Int32::ConstPtr& msg) {
        current_wp = msg->data;
    }
    void localWpCb(const sk_msgs::WaypointArray::ConstPtr& msg) {
        if(received_local_wp) return;
        local_wp=*msg;
        local_wp.frame_id = "local_origin";
        received_local_wp = true;
    }
    void prevSetting() {
        if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                cout<<"Offboard enabled"<<endl;
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                    cout<<"Vehicle armed"<<endl;;
                }
                last_request = ros::Time::now();
            }
        }
    }
    void setpoint() {
        pose.header.frame_id = "local_origin";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = target_wp.point.x;
        pose.pose.position.y = target_wp.point.y;
        pose.pose.position.z = target_wp.point.z;
        pose.pose.orientation.z = double(atan2(local_wp.wp[current_wp+2].point.y - local_wp.wp[current_wp].point.y,
                                        local_wp.wp[current_wp+2].point.x - local_wp.wp[current_wp].point.x)) - PI/2;
        //pose.pose.orientation.z = PI/2;
        pose.pose.orientation.w = 1.0;
        //ROS_INFO("%f",pose.pose.orientation.z);

        local_pos_pub.publish(pose);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "setpoint_commander");
    SetpointCommander sc;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    while(ros::ok() && !sc.localWpExist()){
        ros::spinOnce();
        rate.sleep();
    }
    cout<<"Received local waypoints."<<endl;

    // wait for FCU connection
    while(ros::ok() && !sc.connected()){
        ros::spinOnce();
        rate.sleep();
    }

    cout<<"FCU connected."<<endl;

    for(int i = 100; ros::ok() && i > 0; --i){
        sc.setpoint();
        ros::spinOnce();
        rate.sleep();
    }

    cout<<"Requesting arming and OFFBOARD mode."<<endl;

    sc.last_request = ros::Time::now();

    while(ros::ok()){
        sc.prevSetting();
        sc.setpoint();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
