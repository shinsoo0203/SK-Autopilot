#include <ros/ros.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "sk_msgs/Waypoint.h"
#include "sk_msgs/WaypointArray.h"
#include "sk_msgs/Object.h"
#include "sk_msgs/ObjectArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"

using namespace std;

class TrajectoryPlanner {
private:
    ros::NodeHandle nh;

    ros::Subscriber sub_curr_wp;
    ros::Subscriber sub_local_wp_arr;
    ros::Subscriber sub_object;

    ros::Publisher pub_target_wp;
    ros::Publisher pub_traj_arr_marker;
    ros::Publisher pub_selected_traj;

    sk_msgs::WaypointArray local_wp_arr;
    sk_msgs::WaypointArray fcu_wp_arr;
    sk_msgs::ObjectArray object_arr_fcu;
    sk_msgs::ObjectArray object_arr_local;
    sk_msgs::Waypoint target_wp;
    visualization_msgs::MarkerArray traj_arr_marker;

    tf::TransformListener m_rosTfListener;

    std::vector<sk_msgs::WaypointArray> fcu_traj_vec;

    int curr_wp;
    bool received_local_wp_arr;
    bool received_curr_wp;
    float trajectory_interval;
    int num_of_trajectory;

    int look_ahead;
    bool finished;
    int selected_idx;

public:
    TrajectoryPlanner() {
        nh.param ("trajectory_planner/trajectory_interval", trajectory_interval, (float)2.0);
        nh.param ("trajectory_planner/num_of_trajectory", num_of_trajectory, 5);

        sub_curr_wp = nh.subscribe(
                    "/current_waypoint", 10, &TrajectoryPlanner::currWpCb, this);
        sub_local_wp_arr = nh.subscribe(
                    "/local_waypoints", 10, &TrajectoryPlanner::localWpCb, this);
        sub_object = nh.subscribe(
                    "/objects",1,&TrajectoryPlanner::objectCb, this);

        pub_target_wp = nh.advertise<sk_msgs::Waypoint>(
                    "/target_waypoint", 10);
        pub_traj_arr_marker = nh.advertise<visualization_msgs::MarkerArray>(
                    "/trajectory_array_marker",1);
        pub_selected_traj = nh.advertise<sk_msgs::WaypointArray>(
                    "/selected_trajectory",1);

        received_local_wp_arr = false;
        received_curr_wp = false;
        look_ahead = 30;
        finished = false;
        selected_idx = num_of_trajectory/2;
        for(int i=0;i<num_of_trajectory;i++) {
            fcu_traj_vec.push_back(sk_msgs::WaypointArray());
        }
    }
    bool localWpExist() {
        return received_local_wp_arr;
    }
    bool currWpExist() {
        return received_curr_wp;
    }
    void currWpCb(const std_msgs::Int32::ConstPtr& msg) {
        curr_wp = msg->data;
        received_curr_wp = true;
    }
    void localWpCb(const sk_msgs::WaypointArray::ConstPtr& msg) {
        if(!received_local_wp_arr) received_local_wp_arr = true;
        local_wp_arr=*msg;
        local_wp_arr.frame_id = "/local_origin";
        fcu_wp_arr.frame_id = "/fcu";

        sk_msgs::Waypoint tmp_wp;
        geometry_msgs::PointStamped point_local;
        geometry_msgs::PointStamped point_fcu;

        point_local.header.frame_id = "/local_origin";
        point_local.header.stamp = ros::Time(0);
        point_fcu.header.frame_id = "/fcu";
        point_fcu.header.stamp = ros::Time(0);

        fcu_wp_arr.wp.clear();
        for(const auto& wp : local_wp_arr.wp) {
            point_local.point = wp.point;
            m_rosTfListener.transformPoint("/fcu", point_local, point_fcu);

            tmp_wp = wp;
            tmp_wp.frame_id = fcu_wp_arr.frame_id;
            tmp_wp.point = point_fcu.point;

            fcu_wp_arr.wp.push_back(tmp_wp);
        }
    }
    void objectCb(const sk_msgs::ObjectArray::ConstPtr& msg) {
        object_arr_fcu = *msg;
    }
    void generateTrajectory() {
        sk_msgs::WaypointArray traj;
        traj.frame_id = fcu_wp_arr.frame_id;

        fcu_traj_vec.clear();

        for(int i=curr_wp;i<curr_wp+60;i++) {
            if(i<fcu_wp_arr.wp.size()) {
                traj.wp.push_back(fcu_wp_arr.wp[i]);
            }

        }

        for(int i=-num_of_trajectory/2;i<=num_of_trajectory/2;i++){
            sk_msgs::WaypointArray tmp_traj;
            tmp_traj=traj;
            tmp_traj.wp.clear();
            for(const auto& wp : traj.wp) {
                sk_msgs::Waypoint tmp_wp;
                tmp_wp = wp;
                tmp_wp.point.x += (trajectory_interval * i);
                tmp_traj.wp.push_back(tmp_wp);
            }

            fcu_traj_vec.push_back(tmp_traj);
            //std::cout<<fcu_traj_vec.front().wp.front().frame_id<<endl;

        }
        //ROS_INFO("%d",fcu_traj_vec.size());
    }
    void checkCollision() {
        //selected_idx = fcu_traj_vec.size()/2;
        std::vector<bool> collision_vec;
        for(int i=0;i<fcu_traj_vec.size();i++) {
            bool collision = false;
            collision_vec.push_back(collision);
        }

        for(int i=0;i<fcu_traj_vec.size();i++) {
            for(const auto& wp : fcu_traj_vec[i].wp) {
                for(const auto& object : object_arr_fcu.objects) {
                    if(sqrt(pow(wp.point.x - object.pose.position.x,2) +
                            pow(wp.point.y - object.pose.position.y,2)) < 2.0) {
                        collision_vec[i]=true;
                        break;
                    }
                }
                if(collision_vec[i]) break;
            }
        }

        bool empty=true;
        for(const auto& collision : collision_vec) {
            if(collision) empty=false;
            std::cout<<collision<<endl;
        }
        if(empty) {
            selected_idx = fcu_traj_vec.size()/2;
            return;
        }

        for(int i=0;i<fcu_traj_vec.size();i++) {
            if(selected_idx+i >= fcu_traj_vec.size()) return;
            if(collision_vec[selected_idx+i] == false) {
                selected_idx = selected_idx + i;
                break;
            }
            if(selected_idx-i < 0) return;
            if(collision_vec[selected_idx-i] == false) {
                selected_idx = selected_idx - i;
                break;
            }
        }
    }
    bool checkCollision(const sk_msgs::ObjectArray& object_arr_fcu, const sk_msgs::WaypointArray& fcu_traj) {

    }
    void setTargetWp() {
        target_wp = local_wp_arr.wp[0];

        geometry_msgs::PointStamped point_local;
        geometry_msgs::PointStamped point_fcu;

        point_local.header.frame_id = "/local_origin";
        point_local.header.stamp = ros::Time(0);
        point_fcu.header.frame_id = "/fcu";
        point_fcu.header.stamp = ros::Time(0);

        point_fcu.point = fcu_traj_vec[selected_idx].wp[look_ahead].point;

        m_rosTfListener.transformPoint("/local_origin", point_fcu, point_local);

        target_wp.point = point_local.point;



    }
    void markTrajArr() {
        traj_arr_marker.markers.clear();
        int i = 0;
        int j = 0;
        for(const auto& traj : fcu_traj_vec)
        {
            int k = 0;
            for(const auto& wp : traj.wp)
            {
                if(k%5 == 0){
                    visualization_msgs::Marker traj_marker;
                    traj_marker.lifetime = ros::Duration(0.1);
                    traj_marker.header.frame_id = traj.frame_id;
                    traj_marker.id = i++;

                    if(j == selected_idx) {
                        traj_marker.color.r = 0.9;
                        traj_marker.color.g = 0.0;
                        traj_marker.color.b = 0.6;
                        traj_marker.color.a = 2.0;
                    }else {
                        traj_marker.color.r = 0.0;
                        traj_marker.color.g = 0.9;
                        traj_marker.color.b = 0.6;
                        traj_marker.color.a = 2.0;
                    }

                    traj_marker.type = visualization_msgs::Marker::ARROW;
                    traj_marker.action = visualization_msgs::Marker::ADD;

                    traj_marker.scale.x = 0.4;
                    traj_marker.scale.y = 0.2;
                    traj_marker.scale.z = 0.2;
                    traj_marker.points.clear();
                    traj_marker.pose.position.x = wp.point.x;
                    traj_marker.pose.position.y = wp.point.y;
                    traj_marker.pose.position.z = wp.point.z;
                    traj_marker.pose.orientation.y = 1.0;
                    traj_arr_marker.markers.push_back (traj_marker);
                }
                k++;

            }
            j++;
        }
        //ROS_INFO("%d",traj_arr_marker.markers.size());
    }
    void publish() {
        pub_target_wp.publish(target_wp);
        pub_traj_arr_marker.publish(traj_arr_marker);
    }
    void checkFinish() {
        if(curr_wp == local_wp_arr.wp.size()-1) {
            finished = true;
        }
        if(finished) {
            target_wp = local_wp_arr.wp.front();
            if(curr_wp==0) {
                finished = false;
            }
        }
    }
    void core() {
        generateTrajectory();
        checkCollision();
        setTargetWp();
        markTrajArr();
        checkFinish();
        publish();
    }

    void mainLoop()
    {
        ros::Rate loop_rate(50);

        while(ros::ok())
        {
            core();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "trajectory_planner");
    TrajectoryPlanner tp;
    ros::Rate loop_rate(50);

    cout<<"Waiting for waypoints..."<<endl;

    while(ros::ok() && (!tp.localWpExist() || !tp.currWpExist()))
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    cout<<"Received waypoints."<<endl;

    tp.mainLoop();

    return 0;
}
