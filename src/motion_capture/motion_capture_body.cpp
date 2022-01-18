#include "motion_capture_body.h"
#include "nav_msgs/Odometry.h"

motion_capture_body::motion_capture_body(ros::NodeHandle &n){
    motion_sub = n.subscribe("/gazebo/model_states", 10, &motion_capture_body::motion_captureCallback, this);
    ekf_vo_pub = n.advertise<nav_msgs::Odometry>("/smart_car_mc110/vo", 10);
}

motion_capture_body::~motion_capture_body(){

}

void motion_capture_body::motion_captureCallback(const gazebo_msgs::ModelStates::ConstPtr& msg){
    nav_msgs::Odometry pub_msg;
    pub_msg.header.stamp = ros::Time::now();
    pub_msg.pose.pose = msg->pose[6];
    pub_msg.pose.covariance = {1e-5, 0, 0, 0, 0, 0,
                               0, 1e-5, 0, 0, 0, 0,
                               0, 0, 1e-5, 0, 0, 0,
                               0, 0, 0, 1e-5, 0, 0,
                               0, 0, 0, 0, 1e-5, 0,
                               0, 0, 0, 0, 0, 1e-5
    };
    ekf_vo_pub.publish(pub_msg);
}