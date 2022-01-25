#include "motion_capture_body.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

motion_capture_body::motion_capture_body(ros::NodeHandle &n):seq(0){
    motion_sub = n.subscribe("/gazebo/model_states", 10, &motion_capture_body::motion_captureCallback, this);
    ekf_vo_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/smart_car_mc110/vo", 10);

    // only use when simulate
    imu_sub = n.subscribe("/mobile_base/sensors/imu_data", 10, &motion_capture_body::imu_dataCallback, this);
    ekf_imu_pub = n.advertise<sensor_msgs::Imu>("/smart_car_mc110/imu_data", 10);
}

motion_capture_body::~motion_capture_body(){

}

void motion_capture_body::motion_captureCallback(const gazebo_msgs::ModelStates::ConstPtr& msg){
    geometry_msgs::PoseWithCovarianceStamped pub_msg;
    seq++;
    pub_msg.header.seq = seq;
    pub_msg.header.stamp = ros::Time::now();
    pub_msg.header.frame_id = "odom";
    pub_msg.pose.pose = msg->pose[6];
    pub_msg.pose.covariance = {1e-4, 0, 0, 0, 0, 0,
                               0, 1e-4, 0, 0, 0, 0,
                               0, 0, 1e-4, 0, 0, 0,
                               0, 0, 0, 1e-3, 0, 0,
                               0, 0, 0, 0, 1e-3, 0,
                               0, 0, 0, 0, 0, 1e-3
    };
    ekf_vo_pub.publish(pub_msg);
}

void motion_capture_body::imu_dataCallback(const sensor_msgs::Imu::ConstPtr& msg){
    sensor_msgs::Imu pub_msg;
    pub_msg = *msg;
    pub_msg.orientation_covariance = {1, 0, 0,
                                      0, 1, 0,
                                      0, 0, 1e-4

    };
    pub_msg.angular_velocity_covariance = {1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1e-4

    };
    pub_msg.linear_acceleration_covariance = {10, 0, 0,
                                              0, 10, 0,
                                              0, 0, 1e-2

    };
    ekf_imu_pub.publish(pub_msg);
}