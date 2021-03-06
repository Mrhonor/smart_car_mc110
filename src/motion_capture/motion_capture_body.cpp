#include "motion_capture_body.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define pi 3.14159
motion_capture_body::motion_capture_body(ros::NodeHandle &n):seq(0){
    motion_sub = n.subscribe("/vrpn_client_node/mc110_2/pose", 10, &motion_capture_body::motion_captureCallback, this);
    ekf_vo_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/smart_car_mc110/vo", 10);

    // only use when simulate
    imu_sub = n.subscribe("/mobile_base/sensors/imu_data", 10, &motion_capture_body::imu_dataCallback, this);
    ekf_imu_pub = n.advertise<sensor_msgs::Imu>("/smart_car_mc110/imu_data", 10);
}

motion_capture_body::~motion_capture_body(){

}

void motion_capture_body::motion_captureCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    // 出错数据丢弃
    if(msg->pose.position.x > 9990000) return;

    
    tf2::Quaternion q(0, 0, msg->pose.orientation.z, msg->pose.orientation.w);
    q.normalize();
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);//进行转换
    // 修正motion capture的角度定位误差
    yaw -= (3.7 * pi / 180);
    while(yaw > pi) yaw -= 2*pi;
    while(yaw < -pi) yaw += 2*pi;

    // ROS_INFO("yaw : %lf", yaw*180/pi);

    q.setRPY(roll,pitch,yaw); 

    // ROS_INFO("yaw: %lf", yaw);

    geometry_msgs::PoseWithCovarianceStamped pub_msg;
    seq++;
    pub_msg.header.seq = seq;
    pub_msg.header.stamp = ros::Time::now();
    pub_msg.header.frame_id = "odom";
    pub_msg.pose.pose.position.x = msg->pose.position.x / 1000.0;
    pub_msg.pose.pose.position.y = msg->pose.position.y / 1000.0;
    pub_msg.pose.pose.position.z = msg->pose.position.z / 1000.0;
    pub_msg.pose.pose.orientation = tf2::toMsg(q);
    pub_msg.pose.covariance = {1e-4, 0, 0, 0, 0, 0,
                               0, 1e-4, 0, 0, 0, 0,
                               0, 0, 1e-4, 0, 0, 0,
                               0, 0, 0, 1e-4, 0, 0,
                               0, 0, 0, 0, 1e-4, 0,
                               0, 0, 0, 0, 0, 1e-4
    };
    ekf_vo_pub.publish(pub_msg);
}

void motion_capture_body::imu_dataCallback(const sensor_msgs::Imu::ConstPtr& msg){
    sensor_msgs::Imu pub_msg;
    pub_msg = *msg;
    
    //pub_msg.orientation.w += 0.1; 
    if(pub_msg.orientation.w > 1) pub_msg.orientation.w = 1;
    pub_msg.orientation_covariance = {1, 0, 0,
                                      0, 1, 0,
                                      0, 0, 1e-3

    };
    pub_msg.angular_velocity_covariance = {1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1e-4

    };
    pub_msg.linear_acceleration_covariance = {1000, 0, 0,
                                              0, 1000, 0,
                                              0, 0, 1e-2

    };
    ekf_imu_pub.publish(pub_msg);
}
