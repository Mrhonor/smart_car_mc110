#include <ros/ros.h>
#include "smart_car_mc110/hedge_imu_raw.h"
#include "smart_car_mc110/hedge_imu_fusion.h"
#include "smart_car_mc110/hedge_pos_ang.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/Imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>



#define pi 3.14159
#define ACCLSB 0.01
#define GYROLSB 0.0175

ros::Publisher pose_pub, imu_pub, position_pub;
int imuFusionSeq = 0, imuRawSeq = 0, uwbPoseSeq = 0;
double gyro_x = 0, gyro_y = 0, gyro_z = 0;


void imuFusionCallback(const smart_car_mc110::hedge_imu_fusionConstPtr& msg){
    ros::Time timeStamp = ros::Time::now();
    tf2::Quaternion q(0, 0, msg->qz, msg->qw);
    q.normalize();
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);
    // tf2::Matrix3x3 R1(0, -1, 0,
    //     1, 0, 0,
    //     0, 0, 1);

    // tf2::Matrix3x3 Sz(1, 0, 0,
    //     0, -1, 0,
    //     0, 0, 1);
    // m = Sz * R1 * Sz * m;
    m.getRPY(roll, pitch, yaw);//进行转换
    // if(yaw > 0) yaw -= pi;
    // else yaw += pi;
    // // yaw *= -1;
    // roll  *= -1;
    // pitch *= -1;
    // double temp = roll;
    // roll = pitch;
    // pitch = temp;

    yaw = -yaw;


    // ROS_INFO("yaw : %lf", yaw*180/pi);

    q.setEuler(yaw,pitch,roll); // yaw picth roll
    geometry_msgs::PoseWithCovarianceStamped pub_msg;
    
    pub_msg.header.stamp = timeStamp;
    pub_msg.header.seq = imuFusionSeq;
    imuFusionSeq++;
    pub_msg.header.frame_id = "odom";
    pub_msg.pose.covariance = {1,0,0,0,0,0,
                               0,1,0,0,0,0,
                               0,0,1,0,0,0,
                               0,0,0,1e-3,0,0,
                               0,0,0,0,1e-3,0,
                               0,0,0,0,0,1e-3};
                            
    pub_msg.pose.pose.orientation = tf2::toMsg(q);
    pose_pub.publish(pub_msg);
}

void imuRawCallback(const smart_car_mc110::hedge_imu_rawConstPtr& msg){
    ros::Time timeStamp = ros::Time::now();
    double acc_x = msg->acc_x * ACCLSB;
    double acc_y = msg->acc_y * ACCLSB;
    double acc_z = msg->acc_z * ACCLSB;

    // double g_x = msg->gyro_x * GYROLSB;
    // double g_y = msg->gyro_y * GYROLSB;
    // double g_z = msg->gyro_z * GYROLSB;


    // deduct system error
    double g_x = (msg->gyro_x - 56.563589) * GYROLSB;
    double g_y = (msg->gyro_y + 85.342666) * GYROLSB;
    double g_z = (msg->gyro_z + 26.876709) * GYROLSB;


    // gyro_x = gyro_x * (imuRawSeq / (imuRawSeq + 1.0)) + g_x / (imuRawSeq + 1);
    // gyro_y = gyro_y * (imuRawSeq / (imuRawSeq + 1.0)) + g_y / (imuRawSeq + 1);
    // gyro_z = gyro_z * (imuRawSeq / (imuRawSeq + 1.0)) + g_z / (imuRawSeq + 1);

    // ROS_INFO("acc_x: %lf, acc_y :%lf, acc_z : %lf", acc_x, acc_y, acc_z);

    Eigen::Matrix<double,3,3> R1, Sz;
    Eigen::Matrix<double,3,1> Acc, Gyro;
    R1 << 0, -1, 0,
        1, 0, 0,
        0, 0, 1;
    Sz <<  1,    0,     0,
           0,    -1,    0,
           0,     0,    1;
    Acc << acc_x, acc_y, acc_z;
    Gyro << g_x, g_y, g_z;

    Acc = Sz*R1*Sz*Acc;
    Gyro = Sz*R1*Sz*Gyro;
    ROS_INFO("gyro_x: %lf, gyro_y :%lf, gyro_z : %lf", Gyro(0,0), Gyro(1,0), Gyro(2,0));

    sensor_msgs::Imu pub_msg;
    pub_msg.header.frame_id = "base_link";
    pub_msg.header.seq = imuRawSeq;
    pub_msg.header.stamp = timeStamp;
    imuRawSeq++;
    pub_msg.linear_acceleration.x = Acc(0,0);
    pub_msg.linear_acceleration.y = Acc(1,0);
    pub_msg.linear_acceleration.z = Acc(2,0);
    pub_msg.linear_acceleration_covariance = {1e-3,0,0,
                                              0,1e-3,0,
                                              0,0,1e-3};
    pub_msg.angular_velocity.x = Gyro(0,0);
    pub_msg.angular_velocity.y = Gyro(1,0);
    pub_msg.angular_velocity.z = Gyro(2,0);
    pub_msg.linear_acceleration_covariance = {1e-3,0,0,
                                              0,1e-3,0,
                                              0,0,1e-3};

    imu_pub.publish(pub_msg);
    
}

void uwbPositionCallback(const smart_car_mc110::hedge_pos_angConstPtr& msg){
    geometry_msgs::PoseWithCovarianceStamped pub_msg;
    
    pub_msg.header.stamp = ros::Time::now();
    pub_msg.header.seq = uwbPoseSeq;
    uwbPoseSeq++;
    pub_msg.header.frame_id = "odom";
    pub_msg.pose.covariance = {1e-3,0,0,0,0,0,
                               0,1e-3,0,0,0,0,
                               0,0,1e-3,0,0,0,
                               0,0,0,1e-3,0,0,
                               0,0,0,0,1e-3,0,
                               0,0,0,0,0,1e-3};
                            
    pub_msg.pose.pose.position.x = msg->x_m;
    pub_msg.pose.pose.position.y = msg->y_m;
    pub_msg.pose.pose.position.z = msg->z_m;
    position_pub.publish(pub_msg);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "uwb");


    ros::NodeHandle node;
    ros::Subscriber orientation_sub = node.subscribe("/hedge_imu_fusion", 10, &imuFusionCallback);
    ros::Subscriber imu_sub = node.subscribe("/hedge_imu_raw", 10, &imuRawCallback);
    ros::Subscriber position_sub = node.subscribe("/hedge_pos_ang", 10, &uwbPositionCallback);

    pose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/uwb/orientation",10);
    imu_pub = node.advertise<sensor_msgs::Imu>("/uwb/imu", 10);
    position_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/uwb/position", 10);

    ros::spin();
    
    return 0;
};