#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>


ros::Publisher pub;

void poseCallback(const sensor_msgs::ImuConstPtr& msg){
  Eigen::Matrix<double,3,3> R;
  Eigen::Matrix<double,3,1> V;
  R << 0.999, 0.0312, 0.0316,
       0.0312, 0.0122, -0.9994,
       -0.0316, 0.9994, 0.0112;
  V << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  V = R*V;
  sensor_msgs::Imu pub_msg;
  pub_msg.linear_acceleration.x = V(0,0);
  pub_msg.linear_acceleration.y = V(1,0);
  pub_msg.linear_acceleration.z = V(2,0);
  pub.publish(pub_msg);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "open_vins");


  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/camera/imu", 10, &poseCallback);
  pub = node.advertise<sensor_msgs::Imu>("/imu", 10);

  ros::spin();
  return 0;
};