#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>


std::string static_turtle_name;

int main(int argc, char **argv)
{
  ros::init(argc,argv, "open_vins_tf"); //初始化ros节点
  
  
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  
  geometry_msgs::TransformStamped static_transformStamped; // 声明转换信息

  // 赋值 转换信息
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "odom";
  static_transformStamped.child_frame_id = "global";
  static_transformStamped.transform.translation.x = 0;
  static_transformStamped.transform.translation.y = 0;
  static_transformStamped.transform.translation.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, -3.1415926 / 2);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();

  
  // 通过  StaticTransformBroadcaster  把 转换信息发送出去
  static_broadcaster.sendTransform(static_transformStamped);
  
  // 终端 显示
  ROS_INFO("Spinning until killed publishing global to world");
  ros::spin();
  return 0;
};