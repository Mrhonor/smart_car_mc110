#ifndef __MOTION_CAPTURE_MAIN__
#define __MOTION_CAPTURE_MAIN__

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"


class motion_capture_body
{
private:
    ros::Subscriber motion_sub;
    ros::Publisher ekf_vo_pub;
    int seq;

    // only use when simulate
    ros::Subscriber imu_sub;
    ros::Publisher ekf_imu_pub;

public:
    motion_capture_body(ros::NodeHandle &n);
    ~motion_capture_body();

private:
    void motion_captureCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    // only use when simulate
    void imu_dataCallback(const sensor_msgs::Imu::ConstPtr& msg);
};



#endif