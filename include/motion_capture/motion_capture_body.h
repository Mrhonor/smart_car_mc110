#ifndef __MOTION_CAPTURE_MAIN__
#define __MOTION_CAPTURE_MAIN__

#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"



class motion_capture_body
{
private:
    ros::Subscriber motion_sub;
    ros::Publisher ekf_vo_pub;


public:
    motion_capture_body(ros::NodeHandle &n);
    ~motion_capture_body();

private:
    void motion_captureCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

};



#endif