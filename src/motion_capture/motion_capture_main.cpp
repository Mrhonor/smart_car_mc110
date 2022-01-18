#include "motion_capture_body.h"

#include "ros/ros.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "smart_car_motion_capture");

	ros::NodeHandle n;
	ROS_INFO("START");
	motion_capture_body node(n);

	
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

	return 0;
}
