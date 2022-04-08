#include "single_eye_body.h"

#include "ros/ros.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "smart_car_single_eye");

	ros::NodeHandle n;
	ROS_INFO("START");
	single_eye_body node(n);

	
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

	return 0;
}
