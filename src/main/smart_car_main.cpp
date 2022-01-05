#include "smart_car_controller.h"

#include "ros/ros.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "smart_car_mc110");

	ros::NodeHandle n;
	ROS_INFO("START");
	smart_car_controller node(n);

	
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

	return 0;
}
