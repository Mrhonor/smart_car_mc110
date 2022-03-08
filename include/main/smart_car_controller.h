#ifndef __CONTROLLER__
#define __CONTROLLER__

// ros
#include "ros/ros.h"
// inc
#include "smart_car_communicator.h"
// cpp
#include <semaphore.h>


class smart_car_controller
{
public:

    smart_car_controller(ros::NodeHandle &);
    ~smart_car_controller();
    
    void trackPublish();

private:

    smart_car_communicator* comUart;
    void controllerThreadHandle();

    ros::Publisher state_pub_;
    ros::Publisher reference_path_pub_;
    ros::Publisher track_boundary_left_pub_;
    ros::Publisher track_boundary_right_pub_;

};




#endif
