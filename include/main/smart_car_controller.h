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
    

private:

    smart_car_communicator* comUart;
    void controllerThreadHandle();

};




#endif
