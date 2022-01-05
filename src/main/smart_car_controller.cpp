#include "smart_car_controller.h"




using namespace std;

smart_car_controller::smart_car_controller(ros::NodeHandle &n){
    ROS_INFO("controller init");
    comUart = smart_car_communicator();
}

smart_car_controller::~smart_car_controller(){

}

