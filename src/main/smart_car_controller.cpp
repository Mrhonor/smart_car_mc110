#include "smart_car_controller.h"
#include "smart_car_public.h"



using namespace std;

smart_car_controller::smart_car_controller(ros::NodeHandle &n){
    ROS_INFO("controller init");
    comUart = smart_car_communicator();
}

smart_car_controller::~smart_car_controller(){

}

void smart_car_controller::controllerThreadHandle(){
    SCommandDataStru data;
    data.Init();
    while (ros::ok())
    {
        comUart.uartTxHandle(data);
        ros::Duration(0.1).sleep();
    }
    
}