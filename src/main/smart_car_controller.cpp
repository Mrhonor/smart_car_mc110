#include "smart_car_controller.h"
#include "smart_car_public.h"

#include <thread>

using namespace std;

smart_car_controller::smart_car_controller(ros::NodeHandle &n){
    ROS_INFO("controller init");
    comUart = new smart_car_communicator();

    // 开启线程
	thread mainThread(&smart_car_controller::controllerThreadHandle, this);
	mainThread.detach();
}

smart_car_controller::~smart_car_controller(){
    delete comUart;
}

void smart_car_controller::controllerThreadHandle(){
    SCommandDataStru data;
    data.Init();
    data.TargetAngle = 45;
    data.TargetSpeed = 2000;
    data.RecommendCarSpeed = 2000;
    data.LimitSpeed = 2000;
    data.control_type = 1;
    data.remote_control = 1;
    data.throttle = 1;
    data.turnAngle = 45;
    data.gear = 1;
    while (ros::ok())
    {
        comUart->uartTxHandle(data);
        ros::Duration(0.01).sleep();
    }
    
}