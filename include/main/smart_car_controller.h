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
private:

    smart_car_communicator* comUart;

    ros::Publisher encoder_pub;
    ros::Publisher imu_pub;

    int32 imu_seq;
    int32 encoder_seq;

public:

    smart_car_controller(ros::NodeHandle &);
    ~smart_car_controller();
    

private:

    void controllerThreadHandle();
    void SensorInfoPublish(const SRealDataStru& data, const ros::Time &timeStamp);
    void ImuDataPublish(int16 * gypo, const ros::Time &timeStamp);
    void encoderDataPublish(const int16 & vel, const ros::Time &timeStamp);

};




#endif
