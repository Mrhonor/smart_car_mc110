#ifndef __CONTROLLER__
#define __CONTROLLER__

// ros
#include "ros/ros.h"
// inc
#include "smart_car_communicator.h"
// cpp
#include <semaphore.h>

// temp use for simulation
#include "integrator.h"
#include "types.h"
#include "std_msgs/Float64MultiArray.h"

// simulation
using namespace mpcc;

class smart_car_controller
{
private:

    smart_car_communicator* comUart;

    ros::Publisher encoder_pub;
    ros::Publisher imu_pub;

    int32 imu_seq;
    int32 encoder_seq;

    // simulation
    mpcc::Integrator* integrator;
    State x0;
    ros::Publisher state_pub_;
    ros::Publisher reference_path_pub_;

    ros::Subscriber control_sub_;
    double Ts;
    int TempSimuEnd;

public:

    smart_car_controller(ros::NodeHandle &);
    ~smart_car_controller();
    
    // simulation
    void trackPublish();
    void statePublish();
    void mpccControlCallback(const std_msgs::Float64MultiArrayConstPtr&);

private:

    void controllerThreadHandle();
    void SensorInfoPublish(const SRealDataStru& data, const ros::Time &timeStamp);
    void ImuDataPublish(const int16 * gypo, const ros::Time &timeStamp);
    void encoderDataPublish(const int16 & vel, const ros::Time &timeStamp);


};




#endif
