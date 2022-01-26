#include "smart_car_controller.h"
#include "smart_car_public.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

#include <thread>

using namespace std;

smart_car_controller::smart_car_controller(ros::NodeHandle &n):
    imu_seq(0),
    encoder_seq(0)
{
    ROS_INFO("controller init");
    comUart = new smart_car_communicator();

    encoder_pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/smart_car_mc110/encoder", 10);
    imu_pub = n.advertise<sensor_msgs::Imu>("/smart_car_mc110/imu_data", 10);

    // 开启线程
	thread mainThread(&smart_car_controller::controllerThreadHandle, this);
	mainThread.detach();
}

smart_car_controller::~smart_car_controller(){
    delete comUart;
}

void smart_car_controller::controllerThreadHandle(){
    SRealDataStru data;
    ros::Time curTimeStamp = ros::Time::now();
    while (ros::ok())
    {
        // 获取当前时间戳
        curTimeStamp = ros::Time::now();
        // 检查是否接收到数据
        if(comUart->uartRxHandle(data)){
            // 发布数据
            SensorInfoPublish(data, curTimeStamp);
        }
        ros::Duration(0.01).sleep();
    }
    
}

// 发布传感器数据
void smart_car_controller::SensorInfoPublish(const SRealDataStru& data, const ros::Time &timeStamp){
    ImuDataPublish(data.Gyro, timeStamp);
    encoderDataPublish(data.CarSpeed, timeStamp);
}

// 发布Imu数据
void smart_car_controller::ImuDataPublish(int16 * gypo, const ros::Time &timeStamp){
    // 解包MPU9250数据
    // MPU9250数据给出int16位的数据，数据表示范围为MPU9250_ANGLEVEL_RANGE、MPU9250_ACC_RANGE
    double RollVel = gypo[0] / 32768.0 * MPU9250_ANGLEVEL_RANGE;
    double PitchVel = gypo[1] / 32768.0 * MPU9250_ANGLEVEL_RANGE;
    double YawVel = gypo[2] / 32768.0 * MPU9250_ANGLEVEL_RANGE;

    double XAcc = gypo[3] / 32768.0 * MPU9250_ACC_RANGE;
    double YAcc = gypo[4] / 32768.0 * MPU9250_ACC_RANGE;
    double ZAcc = gypo[5] / 32768.0 * MPU9250_ACC_RANGE;

    // 计算方向角，程序来源自MPU6050解包方式，不一定相同，有待测试
    double Roll = gypo[6] * 0.00549;
    double Pitch = gypo[7] * 0.00549;
    double Yaw = gypo[8] * 0.00549;

    double Quat_x = sin(Roll*PI/180.0/2)*cos(Yaw*PI/180.0/2)*cos(Pitch*PI/180.0/2)-cos(Roll*PI/180.0/2)*sin(Yaw*PI/180.0/2)*sin(Pitch*PI/180.0/2);
    double Quat_y = cos(Roll*PI/180.0/2)*sin(Yaw*PI/180.0/2)*cos(Pitch*PI/180.0/2)+sin(Roll*PI/180.0/2)*cos(Yaw*PI/180.0/2)*sin(Pitch*PI/180.0/2);
    double Quat_z = cos(Roll*PI/180.0/2)*cos(Yaw*PI/180.0/2)*sin(Pitch*PI/180.0/2)-sin(Roll*PI/180.0/2)*sin(Yaw*PI/180.0/2)*cos(Pitch*PI/180.0/2);
    double Quat_w = cos(Roll*PI/180.0/2)*cos(Yaw*PI/180.0/2)*cos(Pitch*PI/180.0/2)+sin(Roll*PI/180.0/2)*sin(Yaw*PI/180.0/2)*sin(Pitch*PI/180.0/2);

    // 发布数据
    sensor_msgs::Imu pub_msg;
    imu_seq++;
    pub_msg.header.seq = imu_seq;
    pub_msg.header.stamp = timeStamp;
    pub_msg.header.frame_id = "base_link";
    pub_msg.orientation.w = Quat_w;
    pub_msg.orientation.x = Quat_x;
    pub_msg.orientation.y = Quat_y;
    pub_msg.orientation.z = Quat_z;
    pub_msg.orientation_covariance = {1, 0, 0,
                                      0, 1, 0,
                                      0, 0, 1e-4

    };

    pub_msg.angular_velocity.x = RollVel;
    pub_msg.angular_velocity.y = PitchVel;
    pub_msg.angular_velocity.z = YawVel;
    pub_msg.angular_velocity_covariance = {1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1e-4

    };

    pub_msg.linear_acceleration.x = XAcc;
    pub_msg.linear_acceleration.Y = YAcc;
    pub_msg.linear_acceleration.Z = ZAcc;
    pub_msg.linear_acceleration_covariance = {1e-4, 0, 0,
                                              0, 1e-4, 0,
                                              0, 0, 1e-4

    };
    imu_pub.publish(pub_msg);
}

void smart_car_controller::encoderDataPublish(const int16 & vel, const ros::Time &timeStamp){
    double XVel = (double)vel / 1000.0;
    geometry_msgs::TwistWithCovarianceStamped pub_msg;
    encoder_seq++;
    pub_msg.header.seq = encoder_seq;
    pub_msg.header.stamp = timeStamp;
    pub_msg.header.frame_id = "base_link";
    pub_msg.twist.twist.linear.x = XVel;
    pub_msg.twist.covariance = {1e-2,0,0,0,0,0,
                                0,1,0,0,0,0,
                                0,0,1,0,0,0,
                                0,0,0,1,0,0,
                                0,0,0,0,1,0,
                                0,0,0,0,0,1

    };

    encoder_pub.publish(pub_msg);
}