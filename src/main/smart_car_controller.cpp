#include "smart_car_controller.h"
#include "smart_car_public.h"

#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "tf/transform_datatypes.h"


#include <thread>
// simulation
#include <nlohmann/json.hpp>
#include <fstream>


using namespace std;
using json = nlohmann::json;
smart_car_controller::smart_car_controller(ros::NodeHandle &n):
    imu_seq(0),
    encoder_seq(0)
{
    ROS_INFO("controller init");
    comUart = new smart_car_communicator();
    ControlState.Init();

    state_pub_ = n.advertise<std_msgs::Float64MultiArray>("/EKF/State", 10);
    reference_path_pub_ = n.advertise<std_msgs::Float64MultiArray>("/RefPath", 10);

    encoder_pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/smart_car_mc110/encoder", 10);
    imu_pub = n.advertise<sensor_msgs::Imu>("/smart_car_mc110/imu_data", 10);

    control_sub_ = n.subscribe("/MPCC/Control", 10, &smart_car_controller::mpccControlCallback, this);
    ekf_sub = n.subscribe("/odometry/filtered", 10, &smart_car_controller::ekfCallback, this);

    cmd_vel_sub  = n.subscribe("cmd_vel",     100, &smart_car_controller::Cmd_Vel_Callback, this); 

    curMode = "path1";
    ros::param::get("~configPath", configPath);
    ros::param::get("~trackMode", trackMode);
    ros::param::get("~trackPath1", trackPath1);
    ros::param::get("~trackPath2", trackPath2);


    ROS_INFO_STREAM("config path:" << configPath);

    ifstream iConfig(configPath);
    json jsonConfig;
    iConfig >> jsonConfig;

    PathToJson json_paths {jsonConfig["model_path"],
                           jsonConfig["cost_path"],
                           jsonConfig["bounds_path"],
                           jsonConfig["track_path"],
                           jsonConfig["normalization_path"]};

    integrator = new Integrator(jsonConfig["Ts"],json_paths);
    // x0 = {0, 0,0,jsonConfig["v0"],0,0,0,0.5,0,jsonConfig["v0"]};
    x0 = {0, 0,0,0,0,0,0,0,0,0};
    Ts = jsonConfig["Ts"];
    TempSimuEnd = 0;

    u0.setZero();

    LastTxTime = ros::Time::now();

    // 开启线程
    thread mainThread(&smart_car_controller::controllerThreadHandle, this);
    mainThread.detach();
}

smart_car_controller::~smart_car_controller(){
    ros::Duration(0.2).sleep();
    ControlState.TargetVelocity = 0;
    comUart->uartTxHandle(ControlState);
    ros::Duration(1).sleep();
    delete comUart;
    delete integrator;
}

void smart_car_controller::controllerThreadHandle(){
    SRealDataStru data;
    data.Init();
    ros::Duration(1).sleep();

    trackPublish(trackPath1);
    trackPublish(trackPath1);
    ros::Duration(1).sleep();
    ros::Time staTime = ros::Time::now();

    while (ros::ok())
    {
        
        if(comUart->uartRxHandle(data)){
            ros::Time timeStamp = ros::Time::now();
            SensorInfoPublish(data, timeStamp);
        }
        

        ros::Time curTime = ros::Time::now();
        double dt = (curTime.toSec() - LastTxTime.toSec());
        // x0.D += u0.dD * dt;
        x0.delta += u0.dDelta * dt;
        x0.vs += u0.dVs * dt;

        // if(x0.D > 0.05)       ControlState.TargetVelocity = x0.D * 0.75 + 0.25;
        // else if(x0.D < -0.05) ControlState.TargetVelocity = x0.D * 0.75 - 0.25;
        // else                  ControlState.TargetVelocity = 0;

        if(trackMode == "eight"){
            if(curMode == "path1" && x0.X > -0.1){
                curMode = "path2_s";
                trackPublish(trackPath2);
            }
            else if(curMode == "path2_s" && x0.X > 3){
                curMode = "path2";
            }
            else if(curMode == "path2" && x0.X < 0.1){
                curMode = "path1_s";
                trackPublish(trackPath1);
            }
            else if(curMode == "path1_s" && x0.X < -3){
                curMode = "path1";
            }
        }
        
        ControlState.TargetAngle = -u0.dDelta * 180 / 3.14159;  
        if(ControlState.TargetAngle > 30) ControlState.TargetAngle = 30;
        else if(ControlState.TargetAngle < -30) ControlState.TargetAngle = -30;

        ControlState.TargetVelocity = 0.5 - fabs(ControlState.TargetAngle) * 0.01 ;
        
        if(trackMode == "debug" && (curTime.toSec() - staTime.toSec())>3){
            ControlState.TargetVelocity = 0;
            ROS_WARN("Debug!Stop!");
        }

        comUart->uartTxHandle(ControlState);
            
        LastTxTime = curTime;

        ros::Duration(0.01).sleep();
    }
    // ros::Duration(1).sleep();
    // trackPublish();
}

void smart_car_controller::trackPublish(std::string filePath){
    std::ifstream iTrack(filePath);
    json jsonTrack;
    iTrack >> jsonTrack;
    std::vector<double> x_center = jsonTrack["X"];
    std::vector<double> y_center = jsonTrack["Y"];
    std::vector<double> x_in = jsonTrack["X_i"];
    std::vector<double> y_in = jsonTrack["Y_i"];
    std::vector<double> x_out = jsonTrack["X_o"];
    std::vector<double> y_out = jsonTrack["Y_o"];


    std_msgs::Float64MultiArray ref_msg;
    std_msgs::MultiArrayDimension dim_msg;
    dim_msg.label = "X";
    dim_msg.size = x_center.size();
    ref_msg.layout.dim.push_back(dim_msg);
    for(auto i : x_center){
        ref_msg.data.push_back(i);
    }

    dim_msg.label = "Y";
    dim_msg.size = y_center.size();
    ref_msg.layout.dim.push_back(dim_msg);
    for(auto i : y_center){
        ref_msg.data.push_back(i);
    }
    dim_msg.label = "Xin";
    dim_msg.size = x_in.size();
    ref_msg.layout.dim.push_back(dim_msg);
    for(auto i : x_in){
        ref_msg.data.push_back(i);
    }
    dim_msg.label = "Yin";
    dim_msg.size = y_in.size();
    ref_msg.layout.dim.push_back(dim_msg);
    for(auto i : y_in){
        ref_msg.data.push_back(i);
    }
    dim_msg.label = "Xout";
    dim_msg.size = x_out.size();
    ref_msg.layout.dim.push_back(dim_msg);
    for(auto i : x_out){
        ref_msg.data.push_back(i);
    }
    dim_msg.label = "Yout";
    dim_msg.size = y_out.size();
    ref_msg.layout.dim.push_back(dim_msg);
    for(auto i : y_out){
        ref_msg.data.push_back(i);
    }
    
    reference_path_pub_.publish(ref_msg);

    // ros::Duration(1).sleep();
    // statePublish();
    // nav_msgs::Odometry msg;
    // msg.header.seq = 0;
    // msg.header.stamp = ros::Time::now();
    // msg.header.frame_id = "odom";
    // msg.child_frame_id = "base_link";
    // msg.pose.pose.position.x = 2.5;
    // msg.pose.pose.position.y = 0;
    // msg.pose.pose.orientation.x = 0;
    // msg.pose.pose.orientation.y = 0;
    // msg.pose.pose.orientation.z = 0;
    // msg.pose.pose.orientation.w = 1;
    // msg.twist.twist.linear.x = 2;
    // msg.twist.twist.linear.y = 0;
    // msg.twist.twist.angular.z = 0;
    // ros::Duration(1).sleep();
    // state_pub_.publish(msg);
}

void smart_car_controller::statePublish(){
    std_msgs::Float64MultiArray state_msg;
    state_msg.data.push_back(x0.X);
    state_msg.data.push_back(x0.Y);
    state_msg.data.push_back(x0.phi);
    state_msg.data.push_back(x0.vx);
    state_msg.data.push_back(x0.vy);
    state_msg.data.push_back(x0.r);
    state_msg.data.push_back(x0.D);
    state_msg.data.push_back(x0.delta);
    state_msg.data.push_back(x0.vs);
    

    // temp for simulation
    TempSimuEnd++;
    state_msg.data.push_back(TempSimuEnd);

    state_pub_.publish(state_msg);
}

void smart_car_controller::mpccControlCallback(const std_msgs::Float64MultiArrayConstPtr& msg){
    // Input u0;
    u0.dD = msg->data[0];
    u0.dDelta = msg->data[1];
    u0.dVs = msg->data[2];
    
    // statePublish();
}

// 发布传感器数据
void smart_car_controller::SensorInfoPublish(const SRealDataStru& data, const ros::Time &timeStamp){
    ImuDataPublish(data.Gyro, timeStamp);
    encoderDataPublish(data.CarSpeed, timeStamp);
}

// 发布Imu数据
void smart_car_controller::ImuDataPublish(const int16 * gypo, const ros::Time &timeStamp){
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
                                      0, 0, 1e-1

    };

    pub_msg.angular_velocity.x = RollVel;
    pub_msg.angular_velocity.y = PitchVel;
    pub_msg.angular_velocity.z = YawVel;
    pub_msg.angular_velocity_covariance = {1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1e-1

    };

    pub_msg.linear_acceleration.x = XAcc;
    pub_msg.linear_acceleration.y = YAcc;
    pub_msg.linear_acceleration.z = ZAcc;
    pub_msg.linear_acceleration_covariance = {1e-1, 0, 0,
                                              0, 1e-1, 0,
                                              0, 0, 1e-1

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
    pub_msg.twist.covariance = {1e-1,0,0,0,0,0,
                                0,1,0,0,0,0,
                                0,0,1,0,0,0,
                                0,0,0,1,0,0,
                                0,0,0,0,1,0,
                                0,0,0,0,0,1

    };

    encoder_pub.publish(pub_msg);
}

void smart_car_controller::ekfCallback(const nav_msgs::OdometryConstPtr& msg){
    x0.X = msg->pose.pose.position.x;
    x0.Y = msg->pose.pose.position.y;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);

    
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    x0.phi = yaw;
    x0.vx = msg->twist.twist.linear.x;
    x0.vy = msg->twist.twist.linear.y;
    x0.r = msg->twist.twist.angular.z;
/*
    if(TempSimuEnd < 1){
         x0.vx = 0.3;
         ControlState.TargetVelocity = 0.3;
         x0.D = 0.2;
    }
*/
    statePublish();
}
void smart_car_controller::Cmd_Vel_Callback(const geometry_msgs::TwistConstPtr& msg){
    SCommandDataStru sendData;
    sendData.XVel = msg->linear.x;
    sendData.YVel = msg->linear.y;
    sendData.YawVel = msg->angular.z;
    comUart->uartTxHandle(sendData);
}
