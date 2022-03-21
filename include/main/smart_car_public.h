#ifndef __PUBLIC__
#define __PUBLIC__

#include <string.h>

typedef signed char         int8;   // 8 bits signed integer 
typedef unsigned char       uint8;  // 8 bits unsigned integer 
typedef short int           int16;  // 16 bits signed integer 
typedef unsigned short int  uint16; // 16 bits unsigned integer 
typedef int					int32;  // 32bits signed integer
typedef unsigned int		uint32; // 32bits unsigned integer
typedef float               float32;// 32 bits floating point

#define UARTDEVNAME                     "/dev/ttyS4"
#define UARTBAUDRATE                    115200
#define UARTDATABIT                     8
#define UARTCHECK	                    0
#define UARTSTOPBIT                     1
#define INITRETVAL	                    -100
#define UART_AVERAGE_ONEFRAME_LEN	    52
#define UART_QUEUE_STORE_NUMS		    100

#define ROBOT_CAR_INITSPEED			500
#define ROBOT_CAR_INITDIREC			0
#define ROBOT_CAR_INITANGLE			0
#define ROBOT_CAR_INITRMODE			0
#define ROBOT_CAR_INITSTOPID		100
#define ROBOT_CAR_INITFLAG          0
#define ROBOT_CAR_FRONT_DISTANCE    8191
#define TRAFFIC_LIGHT_STA           2
#define TARFFIC_LEFT_SECONDS        10
#define LEFT_PARK_NUM               6
#define CARID                       001
#define RESERVED   					0
#define ROBOT_MODE_AGV				0
#define ROBOT_MODE_INS				1

#define ROBOT_CAR_CALLPOLICEMODE    0
#define ROBOT_CAR_RACINGCOUNT       0
#define ROBOT_CAR_REMOTE_CONTROL    0
#define ROBOT_CAR_CONTROL_TYPE      0
#define ROBOT_CAR_THROTTLE          0
#define ROBOT_CAR_TURNANGLE         0
#define ROBOT_CAR_BRAKE             0
#define ROBOT_CAR_GEAR              0

#define RFIDANGLE                   100
#define REALSENSEDIS                0

#define PROTOCOL_TX_LEN   			64

#define TRUE   1
#define FALSE  0

#define STATUS_FLAG_OK  0x55
#define STATUS_FLAG_ERR  0xAA

#define  TEMP_BUFF_LEN  2500  //modify by andy 20210606
#define  TX_BUFF_LEN   TEMP_BUFF_LEN
#define  RX_BUFF_LEN   TEMP_BUFF_LEN   


#define INCLOOP(a,b,max)                a = (a+b)%max
#define LOCATERR			            printf("%s | %s | %d: ",__FILE__,__func__,__LINE__)
#define HIBYTE(w)   ((uint8) (((uint16) (w) >> 8) & 0xFF)) 
#define LOBYTE(w)   ((uint8) (w))
#define HIWORD(l)   ((uint16)(((uint32)(l) >> 16) & 0xFFFF))
#define LOWORD(l)   ((uint16)(l))

#define MPU9250_ANGLEVEL_RANGE 2000 // MPU9250测量角速度范围±250
#define MPU9250_ACC_RANGE	   16*9.8 // MPU9250测量加速度范围±2g
#define MPU9250_MAG_RANGE      4800  // MPU9250测量磁场强度范围±4800μT
#define PI 					   3.14159265

// typedef struct _tagCommandData
// {
// 	int16 TargetSpeed;
// 	int16 TargetAngle;
// 	uint8 CarSpeed;   /*行驶方向 左：1   右：0*/
// 	uint8 DestanceSense;
// 	uint32 FRID_CardNo;
// 	uint32 MagneticStripeFlag;  /*是否需要磁条导轨标记,默认需要：0；   不需要：1*/  /*add by andy 20190907*/

// 	int16 RecommendCarSpeed;   /*推荐车速　单位：ｃｍ/s*/
// 	int16 FrontDistance;       /*前方车距   单位：cm*/
// 	int16 LimitSpeed;          /*限速值    单位：ｃｍ/s*/
// 	uint8 TrafficLightSta;     /*0=初始化，1=红灯，2=绿灯，3=黄灯。其他无效*/
// 	uint8 TraficLeftSeconds;   /*交通灯剩余秒数　单位：s*/
// 	uint8 LeftParkNum;         /*剩余车位*/
// 	uint8 CarID;               /*车辆ID*/

// 	uint8 CallPoliceMode;    /*是否出警标记 默认0不出警  1：出警*/
    
// 	uint8 RacingCount;       /*赛车圈数 默认0 */
// 	/*add by andy 20210328--模拟驾驶*/
//     uint8 remote_control;    //遥控指令 默认0中控系统控制  1：模拟驾驶系统控制
//     uint8 control_type;      //控制类型 默认：00 只控制速度，角度只控制变道  01任意控制，脱离磁条控制方向和速度
//     uint16 throttle;          //油门
//     int16 turnAngle;         //转向角度
//     int16 brake;             //刹车
//     uint8 gear;              //档位 0 N 1 D 2 R 

// 	/*add by andy 20210411--单目摄像头基于opencv识别车道线巡线*/
//     uint8  RfidAngle;         /*车辆要到达的下一个RFID粘贴角度 add by andy 20210411*/
	
// 	int16 realSenseDis;       /*双目摄像头测距--单位mm---add by andy 20210411*/

// 	uint8  reserved;          /*保留*/

// 	_tagCommandData()
// 	{
// 		Init();
// 	}
	
// 	void Init()
// 	{
// 		memset(this,0,sizeof(_tagCommandData));
// 		TargetSpeed        = ROBOT_CAR_INITSPEED;
// 	    TargetAngle        = ROBOT_CAR_INITANGLE;
// 	    CarSpeed           = ROBOT_CAR_INITDIREC;
// 	    DestanceSense      = ROBOT_CAR_INITRMODE;
// 	    FRID_CardNo        = ROBOT_CAR_INITSTOPID;
// 	    MagneticStripeFlag = ROBOT_CAR_INITFLAG;
// 	    RecommendCarSpeed  = ROBOT_CAR_INITSPEED;
// 	    FrontDistance 	   = ROBOT_CAR_FRONT_DISTANCE;
// 	    LimitSpeed         = ROBOT_CAR_INITSPEED;
// 	    TrafficLightSta    = TRAFFIC_LIGHT_STA;
// 	    TraficLeftSeconds  = TARFFIC_LEFT_SECONDS;
// 	    LeftParkNum        = LEFT_PARK_NUM;
// 	    CarID              = CARID;

// 		CallPoliceMode     = ROBOT_CAR_CALLPOLICEMODE;
// 		RacingCount        = ROBOT_CAR_RACINGCOUNT;
// 		remote_control     = ROBOT_CAR_REMOTE_CONTROL;
// 		control_type       = ROBOT_CAR_CONTROL_TYPE;
// 		throttle           = ROBOT_CAR_THROTTLE;
// 		turnAngle          = ROBOT_CAR_TURNANGLE;
// 		brake              = ROBOT_CAR_BRAKE;
// 		gear               = ROBOT_CAR_GEAR;

// 		RfidAngle          = RFIDANGLE;
//         realSenseDis       = REALSENSEDIS;
// 	    reserved		   = RESERVED;
// 	}
	
// 	void operator =(_tagCommandData data)
// 	{
// 		Init();
// 		TargetSpeed        = data.TargetSpeed;
// 	    TargetAngle        = data.TargetAngle;
// 	    CarSpeed           = data.CarSpeed;
// 	    DestanceSense      = data.DestanceSense;
// 	    FRID_CardNo        = data.FRID_CardNo;
// 	    MagneticStripeFlag = data.MagneticStripeFlag;
// 	    RecommendCarSpeed  = data.RecommendCarSpeed;
// 	    FrontDistance 	   = data.FrontDistance;
// 	    LimitSpeed         = data.LimitSpeed;
// 	    TrafficLightSta    = data.TrafficLightSta;
// 	    TraficLeftSeconds  = data.TraficLeftSeconds;
// 	    LeftParkNum        = data.LeftParkNum;
// 	    CarID              = data.CarID;
// 		CallPoliceMode     = data.CallPoliceMode;
// 		RacingCount        = data.RacingCount;
// 		remote_control     = data.remote_control;
// 		control_type       = data.control_type;
// 		throttle           = data.throttle;
// 		turnAngle          = data.turnAngle;
// 		brake              = data.brake;
// 		gear               = data.gear;
// 		RfidAngle          = data.RfidAngle;
// 		realSenseDis       = data.realSenseDis;
// 	    reserved           = data.reserved;
// 	}
	
// }SCommandDataStru;

typedef struct _tagCommandData
{
	float32 TimeStamp; // 时间戳

	// states
	float32 XPos;
	float32 YPos;
	float32 ZPos;

	float32 Roll;
	float32 Pitch;
	float32 Yaw;

	float32 XVel;
	float32 YVel;
	float32 ZVel;

	float32 RollVel;
	float32 PitchVel;
	float32 YawVel;

	float32 XAcc;
	float32 YAcc;
	float32 ZAcc;

	// control command
	float32 TargetVelocity; //目标油门
	float32 TargetAngle; //目标角度
	
	uint8 ControlMode; //控制模式
	

	_tagCommandData()
	{
		Init();
	}
	
	void Init()
	{
		memset(this,0,sizeof(_tagCommandData));
	    
	}
	
	void operator =(_tagCommandData data)
	{
		Init();
		memcpy(this, &data, sizeof(_tagCommandData));
	}
	
}SCommandDataStru;

typedef struct _tagRealData
{
	int16 SteerAngle;
	int16 PWM_Signal;
	int16 CarSpeed;
	int8 AGVPos[2];
	uint16 InfraRedFront;       //InfraRed1
	uint16 InFraRedLeftFront;   //InfraRed2
	uint16 InFraRedLeftBack;    //InfraRed3
	uint16 InFraRedRightFront;  //InfraRed4
	uint16 InFraRedRightBack;   //InfraRed5
	uint16 InFraRedBack;        //InfraRed6
	uint16 Energy;
	uint32 FRIDCardNo;
	int16 Gyro[9];
	uint8 ErrorCode;
	
	_tagRealData()
	{
		Init();
	}
	
	void Init()
	{
		memset(this,0,sizeof(_tagRealData));
	}
	void operator =(_tagRealData data)
	{
		Init();
		SteerAngle = data.SteerAngle;
		PWM_Signal = data.PWM_Signal;
		CarSpeed = data.CarSpeed;
		memcpy(AGVPos,data.AGVPos,sizeof(int8)*2);
		InfraRedFront      = data.InfraRedFront;
		InFraRedLeftFront  = data.InFraRedLeftFront;
		InFraRedLeftBack   = data.InFraRedLeftBack;
		InFraRedRightFront = data.InFraRedRightFront;
		InFraRedRightBack  = data.InFraRedRightBack;
		InFraRedBack       = data.InFraRedBack;
		Energy = data.Energy;
		FRIDCardNo = data.FRIDCardNo;
		memcpy(Gyro,data.Gyro,sizeof(int16)*9);
		ErrorCode = data.ErrorCode;
	}
	
}SRealDataStru;

#endif