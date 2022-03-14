#include "smart_car_communicator.h"
#include "smart_car_uart.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <thread>

#include "ros/ros.h"

using namespace std;

// vxWorks claims to implement gettimeofday in sys/time.h
// but nevertheless does not provide it! See
// https://support.windriver.com/olsPortal/faces/maintenance/techtipDetail_noHeader.jspx?docId=16442&contentId=WR_TECHTIP_006256
// We implement a surrogate version here via clock_gettime:
inline int gettimeofday2(struct timeval *tv, void * /*tzv*/) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  tv->tv_sec  = ts.tv_sec;
  tv->tv_usec = ts.tv_nsec / 1000;
  return 0;
}

smart_car_communicator::smart_car_communicator(){
	// 初始化资源
    uartToMainQ = new smart_car_loop_queue(true);
    mainToUartQ = new smart_car_loop_queue(true);
	uartProtocol = new smart_car_protocol();

	// 开启串口通信线程
	thread uartThread(&smart_car_communicator::uartThreadHandler, this);
	uartThread.detach();
}

smart_car_communicator::~smart_car_communicator()
{
	delete uartToMainQ;
	delete mainToUartQ;
	delete uartProtocol;
}

void smart_car_communicator::uartThreadHandler()//串口任务
{
	robotLinkLayer * linkP = new smart_car_uart;//传输层句柄
	char interBuf[UART_AVERAGE_ONEFRAME_LEN];
	char buf[1024];
	memset(buf,0,sizeof(buf));
	memset(interBuf,0,sizeof(interBuf));
	int count = 0,tmpCount,tmpCount1;
	linkP->rbCreatConnect(NULL);//创建连接flagchen 此处未作返回值判断
	while(1)
	{
        if(mainToUartQ->getCurrentLenInBytes())//有未发送数据
        {
            memset(interBuf,0,sizeof(interBuf));
            tmpCount = mainToUartQ->getCurrentLenInBytes();
            tmpCount1 = sizeof(interBuf);
            count = tmpCount > tmpCount1 ? tmpCount1 : tmpCount; 
            mainToUartQ->outQueueToBuffer(interBuf, count);

            linkP->rbSendData(interBuf , count);//flagchen 因为是非阻塞方式，此处有可能没发送或只发送了一部分，需要做处理
        }
        else
            usleep(1000);

        count = linkP->rbRecvData(buf,1024);//接收处理
        if(count > 0)
        {
            uartToMainQ->inQueue(buf,count);
            memset(buf,0,sizeof(buf));
        }
	}
	
	if(linkP) delete linkP;
}

//串口发送处理，实际就是放到串口队列,originData为要发送的原始数据
void smart_car_communicator::uartTxHandle(SCommandDataStru & originData)
{
	void * sendP;
	int sendCount = 0;
	uartProtocol->setCommandDataStru(originData);//设置原始数据
	sendP = uartProtocol->protocolTX(sendCount);//组帧
	mainToUartQ->inQueue(sendP,sendCount);//入队
}

//串口数据处理，RecvData为收到的数据
bool smart_car_communicator::uartRxHandle(SRealDataStru & RecvData)
{
	int currentQueueCount;
	uint32 outQueueCount;
	int isSuccess;

	struct timeval tv_begin;
	 
	if((currentQueueCount = uartToMainQ->getCurrentLenInBytes()) >= UART_AVERAGE_ONEFRAME_LEN)
	{
		gettimeofday(&tv_begin, NULL);
		
#ifdef PRINTCLASSTIMECOUNTPRTC
		printf("^^^^^^^^^^^^recv data:%d second %d usecond^^^^^^^^^^^^\n",tv_begin.tv_sec,tv_begin.tv_usec);
#endif

		int tmpCount = currentQueueCount > 512 ? 512 : currentQueueCount;
		char  buf[tmpCount];
		memset(buf,0,tmpCount);
		uartToMainQ->getDataWithoutOutQueue(buf,tmpCount);
		//串口数据解析
		outQueueCount = uartProtocol->protocolRX(buf,tmpCount,isSuccess);
		//解析的字节数
		if(outQueueCount > 0)
		{
			uartToMainQ->outQueueDirect(outQueueCount);//从队列中出队
		}
		if(isSuccess)//成功解析出一帧数据
		{
			RecvData = uartProtocol->getRealDataStruContext();//提取有效数据

#ifdef PRINTCLASSSTM32UPLOADINFO
			C_Debug("############################Recv Data From STM32F103############################");
					
			printf("steerAngle: %d\n",RecvData.SteerAngle);
			printf("PWM_signal: %d\n",RecvData.PWM_Signal);
			printf("car_speed : %d\n",RecvData.CarSpeed);
			printf("AGV_POS1  : %d\n",RecvData.AGVPos[0]);
			printf("AGV_POS2  : %d\n",RecvData.AGVPos[1]);
			printf("InfraRed1 : %u\n",RecvData.InfraRedFront);
			printf("InfraRed2 : %u\n",RecvData.InFraRedLeftFront);
			printf("InfraRed3 : %u\n",RecvData.InFraRedLeftBack);
			printf("InfraRed4 : %u\n",RecvData.InFraRedRightFront);
			printf("InfraRed5 : %u\n",RecvData.InFraRedRightBack);
			printf("InfraRed6 : %u\n",RecvData.InFraRedBack);
			printf("Energy	  : %u\n",RecvData.Energy);
			printf("FRIDCardNO: %x\n",RecvData.FRIDCardNo);
			printf("JiuZhou1  : %d\n",RecvData.JiuZhou[0]);
			printf("JiuZhou2  : %d\n",RecvData.JiuZhou[1]);
			printf("JiuZhou3  : %d\n",RecvData.JiuZhou[2]);
			printf("JiuZhou4  : %d\n",RecvData.JiuZhou[3]);
			printf("JiuZhou5  : %d\n",RecvData.JiuZhou[4]);
			printf("JiuZhou6  : %d\n",RecvData.JiuZhou[5]);
			printf("JiuZhou7  : %d\n",RecvData.JiuZhou[6]);
			printf("JiuZhou8  : %d\n",RecvData.JiuZhou[7]);
			printf("JiuZhou9  : %d\n",RecvData.JiuZhou[8]);
			printf("Error_Code: %u\n",RecvData.ErrorCode);
									
			printf("############################Recv Data From STM32F103 Endl############################\n");
#endif
#ifdef PRINTCLASSSTM32UPLOADINFOFORMAT
			printf("%d	",RecvData.SteerAngle);
			printf("%d	",RecvData.PWM_Signal);
			printf("%d	",RecvData.CarSpeed);
			printf("%d	",RecvData.AGVPos[0]);
			printf("%d	",RecvData.AGVPos[1]);
			printf("%u  ",RecvData.InfraRedFront);
			printf("%u  ",RecvData.InFraRedLeftFront);
			printf("%u  ",RecvData.InFraRedLeftBack);
			printf("%u  ",RecvData.InFraRedRightFront);
			printf("%u  ",RecvData.InFraRedRightBack);
			printf("%u  ",RecvData.InFraRedBack);
			printf("%u	",RecvData.Energy);
			printf("%x	",RecvData.FRIDCardNo);
			printf("%d	",RecvData.JiuZhou[0]);
			printf("%d	",RecvData.JiuZhou[1]);
			printf("%d	",RecvData.JiuZhou[2]);
			printf("%d	",RecvData.JiuZhou[3]);
			printf("%d	",RecvData.JiuZhou[4]);
			printf("%d	",RecvData.JiuZhou[5]);
			printf("%d	",RecvData.JiuZhou[6]);
			printf("%d	",RecvData.JiuZhou[7]);
			printf("%d	",RecvData.JiuZhou[8]);
			printf("%u	\n",RecvData.ErrorCode);
#endif
#ifdef PRINTCLASSSTM32UPLOADINFODBG		
			printf("%d	",RecvData.JiuZhou[0]);
			printf("%d	",RecvData.JiuZhou[1]);
			printf("%d	",RecvData.JiuZhou[2]);
			printf("%d	",RecvData.JiuZhou[3]);
			printf("%d	",RecvData.JiuZhou[4]);
			printf("%d	",RecvData.JiuZhou[5]);
			printf("%d	",RecvData.JiuZhou[6]);
			printf("%d	",RecvData.JiuZhou[7]);
			printf("%d	\n",RecvData.JiuZhou[8]);
#endif

#ifdef PRINTCLASSSTM32UPLOADINFOINFR
			printf("%u  ",RecvData.InfraRedFront);
			printf("%u  ",RecvData.InFraRedLeftFront);
			printf("%u  ",RecvData.InFraRedLeftBack);
			printf("%u  ",RecvData.InFraRedRightFront);
			printf("%u  ",RecvData.InFraRedRightBack);
			printf("%u  \n\n",RecvData.InFraRedBack);
#endif
			return true;
		}
		else{
#ifndef PRINTCLASSSTM32UPLOADINFO
			printf("uart recv failed !\n");
#endif
		}
	}
	
	return false;
}