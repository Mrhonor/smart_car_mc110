#include "smart_car_protocol.h"

#include "crcThings.h"

#include <string.h>


#define PROTOCOL_HEAD_LEN	4
#define PROTOCOL_BODY_LEN	45
#define PROTOCOL_TAIL_LEN	5



smart_car_protocol::smart_car_protocol()
{
	memset(m_TxBuff,0,sizeof(m_TxBuff));
	memset(m_RxBuff,0,sizeof(m_RxBuff));
	
	m_StatusFlag = 0;
}

smart_car_protocol::~smart_car_protocol()
{

}

int smart_car_protocol::protocolRX(void* rx,int len, int & isSuccess)//chenFlag   return value bool to int
{
	if(rx == NULL)
		return FALSE;
	return frameRecvProc(rx,len,isSuccess);
}

void* smart_car_protocol::protocolTX(int& len)
{
	return frameSendProc(len);
}

int  smart_car_protocol::getInfoByteCount(uint8* buff)
{
	if(buff == NULL)
		return 0;
	else
	{
		return *((uint16 *)(&(buff[2])));//chenflag 变为两字节//buff[2]/**256 + buff[3]*/;
	}
}

uint32  smart_car_protocol::makeInt32Data(uint8* buff)
{
	uint32 iTempData = buff[3]*256*256*256 + buff[2]*256*256 + buff[1]*256 + buff[0];
	return iTempData;
}

uint16  smart_car_protocol::makeInt16Data(uint8* buff)
{
	uint16 iTempData = buff[1]*256 + buff[0];
	return iTempData;
}

int  smart_car_protocol::frameDataProc(uint8* buff,int len)
{
	//数据长度、数据类型
	if(buff[0] != PROTOCOL_BODY_LEN || buff[1] != 0x00)//chenflag  0x12B  改为 0x2B
		return FALSE;
	
	m_RecvData.Init();
	
	uint8* pCurr = buff;
	pCurr += 2;
	int i;
	
	m_RecvData.SteerAngle          = makeInt16Data(pCurr);     pCurr += 2;
	m_RecvData.PWM_Signal          = makeInt16Data(pCurr);	   pCurr += 2;
	m_RecvData.CarSpeed            = makeInt16Data(pCurr);     pCurr += 2;
	m_RecvData.AGVPos[0]           = *pCurr++;
	m_RecvData.AGVPos[1]           = *pCurr++;
	
	m_RecvData.InfraRedFront       = makeInt16Data(pCurr);		pCurr += 2;
	m_RecvData.InFraRedLeftFront   = makeInt16Data(pCurr);		pCurr += 2;
	m_RecvData.InFraRedLeftBack    = makeInt16Data(pCurr);		pCurr += 2;
	m_RecvData.InFraRedRightFront  = makeInt16Data(pCurr);		pCurr += 2;
	m_RecvData.InFraRedRightBack   = makeInt16Data(pCurr);	    pCurr += 2;
	m_RecvData.InFraRedBack        = makeInt16Data(pCurr);	    pCurr += 2;
	m_RecvData.Energy              = makeInt16Data(pCurr);	    pCurr += 2;
	m_RecvData.FRIDCardNo          = makeInt32Data(pCurr);		pCurr += 4;

	for(i = 0;i<9;++i){
		m_RecvData.Gyro[i] = makeInt16Data(pCurr);	    
		pCurr += 2;
	}
    
	m_RecvData.ErrorCode = *pCurr++;
#ifdef ANDYTEST
	C_Debug("[Andy_test 20190717]m_RecvData.CarSpeed = %d, m_RecvData.FRIDCardNo = 0X%0X ", m_RecvData.CarSpeed, m_RecvData.FRIDCardNo);
#endif
	return TRUE;
}
	
int  smart_car_protocol::frameRecvProc(void* rxData,int len,int & isSuccess)
{
	int i = 0;
	int tmp = 0;//flagchen  用于存储找到帧头前的数据个数
	isSuccess = 0;
	if(len >= UART_AVERAGE_ONEFRAME_LEN)//chenflag 超过帧长度则处理，否则不处理
	{
		memcpy((void*)m_RxBuff,rxData,len);
		uint8* pHeadr = m_RxBuff;
		//找帧头
		for(i=0;i< len;++i)//chenflag  此处如果i=len仍然没有找到帧头要退出，并返回len,否则会越界。如果在i=n时找到帧头要记录
		{
			if(m_RxBuff[i] != 0x7D)
            {
				pHeadr++;
				continue;
			}		
            break;			
		}

		/***chenflag*****/
		if(i == len)
		{
			return len;
		}
		else if(i < len)
		{
			tmp = i;
		}
		/***chenflag end*****/
		
		m_StatusFlag = pHeadr[1];
		int iInfoByteCount = getInfoByteCount(pHeadr);
		if (iInfoByteCount != PROTOCOL_BODY_LEN)//chenflag
		{
#ifdef PRINTCLASSUARTPROTOCOLINFO
			printf("frame len error: %d \n", iInfoByteCount);
#endif
			return tmp + 1;//chenflag
		}
		else if(tmp + UART_AVERAGE_ONEFRAME_LEN > len)//chenflag  数据接收不全
		{
			return tmp;
		}
		uint32 crcCheckRes = makeFrameCheck(pHeadr,iInfoByteCount + 4);

		uint32 crcCodeCmp = makeInt32Data(pHeadr + iInfoByteCount + 4);

		uint8 ucFrameTail = pHeadr[iInfoByteCount+9-1];

		if(crcCodeCmp != crcCheckRes)
		{
#ifdef PRINTCLASSUARTPROTOCOLINFO
			printf("CRC error\n");
#endif
			//帧校验出错
			return tmp + 1;
		}
		if(ucFrameTail != 0x7E)
		{
			//帧尾错误
#ifdef PRINTCLASSUARTPROTOCOLINFO
			printf("frame tail error: %x \n", ucFrameTail);
#endif
			return tmp + 1;
		}
		
		isSuccess = frameDataProc(pHeadr+2,iInfoByteCount);//chenflag
		
		return tmp + UART_AVERAGE_ONEFRAME_LEN;//chenflag
	}
	else//chenflag
		return false;
}


void*  smart_car_protocol::frameSendProc(int& len)
{
	memset(m_TxBuff,0,sizeof(m_TxBuff));
	uint8* pCurr = m_TxBuff;
	//帧头
	*pCurr++ = 0x7D;
	//*pCurr++ = 0xEE;
	*pCurr++ = m_StatusFlag;
	
	*(((uint16 *)pCurr)) = PROTOCOL_TX_LEN;
	pCurr+=2;

	*(((uint32 *)pCurr)) = m_SendData.TimeStamp;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.XPos;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.YPos;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.ZPos;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.Roll;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.Pitch;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.Yaw;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.XVel;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.YVel;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.ZVel;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.RollVel;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.PitchVel;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.YawVel;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.XAcc;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.YAcc;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.ZAcc;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.TargetVelocity;
	pCurr+=4;

	*(((uint32 *)pCurr)) = m_SendData.TargetAngle;
	pCurr+=4;

	*pCurr++ = m_SendData.ControlMode;

	uint32 crc = makeFrameCheck(m_TxBuff,pCurr-m_TxBuff);
	*pCurr++ = LOBYTE(LOWORD(crc));
	*pCurr++ = HIBYTE(LOWORD(crc));
	*pCurr++ = LOBYTE(HIWORD(crc));
	*pCurr++ = HIBYTE(HIWORD(crc));
	
	*pCurr++ = 0x7E;
	//*pCurr++ = 0xFF;

	len = pCurr - m_TxBuff;
	
	return (void*)m_TxBuff;
}

// void*  smart_car_protocol::frameSendProc(int& len)
// {
// 	memset(m_TxBuff,0,sizeof(m_TxBuff));
// 	uint8* pCurr = m_TxBuff;
// 	//帧头
// 	*pCurr++ = 0x7D;
// 	//*pCurr++ = 0xEE;
// 	*pCurr++ = m_StatusFlag;
	
// 	*pCurr++ = 0x26;  /*20210328--帧长度25+10+1+2*/
// 	//*pCurr++ = 0x19;  /*帧长度25*/
//     //*pCurr++ = 0x18;  /*帧长度24*/
// 	//*pCurr++ = 0x0E;  /*帧长度14*/
// 	*pCurr++ = 0x02;
	
// 	*pCurr++ = LOBYTE(m_SendData.TargetSpeed);
// 	*pCurr++ = HIBYTE(m_SendData.TargetSpeed);
	
// 	*pCurr++ = LOBYTE(m_SendData.TargetAngle);
// 	*pCurr++ = HIBYTE(m_SendData.TargetAngle);

// 	*pCurr++ = m_SendData.CarSpeed;
// 	*pCurr++ = m_SendData.DestanceSense;
	
// 	*(((uint32 *)pCurr)) = m_SendData.FRID_CardNo;
// 	pCurr+=4;
	
// 	*(((uint32 *)pCurr)) = m_SendData.MagneticStripeFlag;  /*add by andy 20190909*/
// 	pCurr+=4;

// 	*pCurr++ = LOBYTE(m_SendData.RecommendCarSpeed);
// 	*pCurr++ = HIBYTE(m_SendData.RecommendCarSpeed);

// 	*pCurr++ = LOBYTE(m_SendData.FrontDistance);
// 	*pCurr++ = HIBYTE(m_SendData.FrontDistance);

// 	*pCurr++ = LOBYTE(m_SendData.LimitSpeed);
// 	*pCurr++ = HIBYTE(m_SendData.LimitSpeed);

// 	*pCurr++ = m_SendData.TrafficLightSta;
// 	*pCurr++ = m_SendData.TraficLeftSeconds;
// 	*pCurr++ = m_SendData.LeftParkNum;
// 	*pCurr++ = m_SendData.CarID;

//     /*add by andy 20210328*/
// 	*pCurr++ = m_SendData.CallPoliceMode; 
// 	*pCurr++ = m_SendData.remote_control;
// 	*pCurr++ = m_SendData.control_type;
	
// 	*pCurr++ = LOBYTE(m_SendData.throttle);
// 	*pCurr++ = HIBYTE(m_SendData.throttle);

// 	*pCurr++ = LOBYTE(m_SendData.turnAngle);
// 	*pCurr++ = HIBYTE(m_SendData.turnAngle);

// 	*pCurr++ = LOBYTE(m_SendData.brake);
// 	*pCurr++ = HIBYTE(m_SendData.brake);

// 	*pCurr++ = m_SendData.gear;
	
// 	*pCurr++ = m_SendData.RfidAngle; /*add by andy 20210411*/

// 	*pCurr++ = LOBYTE(m_SendData.realSenseDis);
// 	*pCurr++ = HIBYTE(m_SendData.realSenseDis);

// 	*pCurr++ = m_SendData.reserved;

// 	uint32 crc = makeFrameCheck(m_TxBuff,pCurr-m_TxBuff);
// 	*pCurr++ = LOBYTE(LOWORD(crc));
// 	*pCurr++ = HIBYTE(LOWORD(crc));
// 	*pCurr++ = LOBYTE(HIWORD(crc));
// 	*pCurr++ = HIBYTE(HIWORD(crc));
	
// 	*pCurr++ = 0x7E;
// 	//*pCurr++ = 0xFF;

// 	len = pCurr - m_TxBuff;
	
// 	return (void*)m_TxBuff;
// }