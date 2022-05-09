#ifndef __PROTOCOL__
#define __PROTOCOL__

#include "smart_car_loop_queue.h"
#include "smart_car_public.h"



// protocolBase
class smart_car_protocol
{

public:
	smart_car_protocol();
	~smart_car_protocol();


public: //impl
	int protocolRX(void* rx,int len, int & isSuccess);//chenFlag   return value bool to int
	void* protocolTX(int& len);
	
	int  frameRecvProc(void* rxData,int len, int & isSuccess);
	void*  frameSendProc(int& len);
	
	int  frameDataProc(uint8* buff,int len);
	int  getInfoByteCount(uint8* buff);
	
	inline uint32  makeInt32Data(uint8* buff);
	inline uint16  makeInt16Data(uint8* buff);
	
	
	SRealDataStru  getRealDataStruContext(){return m_RecvData;}
	void  setCommandDataStru(SCommandDataStru data) {m_SendData = data;}
	


	
protected:
	uint8 m_TxBuff[TX_BUFF_LEN];
	uint8 m_RxBuff[RX_BUFF_LEN];
	
	uint16 m_StatusFlag;
	
	SRealDataStru m_RecvData;
	SCommandDataStru m_SendData;

	unsigned char Check_Sum(unsigned char Count_Number, uint8* data);
	short IMU_Trans(uint8 Data_High,uint8 Data_Low);
	float Odom_Trans(uint8 Data_High,uint8 Data_Low);
};

#endif