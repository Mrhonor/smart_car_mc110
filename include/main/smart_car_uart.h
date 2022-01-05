#ifndef __UART__
#define __UART__

#include "smart_car_linklayer.h"

class smart_car_uart : public robotLinkLayer
{

public:
	smart_car_uart();
	~smart_car_uart();

	bool rbCreatConnect(void * param);
	bool rbCloseConnect();
	int  rbSendData(void * buf, int len);
	int  rbRecvData(void * buf, int max);
	void rbSetACommMode(int id,int combaud,int databit,int stopbit,int checkmode);
	
private:
	int fileDescriptor;
};



#endif