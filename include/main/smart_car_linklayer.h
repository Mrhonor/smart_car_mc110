#ifndef __LINKLAYER__
#define __LINKLAYER__

class robotLinkLayer{

public:
	robotLinkLayer(){}
	~robotLinkLayer(){}

	virtual bool rbCreatConnect(void * param)=0;
	virtual bool rbCloseConnect()=0;
	virtual int  rbSendData(void * buf, int len)=0;
	virtual int  rbRecvData(void * buf, int max)=0;
	
private:
	
};


#endif