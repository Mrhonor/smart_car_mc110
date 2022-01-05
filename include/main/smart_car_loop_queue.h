#ifndef __LOOP_QUEUE__
#define __LOOP_QUEUE__

#include <pthread.h>

class smart_car_loop_queue {

public:
	smart_car_loop_queue();
	smart_car_loop_queue(bool pIsMultiThread);
	~smart_car_loop_queue();

	void initMutex();
	void lockIt();
	bool tryLockIt();
	void unLockIt();
	void resetQueue();

	bool isQueueFull(int bytesToWrite);
	bool isQueueEmpty();
	int inQueue(void * data, int len);
	int outQueueToBuffer(void * buf, int len);
	int outQueueDirect(int len);
	int getDataWithoutOutQueue(void * buf, int len);

	int getCurrentLenInBytes();
	void * getBufContent();
	char * getReadPBuffer();
	
private:
	pthread_mutex_t mRwlockor;
	bool mIsMultiThread;
	int mReadP;
	int mWriteP;
	int mDataLenInBytes;
	int mBufSize;
	void * mBuf;
};


#endif