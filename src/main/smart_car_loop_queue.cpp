#include "smart_car_loop_queue.h"
#include "smart_car_public.h"

#include <string.h>
#include <stdlib.h>

smart_car_loop_queue::smart_car_loop_queue() : mIsMultiThread(true)
{
	mBuf = NULL;
	initMutex();
	resetQueue();
}

smart_car_loop_queue::smart_car_loop_queue(bool pIsMultiThread) : mIsMultiThread(pIsMultiThread)
{
	mBuf = NULL;
	initMutex();
	resetQueue();
}
smart_car_loop_queue::~smart_car_loop_queue()
{
	pthread_mutex_destroy(&mRwlockor); 
	if(mBuf) free(mBuf);
}

void smart_car_loop_queue::initMutex()
{
	if(mIsMultiThread)
	{
		pthread_mutex_init(&mRwlockor, NULL);
	}
}
void smart_car_loop_queue::lockIt()
{
	if(mIsMultiThread)
	{
		pthread_mutex_lock(&mRwlockor);
	}
}
bool smart_car_loop_queue::tryLockIt()
{
	if(mIsMultiThread)
	{
		if(!pthread_mutex_trylock(&mRwlockor))
			return true;
		else return false;
	}
	else
		return true;
}

void smart_car_loop_queue::unLockIt()
{
	if(mIsMultiThread)
	{
		pthread_mutex_unlock(&mRwlockor);
	}
}

void smart_car_loop_queue::resetQueue()
{
	lockIt();
	if(mBuf) free(mBuf);
	mReadP = 0;
	mWriteP = 0;
	mDataLenInBytes = 0;
	mBufSize = UART_AVERAGE_ONEFRAME_LEN * UART_QUEUE_STORE_NUMS;
	mBuf = malloc(mBufSize);
	if(mBuf == NULL)
	{
#ifdef PRINTCLASSLOOPQUEUE
		LOCATERR;
		perror("reset loop queue");
#endif
	}
	unLockIt();
}

bool smart_car_loop_queue::isQueueFull(int bytesToWrite)
{
	if(mDataLenInBytes + bytesToWrite > mBufSize)
		return true;
	else
		return false;
}
bool smart_car_loop_queue::isQueueEmpty()
{
	if(mReadP == mWriteP && mDataLenInBytes == 0)
		return true;
	else
		return false;
}


int smart_car_loop_queue::inQueue(void * data, int len)
{
	int ret = INITRETVAL;
	int i;

	lockIt();

	if(!data || len <= 0)
	{
		ret = -1;
		goto endFlag_inQ;
	}
	
	if(!isQueueFull(len))
	{
		for(i=0; i<len; i++)
		{
			((unsigned char *)mBuf)[(mWriteP+i)%mBufSize] = ((unsigned char *)data)[i];
		}
		//memcpy(&(((unsigned char *)mBuf)[mWriteP]),data,len);
		INCLOOP(mWriteP,len,mBufSize);
		mDataLenInBytes += len;
		ret = len;
		goto endFlag_inQ;
	}
	else
	{
		ret = 0;
		goto endFlag_inQ;
	}
endFlag_inQ:
	unLockIt();
	return ret;
}


int smart_car_loop_queue::outQueueToBuffer(void * buf, int len)
{
	int ret = INITRETVAL;
	int count = INITRETVAL;
	int i;

	lockIt();

	if(!buf || len <=0)
	{
		ret = -1;
		goto endFlag_outQ;
	}
	
	if(!isQueueEmpty())
	{
		count = mDataLenInBytes > len ? len : mDataLenInBytes;
		for(i=0; i<len; i++)
		{
			((unsigned char *)buf)[i] = ((unsigned char *)mBuf)[(mReadP+i)%mBufSize];
		}
		//memcpy(buf,&(((unsigned char *)mBuf)[mReadP]),count);
		INCLOOP(mReadP,count,mBufSize);
		mDataLenInBytes -= count;
		ret = count;
		goto endFlag_outQ;
	}
	else
	{
		ret = 0;
		goto endFlag_outQ;
	}
endFlag_outQ:
	unLockIt();
	return ret;
}
int smart_car_loop_queue::getDataWithoutOutQueue(void * buf, int len)
{
	int ret = INITRETVAL;
	int count = INITRETVAL;
	int i;

	lockIt();

	if(!buf || len <=0)
	{
		ret = -1;
		goto endFlag_outQ;
	}
	
	if(!isQueueEmpty())
	{
		count = mDataLenInBytes > len ? len : mDataLenInBytes;
		for(i=0; i<len; i++)
		{
			((unsigned char *)buf)[i] = ((unsigned char *)mBuf)[(mReadP+i)%mBufSize];
		}
		//memcpy(buf,&(((unsigned char *)mBuf)[mReadP]),count);
		//INCLOOP(mReadP,count,mBufSize);
		//mDataLenInBytes -= count;
		ret = count;
		goto endFlag_outQ;
	}
	else
	{
		ret = 0;
		goto endFlag_outQ;
	}
endFlag_outQ:
	unLockIt();
	return ret;
}

int smart_car_loop_queue::outQueueDirect(int len)
{
	int ret = INITRETVAL;
	int count = INITRETVAL;

	lockIt();

	if(len <=0)
	{
		ret = -1;
		goto endFlag_outQD;
	}
	
	if(!isQueueEmpty())
	{
		count = mDataLenInBytes > len ? len : mDataLenInBytes;
		INCLOOP(mReadP,count,mBufSize);
		mDataLenInBytes -= count;
		ret = count;
		goto endFlag_outQD;
	}
	else
	{
		ret = 0;
		goto endFlag_outQD;
	}
endFlag_outQD:
	unLockIt();
	return ret;
}
int smart_car_loop_queue::getCurrentLenInBytes()
{
	int ret;
	lockIt();
	ret = mDataLenInBytes;
	unLockIt();
	return ret;
}
void * smart_car_loop_queue::getBufContent()//此函数存在隐患，被外部对象调用可能会引起多线程访问冲突,不是多任务的情况下可以直接使用
{
	return mBuf;
}
char * smart_car_loop_queue::getReadPBuffer()//此函数存在隐患，被外部对象调用可能会引起多线程访问冲突，不是多任务的情况可以直接使用
{
	return &(((char *)mBuf)[mReadP]);
}
