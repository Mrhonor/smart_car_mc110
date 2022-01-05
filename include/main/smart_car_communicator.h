#ifndef __COMMUNICATOR__
#define __COMMUNICATOR__

#include "smart_car_loop_queue.h"
#include "smart_car_public.h"
#include "smart_car_protocol.h"

// STM32通信控制类
class smart_car_communicator
{
private:
    // 接收缓存区
    smart_car_loop_queue uartToMainQ;
    // 发送缓冲区
    smart_car_loop_queue mainToUartQ;

    smart_car_protocol* uartProtocol;

    void uartThreadHandler();
    
public:
    smart_car_communicator();
    ~smart_car_communicator();

    // 向串口发送数据
    void uartTxHandle(SCommandDataStru & originData);
    // 接收数据
    bool uartRxHandle(SRealDataStru & RecvData);
};




#endif