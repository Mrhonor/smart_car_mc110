#include "smart_car_uart.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>

#include "smart_car_public.h"

static  int speed_arr[] = { B1500000, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
static  int name_arr[] =  { 1500000,  115200,  57600,  38400,  19200,  9600,  4800,  2400,  1200,  300};  

using namespace std;

smart_car_uart::smart_car_uart()
{
	fileDescriptor = -1;
}
smart_car_uart::~smart_car_uart()
{
	
}
bool smart_car_uart::rbCreatConnect(void * param)
{
	fileDescriptor= open(UARTDEVNAME,O_RDWR|O_NONBLOCK);
	if(-1 == fileDescriptor)
	{
#ifndef PRINTCLASSUARTLINKINFO
		LOCATERR;
		perror("uart creat connect");
#endif
		return false;
	}
	rbSetACommMode(fileDescriptor,UARTBAUDRATE,UARTDATABIT,UARTSTOPBIT,UARTCHECK);
	return true;
}
bool smart_car_uart::rbCloseConnect()
{
	int ret = INITRETVAL;
	ret = close(fileDescriptor);
	if(-1 == ret)
	{
#ifdef PRINTCLASSUARTLINKINFO
		LOCATERR;
		perror("uart close connect");
#endif
		return false;
	}
	return true;
}
int smart_car_uart::rbSendData(void * buf, int len)
{
	int ret = INITRETVAL;
	if(len <= 0 || buf == NULL)
	{
#ifdef PRINTCLASSUARTLINKINFO
		LOCATERR;
		printf("uart send param error\n");
#endif
		return ret;
	}
	ret = write(fileDescriptor,buf,len);
	if(-1 == ret && strcmp(strerror(errno),"Resource temporarily unavailable"))
	{
#ifdef PRINTCLASSUARTLINKINFO
		LOCATERR;
		perror("uart send data");
#endif
	}
	else if(0 == ret)
	{
#ifdef PRINTCLASSUARTLINKINFO
		LOCATERR;
		printf("uart send 0 byte,intended to send %d",len);
#endif
	}
	return ret;
	
}
int smart_car_uart::rbRecvData(void * buf, int max)
{
	int ret = INITRETVAL;
	if(max <= 0 || buf == NULL)
	{
#ifdef PRINTCLASSUARTLINKINFO
		LOCATERR;
		printf("uart recv param error\n");
#endif
		return ret;
	}
	ret = read(fileDescriptor,buf,max);
	if(-1 == ret)
	{
		//LOCATERR;
		//perror("uart recv data");
	}
	else if(0 == ret)
	{
#ifdef PRINTCLASSUARTLINKINFO
		LOCATERR;
		printf("uart recv 0 byte,intended to recv max %d",max);
#endif
	}
	return ret;
}

void smart_car_uart::rbSetACommMode(int id,int combaud,int databit,int stopbit,int checkmode)
{
	int ret = INITRETVAL;
	int i,status;
	struct  termios tty_termios;
	int	flags;
	
	if ((tcgetattr (id, &tty_termios) == -1)) {
#ifdef PRINTCLASSUARTLINKINFO
		perror ("tcgetattr() failed");
#endif
		return;
	}
	//设置为非终端方式
	cfmakeraw(&tty_termios);
	
	//设置波特率
	for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) 
	{ 
		if  (combaud == name_arr[i]) 
		{     
			ret = cfsetispeed(&tty_termios, speed_arr[i]);  
			if(ret == -1)
			{
#ifdef PRINTCLASSUARTLINKINFO
				LOCATERR;
				printf("uart set in speed failed!\n");
#endif
			}
			ret = cfsetospeed(&tty_termios, speed_arr[i]);
			if(ret == -1)
			{
#ifdef PRINTCLASSUARTLINKINFO
				LOCATERR;
				printf("uart set out speed failed!\n");
#endif
			}
			break;
		}  
	}
	//设置停止位、数据位、校验
	tty_termios.c_cflag &= ~CSIZE; 
	switch (databit) //设置数据位
	{   
	case 7:		
		tty_termios.c_cflag |= CS7; 
		break;
	case 8:     
	default:    
		tty_termios.c_cflag |= CS8;
		break;   
	}
	//
	switch (checkmode) 
	{   
	case 0: //无校验
		tty_termios.c_cflag &= ~PARENB;   /* Clear parity enable */
		tty_termios.c_iflag &= ~INPCK;     /* Enable parity checking */ 
		break;  
	case 1: 
		tty_termios.c_cflag |= (PARODD | PARENB); /*奇校验*/  
		tty_termios.c_iflag |= INPCK;             /* Disnable parity checking */ 
		break;  
	case 2:  
		tty_termios.c_cflag |= PARENB;     /* Enable parity */    
		tty_termios.c_cflag &= ~PARODD;   /*偶校验*/     
		tty_termios.c_iflag |= INPCK;       /* Disable parity checking */
		break;
	default:
		tty_termios.c_cflag &= ~PARENB;   /* Clear parity enable */
		tty_termios.c_iflag &= ~INPCK;     /* Enable parity checking */ 
		break;  
	}  
	/* Set input parity option */ 
	if (checkmode)   
		tty_termios.c_iflag |= INPCK; 
	/*设置停止位*/  
	switch (stopbit)
	{   
	case 1:    
		tty_termios.c_cflag &= ~CSTOPB;  
		break;  
	case 2:    
		tty_termios.c_cflag |= CSTOPB;  
		break;
	default:    
		tty_termios.c_cflag &= ~CSTOPB;  
		break;  
	} 
	
	tty_termios.c_cc[VTIME] = 150; /*设置超时15s*/   
	tty_termios.c_cc[VMIN] = 0; /* Update the tty_termios and do it NOW */
	if (tcsetattr(id,TCSANOW,&tty_termios) != 0)   
	{ 
#ifdef PRINTCLASSUARTLINKINFO
		perror("SetupSerial");   
#endif
	} 
	tcflush(id,TCIOFLUSH);
	
	flags = fcntl(id, F_GETFL);
	fcntl(id, F_SETFL, flags |  O_NONBLOCK);
}




