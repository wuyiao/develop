#include <stdio.h>      /*标准输入输出定义*/    
#include <stdlib.h>     /*标准函数库定义*/    
#include <unistd.h>     /*Unix 标准函数定义*/    
#include <sys/types.h>      
#include <sys/stat.h>       
#include <fcntl.h>      /*文件控制定义*/    
#include <termios.h>    /*PPSIX 终端控制定义*/    
#include <errno.h>      /*错误号定义*/    
#include <string.h>
#include <sched.h> 
#include <sys/ioctl.h> 
#include <linux/serial.h> 
#include <math.h>
#include <pthread.h>
#include <stdbool.h>

#include <libubox/ulog.h>

#include "common.h"
#include "context.h"
#include "uart.h"


//串口关闭
void uart_close(int fd)
{
	close(fd);
}

/* 串口设置参数

 *入口参数： fd          串口文件描述符

 *           speed       串口速度  

 *           flow_ctrl   数据流控制  

 *           databits    数据位   （取值为7或者8） 

 *           stopbits    停止位   （取值为1或者2）  

 *           parity      效验类型 （取值为N,E,O,S） 

*/ 
int uart_set(int fd,int speed, int flow_ctrl, int databits, int stopbits, char parity)
{
	unsigned int i = 0;
	//int status;
	int speed_arr[] = {B3000000,B2000000,B115200, B57600, B19200, B9600, B4800, B2400};
	int name_arr[] = {3000000,2000000,115200, 57600, 19200, 9600, 4800, 2400};
	struct termios options;
	struct serial_struct serial;
	/*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数
	还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，
	函数返回值为1.  */  
	if(tcgetattr(fd, &options) != 0)
	{
		XDEBUG("SetupSerial 1\n");
		return ERROR;
	}
	// 设置串口输入波特率和输出波特率
	for(i=0; i<(sizeof(speed_arr)/sizeof(int)); i++)
	{
		if(speed == name_arr[i])
		{
			cfsetispeed(&options, speed_arr[i]);
			cfsetospeed(&options, speed_arr[i]);
		}
	}
	//修改串口的缓冲区大小
	ioctl(fd,TIOCGSERIAL,&serial); 
	// #if TEST_LOW_LATENCY 
	// serial.flags |= ASYNC_LOW_LATENCY; 
	// #else 
	// serial.flags &= ~ASYNC_LOW_LATENCY; 
	// #endif 
	serial.xmit_fifo_size = 1024*1024;  
	ioctl(fd,TIOCSSERIAL,&serial); 

	// 修改控制模式，保证程序不会占用串口
	options.c_cflag |= CLOCAL;
	// 修改控制模式，使得能够从串口中读取输入数据
	options.c_cflag |= CREAD;
	// 设置数据流控制
	switch(flow_ctrl)
	{
		case 0 : // 不使用流控制
			options.c_cflag &= ~CRTSCTS;
			break;
		case 1 : // 使用硬体流控制
			options.c_cflag |= CRTSCTS;
			break;
		case 2 : // 使用软件流控制
			options.c_cflag |= IXON | IXOFF | IXANY;
			break;
		default:
			XDEBUG("flow control is error\n");
			break;
	}
	// 设置数据位
	// 屏蔽其他标志位
	options.c_cflag &= ~CSIZE;
	switch (databits)
	{
		case 5:
			options.c_cflag |= CS5;
			break;
		case 6:
			options.c_cflag |= CS6;
			break;
		case 7:
			options.c_cflag |= CS7;
			break;
		case 8:
			options.c_cflag |= CS8;
			break;
		default:
			fprintf(stderr, "Unsupport data size\n");
			return ERROR;
	}
	// 设置校验位
	switch(parity)
	{
		case 'N':// 无奇偶校验位
			options.c_cflag &= ~PARENB;
			//options.c_cflag &= ~INPCK;
			break;
		case 'O':// 设置为奇校验
			options.c_cflag |= PARENB;
			options.c_cflag |= PARODD;
			options.c_cflag |= (INPCK | ISTRIP);
			break;    
 	    case 'E'://设置为偶校验      
        		 options.c_cflag |= PARENB;           
           		 options.c_cflag &= ~PARODD;           
           		 options.c_cflag |= (INPCK | ISTRIP);          
           		 break;     
       	case 'S':// 设置为空格     
           		 options.c_cflag &= ~PARENB;    
           		 options.c_cflag &= ~CSTOPB;    
           		 break;     
       	default:      
           		 fprintf(stderr,"Unsupported parity\n");        
           		 return ERROR;  
    }  
	// 设置停止位     
    switch (stopbits)    
    {      
        case 1:       
            options.c_cflag &= ~CSTOPB; 
			break;     
        case 2:       
            options.c_cflag |= CSTOPB; 
			break;    
        default:       
            fprintf(stderr,"Unsupported stop bits\n");     
            return ERROR;    
    }    	
    //修改输出模式，原始数据输出    
    options.c_oflag &= ~OPOST;    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
	//options.c_lflag &= ~(ISIG | ICANON); 	
	options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);   
   
    //设置等待时间和最小接收字符    
    options.c_cc[VTIME] = 20; /* 读取一个字符等待1*(1/10)s */      
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */         
    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读    
    tcflush(fd,TCIFLUSH);    
    //激活配置 (将修改后的termios数据设置到串口中） 
    int a = tcsetattr(fd,TCSANOW,&options);  
    if (a != 0)      
    {   
		XDEBUG("com set error\n");     
        return ERROR;     
    }    
    return 0;
}

// 串口初始化
int uart_set_init(int fd) 
{
	if(uart_set(fd, 9600, 0, 8, 1, 'N') == -1)
		return ERROR;
	else 
		return OK;
}

// 串口接收数据
int uart_recv(int fd, unsigned char *rcv_buf, int data_len)
{
	int len, fs_sel;    
    fd_set fs_read;         
    FD_ZERO(&fs_read);    
    FD_SET(fd, &fs_read);     
    fs_sel = select(fd+1, &fs_read, NULL, NULL, NULL);     
    if(fs_sel)    
    {    
        len = read(fd, rcv_buf, data_len); 

        return len;    
    }    
    else    
		return -1;    
}

// 串口发送数据
int uart_send(int fd, unsigned char *send_buf, int data_len)
{
	int len = 0;    
    len = write(fd, send_buf, data_len); 
	
    if (len == data_len )    
    {   
		return len;    
    }         
    else       
    {              
        tcflush(fd, TCOFLUSH);    
        return -1;    
    }  
}

// 打开uart0串口
int uart_init()
{
	int uart_fd = 0;
    int ret = 0;

    //打开设备文件描述符
	uart_fd = open(UART, O_RDWR | O_SYNC);
    if(uart_fd < 0)
    {
        XDEBUG("uart open failed\n");
    }
    //串口设备初始化
	ret = uart_set_init(uart_fd);
    if(ret < 0)
    {
        XDEBUG("uart set init failed\n");
    }
    
    //XDEBUG("uart init success\n");
	return uart_fd;
}

//ARM接收组态屏发送的串口数据
void * thread_vgus_uart_recv(void *ctx)
{
    context_t *context = (context_t *)ctx;
	int ret = 0,recvlen = 3,data_length = 0;
	uchar readbuf[256]={0};

    while(1)
    {
        ret = uart_recv(context->device_fd.uart_fd,readbuf,recvlen);
		ULOG_DEBUG("buff0: 0x%x",readbuf[0]);
    	ULOG_DEBUG("buff1: 0x%x",readbuf[1]);
    	ULOG_DEBUG("buff2: 0x%x",readbuf[2]);
		ULOG_DEBUG("buff3: 0x%x",readbuf[3]);
    }
}

void * thread_vgus_uart_send(void *ctx)
{
	context_t *context = (context_t *)ctx;
    uchar a[3] = {0};
	a[0] = 0xc1;
	a[1] = 0x00;
	a[2] = 0x04;

    while(1)
    {
    	uart_send(context->device_fd.uart_fd, a, 3);
		ULOG_DEBUG("------------------------------------\n");
		ULOG_DEBUG("buff0: 0x%x",a[0]);
    	ULOG_DEBUG("buff1: 0x%x",a[1]);
    	ULOG_DEBUG("buff2: 0x%x",a[2]);
		ULOG_DEBUG("------------------------------------\n");

		sleep(3);
    }
}

void * test1(void *ctx)
{
	context_t *context = (context_t *)ctx;
    uchar a[5] = {0};
	a[0] = 0x5;
	a[1] = 0x4;
	a[2] = 0x3;
	a[3] = 0x2;
	a[4] = 0x1;

    while(1)
    {
		ULOG_DEBUG("------------------------------------\n");
		ULOG_DEBUG("test0: 0x%x",a[0]);
    	ULOG_DEBUG("test1: 0x%x",a[1]);
    	ULOG_DEBUG("test2: 0x%x",a[2]);
		ULOG_DEBUG("test3: 0x%x",a[3]);
		ULOG_DEBUG("test4: 0x%x",a[4]);
		ULOG_DEBUG("------------------------------------\n");

		sleep(3);
    }
}

//串口线程
int vgus_uart_init(void *ctx)
{
    int ret = 0;
    int i = 0;


    pthread_t uart_tid ,uart_tid_send ,test;
    context_t *context = (context_t *)ctx;
   
    context->device_fd.uart_fd = uart_init(ctx);     //串口打开并初始化




    ret = pthread_create(&uart_tid, NULL, thread_vgus_uart_recv, ctx);
    if(ret < 0)
    {
        ULOG_DEBUG("pthread create r uart failed\n");
    }

	ret = pthread_create(&uart_tid_send, NULL, thread_vgus_uart_send, ctx);
    if(ret < 0)
    {
        ULOG_DEBUG("pthread create  w failed\n");
    }

	ret = pthread_create(&test, NULL, test1, ctx);
    if(ret < 0)
    {
        ULOG_DEBUG("pthread create stock test failed\n");
    }
    ULOG_DEBUG(" uart thread success\n");
    
	return 0;
}
