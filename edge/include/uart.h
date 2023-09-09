#ifndef __UART_H__
#define __UART_H__

#define UART "/dev/ttymxc5"

#define VGUS_SYSREG_READ 0x81
#define VGUS_FRAME_HEADER_HIGH 0xA5
#define VGUS_FRAME_HEADER_LOW 0x5A

#define UARTDATA0 0x2000
#define UARTDATA1 0x2001
#define UARTDATA2 0x2002
#define UARTDATA3 0x2003
#define UARTDATA4 0x2004
#define UARTDATA5 0x2005
#define UARTDATA6 0x2006
#define UARTDATA7 0x2007
#define UARTDATA8 0x2008
#define UARTDATA9 0x2009

void * thread_vgus_uart_recv(void *ctx);
int vgus_uart_init(void *ctx);

void uart_close(int fd);
int uart_set(int fd,int speed, int flow_ctrl, int databits, int stopbits, char parity);
int uart_set_init(int fd);
int uart_recv(int fd, unsigned char *rcv_buf, int data_len);
int uart_send(int fd, unsigned char *send_buf, int data_len);
int uart_init();
int arm_uart_recv(int fd,unsigned char* readbuf, int length);

#endif