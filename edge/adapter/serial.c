#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <edge.h>
#include <edev.h>
#include <config.h>
#include <adapter.h>
#include <debug.h>
#include <libubox/ulog.h>

static int fd;
static struct serial_operations serial_ops;
static struct edge_adapter adapter;

static speed_t get_baudrate(int baudrate)
{
    switch (baudrate) {
        case 0: return B0;
        case 50: return B50;
        case 75: return B75;
        case 110: return B110;
        case 134: return B134;
        case 150: return B150;
        case 200: return B200;
        case 300: return B300;
        case 600: return B600;
        case 1200: return B1200;
        case 1800: return B1800;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 576000: return B576000;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 2500000: return B2500000;
        case 3000000: return B3000000;
        case 3500000: return B3500000;
        case 4000000: return B4000000;
        default: return -1;
    }
}

static int open_dev(struct edge_device *dev)
{
    speed_t speed;
    struct termios oldtio, newtio;
    struct serial_conf *conf = dev->conf;
    char serialdev[32] = {0};

    snprintf(serialdev, sizeof(serialdev), "/dev/%s", conf->dev);

    speed = get_baudrate(conf->baudrate);
    fd = open(serialdev, O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        ULOG_ERR("open dev: %s", conf->dev);
        return -1;
    }
    //save to oldtio
    tcgetattr(fd, &oldtio);
    //clear newtio
    bzero(&newtio, sizeof(newtio));

    newtio.c_cflag |= speed | CS8 | CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~PARENB;
    newtio.c_iflag = IGNPAR;  
    newtio.c_oflag = 0;
    //printf("newtio.c_cflag=%x\n",newtio.c_cflag);
    tcflush(fd, TCIFLUSH);  
    tcsetattr(fd, TCSANOW, &newtio);  
    tcgetattr(fd, &oldtio);
    //printf("oldtio.c_cflag=%x\n",oldtio.c_cflag);
    dev->fd[0] = fd;
    ULOG_DEBUG("open serialdev: %s success",serialdev);
    return fd;
}

void close_dev()
{
	close(fd);
}


int serial_recv(unsigned char *rcv_buf, int data_len)
{
    int len = 0;
  
    len = read(fd, rcv_buf, data_len); 
    ULOG_DEBUG("serial read: %d \n",len);

    return len;    
}

int serial_send(unsigned char *send_buf, int data_len)
{
	int len = 0;    
    len = write(fd, send_buf, data_len); 
	
    if (len == data_len )    
    {   
        ULOG_DEBUG("serial write: %d \n",len);
		return len;    
    }         
    else       
    {              
        tcflush(fd, TCOFLUSH);    
        return -1;    
    }  
}


ADAPTER_DECLARE(serial)
{
    memset(&serial_ops, 0, sizeof(serial_ops));
    memset(&adapter, 0, sizeof(adapter));

    serial_ops.open = open_dev;
    serial_ops.close = close_dev;
    serial_ops.write = serial_send;
    serial_ops.read = serial_recv;

    adapter.obj.type = SERIAL;
    adapter.ops = &serial_ops;

    adapter_register(&adapter);

    if (DEBUG(init, INIT))
        ULOG_DEBUG("(2) initialize serial");
}
