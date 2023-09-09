#include <libubox/uloop.h>
#include <libubox/ulog.h>
#include <thread.h>
#include <debug.h>

#include "lorae22.h"
#include "adapter.h"
#include "edev.h"

#define SERIAL_NAME "ttymxc5"

struct uloop_fd ufd;

static struct edge_device *serial_dev;
struct serial_operations *ops;
struct thread thread;

static struct uloop_timeout timeout;

int lorae22_read()
{
    unsigned char buff[1024];
    memset(buff,0,sizeof(buff));

    ops->read(&buff,4);

    ULOG_DEBUG("buff0: 0x%x",buff[0]);
    ULOG_DEBUG("buff1: 0x%x",buff[1]);
    ULOG_DEBUG("buff2: 0x%x",buff[2]);
    ULOG_DEBUG("buff3: 0x%x",buff[3]);

    return 0;
}


static void timeout_uart_send(struct uloop_timeout *t)
{
    int len = 0;
    unsigned char buff[3];
    buff[0] = 0xC1;
    buff[1] = 0x00;
    buff[2] = 0x04;

    ops->write(buff,3);

    uloop_timeout_set(t, 3 * 1000);
}

static void add_thread()
{
    thread_schedule(&edge_ctx.s, &thread);
}

static void lorae22_thread(struct thread *t)
{
    lorae22_read();
    thread_complete(t);
}

int lorae22_init()
{
    serial_dev = edev_get_dev(SERIAL_NAME);

    struct edge_adapter *serial = edev_get_adapter(serial_dev);
    ops = serial->ops;
    ops->open(serial_dev);
    thread.thread = lorae22_thread;
    thread_init(&thread);
    add_thread();
    ufd.fd = serial_dev->fd[0];
    ufd.cb = add_thread;
    uloop_fd_add(&ufd, ULOOP_READ);

    memset(&timeout, 0, sizeof(timeout));
    timeout.cb = timeout_uart_send;
    uloop_timeout_set(&timeout, 3 * 1000);

    return 0;
}