#ifndef __CONTEXT_H__
#define __CONTEXT_H__

typedef struct device_fd_s
{
    int uart_fd;
}device_fd_t;

typedef struct context_s
{
    device_fd_t device_fd;
}context_t;


context_t* init_context();
int uninit_context(context_t* ctx);

#endif