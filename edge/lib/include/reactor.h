#ifndef _EDGE_REACTOR_H_
#define _EDGE_REACTOR_H_

#include <libubox/uloop.h>


typedef struct reactor {
    struct uloop_fd ufd;
    void (react)(struct uloop_fd *fd, unsigned int events);
} reactor_t;

/* dispatch instance for event handle */
typedef struct dispatcher {
} dispatcher_t;

#endif
