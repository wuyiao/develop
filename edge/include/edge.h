#ifndef _EDGE_H_
#define _EDGE_H_
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <fcntl.h>
#include <libubox/ulog.h>
#include <libubox/utils.h>
#include <libubox/uloop.h>
#include <libubus.h>
#include <linklist.h>
#include "thread.h"

struct edge_context {
    struct uci_context *uci_ctx;
    struct uci_package *uci_device;
    struct uci_package *uci_edge;
    struct schedule s;
};

typedef void (*init_t)(void);
typedef void (*exit_t)(void);

#define SANITY_CHECK(cond) \
{ \
    if (!cond) { \
        ULOG_ERR("sanity check failed with %s at %s:%d", #cond, __FUNCTION__, __LINE__); \
    } \
}

#define XDESC(a, b) { .number = a, .name = b }
#define DESC(a) XDESC(a, #a)

#ifndef __INIT1
#define __INIT1 __attribute__((constructor (101)))
#endif

#ifndef __INIT2
#define __INIT2 __attribute__((constructor (102)))
#endif

#ifndef __INIT3
#define __INIT3 __attribute__((constructor (103)))
#endif

#ifndef __INIT4
#define __INIT4 __attribute__((constructor (104)))
#endif

#ifndef __FINI1
#define __FINI1 __attribute__((destructor (101)))
#endif

#ifndef __FINI2
#define __FINI2 __attribute__((destructor (102)))
#endif

#ifndef __FINI3
#define __FINI3 __attribute__((destructor (103)))
#endif

#ifndef __FINI4
#define __FINI4 __attribute__((destructor (104)))
#endif

#define WEAK __attribute__((weak))

#define __define_init(fn, id) \
	static void __INIT##id __init_##fn##id (void)

#define __define_fini(fn, id) \
	static void __FINI##id __fini_##fn##id (void)
    

#ifndef INIT
#define INIT(fn, id)    __define_init(fn, id) 
#endif

#ifndef FINI
#define FINI(fn, id)    __define_fini(fn, id) 
#endif

#define INIT_DECLARE(fn, id)    __define_init(fn, id)
#define FINI_DECLARE(fn, id)    __define_fini(fn, id) 

extern struct edge_context edge_ctx;
#endif
