#ifndef _EDGE_DEBUG_H_
#define _EDGE_DEBUG_H_
#include <libubox/ulog.h>

#define DEBUG_EDGE      0x01
#define DEBUG_EDGE_ADAPTER 0x01

#define DEBUG_EDGE_INIT  0x01

#define DEBUG_EDGE_DEV   0x01
#define DEBUG_EDGE_DRV   0x01

#define DEBUG_EDGE_SERVICE_LORA 0x01


//#define LOG_DEBUG   DEBUG_LEVEL

#define DEBUG(m, p) \
    (debug_edge_##m && DEBUG_EDGE_##p)

#define DEBUG_ON(module, part) \
    debug_edge_##module |= (DEBUG_EDGE_##part)
#define DEBUG_OFF(module, part) \
    debug_edge_##module &= ~(DEBUG_EDGE_##part)
#define DPRINT()

#if (DEBUG_LEVEL >= LOG_INFO)
#endif

#define ULOG_DEBUG(fmt, ...) ulog(LOG_DEBUG, fmt, ## __VA_ARGS__)

extern unsigned long debug_edge;
extern unsigned long debug_edge_init;
extern unsigned long debug_edge_sb;
extern unsigned long debug_edge_dev;
extern unsigned long debug_edge_drv;
extern unsigned long debug_edge_service;
extern unsigned long debug_edge_service_lora;

int debug_init(void);
#endif

