#include <debug.h>
#include <syslog.h>
#include <libubox/ulog.h>

unsigned long debug_edge;
unsigned long debug_edge_init;
unsigned long debug_edge_sb;
unsigned long debug_edge_dev;
unsigned long debug_edge_drv;
unsigned long debug_edge_service;
unsigned long debug_edge_service_lora;

int debug_init()
{
    ulog_open(ULOG_SYSLOG, LOG_USER, "edge");
    //ulog_open(ULOG_STDIO, LOG_USER, "edge");
    ulog_threshold(LOG_DEBUG);
    //DEBUG_ON(gsm, GSM_EVENTS);

    return 0;
}
