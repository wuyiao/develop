/* Edge Sensor Network Subsystem
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>

#include <debug.h>
#include <edge.h>
#include <icc.h>
#include <ubus.h>
#include <edev.h>
#include <edrv.h>
#include <config.h>
#include <softbus.h>
#include <service.h>
#include "context.h"
#include "uart.h"

//#include <component.h>

struct edge_context edge_ctx;

int main(int argc, char **argv)
{
// 	context_t *ctx = init_context();
// 	if(ctx == NULL)
// 	{
// 		goto __end;
// 	}
//     debug_init();
//     vgus_uart_init(ctx);

//     while (1) 
// 	{
// 		sleep(3);
// 	}

// __end:
// 	uninit_context(ctx);
    struct edge_context *edge;
    edge = &edge_ctx;

    ULOG_DEBUG("version: gcc %s\n", __VERSION__);
    ULOG_DEBUG("compile: %s %s\n", __TIME__, __DATE__);
    ULOG_DEBUG("edge start\n");

    uloop_init();

    schedule_init(&edge->s);

    debug_init();

    ubus_init();

    softbus_init();

    icc_init();

    config_init(edge);

    edev_init(edge);
    edrv_init(edge);
    service_init(edge);

    softbus_dump_all_chain();
    softbus_dump_remain_sink();
    //uhdps_init();
    //component_load();

    // lorae22_init();

    ULOG_DEBUG("init done\n");


    uloop_run();
    ulog_close();
    edrv_fini(edge);

    return 0;
}
