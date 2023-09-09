#ifndef _EDGE_EDRV_H_
#define _EDGE_EDRV_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <libubox/avl.h>
#include <libubox/avl-cmp.h>
#include <log.h>
#include <edge.h>
#include <vector.h>
//#include <radio.h>

#define EDRV_INIT(fn)   INIT(fn, 1) 
#define DRIVER_DECLARE(fn) INIT(fn, 2) 

struct edge_context;
struct edge_device;
struct pkt_tx_s;
struct pkt_rx_s;

struct edge_driver {
    struct avl_node node;
    /* the name of a driver instance must be unique across the whole driver framwork */
    char name[32];
    /* driver class */
    char class[32];
    /* */
    int type;
    /* driver conf */
    void *conf;
    /* for private using */
    void *data;
    /* flag */
    int initialized;

    /* auxiliary device we needed */
    vector adev;

    /* base or main device we derived */
    struct edge_device *mdev;
    /* driver initialization */
    int (*init)(struct edge_driver *drv);
    int (*fini)(struct edge_driver *drv);

    /* driver operations */
    void *ops;
};

extern struct list edrv_list;

int edrv_init(struct edge_context *edge);
int edrv_fini(struct edge_context *edge);
void edrv_register(struct edge_driver *drv);
void edrv_unregister(struct edge_driver *drv);
void edrv_set_main_dev(struct edge_driver *drv, struct edge_device *mdev);
struct edge_device *edrv_get_main_dev(struct edge_driver *drv);
int edrv_add_auxiliary_dev(struct edge_driver *drv, int index, struct edge_device *adev);
struct edge_device *edrv_get_auxiliary_dev(struct edge_driver *drv, int index);
struct edge_driver *edrv_get_drv(char *name);

#endif
#if 0
            void (*reset)(void (*callback)(void));
            int (*get_radio_version)(void);
            void (*rx)(uint8_t rxmode);
            int (*rx_irq)(void);
            void (*tx)(struct pkt_tx_s *pkt);
            int (*receive)(struct pkt_rx_s *pkt);
            void (*single_tx)(uint8_t *payload, int size);
            void (*setup_channel)(void);
#endif

