#include <edge.h>
#include "edev.h"
#include "edrv.h"
#include <config.h>
#include <linklist.h>
#include <libubox/avl.h>
#include <libubox/avl-cmp.h>
#include <debug.h>

struct avl_tree edge_drivers;

void edrv_register(struct edge_driver *drv)
{
    drv->node.key = drv->name;
    avl_insert(&edge_drivers, &drv->node);
}

void edrv_unregister(struct edge_driver *drv)
{
    avl_delete(&edge_drivers, &drv->node);
}

void edrv_set_main_dev(struct edge_driver *drv, struct edge_device *mdev)
{
    drv->mdev = mdev;
}

struct edge_device *edrv_get_main_dev(struct edge_driver *drv)
{
    return drv->mdev;
}

int edrv_add_auxiliary_dev(struct edge_driver *drv, int index, struct edge_device *adev)
{
    if (vector_slot(drv->adev, index))
        return -1;

    vector_set_index(drv->adev, index, adev);

    return 0;
}

struct edge_device *edrv_get_auxiliary_dev(struct edge_driver *drv, int index)
{
    return vector_slot(drv->adev, index);
}

struct edge_driver *edrv_get_drv(char *name)
{
    struct edge_driver *drv;

    if (!name)
        return NULL;

    return avl_find_element(&edge_drivers, name, drv, node);
}

EDEV_INIT(edrv_preinit)
{
    avl_init(&edge_drivers, avl_strcmp, false, NULL);
    if (DEBUG(drv, DRV))
        ULOG_DEBUG("(1) initialize edrv\n");
}

int edrv_init(struct edge_context *edge)
{
    struct edge_driver *drv;

    config_parse_edge();

    avl_for_each_element(&edge_drivers, drv, node) {
		if (!drv->init)
			continue;
        drv->init(drv);
	}

    return 0;
}

int edrv_fini(struct edge_context *edge)
{
    struct edge_driver *drv;

    avl_for_each_element(&edge_drivers, drv, node) {
		if (!drv->fini)
			continue;
        drv->fini(drv);
	}

    return 0;
}
