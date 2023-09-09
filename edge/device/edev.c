#include <ctype.h>
#include <assert.h>
#include <edge.h>
#include <edev.h>
#include <edrv.h>
#include <adapter.h>
#include <config.h>
#include <libubox/avl.h>
#include <libubox/avl-cmp.h>
#include <debug.h>

struct device_desc dev_type_desc[] = {
    DESC(UNKNOWN),
    DESC(GPIO),
    DESC(SERIAL),
    DESC(SPI),
},

dev_proto_desc[] = {
    DESC(USB),
    DESC(MODBUS),
};

struct avl_tree edge_devices;

void edev_register(struct edge_device *dev)
{
    dev->node.key = dev->name;
    avl_insert(&edge_devices, &dev->node);
}

void edev_unregister(struct edge_device *dev)
{
    avl_delete(&edge_devices, &dev->node);
}

static int edev_set_adapter(struct edge_device *dev)
{
    struct edge_adapter *adapter = adapter_request(dev->type);

    if (adapter)
        dev->adapter = adapter;

    return 0;
}

struct edge_adapter *edev_get_adapter(struct edge_device *dev)
{
    return dev->adapter;
}

int edev_create_dev(int type, char *name, void *conf)
{
    struct edge_device *dev;

    dev = calloc(1, sizeof(struct edge_device));

    if (!dev) {
        ULOG_ERR("edev create dev failed");
        return -1;
    }

    dev->type = type;
    dev->name = name;
    dev->conf = conf;

    edev_register(dev);
    edev_set_adapter(dev);

    ULOG_DEBUG("create dev: %s\n", name);

    return 0;
}

struct edge_device *edev_get_dev(char *name)
{
    struct edge_device *dev;

    if (!name)
        return NULL;

    return avl_find_element(&edge_devices, name, dev, node);
}

void edev_set_drv(struct edge_device *dev, struct edge_driver *drv)
{
    dev->drv = drv;
}

int edev_get_type(char *type)
{
    int i, len;
    char uppercase[32] = {0};

    len = strlen(type);
    len = len > 32 ? 32 : len;

    for (i = 0; i < len; i++) {
        uppercase[i] = toupper(type[i]);
    }

    //ULOG_DEBUG("type(%d): %s", len, uppercase);

    for (i = UNKNOWN; i < __DEV_TYPE_MAX; i++) {
        //ULOG_DEBUG("cmp: %s -> %s", dev_type_desc[i].name, uppercase);
        if (strncmp(dev_type_desc[i].name, uppercase, len) == 0)
            return dev_type_desc[i].number;
    }

    return UNKNOWN;
}

int edev_get_proto(char *type)
{
    int i, len;
    char uppercase[32];

    len = strlen(type);
    len = len > 32 ? 32 : len;

    for (i = 0; i < len; i++) {
        uppercase[i] = toupper(type[i]);
    }


    for (i = UNKNOWN; i < __DEV_TYPE_MAX; i++) {
        if (strncpy(dev_proto_desc[i].name, uppercase, len) == 0)
            return dev_proto_desc[i].number;
    }

    return UNKNOWN;
}

EDEV_INIT(edev_preinit)
{
    avl_init(&edge_devices, avl_strcmp, false, NULL);
    if (DEBUG(dev, DEV))
        ULOG_DEBUG("(1) initialize edev\n");
}

int edev_init(struct edge_context *edge)
{
    config_parse_device();

    return 0;
}
