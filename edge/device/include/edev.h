#ifndef _EDGE_EDEV_H_
#define _EDGE_EDEV_H_

#include <edge.h>
#include <linklist.h>
#include <libubox/avl.h>
#include <libubox/avl-cmp.h>

enum {
    UNKNOWN,
    GPIO,
    SERIAL,
    SPI,
    __DEV_TYPE_MAX
};

enum {
    USB,
    MODBUS,
    __DEV_PROTO_MAX
};

#define EDEV_INIT(fn)   INIT(fn, 1)
struct edge_context;

struct device_desc {
    unsigned number;
    char name[32];
}; 

struct edge_device {
    struct avl_node node;
    int index;
    /* name as searching index
     * the name of a device instance must be unique across the whole device framework
     */
    const char *name;
    /* indicate a device class, the name of a device instance must be unique across the whole device framwork */
    char class[32];
    /* device fd */
    int fd[8];
    /* number of used fd */
    int used_fd;
    /* device type */
    int type;
    //int proto;
    void *conf;
    /* the device is in use or not ? */
    int flag;
    int detected;
    void *data;
    struct edge_driver *drv;
    struct edge_adapter *adapter;
    int (*detect)(struct edge_device *dev);
    int (*init)(void);
};

extern struct device_desc dev_type_desc[];
extern struct list edev_list;
extern struct avl_tree edge_devices;

int edev_init(struct edge_context *edge);
void edev_register(struct edge_device *dev);
void edev_unregister(struct edge_device *dev);

int edev_get_type(char *type);
int edev_get_proto(char *type);
struct edge_adapter *edev_get_adapter(struct edge_device *dev);
int edev_create_dev(int type, char *name, void *conf);
struct edge_device *edev_get_dev(char *name);
void edev_set_drv(struct edge_device *dev, struct edge_driver *drv);

#endif
