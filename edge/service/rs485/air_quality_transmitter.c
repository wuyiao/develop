#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <tlv_box.h>
#include <modbus/modbus.h>
#include <ei-dbg.h>

#include "edge.h"
#include "edev.h"
#include "ipc.h"

static modbus_t *ctx;
static struct edge_device dev = {.name = "AQT"};
static struct uloop_timeout read_loop_timeout;

static void modbus_read_cb(struct uloop_timeout *timeout)
{
    int rc;
    u_int16_t regs[3] = {0};

    rc = modbus_read_registers(ctx, 0x07, 3, regs);
    if (rc < 0) {
        DBG_ERR("read registers failed '%s'", strerror(errno));
        goto out;
    }

    DBG_INFO("read %d registers, [0]: %d, [1]: %d, [2]: %d", rc, regs[0], regs[1], regs[2]);

    tlv_box_t *box = tlv_box_create();
    tlv_box_put_bytes(box, 0xc0, regs, 6);

    if (tlv_box_serialize(box) != 0) {
        DBG_ERR("tlv serialize failed");
        return;
    }

    tlv_box_t *boxes = tlv_box_create();
    tlv_box_put_object(boxes, 0xe0, box);

    if (tlv_box_serialize(boxes) != 0) {
        DBG_ERR("tlv serialize failed");
        return;
    }

    rc = ipc_stream_write(tlv_box_get_buffer(boxes), tlv_box_get_size(boxes));
    DBG_INFO("aqt stream write %d", rc);

    tlv_box_destroy(box);
    tlv_box_destroy(boxes);
out:
    uloop_timeout_set(&read_loop_timeout, 10 * 1000);
}


static int modbus_init()
{
    int rc;
    int mode;
    int rts;

    ctx = modbus_new_rtu("/dev/ttymxc1", 9600, 'N', 8, 1);
    if (ctx == NULL) {
        DBG_ERR("Unable to create the libmodbus context");
        return -1;
    }

    rc = modbus_connect(ctx);
    if (rc < 0)
        DBG_ERR("modbus connect error");

    rc = modbus_set_response_timeout(ctx, 0, 200000);
    if (rc < 0) {
        DBG_ERR("modbus set response timeout error");
    }

    rc = modbus_set_slave(ctx, 0x01);
    if (rc < 0) {
        DBG_ERR("modbus set slave error");
    }

    mode = modbus_rtu_get_serial_mode(ctx);
    switch (mode) {
        case MODBUS_RTU_RS232:
            DBG_INFO("modbus serial RS232 mode %d", mode);
            break;
        case MODBUS_RTU_RS485:
            break;
    }

    rc = modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS485);
    if (rc < 0)
        DBG_INFO("modbus set serial RS485 mode error");

    rts = modbus_rtu_get_rts(ctx);
    
    DBG_INFO("modbus serial rts %d", rts);

    memset(&read_loop_timeout, 0, sizeof(read_loop_timeout));
    read_loop_timeout.cb = modbus_read_cb;
    uloop_timeout_set(&read_loop_timeout, 1000);

    return 0;
}

static int modbus_probe(struct edge_device *dev)
{
    return false;
}

static void _INIT init(void)
{
    memset(&dev, 0, sizeof(dev));

    dev.init = modbus_init;
    dev.probe = modbus_probe;

    edev_register(&dev);
}
