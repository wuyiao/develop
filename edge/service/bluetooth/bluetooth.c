#include <stdio.h>
#include <jhash.h>
#include <json-c/json.h>
#include <tlv_box.h>
#include "edge.h"
#include "edev.h"
#include "ipc.h"
#include "bluetooth.h"
#include "command.h"
#include "gsm.h"

static struct edge_device dev = {.name = "BT"};
static struct bt_provisioner provisioner;
static struct uloop_timeout bt_access_network_timeout;
static struct uloop_timeout bt_mesh_node_keepalive_timeout;
static struct uloop_timeout bt_reset_timeout;
static struct hlist_head rtu[BUCKETSIZE];

int bt_mesh_node_add(struct bt_mesh_node *node)
{
    int index;

    index = jhash(node->mac, 6, 0) & BUCKETMASK;
    INIT_HLIST_NODE(&node->hnode);
    hlist_add_head(&node->hnode, &rtu[index]);

    return 0;
}

int bt_access_network_timer_start(struct bt_provisioner *bt)
{
    return uloop_timeout_remaining(&bt_access_network_timeout) > 0 ? true : false;
}

void bt_access_network_defer_start(struct bt_provisioner *bt)
{
    /* one minute again */
    DBG_INFO("bt access network defer timer start");
    uloop_timeout_set(&bt_access_network_timeout, 60 * 1000);
}

static void bt_access_network_retry(struct uloop_timeout *timeout)
{
    //BT_GSM_EVENT_SCHEDULE(&provisioner, GCEV_NW);
    bt_access_network(&provisioner);
    bt_access_network_defer_start(&provisioner);
}

static void bt_mesh_node_keepalive_handler(struct uloop_timeout *timeout)
{
    int i;
    struct bt_mesh_node *node;
    struct hlist_node *hnode;

    for (i = 0; i < BUCKETSIZE; i++) {
        hlist_for_each_entry_safe(node, hnode, &rtu[i], hnode) {

            node->received_packets += node->pps;
            node->keepalive++;


            if (!node->pps) {
                node->dying_score++;
            }
            else {
                node->dying_score = 0;
            }

            if (node->dying_score >= provisioner.ttl) {
                ERASE_BIT(node->status, NODE_ONLINE);
                ERASE_BIT(node->status, NODE_JOINED);
                SET_BIT(node->status, NODE_OFFLINE);
                SET_BIT(node->status, NODE_DISCOVERY);
            }

            u_int8_t data[64] = {0};

            data[0] = 0x01;

            if ((BIT(node->status, NODE_ONLINE) && (BIT(node->status, NODE_JOINED) || BIT(node->status, NODE_DISCOVERY))) || 
                    (BIT(node->status, NODE_OFFLINE) && (node->dying_score <= 120))) {
                if (node->keepalive >= 20) {
                    node->keepalive = 0;
                    DBG_INFO("sending KEEPALIVE to node ["MACSTR" %02x%02x]", MAC2STR(node->mac), node->na[0], node->na[1]);
                    bt_send_msg_to_node(&provisioner, *(u_int16_t *)node->na, data, 1);
                }
            }

            if (node->status != node->old_status) {
                DBG_INFO("["MACSTR" %02x%02x]: going %s", MAC2STR(node->mac), node->na[0], node->na[1], (BIT(node->status, NODE_OFFLINE)) ? "OFFLINE" : "ONLINE");
                node->old_status = node->status;
            }

            if (node->print_limit++ >= 10) {
                DBG_INFO("["MACSTR" %02x%02x]: pps(%d) received_packets(%d) dying score(%d) status(%x)", 
                    MAC2STR(node->mac), node->na[0], node->na[1], node->pps, node->received_packets, node->dying_score, node->status);
                node->print_limit = 0;
            }

            node->pps = 0;

            /* three mintues later, delete it */
            if (node->dying_score >= 180) {
                DBG_INFO("["MACSTR" %02x%02x]: deleted", MAC2STR(node->mac), node->na[0], node->na[1]);
                hlist_del(&node->hnode);
                free(node);
            }
        }
    }

    uloop_timeout_set(&bt_mesh_node_keepalive_timeout, 1 * 1000);
}

void bt_wait_respawn_start(struct bt_provisioner *bt)
{
    uloop_timeout_set(&bt_reset_timeout, 5 * 1000);
}

static void bt_reset_timeout_handler(struct uloop_timeout *timeout)
{
    bt_gsm_enter_state(&provisioner, GCSM_QUERY, GCEV_INIT);
}

static void bt_event_process(u_int8_t opcode, u_int8_t *data, int len)
{
    switch (opcode) {
        case OPCODE_CONF_NK:
            break;
        case OPCODE_QUERY_NK:
            memcpy(provisioner.nk, data, 16);
            BT_GSM_EVENT_SCHEDULE(&provisioner, GCEV_NK_RECVED);
            break;
        case OPCODE_CONF_AK:
            break;
        case OPCODE_QUERY_AK:
            memcpy(provisioner.nk, data, 16);
            BT_GSM_EVENT_SCHEDULE(&provisioner, GCEV_AK_RECVED);
            break;
        case OPCODE_CONF_NA:
            break;
        case OPCODE_QUERY_NA:
            provisioner.na[0] = data[0];
            provisioner.na[1] = data[1];
            BT_GSM_EVENT_SCHEDULE(&provisioner, GCEV_NA_RECVED);
            break;
        case OPCODE_CONF_ACL:
            break;
        case OPCODE_DEL_ACL:
            break;
        case OPCODE_QUERY_ACL:
            break;
        case OPCODE_ACCESS_NWK:
            {
                int i;
                u_int8_t matched = 0;
                u_int8_t na[2] = {0};
                struct bt_mesh_node *node;

                for (i = 0; i < BUCKETSIZE; i++) {
                    hlist_for_each_entry(node, &rtu[i], hnode) {
                        //DBG_INFO("compare: %d %d", *(short *)node->na, *(short *)na);
                        if (memcmp(node->mac, data, 6) == 0) {
                            na[0] = data[6];
                            na[1] = data[7];

                            if (*(short *)node->na != *(short *)na) {
                                node->na[0] = na[0];
                                node->na[1] = na[1];
                                DBG_WARNING("["MACSTR" %02x%02x]: rejoin the network, na changed", MAC2STR(node->mac), na[0], na[1]);
                            }
                            else {
                                DBG_INFO("["MACSTR" %02x%02x]: rejoin the network, na unchange", MAC2STR(node->mac), node->na[0], node->na[1]);
                            }

                            if (BIT(node->status, NODE_OFFLINE)) {
                                ERASE_BIT(node->status, NODE_OFFLINE);
                                SET_BIT(node->status, NODE_ONLINE);
                            }

                            if (!BIT(node->status, NODE_JOINED))
                                SET_BIT(node->status, NODE_JOINED);

                            matched = 1;
                        }
                    }
                }

                if (!matched) {
                    struct bt_mesh_node *node = calloc(1, sizeof(struct bt_mesh_node));
                    if (node) {
                        memcpy(node->mac, data, 6);
                        node->na[0] = data[6];
                        node->na[1] = data[7];
                        bt_mesh_node_add(node);
                        SET_BIT(node->status, NODE_ONLINE);
                        SET_BIT(node->status, NODE_JOINED);
                        node->old_status = node->status;
                        DBG_INFO("["MACSTR" %02x%02x] new node is joining the network", MAC2STR(node->mac), node->na[0], node->na[1]);
                    }
                    else {
                        DBG_ERR("node alloc failed");
                    }
                }

                BT_GSM_EVENT_SCHEDULE(&provisioner, GCEV_NN_RECVED);
            }

            break;
        case OPCODE_CONF_BR:
            break;
        case OPCODE_QUERY_BR:
            break;
        case OPCODE_QUERY_MAC:
            memcpy(provisioner.mac, data, 6);
            BT_GSM_EVENT_SCHEDULE(&provisioner, GCEV_MAC_RECVED);
            break;
        case OPCODE_CONF_PW:
            break;
        case OPCODE_QUERY_PW:
            provisioner.pw = data[0];
            BT_GSM_EVENT_SCHEDULE(&provisioner, GCEV_PW_RECVED);
            break;
        case OPCODE_RESTART:
            break;
        case OPCODE_RESET:
            BT_GSM_EVENT_SCHEDULE(&provisioner, GCEV_RESET_END);
            break;
        case OPCODE_QUERY_VER:
            provisioner.ver[0] = data[0];
            provisioner.ver[1] = data[1];
            BT_GSM_EVENT_SCHEDULE(&provisioner, GCEV_VER_RECVED);
            break;
        default:
            break;
    }

}

static void bt_sig_msg_process(u_int8_t opcode, u_int8_t *data, int len)
{
    int i;
    int matched = 0;
    u_int8_t na[2] = {opcode, data[0]};
    struct bt_mesh_node *node;

    for (i = 0; i < BUCKETSIZE; i++) {
        hlist_for_each_entry(node, &rtu[i], hnode) {
            if (*(short *)node->na == *(short *)na) {
                matched = 1;
            }
        }
    }

    if (matched)
        DBG_WARNING("["MACSTR" %02x%02x]: SIG notify node deleted", MAC2STR(node->mac), na[0], na[1]);
    else
        DBG_WARNING("[UNKOWN %02x%02x]: SIG notify node deleted", na[0], na[1]);
}

static void bt_transparent_data_process(u_int8_t *mac, u_int8_t *data, int len)
{
    //int i;
    int rc; 
    //char buf[256] = {0};

    // for (i = 0; i < len; i++) {
        // sprintf(buf + i * 2, "%02x", data[i]);
    // }

    // DBG_INFO("["MACSTR"]: %s", MAC2STR(mac), buf);

    tlv_box_t *box = tlv_box_create();
    tlv_box_put_bytes(box, 0xc0, mac, 6);
    tlv_box_put_bytes(box, 0xc1, data, len);

    if (tlv_box_serialize(box) != 0) {
        DBG_ERR("tlv serialize failed");
        return;
    }

    tlv_box_t *boxes = tlv_box_create();
    tlv_box_put_object(boxes, 0xe1, box);

    if (tlv_box_serialize(boxes) != 0) {
        DBG_ERR("tlv serialize failed");
        return;
    }

    rc = ipc_stream_write(tlv_box_get_buffer(boxes), tlv_box_get_size(boxes));
    DBG_INFO("ipc stream write %d", rc);

    tlv_box_destroy(box);
    tlv_box_destroy(boxes);
}

static void serial_cb(struct uloop_fd *fd, unsigned int events)
{
    u_int8_t len = 0;
    u_int8_t ins = 0;
    u_int8_t opcode = 0;
    u_int8_t ecode = 0;
    u_int8_t na[2] = {0};
    u_int8_t event = GCEV_NOEVT;

    int i, nbytes;
    u_int8_t cmd[32] = {0};
    /* serial MTU roughly 80 bytes, it should be safe here */
    u_int8_t data_buf[128] = {0};
    u_int8_t *data = data_buf;

    char printbuf[256] = {0};

    /* when provisioner access to the network, scan around, 10s later,
     * if OK, it will continue process, otherwise serial returns the
     * error code, so this's the change to drive the bt state machine
     * */

    cmd[0] = 2;

    nbytes = read_cmd(fd->fd, cmd, data, sizeof(data_buf));
    if (nbytes < 0) {
        DBG_ERR("read_cmd: %s", strerror(errno));
    }
    else {
        DBG_INFO("read: %d bytes", nbytes); 
    }

    len = cmd[0];
    ins = cmd[1];
    opcode = cmd[2];
    ecode = data[0];

    DBG_INFO("len(%d) ins(%x:%s) opcode(%x:%s)", len, ins, ins_to_str(ins), opcode, opcode_to_str(opcode));

    for (i = 0; i < nbytes - 3; i++) {
        sprintf(printbuf + i * 2, "%02x", data[i]);
    }

    DBG_INFO("%s", printbuf);

    if (len != nbytes -1) {
        DBG_INFO("reading %d bytes, but it not the expected len %d", nbytes, len);
        return;
    }
#if 0
    if (provisioner.wait_left_bytes) {

        DBG_INFO("provisioner wait left bytes");
        memcpy(&provisioner.buf[provisioner.received_bytes], cmd, 3);
        if (nbytes - 3) {
            memcpy(&provisioner.buf[provisioner.received_bytes], data, nbytes);
        }

        // fix variables
        nbytes += provisioner.received_bytes;
        len = provisioner.expection_bytes;

        // finally, we got a compelte packet
        data = provisioner.buf;

        if (len != nbytes - 1) {
            DBG_ERR("bad packet, discard!");
        }
    }

    if (len != nbytes - 1) {
        provisioner.expection_bytes = len;
        provisioner.received_bytes = nbytes;
        provisioner.wait_left_bytes = 1;

        memcpy(provisioner.buf, cmd, 3);
        memcpy(&provisioner.buf[3], data, nbytes);

        DBG_INFO("reading %d bytes, but it not the expected len %d", nbytes, len);
        return;
    }
    else {
        provisioner.wait_left_bytes = 0;
        provisioner.received_bytes = 0;
        provisioner.expection_bytes = 0;
        DBG_INFO("reading %d bytes, expected len %d", nbytes, len);
    }
#endif

    switch (ins) {
        case INS_SEND:
            break;
        case INS_RECV:
            if ((len == 3 && nbytes == 4) &&
                (ecode >= ECODE_BASE && ecode <= ECODE_MAX)) {
                event = ecode - ECODE_BASE;
                DBG_INFO("catched exception: (%d) %s", event, event_to_str(event));
                provisioner.event = event;
                bt_gsm_event(&provisioner);
            }
            else {
                bt_event_process(opcode, data, len - 2);
            }

            break;
        case INS_SIG_SEND:
            break;
        case INS_SIG_RECV:
            bt_sig_msg_process(opcode, data, len -2);

            break;
        case INS_TRANSPARENT_DATA_SEND:
            break;
        case INS_TRANSPARENT_DATA_RECV:
            na[0] = opcode;
            na[1] = data[0];

            {
                struct bt_mesh_node *node;

                for (i = 0; i < BUCKETSIZE; i++) {
                    hlist_for_each_entry(node, &rtu[i], hnode) {
                        //DBG_INFO("compare: %d %d", *(short *)node->na, *(short *)na);
                        if (*(short *)node->na == *(short *)na) {
                            node->pps++;
                            if (node) {
                                DBG_INFO("["MACSTR" %02x%02x]: matched", MAC2STR(node->mac), node->na[0], node->na[1]);
                                bt_transparent_data_process(node->mac, data + 3, len - 5);

                                if (BIT(node->status, NODE_OFFLINE)) {
                                    ERASE_BIT(node->status, NODE_OFFLINE);
                                }

                                if (!BIT(node->status, NODE_ONLINE)) {
                                    SET_BIT(node->status, NODE_ONLINE);
                                }
                            }
                        }
                    }
                }
            }

            /* network byte order to host byte order. */
            //bt_transparent_data_process(ntohs(*(u_int16_t *)na), data + 3, len - 5);
            break;
        case INS_RESET_OR_RESTART:
            if ((len == 3 && nbytes == 4) &&
                (ecode >= ECODE_BASE && ecode <= ECODE_MAX)) {
                event = ecode - ECODE_BASE;
                DBG_INFO("catched exception: (%d) %s", event, event_to_str(event));
                provisioner.event = event;
                bt_gsm_event(&provisioner);
            }
            else {
                bt_event_process(opcode, data, len - 2);
            }

            break;
        default:
            DBG_ERR("unknown instruction: %d", ins);
            break;
    }
}

static void first_cb(struct uloop_fd *fd, unsigned int events)
{
    //int i;
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[128] = {0};

    /* we don't expect reading something here, but if it has, we
     * just print it */
    if (events & ULOOP_READ) {
        cmd[0] = 2;
        nbytes = read_cmd(fd->fd, cmd, data, sizeof(data));
        if (nbytes < 0) {
            DBG_INFO("read: %s", strerror(errno));
        }
        else
            DBG_INFO("read: %d bytes", nbytes); 

        DBG_INFO("len(%d) ins(%s) opcode(%s) ecode(%02x)", cmd[0], ins_to_str(cmd[1]), opcode_to_str(cmd[2]), data[0]);

        // for (i = 0; i < nbytes - 3; i++) {
            // printf("%x", data[i]);
            // fflush(stdout);
            // if (i % 16 == 0 && i !=0 )
                // printf("\n");
        // }
    }
    else {
        /* move to normal state */
        uloop_fd_delete(fd);
        fd->cb = serial_cb;
        uloop_fd_add(&provisioner.serial, ULOOP_READ);

        /* for first, or restart, whatever, we enter query state
        */
        //bt_gsm_enter_state(&provisioner, GCSM_QUERY, GCEV_INIT);
        bt_gsm_enter_state(&provisioner, GSM_RESET, GCEV_INIT);
    }
}

static int open_serial(struct bt_provisioner *bt)
{
    /* later need config for it */
    bt->serial.fd = etu_open_dev("/dev/ttyUSB0", 115200);
    if (bt->serial.fd < 0) {
        DBG_ERR("bluetooth open device 'ttyUSB0' failed!");
        return -1;
    }
    else
        DBG_INFO("bluetooth open serial: %d", bt->serial.fd);

    return 0;
}

int serial_reset(struct bt_provisioner *bt)
{
    uloop_fd_delete(&bt->serial);
    close(bt->serial.fd);
    open_serial(bt);
    bt->serial.cb = serial_cb;
    uloop_fd_add(&bt->serial, ULOOP_READ);

    return 0;
}


int bluetooth_init(void)
{
    int i;
    int rc;

    rc = open_serial(&provisioner);
    if (rc < 0)
        return -1;
    provisioner.serial.cb = first_cb;
    uloop_fd_add(&provisioner.serial, ULOOP_READ | ULOOP_WRITE);

    provisioner.ttl = 60;
    provisioner.wait_left_bytes = 0;

    for (i = 0; i < BUCKETSIZE; i++) {
        INIT_HLIST_HEAD(&rtu[i]);
    }

    /* initialize timer */
    memset(&bt_access_network_timeout, 0, sizeof(bt_access_network_timeout));
    bt_access_network_timeout.cb = bt_access_network_retry;

    memset(&bt_mesh_node_keepalive_timeout, 0, sizeof(bt_mesh_node_keepalive_timeout));
    bt_mesh_node_keepalive_timeout.cb = bt_mesh_node_keepalive_handler;
    uloop_timeout_set(&bt_mesh_node_keepalive_timeout, 1000);

    memset(&bt_reset_timeout, 0, sizeof(bt_reset_timeout));
    bt_reset_timeout.cb = bt_reset_timeout_handler;

    return 0;
}

static bluetooth_probe(struct edge_device *dev)
{
    return false;
}

static void _INIT init(void)
{
    memset(&dev, 0, sizeof(dev));

    dev.type = SERIAL:
    dev.init = bluetooth_init;
    dev.detect = bluetooth_probe;

    edev_register(&dev);
}
