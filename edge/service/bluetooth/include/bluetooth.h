#include <sys/types.h>
#include <hlist.h>
#include "edge.h"

struct bt_provisioner {
    u_int8_t nk[32];
    u_int8_t ak[32];
    u_int8_t na[2];
    u_int8_t pw;
    u_int8_t mac[6];
    u_int8_t ver[2];
    u_int8_t state;
    u_int8_t event;
    int ttl;
    int state_change;
    int expection_bytes;
    int received_bytes;
    int wait_left_bytes;
    u_int8_t buf[256];
    struct uloop_fd serial;
};

//typedef struct bt_provisioner;

struct bt_mesh_node {
    struct hlist_node hnode;
    u_int8_t na[2];
    u_int8_t mac[6];
    int status;
    int old_status;
    /*  */
    int dying_score;
    int received_packets;
    int pps;
    int keepalive;
    int print_limit;
};

enum {
    NODE_UNKNOWN,
    NODE_ONLINE,
    NODE_OFFLINE,
    NODE_DISCOVERY,
    NODE_JOINED,
};

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

#define BUCKETSIZE              (jhash_size(8))
#define BUCKETMASK              (jhash_mask(8))

#define SET_BIT(v, b)           ((v) |= (1 << (b)))
#define ERASE_BIT(v, b)         ((v) &= ~(1 << (b)))
#define BIT(v, b)               ((v) & (1 << (b)))


int bluetooth_init(void);
int serial_reset(struct bt_provisioner *bt);
int bt_access_network_timer_start(struct bt_provisioner *bt);
void bt_access_network_defer_start(struct bt_provisioner *bt);
int bt_mesh_node_add(struct bt_mesh_node *node);
int bt_broadbast_msg(struct bt_provisioner *bt, void *data, int len);
void bt_wait_respawn_start(struct bt_provisioner *bt);

