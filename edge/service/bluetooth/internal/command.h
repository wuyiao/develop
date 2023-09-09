#include <sys/types.h>
#define INS_SEND    0xc0
#define INS_RECV    0x40
#define INS_SIG_SEND    0xc1
#define INS_SIG_RECV    0x41
#define INS_TRANSPARENT_DATA_SEND   0xc2
#define INS_TRANSPARENT_DATA_RECV   0x42
#define INS_RESET_OR_RESTART  0x43

#define ECODE_BASE      0xf7
#define ECODE_MAX       0xff

enum {
    PW_P8DBM,
    PW_P4DBM,
    PW_0DBM,
    PW_N4DBM,
    PW_N10DBM,
    PW_N14DBM,
    PW_N20DBM,
    PW_N24DBM,
    PW_N28DBM,
    PW_N30DBM,
    PW_N37DBM,
    PW_MAX
};

enum {
    OPCODE_CONF_NK,                 /* 0x00 */
    OPCODE_QUERY_NK,                /* 0x01 */
    OPCODE_CONF_AK,                 /* 0x02 */
    OPCODE_QUERY_AK,                /* 0x03 */
    OPCODE_CONF_NA,                 /* 0x04 */
    OPCODE_QUERY_NA,                /* 0x05 */
    OPCODE_CONF_ACL,                /* 0x06 */
    OPCODE_DEL_ACL,                 /* 0x07 */
    OPCODE_QUERY_ACL,               /* 0x08 */
    OPCODE_ACCESS_NWK,               /* 0x09 */
    OPCODE_CONF_BR = 0x0e,          /* 0x0e */
    OPCODE_QUERY_BR,                /* 0x0f */
    OPCODE_QUERY_MAC = 0x11,        /* 0x11 */
    OPCODE_CONF_PW,                 /* 0x12 */
    OPCODE_QUERY_PW,                /* 0x13 */
    OPCODE_RESTART,                 /* 0x14 */
    OPCODE_RESET,                   /* 0x15 */
    OPCODE_QUERY_VER,               /* 0x16 */
    OPCODE_MAX
};

struct bt_provisioner;

u_int8_t *splice_cmd(u_int8_t *cmd, ...);
int write_cmd(int fd, u_int8_t *cmd, u_int8_t *data, int len);
int read_cmd(int fd, u_int8_t *cmd, u_int8_t *data, int len);

char *ins_to_str(u_int8_t ins);
char *ecode_to_str(u_int8_t ecode);
char *event_to_str(u_int8_t event);
char *power_to_str(u_int8_t pw);
char *opcode_to_str(u_int8_t opcode);

void bt_config_netkey(struct bt_provisioner *bt);
void bt_config_appkey(struct bt_provisioner *bt);
void bt_config_unicast_addr(struct bt_provisioner *bt);
void bt_config_power(struct bt_provisioner *bt);
void bt_query_netkey(struct bt_provisioner *bt);
void bt_query_appkey(struct bt_provisioner *bt);
void bt_query_unicast_addr(struct bt_provisioner *bt);
void bt_query_power(struct bt_provisioner *bt);
void bt_query_mac(struct bt_provisioner *bt);
void bt_query_version(struct bt_provisioner *bt);
void bt_access_network(struct bt_provisioner *bt);
void bt_restart(struct bt_provisioner *bt);
void bt_reset(struct bt_provisioner *bt);
void bt_query_commander(struct bt_provisioner *bt, u_int8_t opcode);
int bt_send_msg_to_node(struct bt_provisioner *bt, u_int16_t na, void *data, int len);
int bt_prepare_delete_node(struct bt_provisioner *bt);
int bt_delete_node(struct bt_provisioner *bt);
