#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <math.h>
#include "bluetooth.h"
#include "command.h"
#include "gsm.h"

struct ecode_str {
    u_int8_t ecode;
    char estr[64];
} estring[] = {
    {ECODE_BASE, "The last networking has not ended"},
    {ECODE_BASE + 1, "Appkey bound failed"},
    {ECODE_BASE + 2, "Appkey bound timeout"},
    {ECODE_BASE + 3, "Netkey distribution failed"},
    {ECODE_BASE + 4, "Get parameters failed"},
    {ECODE_BASE + 5, "No networkable devieces are found"},
    {ECODE_BASE + 6, "Invalid command"},
    {ECODE_BASE + 7, "Operation busy"},
    {ECODE_BASE + 8, "Invalid parameters"},
};

struct pw_str {
    u_int8_t pw;
    char str[64];
} pw_string[] = {
    {PW_P8DBM, "+8dBm"},
    {PW_P4DBM, "+4dBm"},
    {PW_0DBM, "0dBm"},
    {PW_N4DBM, "-4dBm"},
    {PW_N10DBM, "-10dBm"},
    {PW_N14DBM, "-14dBm"},
    {PW_N20DBM, "-20dBm"},
    {PW_N24DBM, "-24dBm"},
    {PW_N28DBM, "-28dBm"},
    {PW_N30DBM, "-30dBm"},
    {PW_N37DBM, "-37dBm"},
};

struct opcode_str {
    u_int8_t opcode;
    char str[32];
} opcode_string[] = {
    {OPCODE_CONF_NK, "CONF NK"},
    {OPCODE_QUERY_NK, "QUERY NK"},
    {OPCODE_CONF_AK, "CONF AK"},
    {OPCODE_QUERY_AK, "QUERY AK"},
    {OPCODE_CONF_NA, "CONF NA"},
    {OPCODE_QUERY_NA, "QUERY NA"},
    {OPCODE_CONF_ACL, "CONF ACL"},
    {OPCODE_DEL_ACL, "DEL ACL"},
    {OPCODE_QUERY_ACL, "QUERY ACL"},
    {OPCODE_ACCESS_NWK, "ACCESS NWK"},
    {0x0a, ""},
    {0x0b, ""},
    {0x0c, ""},
    {0x0d, ""},
    {OPCODE_CONF_BR, "CONF BR"},
    {OPCODE_QUERY_BR, "QUERY BR"},
    {0x10, ""},
    {OPCODE_QUERY_MAC, "QUERY MAC"},
    {OPCODE_CONF_PW, "CONF PW"},
    {OPCODE_QUERY_PW, "QUERY PW"},
    {OPCODE_RESTART, "RESTART"},
    {OPCODE_RESET, "RESET"},
    {OPCODE_QUERY_VER, "QUERY VER"},
};

struct instruction_str {
    u_int8_t ins;
    char str[32];
} ins_string[] = {
    {INS_SEND, "SEND"},
    {INS_RECV, "RECV"},
    {INS_TRANSPARENT_DATA_SEND, "DATA SEND"},
    {INS_TRANSPARENT_DATA_RECV, "DATA RECV"},
    {INS_RESET_OR_RESTART, "RESET/RESTART/NWK"},
};

char *ecode_to_str(u_int8_t ecode)
{
    if (ecode >= ECODE_BASE && ecode <= ECODE_MAX)
        return estring[ecode - ECODE_BASE].estr;
    else
        return "";
}

char *event_to_str(u_int8_t event)
{
    if (event >= BT_GSM_EVENT_MAX)
        return "";
    else
        return estring[event].estr;
}

char *power_to_str(u_int8_t pw)
{
    if (pw < PW_MAX)
        return pw_string[pw].str;
    else
        return "";
}

char *opcode_to_str(u_int8_t opcode)
{
    if (opcode < OPCODE_MAX)
        return opcode_string[opcode].str;
    else
        return "";
}

char *ins_to_str(u_int8_t ins)
{
    int i;

    for (i = 0; i < sizeof(ins_string) / sizeof(struct instruction_str); i++)
    {
        if (ins == ins_string[i].ins)
            return ins_string[i].str;
    }

    return "";
}
 
static int hex_to_dec(int x)
{
    int decimal_number, remainder, count = 0;

    while(x > 0) {
        remainder = x % 10;
        decimal_number = decimal_number + remainder * pow(16, count);
        x = x / 10;
        count++;
    }

    return decimal_number;
}

u_int8_t *splice_cmd(u_int8_t *cmd, ...)
{
    va_list ap;
    u_int8_t c;
    u_int16_t s;
    u_int32_t l, len;

    /* preserve first byte */
    cmd[0] = 1;

    va_start(ap, cmd);
    len = va_arg(ap, int);

    while (len != 0) {
        switch (len) {
            case 1:
                c = (u_int8_t)va_arg(ap, int);
                cmd[cmd[0]++] = c;
                break;
            case 2:
                s = (u_int16_t)va_arg(ap, int);
                cmd[cmd[0]++] = (u_int8_t)(s & 0x00ff);
                cmd[cmd[0]++] = (u_int8_t)((s & 0xff00) >> 8);
                break;
            case 4:
                /* need a cast here since va_arg only
                   takes fully promoted types */
                l = (u_int32_t)va_arg(ap, int);
                cmd[cmd[0]++] = (u_int8_t)(l & 0x000000ff);
                cmd[cmd[0]++] = (u_int8_t)((l & 0x0000ff00) >> 8);
                cmd[cmd[0]++] = (u_int8_t)((l & 0x00ff0000) >> 16);
                cmd[cmd[0]++] = (u_int8_t)((l & 0xff000000) >> 24);
                break;
            default:
                /* for more data, copy */
                memcpy(&cmd[cmd[0]++], va_arg(ap, char *), l);
                break;
        }
        len = va_arg(ap, int);
    }
    va_end(ap);

    /* finished, we put length into first byte of cmd, exclude first byte */
    cmd[0] = cmd[0] - 1;

    return cmd;
}

int sanity_writev(int fd, struct iovec *iov, int iov_len)
{
    int len = 0;
    int write_len;

    do {
        write_len = writev(fd, iov, iov_len);

        if (write_len < 0) {
            if (errno == EINTR)
                continue;

            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOTCONN)
                break;

            return -1;
        }

        len += write_len;

        while (write_len >= iov->iov_len) {
            write_len -= iov->iov_len;
            iov_len--;
            iov++;

            if (!iov_len) {
                return len;
            }
        }

        iov->iov_base += write_len;
        iov->iov_len -= write_len;
    } while (1);

    return -1;
}

int sanity_readv(int fd, struct iovec *iov, int iov_len)
{
    int read_len; 

    do {
        read_len = readv(fd, iov, iov_len);
    } while (read_len < 0 && errno == EINTR);

    return read_len;
}

int write_cmd(int fd, u_int8_t *cmd, u_int8_t *data, int len)
{
    int res = 0;
    u_int8_t iov_len = 1;
    struct iovec iov[2];

    iov[0].iov_base = cmd;
    iov[0].iov_len = cmd[0] + 1;

    if (len > 0) {
        iov_len++;
        /* fixed total len */
        cmd[0] += len;
        iov[1].iov_base = data;
        iov[1].iov_len = len;
    }

    res = sanity_writev(fd, iov, iov_len);
    if (res < 0) {
        DBG_ERR("writev: %s", strerror(errno));
        return -1;
    }

    //tcdrain();
    return res;
}

int read_cmd(int fd, u_int8_t *cmd, u_int8_t *data, int len)
{
    int res = 0;
    u_int8_t iov_len = 1;
    struct iovec iov[3];

    iov[0].iov_base = cmd;
    iov[0].iov_len = cmd[0] + 1;

    if (len > 0) {
        iov_len++;
        iov[1].iov_base = data;
        iov[1].iov_len = len;
    }

    res = sanity_readv(fd, iov, iov_len);

    if (res < 0) {
        DBG_ERR("readv: %s", strerror(errno));
        return -1;
    }

    return res;
}

void bt_config_netkey(struct bt_provisioner *bt)
{
#define NK "abcdefghijklmnop"
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[64] = {NK};

    DBG_INFO("bt config NK: %s", NK);
    splice_cmd(cmd, 1, INS_SEND, 1, 0x00, 0);

    nbytes = write_cmd(bt->serial.fd, cmd, data, strlen((char *)data));
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 
}

void bt_config_appkey(struct bt_provisioner *bt)
{
#define AK "abcdefghijklmnop"
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[64] = {AK};

    DBG_INFO("bt config AK: %s\n", AK);
    splice_cmd(cmd, 1, INS_SEND, 1, 0x02, 0);

    nbytes = write_cmd(bt->serial.fd, cmd, data, strlen((char *)data));
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 
}

void bt_config_unicast_addr(struct bt_provisioner *bt)
{
#define UA "01"
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[64] = {0};

    DBG_INFO("bt config unicast address: %s", UA);
    splice_cmd(cmd, 1, INS_SEND, 1, 0x04, 0);
    data[0] = 1;

    nbytes = write_cmd(bt->serial.fd, cmd, data, strlen((char *)data));
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 
}

void bt_config_power(struct bt_provisioner *bt)
{
#define PW 0
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[64] = {0};

    DBG_INFO("bt config power: %s", power_to_str(PW));
    splice_cmd(cmd, 1, INS_SEND, 1, 0x12, 0);
    data[0] = PW;

    nbytes = write_cmd(bt->serial.fd, cmd, data, 1);
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 
}


void bt_query_netkey(struct bt_provisioner *bt)
{
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[64] = {0};

    DBG_INFO("bt query netkey");
    splice_cmd(cmd, 1, INS_SEND, 1, 0x01, 0);

    nbytes = write_cmd(bt->serial.fd, cmd, data, 0);
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 
}

void bt_query_appkey(struct bt_provisioner *bt)
{
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[64] = {0};

    DBG_INFO("bt query appkey");
    splice_cmd(cmd, 1, INS_SEND, 1, 0x03, 0);

    nbytes = write_cmd(bt->serial.fd, cmd, data, strlen((char *)data));
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 
}

void bt_query_unicast_addr(struct bt_provisioner *bt)
{
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[64] = {0};

    DBG_INFO("bt query unicast addr");
    splice_cmd(cmd, 1, INS_SEND, 1, 0x05, 0);

    nbytes = write_cmd(bt->serial.fd, cmd, data, strlen((char *)data));
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 
}

void bt_query_power(struct bt_provisioner *bt)
{
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[64] = {0};

    DBG_INFO("bt query power");
    splice_cmd(cmd, 1, INS_SEND, 1, 0x13, 0);

    nbytes = write_cmd(bt->serial.fd, cmd, data, strlen((char *)data));
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 
}

void bt_query_mac(struct bt_provisioner *bt)
{
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[64] = {0};

    DBG_INFO("bt query MAC");
    splice_cmd(cmd, 1, INS_SEND, 1, 0x11, 0);

    nbytes = write_cmd(bt->serial.fd, cmd, data, strlen((char *)data));
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 
}

void bt_query_version(struct bt_provisioner *bt)
{
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[64] = {0};

    DBG_INFO("bt query version");
    splice_cmd(cmd, 1, INS_SEND, 1, 0x16, 0);

    nbytes = write_cmd(bt->serial.fd, cmd, data, strlen((char *)data));
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 
}

void bt_access_network(struct bt_provisioner *bt)
{
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[64] = {0};
 
    DBG_INFO("bt access network");
    splice_cmd(cmd, 1, INS_SEND, 1, 0x09, 0);

    nbytes = write_cmd(bt->serial.fd, cmd, data, 0);
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 
}

void bt_restart(struct bt_provisioner *bt)
{
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[64] = {0};
 
    DBG_INFO("bt restart");
    splice_cmd(cmd, 1, INS_SEND, 1, 0x14, 0);

    nbytes = write_cmd(bt->serial.fd, cmd, data, 0);
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 
}

void bt_reset(struct bt_provisioner *bt)
{
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[64] = {0};
 
    DBG_INFO("bt reset");
    splice_cmd(cmd, 1, INS_SEND, 1, 0x15, 0);

    nbytes = write_cmd(bt->serial.fd, cmd, data, 0);
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 
}

void bt_query_commander(struct bt_provisioner *bt, u_int8_t opcode)
{
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[64] = {0};
 
    DBG_INFO("bt query commander: %s", opcode_to_str(opcode));
    splice_cmd(cmd, 1, INS_SEND, 1, opcode, 0);

    nbytes = write_cmd(bt->serial.fd, cmd, data, 0);
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 
}

int bt_send_msg_to_node(struct bt_provisioner *bt, u_int16_t na, void *data, int len)
{
    int nbytes;
    u_int8_t cmd[32] = {0};

    splice_cmd(cmd, 1, INS_TRANSPARENT_DATA_SEND, 2, na, 0);

    nbytes = write_cmd(bt->serial.fd, cmd, data, len);
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
        return -1;
    }
    // else
        // DBG_INFO("write: %d bytes", nbytes);

    return 0;
}

int bt_broadbast_msg(struct bt_provisioner *bt, void *data, int len)
{
    int nbytes;
    u_int8_t cmd[32] = {0};

    splice_cmd(cmd, 1, INS_TRANSPARENT_DATA_SEND, 2, 0xffff, 0);

    nbytes = write_cmd(bt->serial.fd, cmd, data, len);
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
        return -1;
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 

    return 0;
}

int bt_prepare_delete_node(struct bt_provisioner *bt)
{
    DBG_WARNING("waring: parepare for delete node");
    bt_query_unicast_addr(bt);
    return 0;
}

int bt_delete_node(struct bt_provisioner *bt)
{
    int nbytes;
    u_int8_t cmd[32] = {0};
    u_int8_t data[64] = {0};

    DBG_WARNING("waring: deleting node by SIG msg");
    splice_cmd(cmd, 1, INS_SIG_SEND, 2, (*(short *)bt->na) - 3, 2, 0x8049, 0);

    nbytes = write_cmd(bt->serial.fd, cmd, data, 0);
    if (nbytes < 0) {
        DBG_ERR("write: %s", strerror(errno));
        return -1;
    }
    else
        DBG_INFO("write: %d bytes", nbytes); 

    return 0;

}
