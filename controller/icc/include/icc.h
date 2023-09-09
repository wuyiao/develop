#ifndef PPD_IPC_H
#define PPD_IPC_H

#include <sys/types.h>
#include <sys/socket.h>
#include <ipc.h>

struct mcu_chip_uid {
    uint32_t uid[3];
};

union preamble {
    struct {
        uint16_t vp:10;
        uint16_t res:6;
    } bits;
    uint16_t data;
};

union state {
    struct {
        uint16_t ZW_F:1;
        uint16_t MW_F:1;
        uint16_t ND_F:1;
        uint16_t ZW_H:1;
        uint16_t ZW_L:1;
        uint16_t MW_H:1;

        uint16_t ND_H:1;
        uint16_t ND_L:1;
        uint16_t M_S:1;
        uint16_t SW_L:1;
        uint16_t MJ:1;
        uint16_t ZM:1;
    } bits;
    uint16_t data;
};

int ipc_init(void);
#endif
