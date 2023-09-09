#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "aes.h"
#include "cmac.h"
#include "lm.h"
#include <utilities.h>
#include <linklist.h>
#include <debug.h>

void lm_cpy(uint8_t *dest, uint8_t *src, int len);
int lm_check_zero(uint8_t *src, int len);

int lm_mtype_join_accept(lm_frame_t *frame, lm_mote_t *cur, uint8_t *buf, int len);
int lm_mtype_join_request(lm_frame_t *frame, lm_mote_t *cur, uint8_t *buf, int len);
int lm_mtype_msg_up(lm_frame_t *frame, lm_mote_t *cur, uint8_t *buf, int len);
int lm_mtype_msg_down(lm_frame_t *frame, lm_mote_t *cur, uint8_t *buf, int len);
int lm_mtype_cmsg_up(lm_frame_t *frame, lm_mote_t *cur, uint8_t *buf, int len);
int lm_mtype_cmsg_down(lm_frame_t *frame, lm_mote_t *cur, uint8_t *buf, int len);
int lm_mtype_rfu(lm_frame_t *frame, lm_mote_t *cur, uint8_t *buf, int len);
int lm_mtype_proprietary(lm_frame_t *frame, lm_mote_t *cur, uint8_t *buf, int len);

typedef int (*lm_mtype_func_p) (lm_frame_t *frame, lm_mote_t *cur, uint8_t *buf, int len);

uint8_t lm_dft_nwkskey[16] = {
    0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3c
};

uint8_t lm_dft_appskey[16] = {
    0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3c
};

uint8_t lm_dft_appkey[16] = {
    0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3c
};

lm_dnonce_t lm_dft_dnonce;

lm_mote_t *lm_mote;
lm_mote_t *lm_mote_latest_jr;
//lm_mote_t lm_mote_buf[LM_MAX_NODES];
uint32_t lm_mote_num;
uint32_t lm_deveui_cnt;
uint32_t lm_devaddr_cnt;
lm_frame_t lm_dlframe;
lm_config_t lm_config;
bool lm_v11;
struct list mote_list;

const lm_mtype_func_p lmp_mtye_func[] = {
    lm_mtype_join_request,
    lm_mtype_join_accept,
    lm_mtype_msg_up,
    lm_mtype_msg_down,
    lm_mtype_cmsg_up,
    lm_mtype_cmsg_down,
    lm_mtype_rfu,
    lm_mtype_proprietary,
};

/* Data Rate Scheme */
const uint8_t lm_dr_tab_0[16] = {
    LM_DR(SF12, BW125),    // DR0
    LM_DR(SF11, BW125),    // DR1
    LM_DR(SF10, BW125),    // DR2
    LM_DR(SF9, BW125),     // DR3
    LM_DR(SF8, BW125),     // DR4
    LM_DR(SF7, BW125),     // DR5
    LM_DR(SF7, BW250),     // DR7
    LM_DR(FSK, BW125),     // DR8
    LM_DR_RFU,             // DR9
    LM_DR_RFU,             // DR10
    LM_DR_RFU,             // DR11
    LM_DR_RFU,             // DR12
    LM_DR_RFU,             // DR13
    LM_DR_RFU,             // DR14
    LM_DR_RFU,             // DR15
};
const uint8_t lm_dr_tab_1[16] = {

    LM_DR(SF10, BW125),    // DR0
    LM_DR(SF9, BW125),     // DR1
    LM_DR(SF8, BW125),     // DR2
    LM_DR(SF7, BW125),     // DR3
    LM_DR(SF8, BW500),     // DR4
    LM_DR_RFU,             // DR5
    LM_DR_RFU,             // DR6
    LM_DR_RFU,             // DR7
    LM_DR(SF12, BW500),    // DR8
    LM_DR(SF11, BW500),    // DR9
    LM_DR(SF10, BW500),    // DR10
    LM_DR(SF9, BW500),     // DR11
    LM_DR(SF8, BW500),     // DR12
    LM_DR(SF7, BW500),     // DR13
    LM_DR_RFU,             // DR14
    LM_DR_RFU,             // DR15
};
const uint8_t lm_dr_tab_2[16] = {
    LM_DR(SF12, BW125),    // DR0
    LM_DR(SF11, BW125),    // DR1
    LM_DR(SF10, BW125),    // DR2
    LM_DR(SF9, BW125),     // DR3
    LM_DR(SF8, BW125),     // DR4
    LM_DR(SF7, BW125),     // DR5
    LM_DR(SF8, BW500),     // DR6
    LM_DR_RFU,             // DR7
    LM_DR(SF12, BW500),    // DR8
    LM_DR(SF11, BW500),    // DR9
    LM_DR(SF10, BW500),    // DR10
    LM_DR(SF9, BW500),     // DR11
    LM_DR(SF8, BW500),     // DR12
    LM_DR(SF7, BW500),     // DR13
    LM_DR_RFU,             // DR14
    LM_DR_RFU,             // DR15
};
const uint8_t lm_dr_tab_3[16] = {
    LM_DR(SF12, BW125),    // DR0
    LM_DR(SF11, BW125),    // DR1
    LM_DR(SF10, BW125),    // DR2
    LM_DR(SF9, BW125),     // DR3
    LM_DR(SF8, BW125),     // DR4
    LM_DR(SF7, BW125),     // DR5
    LM_DR_RFU,             // DR7
    LM_DR_RFU,             // DR8
    LM_DR_RFU,             // DR9
    LM_DR_RFU,             // DR10
    LM_DR_RFU,             // DR11
    LM_DR_RFU,             // DR12
    LM_DR_RFU,             // DR13
    LM_DR_RFU,             // DR14
    LM_DR_RFU,             // DR15
};

/* LinkAdrReq ChmskCntl */
const uint16_t lm_chmaskcntl_tab_0[8] = {
    LM_CMC(0, 15),
    LM_CMC_RFU,
    LM_CMC_RFU,
    LM_CMC_RFU,
    LM_CMC_RFU,
    LM_CMC_RFU,
    LM_CMC_ALL_ON,
    LM_CMC_RFU,
};

const uint16_t lm_chmaskcntl_tab_1[8] = {
    LM_CMC(0, 15),
    LM_CMC(16, 31),
    LM_CMC(32, 47),
    LM_CMC(48, 63),
    LM_CMC(64, 71),
    LM_CMC_RFU,
    LM_CMC_ALL_125KHZ_ON,
    LM_CMC_ALL_125KHZ_OFF,
};

const uint16_t lm_chmaskcntl_tab_2[8] = {
    LM_CMC(0, 15),
    LM_CMC(16, 31),
    LM_CMC(32, 47),
    LM_CMC(48, 63),
    LM_CMC(64, 79),
    LM_CMC(80, 95),
    LM_CMC_ALL_ON,
    LM_CMC_RFU,
};

const int8_t lm_max_eirp_tab[16] = {
    8, 10, 12, 13, 14, 16, 18, 20, 21, 24, 26, 27, 29, 30, 33, 36
};

#define LM_DR_TAB_EU868                 lm_dr_tab_0
#define LM_DR_TAB_US915                 lm_dr_tab_1
#define LM_DR_TAB_CN779                 LM_DR_TAB_EU868
#define LM_DR_TAB_EU433                 LM_DR_TAB_EU868
#define LM_DR_TAB_AU915                 lm_dr_tab_2
#define LM_DR_TAB_CN470                 lm_dr_tab_3
#define LM_DR_TAB_AS923                 LM_DR_TAB_EU868
#define LM_DR_TAB_KR920                 LM_DR_TAB_CN470
#define LM_DR_TAB_IN865                 LM_DR_TAB_EU868
#define LM_DR_TAB_RU864                 LM_DR_TAB_EU868

#define LM_CHMSKCNTL_TAB_EU868          lm_chmaskcntl_tab_0
#define LM_CHMSKCNTL_TAB_US915          lm_chmaskcntl_tab_1
#define LM_CHMSKCNTL_TAB_CN779          LM_CHMSKCNTL_TAB_EU868
#define LM_CHMSKCNTL_TAB_EU433          LM_CHMSKCNTL_TAB_EU868
#define LM_CHMSKCNTL_TAB_AU915          LM_CHMSKCNTL_TAB_US915
#define LM_CHMSKCNTL_TAB_CN470          lm_chmaskcntl_tab_2
#define LM_CHMSKCNTL_TAB_AS923          LM_CHMSKCNTL_TAB_EU868
#define LM_CHMSKCNTL_TAB_KR920          LM_CHMSKCNTL_TAB_EU868
#define LM_CHMSKCNTL_TAB_IN865          LM_CHMSKCNTL_TAB_EU868
#define LM_CHMSKCNTL_TAB_RU864          LM_CHMSKCNTL_TAB_EU868

const lm_region_t lm_region_tab[] = {
    {
        EU868,
        "EU868",
        LM_DR_TAB_EU868,
        {5, 7},
        LM_CHMSKCNTL_TAB_EU868,
    },
    {
        US915,
        "US915",
        LM_DR_TAB_US915,
        {13, 10},
        LM_CHMSKCNTL_TAB_US915,
    },
    {
        CN779,
        "CN779",
        LM_DR_TAB_CN779,
        {2, 5},
        LM_CHMSKCNTL_TAB_CN779,
    },
    {
        EU433,
        "EU433",
        LM_DR_TAB_EU433,
        {2, 5},
        LM_CHMSKCNTL_TAB_EU433,
    },
    {
        AU915,
        "AU915",
        LM_DR_TAB_AU915,
        {13, 10},
        LM_CHMSKCNTL_TAB_AU915,
    },
    {
        CN470,
        "CN470",
        LM_DR_TAB_CN470,
        {7, 7},
        LM_CHMSKCNTL_TAB_CN470,
    },
    {
        AS923,
        "AS923",
        LM_DR_TAB_AS923,
        {5, 7},
        LM_CHMSKCNTL_TAB_AS923,
    },
    {
        KR920,
        "KR920",
        LM_DR_TAB_KR920,
        {4, 7},
        LM_CHMSKCNTL_TAB_KR920,
    },
    {
        IN865,
        "IN865",
        LM_DR_TAB_IN865,
        {13, 10},
        LM_CHMSKCNTL_TAB_IN865,
    },
    {
        RU864,
        "RU864",
        LM_DR_TAB_RU864,
        {5, 7},
        LM_CHMSKCNTL_TAB_RU864,
    },
};

const lm_region_t *lm_region;

int8_t lm_pow_tab[16] = {
    /* EU868 */
    20,
    14,
    11,
    8,
    5,
    2,
    LM_POW_RFU,
    LM_POW_RFU,
    LM_POW_RFU,
    LM_POW_RFU,
    LM_POW_RFU,
    LM_POW_RFU,
    LM_POW_RFU,
    LM_POW_RFU,
    LM_POW_RFU,
    LM_POW_RFU,
};

typedef struct {
    uint8_t cmd;
    uint8_t len;
} lm_maccmd_len_t;

const lm_maccmd_len_t lm_mote_maccmd_tab[] = {
    { MOTE_MAC_LINK_CHECK_REQ,          MOTE_MAC_LEN_LINK_CHECK_REQ },
    { MOTE_MAC_LINK_ADR_ANS,            MOTE_MAC_LEN_LINK_ADR_ANS },
    { MOTE_MAC_DUTY_CYCLE_ANS,          MOTE_MAC_LEN_DUTY_CYCLE_ANS },
    { MOTE_MAC_RX_PARAM_SETUP_ANS,      MOTE_MAC_LEN_RX_PARAM_SETUP_ANS },
    { MOTE_MAC_DEV_STATUS_ANS,          MOTE_MAC_LEN_DEV_STATUS_ANS },
    { MOTE_MAC_NEW_CHANNEL_ANS,         MOTE_MAC_LEN_NEW_CHANNEL_ANS },
    { MOTE_MAC_RX_TIMING_SETUP_ANS,     MOTE_MAC_LEN_RX_TIMING_SETUP_ANS },
    { MOTE_MAC_TX_PARAM_SETUP_ANS,      MOTE_MAC_LEN_TX_PARAM_SETUP_ANS },
    { MOTE_MAC_DL_CHANNEL_ANS,          MOTE_MAC_LEN_DL_CHANNEL_ANS },
    { MOTE_MAC_DEVICE_TIME_REQ,         MOTE_MAC_LEN_DEVICE_TIME_REQ },
    { MOTE_MAC_PING_SLOT_INFO_REQ,      MOTE_MAC_LEN_PING_SLOT_INFO_REQ },
    { MOTE_MAC_PING_SLOT_FREQ_ANS,      MOTE_MAC_LEN_PING_SLOT_FREQ_ANS },
    { MOTE_MAC_BEACON_TIMING_REQ,       MOTE_MAC_LEN_BEACON_TIMING_REQ },
    { MOTE_MAC_BEACON_FREQ_ANS,         MOTE_MAC_LEN_BEACON_FREQ_ANS },
};

const lm_maccmd_len_t lm_server_maccmd_tab[] = {
    { SRV_MAC_LINK_CHECK_ANS,           SRV_MAC_LEN_LINK_CHECK_ANS },
    { SRV_MAC_LINK_ADR_REQ,             SRV_MAC_LEN_LINK_ADR_REQ },
    { SRV_MAC_DUTY_CYCLE_REQ,           SRV_MAC_LEN_DUTY_CYCLE_REQ },
    { SRV_MAC_RX_PARAM_SETUP_REQ,       SRV_MAC_LEN_RX_PARAM_SETUP_REQ },
    { SRV_MAC_DEV_STATUS_REQ,           SRV_MAC_LEN_DEV_STATUS_REQ },
    { SRV_MAC_NEW_CHANNEL_REQ,          SRV_MAC_LEN_NEW_CHANNEL_REQ },
    { SRV_MAC_RX_TIMING_SETUP_REQ,      SRV_MAC_LEN_RX_TIMING_SETUP_REQ },
    { SRV_MAC_TX_PARAM_SETUP_REQ,       SRV_MAC_LEN_TX_PARAM_SETUP_REQ },
    { SRV_MAC_DL_CHANNEL_REQ,           SRV_MAC_LEN_DL_CHANNEL_REQ },
    { SRV_MAC_DEVICE_TIME_ANS,          SRV_MAC_LEN_DEVICE_TIME_ANS },
    { SRV_MAC_PING_SLOT_INFO_ANS,       SRV_MAC_LEN_PING_SLOT_INFO_ANS },
    { SRV_MAC_PING_SLOT_CHANNEL_REQ,    SRV_MAC_LEN_PING_SLOT_CHANNEL_REQ },
    { SRV_MAC_BEACON_TIMING_ANS,        SRV_MAC_LEN_BEACON_TIMING_ANS },
    { SRV_MAC_BEACON_FREQ_REQ,          SRV_MAC_LEN_BEACON_FREQ_REQ },
};

const uint16_t lm_eu868_lgw_dr_tab[16] = {
    LM_LGW_DR(DR_LORA_SF12, BW_125KHZ),
    LM_LGW_DR(DR_LORA_SF11, BW_125KHZ),
    LM_LGW_DR(DR_LORA_SF10, BW_125KHZ),
    LM_LGW_DR(DR_LORA_SF9, BW_125KHZ),
    LM_LGW_DR(DR_LORA_SF8, BW_125KHZ),
    LM_LGW_DR(DR_LORA_SF7, BW_125KHZ),
    LM_LGW_DR(DR_LORA_SF7, BW_250KHZ),
    0,                                   // FSK
    0xFFFF,
    0xFFFF,
    0xFFFF,
    0xFFFF,
    0xFFFF,
    0xFFFF,
    0xFFFF,
    0xFFFF,
};

const lm_region_t *lm_get_region(lm_band_t band)
{
    int i;
    for (i = 0; i < sizeof(lm_region_tab) / sizeof(lm_region_tab[0]); i++) {
        if (band == lm_region_tab[i].band) {
            return &lm_region_tab[i];
        }
    }
    return &lm_region_tab[0];
}

int8_t lm_get_mote_maccmd_len(uint8_t cmd)
{
    int j;
    for (j = 0; j < (sizeof(lm_mote_maccmd_tab) / sizeof(lm_maccmd_len_t)); j++) {
        if (lm_mote_maccmd_tab[j].cmd == cmd) {
            return lm_mote_maccmd_tab[j].len;
        }
    }
    return -1;
}

int8_t lm_get_server_maccmd_len(uint8_t cmd)
{
    int j;
    for (j = 0; j < (sizeof(lm_server_maccmd_tab) / sizeof(lm_maccmd_len_t)); j++) {
        if (lm_server_maccmd_tab[j].cmd == cmd) {
            return lm_server_maccmd_tab[j].len;
        }
    }
    return -1;
}


int lm_init(lm_band_t band)
{
    int i;

    //lm_config.rxwin2.dr = 0;          // not used
    lm_config.rxwin2.freq = 869525000;

    lm_region = lm_get_region(band);

    for (i = 0; i < 16; i++) {
        if (i <= lm_region->power.max_tx_power_index) {
            lm_pow_tab[i] = lm_max_eirp_tab[lm_region->power.max_eirp_index] - 2*i;
        } else {
            lm_pow_tab[i] = LM_POW_RFU;
        }
    }

    return 0;
}

lm_mote_t *lm_get_otaa_mote(uint8_t *deveui)
{
    struct list *head = &mote_list;
    struct listnode *node;
    lm_mote_t *mote;

    for (ALL_LIST_ELEMENTS_RO(head, node, mote)) {
        if (0 == memcmp(deveui, mote->deveui, 8)) {
            return mote;
        }
    }

    return NULL;
}

lm_mote_t *lm_get_abp_mote(uint32_t addr)
{
    struct list *head = &mote_list;
    struct listnode *node;
    lm_mote_t *mote;

    for (ALL_LIST_ELEMENTS_RO(head, node, mote)) {
        if (mote->addr.data == addr) {
            return mote;
        }
    }

    return NULL;
}

lm_mote_t *lm_get_mote(uint8_t *addr)
{
    return NULL;
}

void lm_set_deveui(uint8_t *deveui)
{
    /* Little endian */
    deveui[0] = (uint8_t)(lm_deveui_cnt >> 0);
    deveui[1] = (uint8_t)(lm_deveui_cnt >> 8);
    deveui[2] = (uint8_t)(lm_deveui_cnt >> 16);
    deveui[3] = (uint8_t)(lm_deveui_cnt >> 24);
    deveui[4] = 0x00;
    deveui[5] = 0x70;
    deveui[6] = 0x77;
    deveui[7] = 0x6c;
}

// None support duplicated devaddr
int lm_add_mote(lm_mote_t *mote)
{
    mote->dlsize = 0;
    mote->dlbuf = NULL;
    mote->maccmdsize = 0;

    listnode_add(&mote_list, mote);

    return LM_OK;
}

int lm_del_otaa_mote(uint8_t *deveui)
{
    struct list *head = &mote_list;
    struct listnode *tmp, *node;
    lm_mote_t *mote;

    for (ALL_LIST_ELEMENTS(head, tmp, node, mote)) {
        if (0 == memcmp(deveui, mote->deveui, LM_JOIN_EUI_FIELD_SIZE)) {
            listnode_delete(head, mote);
            return LM_OK;
        }
    }

    return LM_ERR_NOT_AVALAIBLE;
}

int lm_del_abp_mote(uint32_t addr)
{
    struct list *head = &mote_list;
    struct listnode *tmp, *node;
    lm_mote_t *mote;

    for (ALL_LIST_ELEMENTS(head, tmp, node, mote)) {
        if (mote->addr.data = addr) {
            listnode_delete(head, mote);
            return LM_OK;
        }
    }

    return LM_ERR_NOT_AVALAIBLE;
}

int lm_add_tx(uint8_t *deveui, uint8_t port, uint8_t *buf, uint16_t size)
{
    lm_mote_t *cur = lm_get_mote(deveui);
    if (cur == NULL) {
        return LM_ERR_UNKOWN_DEVEUI;
    }

    if ((cur->dlbuf != NULL) || (cur->dlsize != 0)) {
        return LM_ERR_TX_BUF_NOT_EMPTY;
    }

    cur->dlbuf = malloc(size);
    if (cur->dlbuf == NULL) {
        return LM_ERR_NO_HEAP;
    }

    memcpy(cur->dlbuf, buf, size);
    cur->dlsize = size;
    cur->dlport = port;

    return LM_OK;
}

int lm_set_key(lm_key_grp_t *kgrp)
{
    if (kgrp->flag.bits.nwkskey) {
        memcpy(lm_dft_nwkskey, kgrp->nwkskey, 16);
    }
    if (kgrp->flag.bits.appskey) {
        memcpy(lm_dft_appskey, kgrp->appskey, 16);
    }
    if (kgrp->flag.bits.appkey) {
        memcpy(lm_dft_appkey, kgrp->appkey, 16);
    }
    return LM_OK;
}

int lm_tx_maccmd(uint8_t *deveui, lm_maccmd_t *maccmd)
{
    lm_mote_t *cur = lm_get_mote(deveui);
    int j, len;

    if (cur == NULL) {
        return LM_ERR_UNKOWN_DEVEUI;
    }

    len = 0;
    for (j = 0; j < (sizeof(lm_server_maccmd_tab) / sizeof(lm_maccmd_len_t)); j++) {
        if (lm_server_maccmd_tab[j].cmd == maccmd->cmd) {
            len = lm_server_maccmd_tab[j].len;
            maccmd->len = len;
            break;
        }
    }

    if (len == 0) {
        return LM_ERR_MACCMD;
    }

    if ((cur->maccmdsize + len) > 15) {
        return LM_ERR_MACCMD;
    }

    memcpy(cur->maccmd + cur->maccmdsize, (uint8_t *)maccmd, len);
    cur->maccmdsize += len;

    return LM_OK;
}

int lm_maccmd_valid(uint8_t mac_header, uint8_t *opts, int len)
{
    int i, j;
    lm_mhdr_t mhdr;

    mhdr.data = mac_header;

    // Traverse all possible commands, if any of them is invalid terminate and return error
    i = 0;
    while (i < len) {
        if ((mhdr.bits.mtype == LM_MTYPE_MSG_UP) || (mhdr.bits.mtype == LM_MTYPE_CMSG_UP)) {
            for (j = 0; j < (sizeof(lm_mote_maccmd_tab) / sizeof(lm_maccmd_len_t)); j++) {
                if (lm_mote_maccmd_tab[j].cmd == opts[i]) {
                    i += lm_mote_maccmd_tab[j].len;
                    break;
                }
            }
            if (j == (sizeof(lm_mote_maccmd_tab) / sizeof(lm_maccmd_len_t))) {
                return LM_ERR_MACCMD_LEN;
            }
        } else if ((mhdr.bits.mtype == LM_MTYPE_MSG_DOWN) || (mhdr.bits.mtype == LM_MTYPE_CMSG_DOWN)) {
            for (j = 0; j < (sizeof(lm_server_maccmd_tab) / sizeof(lm_maccmd_len_t)); j++) {
                if (lm_server_maccmd_tab[j].cmd == opts[i]) {
                    i += lm_server_maccmd_tab[j].len;
                    break;
                }
            }
            if (j == (sizeof(lm_server_maccmd_tab) / sizeof(lm_maccmd_len_t))) {
                return LM_ERR_MACCMD_LEN;
            }
        } else {
            return LM_ERR_MACCMD;
        }
    }
    return LM_OK;
}

int lm_answer(lm_frame_t *frame, lm_rxpkt_t *rxpkt, lm_txpkt_t *txpkt)
{
    lm_frame_t *dlframe = &lm_dlframe;
    lm_mote_t *cur;
    lm_skey_seed_t lm_skey_seed;
    int len;
    bool anwser = false;
    int8_t rxdr, rx1dr = 0;

    memset((uint8_t *)dlframe, 0, sizeof(lm_frame_t));

    cur = lm_get_mote(frame->deveui);
    if (cur == NULL) {
        return LM_ERR_NOT_AVALAIBLE;
    }

    rx1dr = lm_get_dr(rxpkt->modulation, rxpkt->datarate, rxpkt->bandwidth);
    if (rx1dr < 0) {
        return LM_ERR_UNKOWN_DATA_RATE;
    }
    rx1dr -= cur->dlsettings.bits.rx1droft;
    if (rx1dr<DR0) {
        rx1dr = DR0;
    }

    memcpy(dlframe->deveui, frame->deveui, 8);

    switch (frame->mhdr.bits.mtype) {
        case LM_MTYPE_JOIN_REQUEST:
            dlframe->mhdr.bits.mtype = LM_MTYPE_JOIN_ACCEPT;
            dlframe->pld.ja.appnonce.data = cur->appnonce.data;
            dlframe->pld.ja.netid.data = cur->appnonce.data;
            dlframe->pld.ja.addr.data = cur->addr.data;
            dlframe->pld.ja.dlsettings.data = cur->dlsettings.data;
            dlframe->pld.ja.rxdelay.data = cur->rxdelay.data;

            lm_skey_seed.aeskey = cur->appkey;
            lm_skey_seed.anonce.data = cur->appnonce.data;
            lm_skey_seed.dnonce.data = cur->devnonce.data;
            lm_skey_seed.netid.data = cur->netid.data;
            lm_get_skeys(cur->nwkskey, cur->appskey, &lm_skey_seed);

            anwser = true;
            break;
        case LM_MTYPE_CMSG_UP:
            anwser = true;
            dlframe->pld.mac.fhdr.fctrl.dl.ack = 1;
        case LM_MTYPE_MSG_UP:
            dlframe->mhdr.bits.mtype = LM_MTYPE_MSG_DOWN;
            dlframe->pld.mac.fhdr.addr.data = cur->addr.data;
            if (cur->maccmdsize > 0) {
                anwser = true;
                dlframe->pld.mac.fhdr.fctrl.dl.foptslen = cur->maccmdsize;
                memcpy(dlframe->pld.mac.fhdr.fopts, cur->maccmd, cur->maccmdsize);
                cur->maccmdsize = 0;
            }
            dlframe->pld.mac.fhdr.fcnt = cur->dfcnt;
            if (cur->dlsize > 0) {
                anwser = true;
                dlframe->pld.mac.fport = cur->dlport;
                memcpy(dlframe->pld.mac.frmpld, cur->dlbuf, cur->dlsize);
                dlframe->pld.mac.flen = cur->dlsize;
                free(cur->dlbuf);
                cur->dlbuf = NULL;
                cur->dlsize = 0;
            }
            if (frame->pld.mac.fhdr.fctrl.ul.adrackreq == 1) {
                anwser = true;
            }
            break;
        default:
            return LM_ERR_UNKOWN_FRAME;
    }

    if ((anwser == true) && (lm_pack(dlframe, txpkt->payload, &len) > 0)) {
        txpkt->tx_mode = TIMESTAMPED;
        if (cur->rxdelay.bits.del == 0) {
            cur->rxdelay.bits.del++;
        }
        txpkt->count_us = rxpkt->count_us + cur->rxdelay.bits.del*1000000;
        if (cur->rxwin == CLASS_A_RX2) {
            txpkt->count_us += 1000000;
            txpkt->freq_hz = lm_config.rxwin2.freq;
            rxdr = cur->dlsettings.bits.rx2dr;
        } else {
            txpkt->freq_hz = rxpkt->freq_hz;
            rxdr = rx1dr;
        }
        if (lm_get_rf(rxdr, &txpkt->modulation, &txpkt->datarate,
                    &txpkt->bandwidth, &txpkt->f_dev) < 0) {
            return LM_ERR_UNKOWN_DATA_RATE;
        }
        txpkt->rf_chain = 0;
        txpkt->preamble = 8;
        txpkt->coderate = rxpkt->coderate;
        txpkt->rf_power = 14;
        txpkt->invert_pol = true;
        txpkt->no_crc = true;
        txpkt->no_header = false;
        txpkt->size = len;
        cur->dfcnt++;
        return LM_OK;
    }

    return LM_ERR_UNKOWN_FRAME;
}

int lm_pack(lm_frame_t *frame, uint8_t *msg, int *len)
{
    int i, pl_len;
    lm_mic_t mic;
    lm_key_t lm_key;
    lm_skey_seed_t lm_skey_seed;
    lm_mote_t *cur;
    uint8_t out[33];

    i=0;
    msg[i++] = frame->mhdr.data;

    if (frame->mote == NULL) {
        cur = lm_get_mote(frame->deveui);
        if (cur == NULL) {
            return LM_ERR_NOT_AVALAIBLE;
        }
    } else {
        cur = frame->mote;
    }

    switch (frame->mhdr.bits.mtype) {
        case LM_MTYPE_JOIN_REQUEST:
            memcpy(msg+i, frame->appeui, 8);
            i+=8;
            memcpy(msg+i, frame->deveui, 8);
            i+=8;
            msg[i++] = (uint8_t)frame->pld.jr.devnonce.data;
            msg[i++] = (uint8_t)(frame->pld.jr.devnonce.data>>8);
            lm_key.aeskey = cur->appkey;
            lm_key.in = msg;
            lm_key.len = i;
            lm_join_mic(&mic, &lm_key);
            memcpy(msg+i, mic.buf, 4);
            frame->mic.data = mic.data;
            i += 4;
            *len = i;
            return i;
        case LM_MTYPE_JOIN_ACCEPT:
            msg[i++] = (uint8_t)(frame->pld.ja.appnonce.data>>0);
            msg[i++] = (uint8_t)(frame->pld.ja.appnonce.data>>8);
            msg[i++] = (uint8_t)(frame->pld.ja.appnonce.data>>16);
            msg[i++] = (uint8_t)(frame->pld.ja.netid.data>>0);
            msg[i++] = (uint8_t)(frame->pld.ja.netid.data>>8);
            msg[i++] = (uint8_t)(frame->pld.ja.netid.data>>16);
            msg[i++] = (uint8_t)(frame->pld.ja.addr.data>>0);
            msg[i++] = (uint8_t)(frame->pld.ja.addr.data>>8);
            msg[i++] = (uint8_t)(frame->pld.ja.addr.data>>16);
            msg[i++] = (uint8_t)(frame->pld.ja.addr.data>>24);
            msg[i++] = frame->pld.ja.dlsettings.data;
            msg[i++] = frame->pld.ja.rxdelay.data;
            if (frame->pld.ja.cflist_len == 16) {
                memcpy(msg+i, frame->pld.ja.cflist, 16);
                i+=16;
            }
            lm_key.aeskey = cur->appkey;
            lm_key.in = msg;
            lm_key.len = i;
            lm_join_mic(&mic, &lm_key);
            frame->mic.data = mic.data;
            memcpy(msg+i, mic.buf, 4);
            i += 4;
            lm_key.aeskey = cur->appkey;
            lm_key.in = msg+1;
            lm_key.len = i-1;
            lm_join_encrypt(out+1, &lm_key);
            memcpy(msg+1, out+1, lm_key.len);
            *len = i;

            lm_skey_seed.aeskey = cur->appkey;
            lm_skey_seed.anonce = frame->pld.ja.appnonce;
            lm_skey_seed.dnonce = cur->devnonce;
            lm_skey_seed.netid = frame->pld.ja.netid;
            lm_get_skeys(frame->pld.ja.nwkskey, frame->pld.ja.appskey, &lm_skey_seed);
            return i;
        case LM_MTYPE_MSG_UP:
        case LM_MTYPE_MSG_DOWN:
        case LM_MTYPE_CMSG_UP:
        case LM_MTYPE_CMSG_DOWN:
            msg[i++] = (uint8_t)(frame->pld.mac.fhdr.addr.data >> 0);
            msg[i++] = (uint8_t)(frame->pld.mac.fhdr.addr.data >> 8);
            msg[i++] = (uint8_t)(frame->pld.mac.fhdr.addr.data >> 16);
            msg[i++] = (uint8_t)(frame->pld.mac.fhdr.addr.data >> 24);
            msg[i++] = frame->pld.mac.fhdr.fctrl.data;
            msg[i++] = (uint8_t)(frame->pld.mac.fhdr.fcnt >> 0);
            msg[i++] = (uint8_t)(frame->pld.mac.fhdr.fcnt >> 8);
            if (frame->pld.mac.fhdr.fctrl.dl.foptslen > 0){
                memcpy(msg+i, frame->pld.mac.fhdr.fopts, frame->pld.mac.fhdr.fctrl.dl.foptslen);
                i+=frame->pld.mac.fhdr.fctrl.dl.foptslen;
            }

            lm_key.addr.data = frame->pld.mac.fhdr.addr.data;
            lm_key.fcnt32 = frame->pld.mac.fhdr.fcnt;
            switch (frame->mhdr.bits.mtype) {
                case LM_MTYPE_MSG_UP:
                case LM_MTYPE_CMSG_UP:
                    lm_key.link = LM_UPLINK;
                    break;
                case LM_MTYPE_CMSG_DOWN:
                case LM_MTYPE_MSG_DOWN:
                    lm_key.link = LM_DOWNLINK;
                    break;
            }

            if (frame->pld.mac.flen>0) {
                msg[i++] = frame->pld.mac.fport;
                if (frame->pld.mac.fport == 0) {
                    lm_key.aeskey = cur->nwkskey;
                } else {
                    lm_key.aeskey = cur->appskey;
                }
                lm_key.in = frame->pld.mac.frmpld;
                lm_key.len = frame->pld.mac.flen;
                pl_len = lm_encrypt(msg+i, &lm_key);
                i += pl_len;
            }

            lm_key.aeskey = cur->nwkskey;
            lm_key.in = msg;
            lm_key.len = i;

            lm_msg_mic(&mic, &lm_key);
            memcpy(msg+i, mic.buf, 4);
            frame->mic.data = mic.data;
            i += 4;
            *len = i;
            break;
        case LM_MTYPE_RFU:
            *len = 0;
            break;
        case LM_MTYPE_PROPRIETARY:
            *len = 0;
            break;
    }
    return *len;
}

int lm_auto_add(lm_frame_t *frame, uint8_t *msg, int len)
{
    lm_mic_t mic;
    lm_mic_t rxmic;
    lm_key_t lm_key;
    lm_mote_t *mote;
    int ret, id;
    uint32_t cnt;

    memcpy(rxmic.buf, msg + len - LM_MIC_FIELD_SIZE, LM_MIC_FIELD_SIZE);
    ULOG_DEBUG("RXMIC: " DEVADDRSTR "\n", DEVADDR2STR(rxmic.buf));

    switch (frame->mhdr.bits.mtype) {
        case LM_MTYPE_JOIN_REQUEST:
            lm_key.aeskey = lm_dft_appkey;
            lm_key.in = msg;
            lm_key.len = len - LM_MIC_FIELD_SIZE;
            lm_join_mic(&mic, &lm_key);

            if (mic.data != rxmic.data) {
                return LM_ERR_UNKOWN_FRAME;
            }

            memcpy(frame->appeui, msg + LM_JR_OFF_APPEUI, LM_JOIN_EUI_FIELD_SIZE);
            memcpy(frame->deveui, msg + LM_JR_OFF_DEVEUI, LM_DEV_EUI_FIELD_SIZE);

            mote = calloc(1, sizeof(lm_mote_t));
            if (mote) {
                mote->mode = OTAA;
                memcpy(mote->appeui, msg + LM_JR_OFF_APPEUI, LM_JOIN_EUI_FIELD_SIZE);
                memcpy(mote->deveui, msg + LM_JR_OFF_DEVEUI, LM_DEV_EUI_FIELD_SIZE);
                memcpy(mote->appkey, lm_dft_appkey, 16);
                lm_add_mote(mote);
            }

            ret = lm_mtype_join_request(frame, lm_get_otaa_mote(frame->deveui), msg, len);
            if (ret == LM_OK) {
                lm_mote_latest_jr = lm_get_mote(frame->deveui);
            }
            return ret;
        case LM_MTYPE_MSG_UP:
        case LM_MTYPE_MSG_DOWN:
        case LM_MTYPE_CMSG_UP:
        case LM_MTYPE_CMSG_DOWN:
            lm_key.aeskey = lm_dft_nwkskey;
            lm_key.in = msg;
            lm_key.len = len - 4;
            lm_key.addr.data = frame->pld.mac.fhdr.addr.data; 
            switch (frame->mhdr.bits.mtype) {
                case LM_MTYPE_MSG_UP:
                case LM_MTYPE_CMSG_UP:
                    lm_key.link = LM_UPLINK;
                    ULOG_DEBUG("uplink");
                    break;
                case LM_MTYPE_CMSG_DOWN:
                case LM_MTYPE_MSG_DOWN:
                    lm_key.link = LM_DOWNLINK;
                    ULOG_DEBUG("downlink");
                    break;
            }
            lm_key.fcnt32 = ((uint32_t)msg[LM_DATA_OFF_FCNT+1] << 8) + msg[LM_DATA_OFF_FCNT];

            ULOG_DEBUG("FHDR fcnt32: %d\n", lm_key.fcnt32);

            lm_msg_mic(&mic, &lm_key);

            if (mic.data != rxmic.data) {
                hex_dump(mic.buf, LM_MIC_FIELD_SIZE, 8, "MIC");
                return LM_ERR_MIC;
            }

            mote = calloc(1, sizeof(lm_mote_t));
            if (mote) {
                /* Check if  */
                mote->mode = ABP;
                memset(mote->appeui, 0, LM_JOIN_EUI_FIELD_SIZE);
                memset(mote->deveui, 0, LM_DEV_EUI_FIELD_SIZE);
                mote->addr.data = lm_key.addr.data;
                memcpy(mote->nwkskey, lm_dft_nwkskey, 16);
                memcpy(mote->appskey, lm_dft_appskey, 16);
                mote->ufcnt = lm_key.fcnt32;     // Increase frame counter high 16bits
                mote->dfcnt = 0;
                mote->rxwin = CLASS_A_RX2;
                mote->dlsettings.bits.rx1droft = 0;
                mote->dlsettings.bits.rx2dr = 0;      // 0 ~ 7
                mote->rxdelay.bits.del = 1;
                lm_add_mote(mote);
            }

            //memcpy(frame->deveui, mote.deveui, LM_DEV_EUI_FIELD_SIZE);
            //hex_dump(frame->deveui, 8, 16, "frame deveui:");
            ULOG_DEBUG("mote: %x %x\n", mote->addr.data, frame->pld.mac.fhdr.addr.data);
            lm_mtype_msg_up(frame, lm_get_abp_mote(frame->pld.mac.fhdr.addr.data), msg, len);
            return LM_OK;
    }
    return LM_ERR_UNKOWN_FRAME;
}

int lm_parse(lm_frame_t *frame, uint8_t *msg, int len)
{
    int idx = 0, ret;
    struct list *head = &mote_list;
    struct listnode *node;
    lm_mote_t *mote;

    memset((uint8_t *)frame, 0, sizeof(lm_frame_t));

    if (len < LM_FRAME_PAYLOAD_MIN_SIZE) {
        ULOG_ERR("payload size too small\n");
        return LM_ERR_PRM_SIZE;
    }

    frame->mhdr.data = msg[idx++];

    if (frame->mhdr.bits.major != 0) {
        ULOG_ERR("unsupported major version\n");
        return LM_ERR_PARA;
    }

    if (frame->mhdr.bits.mtype > LM_MTYPE_PROPRIETARY) {
        ULOG_ERR("unknown cmd\n");
        return LM_ERR_CMD_UNKNOWN;
    }

    switch (frame->mhdr.bits.mtype) {
        case LM_MTYPE_JOIN_REQUEST:
            if (len != LM_JR_LEN) {
                return LM_ERR_JOINR_LEN;
            }

            memcpy(frame->appeui, msg + LM_JR_OFF_APPEUI, LM_JOIN_EUI_FIELD_SIZE);
            memcpy(frame->deveui, msg + LM_JR_OFF_DEVEUI, LM_DEV_EUI_FIELD_SIZE);

            /* add more sanity check later */
            for (ALL_LIST_ELEMENTS_RO(head, node, mote)) {
                if (mote->mode != OTAA)
                    continue;
                if (0 != memcmp(frame->appeui, mote->appeui, LM_JOIN_EUI_FIELD_SIZE) &&
                        0 != memcmp(frame->deveui, mote->deveui, LM_DEV_EUI_FIELD_SIZE))
                    continue;

                return lm_mtype_join_request(frame, mote, msg, len);
            }

            ret = lm_auto_add(frame, msg, len);
            if (ret == LM_OK) {
                return LM_OK;
            }
            break;
        case LM_MTYPE_JOIN_ACCEPT:
            if (lm_mote_latest_jr == NULL) {
                return LM_ERR_UNKOWN_FRAME;
            }
            return lm_mtype_join_accept(frame, lm_mote_latest_jr, msg, len);
            break;
        case LM_MTYPE_MSG_UP:
        case LM_MTYPE_MSG_DOWN:
        case LM_MTYPE_CMSG_UP:
        case LM_MTYPE_CMSG_DOWN:
            {
                frame->pld.mac.fhdr.addr.data = ((uint32_t)msg[idx++] << 0);
                frame->pld.mac.fhdr.addr.data |= ((uint32_t)msg[idx++] << 8);
                frame->pld.mac.fhdr.addr.data |= ((uint32_t)msg[idx++] << 16);
                frame->pld.mac.fhdr.addr.data |= ((uint32_t)msg[idx++] << 24);
                ULOG_DEBUG("FHDR addr: " DEVADDRSTR "\n", DEVADDR2STR(frame->pld.mac.fhdr.addr.buf));

                for (ALL_LIST_ELEMENTS_RO(head, node, mote)) {
                    if ((mote->mode == ABP || mote->joined) && mote->addr.data == frame->pld.mac.fhdr.addr.data) {
                        ret = lm_mtype_msg_up(frame, mote, msg, len);
                        if (ret == LM_OK) {
                            return LM_OK;
                        }
                    }
                }

                return lm_auto_add(frame, msg, len);
            }
        case LM_MTYPE_RFU:
        case LM_MTYPE_PROPRIETARY:
            return LM_ERR_NOT_AVALAIBLE;
    }

    return LM_ERR_UNKOWN_FRAME;
}

int lm_mtype_join_accept(lm_frame_t *frame, lm_mote_t *cur, uint8_t *buf, int len)
{
    lm_mic_t mic;
    lm_mic_t plmic;
    lm_key_t lm_key;
    lm_skey_seed_t lm_skey_seed;
    uint8_t out[LM_JA_LEN_EXT];
    int pl_len;
    int id;

    if ((len != LM_JA_LEN) && (len != LM_JA_LEN_EXT)) {
        return LM_ERR_JOINA_LEN;
    }

    lm_key.aeskey = cur->appkey;
    lm_key.in = buf+1;
    lm_key.len = len-1;
    out[0] = buf[0];
    pl_len = lm_join_decrypt(out+1, &lm_key);

    if (pl_len > 0) {
        memcpy(plmic.buf, out+len-4, 4);
        lm_key.aeskey = cur->appkey;
        lm_key.in = out;
        lm_key.len = len-4;
        lm_join_mic(&mic, &lm_key);
        if (mic.data != plmic.data) {
            return LM_ERR_MIC;
        }
    }

    lm_skey_seed.aeskey = cur->appkey;
    lm_skey_seed.anonce.data = out[LM_JA_OFF_APPNONCE+0];
    lm_skey_seed.anonce.data |= ((uint32_t)out[LM_JA_OFF_APPNONCE+1] << 8);
    lm_skey_seed.anonce.data |= ((uint32_t)out[LM_JA_OFF_APPNONCE+2] << 16);
    lm_skey_seed.dnonce = cur->devnonce;
    lm_skey_seed.netid.data = out[LM_JA_OFF_NETID+0];
    lm_skey_seed.netid.data |= ((uint32_t)out[LM_JA_OFF_NETID+1] << 8);
    lm_skey_seed.netid.data |= ((uint32_t)out[LM_JA_OFF_NETID+2] << 16);
    lm_get_skeys(frame->pld.ja.nwkskey, frame->pld.ja.appskey, &lm_skey_seed);

    id = LM_JA_OFF_APPNONCE;
    frame->pld.ja.appnonce.data = ((uint32_t)out[id+0] << 0);
    frame->pld.ja.appnonce.data |= ((uint32_t)out[id+1] << 8);
    frame->pld.ja.appnonce.data |= ((uint32_t)out[id+2] << 16);

    id = LM_JA_OFF_NETID;
    frame->pld.ja.netid.data = ((uint32_t)out[id+0] << 0);
    frame->pld.ja.netid.data |= ((uint32_t)out[id+1] << 8);
    frame->pld.ja.netid.data |= ((uint32_t)out[id+2] << 16);

    id = LM_JA_OFF_DEVADDR;
    frame->pld.ja.addr.data = ((uint32_t)out[id+0] << 0);
    frame->pld.ja.addr.data |= ((uint32_t)out[id+1] << 8);
    frame->pld.ja.addr.data |= ((uint32_t)out[id+2] << 16);
    frame->pld.ja.addr.data |= ((uint32_t)out[id+3] << 24);

    id = LM_JA_OFF_DLSET;
    frame->pld.ja.dlsettings.data = out[id];

    if (len == LM_JA_LEN_EXT) {
        id = LM_JA_OFF_CFLIST;
        frame->pld.ja.cflist_len = 16;
        memcpy(frame->pld.ja.cflist, out+id, 16);
    } else {
        frame->pld.ja.cflist_len = 0;
    }

    frame->mic.data = mic.data;

    frame->len = len;
    memcpy(frame->buf, out, frame->len);

    memcpy(frame->deveui, cur->deveui, 8);
    memcpy(frame->appeui, cur->appeui, 8);

    cur->joined = true;
    memcpy(cur->nwkskey, frame->pld.ja.nwkskey, 16);
    memcpy(cur->appskey, frame->pld.ja.appskey, 16);
    cur->addr.data = frame->pld.ja.addr.data;
    cur->netid.data = frame->pld.ja.netid.data;
    cur->appnonce.data = frame->pld.ja.appnonce.data;

    frame->mote = cur;

    return LM_OK;
}

int lm_mtype_join_request(lm_frame_t *frame, lm_mote_t *mote, uint8_t *msg, int len)
{
    lm_mic_t mic;
    lm_mic_t rxmic;
    lm_key_t lm_key;
    int id;

    memcpy(rxmic.buf, msg + len - LM_MIC_FIELD_SIZE, LM_MIC_FIELD_SIZE);

    if (mote != NULL) {
        lm_key.aeskey = mote->appkey;
        lm_key.in = msg;
        lm_key.len = len - LM_MIC_FIELD_SIZE;
        lm_join_mic(&mic, &lm_key);

        if (mic.data != rxmic.data) {
            return LM_ERR_UNKOWN_FRAME;
        }

        memcpy(frame->appeui, msg + LM_JR_OFF_APPEUI, LM_JOIN_EUI_FIELD_SIZE);
        memcpy(frame->deveui, msg + LM_JR_OFF_DEVEUI, LM_DEV_EUI_FIELD_SIZE);

        id = LM_JR_OFF_DEVNONCE;
        frame->pld.jr.devnonce.data = ( (uint32_t)msg[id++] << 0 );
        frame->pld.jr.devnonce.data |= ( (uint32_t)msg[id++] << 8 );

        // Save devnonce, should maintain devnonce list for security issue
        mote->devnonce = frame->pld.jr.devnonce;

        frame->len = len;
        memcpy(frame->buf, msg, len);

        frame->mic.data = mic.data;

        memcpy(frame->deveui, mote->deveui, LM_JOIN_EUI_FIELD_SIZE);
        memcpy(frame->appeui, mote->appeui, LM_DEV_EUI_FIELD_SIZE);

        frame->mote = mote;
        return LM_OK;
    }

    return LM_ERR_UNKOWN_FRAME;
}

int lm_mtype_msg_up(lm_frame_t *frame, lm_mote_t *cur, uint8_t *msg, int len)
{
    int id = 5, foptslen;
    lm_mic_t plmic;
    uint32_t diff;
    uint16_t fcnt16, fcntlsb, fcntmsb;
    int pld_len = 0;
    int pld_index = 0;
    lm_mic_t mic;
    lm_key_t lm_key;

    // TODO: check minimum len
    if (len < 12) {
        ULOG_ERR("len too small\n");
        return LM_ERR_UNKOWN_FRAME;
    }
    if (cur == NULL) {
        ULOG_ERR("mote is nil\n");
        return LM_ERR_UNKOWN_FRAME;
    }

    memcpy(plmic.buf, msg+len-4, 4);

    lm_key.aeskey = cur->nwkskey;
    lm_key.in = msg;
    lm_key.len = len-4;
    lm_key.addr.data = frame->pld.mac.fhdr.addr.data;

    switch (frame->mhdr.bits.mtype) {
        case LM_MTYPE_MSG_UP:
        case LM_MTYPE_CMSG_UP:
            lm_key.link = LM_UPLINK;
            break;
        case LM_MTYPE_CMSG_DOWN:
        case LM_MTYPE_MSG_DOWN:
            lm_key.link = LM_DOWNLINK;
            break;
    }

    fcnt16 = ((uint32_t)msg[LM_DATA_OFF_FCNT+1]<<8) + msg[LM_DATA_OFF_FCNT];
    fcntlsb = (uint16_t)cur->ufcnt;
    fcntmsb = (uint16_t)(cur->ufcnt >> 16);
    if (fcnt16 < fcntlsb) {
        fcntmsb++;
    }
    lm_key.fcnt32 = ((uint32_t)fcntmsb << 16) + fcnt16;
    ULOG_DEBUG("FHDR fcnt32: %d\n", lm_key.fcnt32);

    lm_msg_mic(&mic, &lm_key);
#if 1
    if (mic.data != plmic.data) {
        if (lm_key.fcnt32 == fcnt16) {
            return LM_ERR_MIC;
        }
        lm_key.fcnt32 = fcnt16;
        lm_msg_mic(&mic, &lm_key);
        if (mic.data != plmic.data) {
            return LM_ERR_MIC;
        }
    }
#endif

    switch (frame->mhdr.bits.mtype) {
        case LM_MTYPE_MSG_UP:
        case LM_MTYPE_CMSG_UP:
            if (cur->ufsum == 0) {
                cur->ufsum++;
            } else {
                if (lm_key.fcnt32 > cur->ufcnt) {
                    diff = lm_key.fcnt32 - cur->ufcnt;
                    if (diff == 0) {

                    } else {
                        cur->uflost += (diff - 1);
                        cur->ufsum++;
                    }
                } else if (lm_key.fcnt32 < cur->ufcnt) {
                    /* Counter is restarted  */
                    cur->ufsum++;
                }
            }
            cur->ufcnt = lm_key.fcnt32;
            break;
    }

    frame->pld.mac.fhdr.fcnt = lm_key.fcnt32;
    frame->pld.mac.fhdr.fctrl.data = msg[id++];
    foptslen = frame->pld.mac.fhdr.fctrl.ul.foptslen;

    if (frame->pld.mac.fhdr.fctrl.ul.classb)
        ULOG_DEBUG("classB\n");
    else
        ULOG_DEBUG("classA\n");
    ULOG_DEBUG("FHDR foptslen: %d\n", foptslen);

    //if (foptslen <= 15) {
    //}

    if (len > (8 + 4 + foptslen)) {
        if (len == (8 + 4 + foptslen + 1)) {
            frame->pld.mac.flen = 0;
            frame->pld.mac.fport = msg[LM_DATA_OFF_FOPTS + foptslen];
            ULOG_WARN("PORT (%d) PRESENT WITHOUT PAYLOAD\n", frame->pld.mac.fport);
        } else {
            frame->pld.mac.fport = msg[LM_DATA_OFF_FOPTS + foptslen];
            ULOG_DEBUG("MHDR fport: %d\n", frame->pld.mac.fport);

            pld_index = LM_DATA_OFF_FOPTS + foptslen + 1;
            pld_len  = len - 4 - pld_index;

            if (frame->pld.mac.fport == 0) {
                lm_key.aeskey = cur->nwkskey;
            } else {
                lm_key.aeskey = cur->appskey;
            }
            lm_key.in = msg + pld_index;
            lm_key.len = pld_len;
            pld_len = lm_encrypt(frame->pld.mac.frmpld, &lm_key);
            if (pld_len <= 0) {
                return LM_ERR_DECRYPT;
            }
            frame->pld.mac.flen = pld_len;
        }
    } else {
        frame->pld.mac.flen = 0;
    }

    if ((foptslen != 0) && ((frame->pld.mac.fport == 0) && (frame->pld.mac.flen > 0))) {
        return LM_ERR_FOPTS_PORT0;
    }

    if (foptslen != 0) {
        memcpy(frame->pld.mac.fhdr.fopts, msg+LM_DATA_OFF_FOPTS, foptslen);
    }
    frame->mic.data = plmic.data;

    memcpy(frame->buf, msg, pld_index);        // until port, pl_index equals length of MHDR+FHDR+FPOR
    memcpy(frame->buf + pld_index, frame->pld.mac.frmpld, pld_len);   // payload
    memcpy(frame->buf + len - 4, mic.buf, 4); // mic
    frame->len = len;

    memcpy(frame->deveui, cur->deveui, 8);
    memcpy(frame->appeui, cur->appeui, 8);

    frame->mote = cur;
    return LM_OK;

}

int lm_mtype_msg_down(lm_frame_t *frame, lm_mote_t *cur, uint8_t *buf, int len)
{
    return lm_mtype_msg_up(frame, cur, buf, len);
}

int lm_mtype_cmsg_up(lm_frame_t *frame, lm_mote_t *cur, uint8_t *buf, int len)
{
    return lm_mtype_msg_up(frame, cur, buf, len);
}

int lm_mtype_cmsg_down(lm_frame_t *frame, lm_mote_t *cur, uint8_t *buf, int len)
{
    return lm_mtype_msg_up(frame, cur, buf, len);
}

int lm_mtype_rfu(lm_frame_t *frame, lm_mote_t *cur, uint8_t *buf, int len)
{
    return LM_ERR_NOT_AVALAIBLE;
}

int lm_mtype_proprietary(lm_frame_t *frame, lm_mote_t *cur, uint8_t *buf, int len)
{
    return LM_ERR_NOT_AVALAIBLE;
}

/*****************************************************************************/
uint8_t lgw_util_get_sf(uint8_t sf)
{
    int i;
    for (i = 7; i <= 12; i++) {
        if (sf == (1 << (i - 6))) {
            sf = i;
            break;
        }
    }
    return sf;
}

uint16_t lgw_util_get_bw(uint8_t bw)
{
    uint16_t bwreal = bw;
    switch (bw) {
        case BW_125KHZ:
            bwreal = 125;
            break;
        case BW_250KHZ:
            bwreal = 250;
            break;
        case BW_500KHZ:
            bwreal = 500;
            break;
    }
    return bwreal;
}

uint8_t lgw_util_get_cr(uint8_t cr)
{
    uint8_t crreal = cr;
    switch (cr) {
        case CR_LORA_4_5:
            crreal = 5;
            break;
        case CR_LORA_4_6:
            crreal = 6;
            break;
        case CR_LORA_4_7:
            crreal = 7;
            break;
        case CR_LORA_4_8:
            crreal = 8;
            break;
    }
    return crreal;
}

int8_t lm_get_dr(uint8_t mod, uint32_t datarate, uint8_t bw)
{
    int8_t ret = -1;
    int i;
    uint16_t drtab = LM_LGW_DR(datarate, bw);

    /* EU868 */
    if (mod == MOD_FSK) {
        ret = DR7;
    } else if (mod == MOD_LORA) {
        for (i = 0; i < 16; i++) {
            if (drtab == lm_eu868_lgw_dr_tab[i]) {
                ret = i;
                break;
            }
        }
    }

    return ret;
}

int8_t lm_get_rf(uint8_t dr, uint8_t *mod, uint32_t *datarate, uint8_t *bw, uint8_t *fdev)
{
    uint16_t drtab;
    if (dr > DR15) {
        return -1;
    }
    drtab = lm_eu868_lgw_dr_tab[dr];
    if (drtab == 0xFFFF) {
        return -1;
    } else if (drtab == 0) {
        *mod = MOD_FSK;
        *datarate = 50000;
        *fdev = 3;
    } else {
        *mod = MOD_LORA;
        *datarate = (uint8_t)(drtab & 0xFF);
        *bw = (uint8_t)((drtab>>8) & 0xFF);
    }
    return 0;
}

static char lm_rf_name[30];
const char *lm_get_rf_name(uint8_t mod, uint32_t datarate, uint8_t bw, uint8_t fdev)
{
    lm_rf_name[0] = '\0';
    if (mod == MOD_LORA) {
        sprintf(lm_rf_name, "SF%dBW%d",
                lgw_util_get_sf(datarate),
                lgw_util_get_bw(bw));
    } else {
        sprintf(lm_rf_name, "FSK50K");
    }
    return lm_rf_name;
}

void lm_cpy(uint8_t *dest, uint8_t *src, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        dest[i] = src[len-1-i];
    }
}

int lm_check_zero(uint8_t *src, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        if (src[i] != 0) {
            return i + 1;
        }
    }
    return 0;
}

void lm_write_dw(uint8_t *output, uint32_t input)
{
    uint8_t* ptr = output;

    *(ptr++) = (uint8_t)(input), input >>= 8;
    *(ptr++) = (uint8_t)(input), input >>= 8;
    *(ptr++) = (uint8_t)(input), input >>= 8;
    *(ptr++) = (uint8_t)(input);
}

uint32_t lm_read_dw(uint8_t *buf)
{
    uint32_t ret;

    ret = ((uint32_t)buf[0] << 0);
    ret |= ((uint32_t)buf[1] << 8);
    ret |= ((uint32_t)buf[2] << 16);
    ret |= ((uint32_t)buf[3] << 24);

    return ret;
}

void lm_msg_mic(lm_mic_t* mic, lm_key_t *key)
{
    uint8_t b0[LM_KEY_LEN];
    memset(b0, 0 , LM_KEY_LEN);
    b0[0] = 0x49;
    b0[5] = key->link;

    lm_write_dw(b0+6, key->addr.data);
    lm_write_dw(b0+10, key->fcnt32);
    b0[15] = (uint8_t)key->len;

    AES_CMAC_CTX cmacctx;
    AES_CMAC_Init(&cmacctx);
    AES_CMAC_SetKey(&cmacctx, key->aeskey);

    AES_CMAC_Update(&cmacctx, b0, LM_KEY_LEN);
    AES_CMAC_Update(&cmacctx, key->in, key->len);

    uint8_t temp[LM_KEY_LEN];
    AES_CMAC_Final(temp, &cmacctx);

    memcpy(mic->buf, temp, LM_MIC_LEN);
}

void lm_join_mic(lm_mic_t* mic, lm_key_t *key)
{
    AES_CMAC_CTX cmacctx;
    AES_CMAC_Init(&cmacctx);
    AES_CMAC_SetKey(&cmacctx, key->aeskey);

    AES_CMAC_Update(&cmacctx, key->in, key->len);

    uint8_t temp[LM_KEY_LEN];
    AES_CMAC_Final(temp, &cmacctx);

    memcpy(mic->buf, temp, LM_MIC_LEN);
}

/** Use to generate JoinAccept Payload */
int lm_join_encrypt(uint8_t *out, lm_key_t *key)
{
    if ((key->len == 0) || (key->len % LM_KEY_LEN != 0)) {
        return -1;
    }

    aes_context aesContext;

    aes_set_key(key->aeskey, LM_KEY_LEN, &aesContext);

    // Check if optional CFList is included
    int i;
    for (i = 0; i < key->len; i += LM_KEY_LEN) {
        aes_decrypt(key->in + i, out + i, &aesContext);
    }

    return key->len;
}

/** Use to decrypt JoinAccept Payload */
int lm_join_decrypt(uint8_t *out, lm_key_t *key)
{
    int i;

    if ((key->len == 0) || (key->len % LM_KEY_LEN != 0)) {
        return -1;
    }

    aes_context aesContext;

    aes_set_key(key->aeskey, LM_KEY_LEN, &aesContext);

    // Check if optional CFList is included
    for (i = 0; i < key->len; i += LM_KEY_LEN) {
        aes_encrypt(key->in + i, out + i, &aesContext);
    }

    return key->len;
}

void lm_block_xor(uint8_t const l[], uint8_t const r[], uint8_t out[], uint16_t bytes)
{
    uint8_t const* lptr = l;
    uint8_t const* rptr = r;
    uint8_t* optr = out;
    uint8_t const* const end = out + bytes;

    for (;optr < end; lptr++, rptr++, optr++)
        *optr = *lptr ^ *rptr;
}

int lm_encrypt(uint8_t *out, lm_key_t *key)
{
    if (key->len == 0)
        return -1;

    uint8_t A[LM_KEY_LEN];

    uint16_t const over_hang_bytes = key->len%LM_KEY_LEN;
    int blocks = key->len/LM_KEY_LEN;
    if (over_hang_bytes) {
        ++blocks;
    }

    memset(A, 0, LM_KEY_LEN);

    A[0] = 0x01; //encryption flags
    A[5] = key->link;

    lm_write_dw(A+6, key->addr.data);
    lm_write_dw(A+10, key->fcnt32);

    uint8_t const* blockInput = key->in;
    uint8_t* blockOutput = out;
    uint16_t i;
    for (i = 1; i <= blocks; i++, blockInput += LM_KEY_LEN, blockOutput += LM_KEY_LEN) {
        A[15] = (uint8_t)(i);

        aes_context aesContext;
        aes_set_key(key->aeskey, LM_KEY_LEN, &aesContext);

        uint8_t S[LM_KEY_LEN];
        aes_encrypt(A, S, &aesContext);

        uint16_t bytes_to_encrypt;
        if ((i < blocks) || (over_hang_bytes == 0))
            bytes_to_encrypt = LM_KEY_LEN;
        else
            bytes_to_encrypt = over_hang_bytes;

        lm_block_xor(S, blockInput, blockOutput, bytes_to_encrypt);
    }
    return key->len;
}

void lm_get_skeys(uint8_t *nwkskey, uint8_t *appskey, lm_skey_seed_t *seed)
{
    aes_context aesContext;
    uint8_t b[LM_KEY_LEN];

    memset(&aesContext, 0, sizeof(aesContext));
    memset(b, 0, LM_KEY_LEN);
    b[1] = (uint8_t)(seed->anonce.data >> 0);
    b[2] = (uint8_t)(seed->anonce.data >> 8);
    b[3] = (uint8_t)(seed->anonce.data >> 16);
    b[4] = (uint8_t)(seed->netid.data >> 0);
    b[5] = (uint8_t)(seed->netid.data >> 8);
    b[6] = (uint8_t)(seed->netid.data >> 16);
    b[7] = (uint8_t)(seed->dnonce.data >> 0);
    b[8] = (uint8_t)(seed->dnonce.data >> 8);

    b[0] = 0x01;
    aes_set_key(seed->aeskey, LM_KEY_LEN, &aesContext);
    aes_encrypt(b, nwkskey, &aesContext);

    b[0] = 0x02;
    aes_set_key(seed->aeskey, LM_KEY_LEN, &aesContext);
    aes_encrypt(b, appskey, &aesContext);
}

lm_band_t lm_get_band_type(const char *band)
{
    int i;
    for (i = 0; i < sizeof(lm_region_tab) / sizeof(lm_region_tab[0]); i++) {
        if (0 == strcmp(band, lm_region_tab[i].name)) {
            return (lm_band_t)lm_region_tab[i].band;
        }
    }
    return lm_region_tab[0].band;
}

const char *lm_get_band_name(lm_band_t band)
{
    int i;
    for (i = 0; i < sizeof(lm_region_tab) / sizeof(lm_region_tab[0]); i++) {
        if (band == lm_region_tab[i].band) {
            return lm_region_tab[i].name;
        }
    }
    return lm_region_tab[0].name;
}
