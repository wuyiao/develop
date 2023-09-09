#ifndef _LM_H_
#define _LM_H_
#include <stdint.h>
#include <stdbool.h>
#include "lm-macro.h"
#include "loragw_hal.h"

#if defined(__CC_ARM) || defined(__GNUC__)
#define PACKED                                      __attribute__( ( __packed__ ) )
#elif defined( __ICCARM__ )
#define PACKED                                      __packed
#else
#warning Not supported compiler type
#endif

/*! MAC header field size */
#define LM_MHDR_FIELD_SIZE             1

/*! ReJoinType field size */
#define LM_JOIN_TYPE_FIELD_SIZE        1

/*! Join EUI field size */
#define LM_JOIN_EUI_FIELD_SIZE         8

/*! Device EUI field size */
#define LM_DEV_EUI_FIELD_SIZE          8

/*! End-device nonce field size */
#define LM_DEV_NONCE_FIELD_SIZE        2

/*! Join-server nonce field size */
#define LM_JOIN_NONCE_FIELD_SIZE       3

/*! RJcount0 field size */
#define LM_RJCOUNT_0_FIELD_SIZE        2

/*! RJcount1 field size */
#define LM_RJCOUNT_1_FIELD_SIZE        2

/*! Network ID field size */
#define LM_NET_ID_FIELD_SIZE           3

/*! Device address field size */
#define LM_DEV_ADDR_FIELD_SIZE         4

/*! DLSettings field size */
#define LM_DL_SETTINGS_FIELD_SIZE      1

/*! RxDelay field size */
#define LM_RX_DELAY_FIELD_SIZE         1

/*! CFList field size */
#define LM_CF_LIST_FIELD_SIZE          16

/*! FHDR Device address field size */
#define LM_FHDR_DEV_ADDR_FIELD_SIZE    LM_DEV_ADDR_FIELD_SIZE

/*! FHDR Frame control field size */
#define LM_FHDR_F_CTRL_FIELD_SIZE      1

/*! FHDR Frame control field size */
#define LM_FHDR_F_CNT_FIELD_SIZE       2

/*! FOpts maximum field size */
#define LM_FHDR_F_OPTS_MAX_FIELD_SIZE  15

/*! Port field size */
#define LM_F_PORT_FIELD_SIZE           1

/*! Port field size */
#define LM_MAC_PAYLOAD_FIELD_MAX_SIZE  242

/*! MIC field size */
#define LM_MIC_FIELD_SIZE              4

/*!
 * JoinRequest frame size
 *
 * MHDR(1) + JoinEUI(8) + DevEUI(8) + DevNonce(2) + MIC(4)
 */
#define LM_JOIN_REQ_MSG_SIZE           ( LM_MHDR_FIELD_SIZE + LM_JOIN_EUI_FIELD_SIZE + \
                                              LM_DEV_EUI_FIELD_SIZE + LM_DEV_NONCE_FIELD_SIZE + \
                                              LM_MIC_FIELD_SIZE )

/*!
 * ReJoinRequest type 1 frame size
 *
 * MHDR(1) + ReJoinType(1) + JoinEUI(8) + DevEUI(8) + RJcount1(2) + MIC(4)
 */
#define LM_RE_JOIN_1_MSG_SIZE          ( LM_MHDR_FIELD_SIZE + LM_JOIN_TYPE_FIELD_SIZE + \
                                              LM_JOIN_EUI_FIELD_SIZE + LM_DEV_EUI_FIELD_SIZE + \
                                              LM_RJCOUNT_1_FIELD_SIZE + \
                                              LM_MIC_FIELD_SIZE )

/*!
 * ReJoinRequest type 0 or 2 frame size
 *
 * MHDR(1) + ReJoinType(1) + NetID(3) + DevEUI(8) + RJcount0(2) + MIC(4)
 */
#define LM_RE_JOIN_0_2_MSG_SIZE        ( LM_MHDR_FIELD_SIZE + LM_JOIN_TYPE_FIELD_SIZE + \
                                              LM_NET_ID_FIELD_SIZE + LM_DEV_EUI_FIELD_SIZE + \
                                              LM_RJCOUNT_0_FIELD_SIZE + \
                                              LM_MIC_FIELD_SIZE )

/*!
 * JoinAccept frame minimum size
 *
 * MHDR(1) + AppNonce(3) + NetID(3) + DevAddr(4) + DLSettings(1) + RxDelay(1) + MIC(4)
 */
#define LM_JOIN_ACCEPT_FRAME_MIN_SIZE  ( LM_MHDR_FIELD_SIZE + LM_JOIN_NONCE_FIELD_SIZE + \
                                              LM_NET_ID_FIELD_SIZE + LM_DEV_ADDR_FIELD_SIZE + \
                                              LM_DL_SETTINGS_FIELD_SIZE + LM_RX_DELAY_FIELD_SIZE + \
                                              LM_MIC_FIELD_SIZE )

/*!
 * JoinAccept frame maximum size
 *
 * MHDR(1) + AppNonce(3) + NetID(3) + DevAddr(4) + DLSettings(1) + RxDelay(1) + CFList(16) + MIC(4)
 */
#define LM_JOIN_ACCEPT_FRAME_MAX_SIZE  ( LM_MHDR_FIELD_SIZE + LM_JOIN_NONCE_FIELD_SIZE + \
                                              LM_NET_ID_FIELD_SIZE + LM_DEV_ADDR_FIELD_SIZE + \
                                              LM_DL_SETTINGS_FIELD_SIZE + LM_RX_DELAY_FIELD_SIZE + \
                                              LM_CF_LIST_FIELD_SIZE + LM_MIC_FIELD_SIZE )

/*!
 * MIC computation offset
 * \remark required for 1.1.x support
 */
#define JOIN_ACCEPT_MIC_COMPUTATION_OFFSET                                                   \
    ( LM_MHDR_FIELD_SIZE + LM_JOIN_TYPE_FIELD_SIZE + LM_JOIN_EUI_FIELD_SIZE + \
      LM_DEV_NONCE_FIELD_SIZE )

/*!
 * FRMPayload overhead to be used when setting the Radio.SetMaxPayloadLength
 * 
 * Overhead to be used when setting the Radio.SetMaxPayloadLength in RxWindowSetup function.
 *
 * MHDR(1) + FHDR(7) + Port(1) + MIC(4)
 *
 * Maximum PHYPayload = MaxPayloadOfDatarate + LM_FRAME_PAYLOAD_OVERHEAD_SIZE
 */
#define LM_FRAME_PAYLOAD_OVERHEAD_SIZE ( LM_MHDR_FIELD_SIZE + ( LM_FHDR_DEV_ADDR_FIELD_SIZE + \
                                              LM_FHDR_F_CTRL_FIELD_SIZE + LM_FHDR_F_CNT_FIELD_SIZE ) + \
                                              LM_F_PORT_FIELD_SIZE + LM_MIC_FIELD_SIZE )

/*!
 * FRMPayload minimum size
 * 
 * MHDR(1) + FHDR(7) + MIC(4)
 */
#define LM_FRAME_PAYLOAD_MIN_SIZE      ( LM_MHDR_FIELD_SIZE + ( LM_FHDR_DEV_ADDR_FIELD_SIZE + \
                                              LM_FHDR_F_CTRL_FIELD_SIZE + LM_FHDR_F_CNT_FIELD_SIZE ) + \
                                              LM_MIC_FIELD_SIZE )
/*!
 * FRMPayload maximum possible size
 *
 * MHDR(1) + FHDR(7) + Port(1) + MACPayload(242) + MIC(4)
 */
#define LM_FRAME_PAYLOAD_MAX_SIZE      ( LM_MHDR_FIELD_SIZE + ( LM_FHDR_DEV_ADDR_FIELD_SIZE + \
                                              LM_FHDR_F_CTRL_FIELD_SIZE + LM_FHDR_F_CNT_FIELD_SIZE ) + \
                                              LM_F_PORT_FIELD_SIZE + LM_MAC_PAYLOAD_FIELD_MAX_SIZE + \
                                              LM_MIC_FIELD_SIZE )

#define LM_MAX_NODES                        (150)
#define LM_KEY_LEN                          (16)
#define LM_MIC_LEN                          (4)

#define LM_DR(sf, bw)               ((uint8_t)((sf) | ((bw)<<4)))
#define LM_DR_RFU                   (0xFF)
#define LM_POW_RFU                  (-128)

/* Channel Mask Control */
#define LM_CMC(from, to)            ((uint8_t)((from) | ((to)<<8)))
#define LM_CMC_RFU                  (0xFFFF)
#define LM_CMC_ALL_ON               (0xFFFE)
#define LM_CMC_ALL_125KHZ_ON        (0xFFFD)
#define LM_CMC_ALL_125KHZ_OFF       (0xFFFC)

#define LM_FOPTS_MAX_LEN                    (15)
#define LM_MACCMD_MAX_LEN                   (128)

#define LM_BEACON_PERIOD                    (128)

#ifndef DEVADDR2STR
#define DEVADDR2STR(a) (a)[0], (a)[1], (a)[2], (a)[3]
#define DEVADDRSTR "%02x:%02x:%02x:%02x"
#endif

enum {
    DR0 = 0,
    DR1 = 1,
    DR2 = 2,
    DR3 = 3,
    DR4 = 4,
    DR5 = 5,
    DR6 = 6,
    DR7 = 7,
    DR8 = 8,
    DR9 = 9,
    DR10 = 11,
    DR12 = 12,
    DR13 = 13,
    DR14 = 14,
    DR15 = 15,
};

enum {
    FSK = 0,
    SF5 = 5,
    SF6 = 6,
    SF7 = 7,
    SF8 = 8,
    SF9 = 9,
    SF10 = 10,
    SF11 = 11,
    SF12 = 12,
};

enum {
    BW125 = 0,      // 125*1 125*pow(2,n)
    BW250 = 1,      // 125*2
    BW500 = 2,      // 125*4
};

#define LM_LGW_DR(sf,bw)            ( (uint16_t)( (sf) | ((bw)<<8) ))

typedef enum {
    ABP   = 0,
    OTAA  = 1,
} lm_mode_t;

typedef enum {
    RXWIN_IDLE      = -1,
    CLASS_C_RX2_0   = 4,
    CLASS_A_RX1     = 1,
    CLASS_C_RX2_1   = 5,
    CLASS_A_RX2     = 2,
    CLASS_C_RX2     = 0,
    CLASS_B_RX      = 3,
} lm_rxwin_t;

typedef enum {
    CLASS_A  = 0,
    CLASS_B  = 1,
    CLASS_C  = 2,
} lm_class_t;

typedef enum {
    EU868,
    US915,
    CN779,
    EU433,
    AU915,
    CN470,
    AS923,
    KR920,
    IN865,
    RU864,
} lm_band_t;

typedef union {
    uint8_t data;
    struct {
#ifdef ENABLE_BIG_ENDIAN
        uint8_t mtype           : 3;
        uint8_t rfu             : 3;
        uint8_t major           : 2;
#else
        uint8_t major           : 2;
        uint8_t rfu             : 3;
        uint8_t mtype           : 3;
#endif
    } bits;
} PACKED lm_mhdr_t;

typedef union {
    uint32_t data;
    uint8_t buf[4];
    struct {
#ifdef ENABLE_BIG_ENDIAN
        uint32_t nwkaddr        : 25;
        uint32_t nwkid          : 7;
#else
        uint32_t nwkid          : 7;
        uint32_t nwkaddr        : 25;
#endif
    } PACKED bits;
} lm_devaddr_t;

typedef union {
    uint8_t data;
    struct {
#ifdef ENABLE_BIG_ENDIAN
        uint8_t adr             : 1;
        uint8_t adrackreq       : 1;
        uint8_t ack             : 1;
        uint8_t classb          : 1;
        uint8_t foptslen        : 4;
#else
        uint8_t foptslen        : 4;
        uint8_t classb          : 1;
        uint8_t ack             : 1;
        uint8_t adrackreq       : 1;
        uint8_t adr             : 1;
#endif
    } PACKED ul;

    struct {
#ifdef ENABLE_BIG_ENDIAN
        uint8_t adr             : 1;
        uint8_t rfu             : 1;
        uint8_t ack             : 1;
        uint8_t fpending        : 1;
        uint8_t foptslen        : 4;
#else
        uint8_t foptslen        : 4;
        uint8_t fpending        : 1;
        uint8_t ack             : 1;
        uint8_t rfu             : 1;
        uint8_t adr             : 1;
#endif
    } PACKED dl;
} lm_fctrl_t;

typedef struct {
    lm_devaddr_t addr;
    lm_fctrl_t fctrl;
    uint16_t fcnt;
    uint8_t fopts[LM_FOPTS_MAX_LEN];
} PACKED lm_fhdr_t;

typedef union {
    uint32_t data;
    uint8_t buf[4];
} PACKED lm_mic_t;

typedef union {
    uint32_t data               : 24;
    uint8_t buf[3];
} PACKED lm_anonce_t;

typedef lm_anonce_t lm_netid_t;

typedef union {
    uint16_t data;
    uint8_t buf[2];
} PACKED lm_dnonce_t;

typedef union {
    uint8_t data;
    struct {
#ifdef ENABLE_BIG_ENDIAN
        uint8_t rfu             : 4;
        uint8_t del             : 4;
#else
        uint8_t del             : 4;
        uint8_t rfu             : 4;
#endif
    } bits;
} PACKED lm_rxdelay_t;

typedef union {
    uint8_t data;
    struct {
#ifdef ENABLE_BIG_ENDIAN
        uint8_t rfu             : 1;
        uint8_t rx1droft        : 3;
        uint8_t rx2dr           : 4;
#else
        uint8_t rx2dr           : 4;
        uint8_t rx1droft        : 3;
        uint8_t rfu             : 1;
#endif
    } bits;
} PACKED lm_dlset_t;

typedef struct {
    lm_fhdr_t fhdr;
    uint8_t fport;
    uint8_t frmpld[256];
    uint8_t flen;
} lm_pld_mac_t;

typedef struct {
    uint8_t appeui[8];
    uint8_t deveui[8];
    lm_dnonce_t devnonce;
} lm_pld_jr_t;

typedef struct {
    lm_anonce_t appnonce;
    lm_netid_t netid;
    lm_devaddr_t addr;
    lm_dlset_t dlsettings;
    lm_rxdelay_t rxdelay;
    uint8_t cflist[16];
    int cflist_len;
    uint8_t nwkskey[16];
    uint8_t appskey[16];
} lm_pld_ja_t;

//lm.mote.abp.devaddr;
typedef struct lm {
    uint8_t flag;
    lm_mode_t mode;
    uint8_t joined;

    uint8_t appeui[8];
    uint8_t deveui[8];
    lm_devaddr_t addr;
    uint8_t nwkskey[16];
    uint8_t appskey[16];
    uint8_t appkey[16];

    lm_dnonce_t devnonce;
    lm_anonce_t appnonce;
    lm_netid_t netid;

    uint32_t ufsum;
    uint32_t uflost;
    uint32_t ufcnt;
    uint32_t dfcnt;

    bool dlack;
    bool dlready;
    uint8_t dlport;
    uint16_t dlsize;
    uint8_t *dlbuf;
    lm_rxwin_t dlrxwin;

    uint8_t maccmdsize;
    uint8_t maccmd[LM_MACCMD_MAX_LEN];

    lm_rxwin_t rxwin;
    lm_dlset_t dlsettings;
    lm_rxdelay_t rxdelay;

    struct {
        uint8_t prediocity;
        uint8_t dr;
        uint32_t freq;
    } pingslot;

    struct {
        uint32_t freq;
    } beacon;

    //struct lm *next;
} lm_mote_t;

typedef struct {
    lm_mote_t *mote;
    uint8_t deveui[8];
    uint8_t appeui[8];
    lm_mhdr_t mhdr;
    union {
        lm_pld_mac_t mac;
        lm_pld_jr_t jr;
        lm_pld_ja_t ja;
    } pld;
    lm_mic_t mic;

    int len;
    uint8_t buf[256];
    struct {
        uint8_t *buf;
        uint8_t len;
    } maccmd;
} lm_frame_t;

typedef struct {
    uint8_t *aeskey;
    lm_anonce_t anonce;
    lm_netid_t netid;
    lm_dnonce_t dnonce;
} lm_skey_seed_t;

typedef enum {
    LM_UPLINK = 0,
    LM_DOWNLINK = 1,
} lm_link_t;

typedef struct {
    uint8_t *aeskey;
    uint8_t *in;
    uint16_t len;
    lm_devaddr_t addr;
    lm_link_t link;
    uint32_t fcnt32;
} lm_key_t;

typedef struct {
    uint8_t cmd;
    union {
        uint8_t buf[14];
        struct {
            uint8_t margin;
            uint8_t gwcnt;
        } lchk_ans;

        struct {
            union {
                uint8_t data;
                struct {
#ifdef ENABLE_BIG_ENDIAN
                    uint8_t dr              : 4;
                    uint8_t txpow           : 4;
#else
                    uint8_t txpow           : 4;
                    uint8_t dr              : 4;
#endif
                } bits;
            } dr_txpow;
            uint8_t chmsk[2];
            union {
                uint8_t data;
                struct {
#ifdef ENABLE_BIG_ENDIAN
                    uint8_t rfu             : 1;
                    uint8_t chmaskcntl      : 3;
                    uint8_t nbtrans         : 4;
#else
                    uint8_t nbtrans         : 4;
                    uint8_t chmaskcntl      : 3;
                    uint8_t rfu             : 1;
#endif

                } bits;
            } redundancy;
        } ladr_req;

        struct {
            union {
                uint8_t data;
                struct {
#ifdef ENABLE_BIG_ENDIAN
                    uint8_t rfu             : 4;
                    uint8_t maxdc           : 4;
#else
                    uint8_t maxdc           : 4;
                    uint8_t rfu             : 4;
#endif
                } bits;
            } dcpl;
        } dcap_req;

        struct {
            lm_dlset_t dlsettings;
            uint8_t freq[3];
        } dn2p_req;

        //        struct{
        //        }devs_req;

        struct {
            uint8_t chindex;
            uint8_t freq[3];
            union {
                uint8_t data;
                struct {
#ifdef ENABLE_BIG_ENDIAN
                    uint8_t max             : 4;
                    uint8_t min             : 4;
#else
                    uint8_t min             : 4;
                    uint8_t max             : 4;
#endif

                } bits;
            } drrange;
        } snch_req;

        struct {
            union {
                uint8_t data;
                struct {
#ifdef ENABLE_BIG_ENDIAN
                    uint8_t rfu             : 4;
                    uint8_t del             : 4;
#else
                    uint8_t del             : 4;
                    uint8_t rfu             : 4;
#endif
                } bits;
            } rxtspl;
        } rxts_req;

        struct {
            uint8_t freq[3];
            union {
                uint8_t data;
                struct {
                    uint8_t val             : 4;
                    uint8_t rfu             : 4;
                } bits;
            } dr;
        } psch_req;

        struct {
            uint8_t delay[2];
            uint8_t channel;
        } bcnt_ans;

        struct {
            uint8_t freq[3];
        } bfreq_req;

        struct {
            uint8_t epoch[4];
            uint8_t epoch_ms;
        } devt_req;

        struct {
            uint8_t chindex;
            uint8_t freq[3];
        } dlch_req;
    } pld;
    uint8_t len;
} lm_maccmd_t;

typedef struct {
    uint32_t freq;
    uint32_t dr;
} lm_beacon_t;

typedef struct {
    uint32_t freq;
    uint32_t dr;
    uint32_t prediocity;
} lm_ping_t;

typedef struct{
    union {
        uint8_t data;
        struct {
            uint8_t nwkskey :1;
            uint8_t appskey :1;
            uint8_t appkey  :1;
        } bits;
    } flag;
    uint8_t *nwkskey;
    uint8_t *appskey;
    uint8_t *appkey;
} lm_key_grp_t;

typedef struct {
    uint32_t freq[5];
    uint8_t len;
} lm_cflist_t;

typedef struct lgw_pkt_rx_s lm_rxpkt_t;
typedef struct lgw_pkt_tx_s lm_txpkt_t;

#include "lm-log.h"
//#include "lm-ch.h"

typedef struct{
    lm_band_t band;
    const char *name;
    const uint8_t *dr_to_sfbw_tab;
    struct {
        uint8_t max_eirp_index;
        uint8_t max_tx_power_index;
    } power;
    const uint16_t *chmaskcntl_tab;
} lm_region_t;

typedef struct {
    const lm_region_t *region;
    struct {
        uint32_t freq;
        uint8_t dr;
    } rxwin2;
    uint8_t rxdelay;
    lm_ping_t ping;
    struct {
        uint8_t buf[16];
        uint8_t len;
    } cflist;
    bool mac_cmd_dup;
    bool force_port0;
} lm_config_t;

int lm_init(lm_band_t band);
int lm_add(lm_mote_t *mote);
lm_mote_t *lm_get_mote(uint8_t *deveui);
int lm_del(uint8_t *deveui);
int lm_set_key(lm_key_grp_t *kgrp);
int lm_add_tx(uint8_t *deveui, uint8_t port, uint8_t *buf, uint16_t size);
int lm_tx_maccmd(uint8_t *deveui, lm_maccmd_t *maccmd);
int lm_parse(lm_frame_t *frame, uint8_t *buf, int len);
int lm_pack(lm_frame_t *frame, uint8_t *buf, int *len);
int lm_answer(lm_frame_t *frame, lm_rxpkt_t *rxpkt, lm_txpkt_t *txpkt);

void lm_cpy(uint8_t *dest, uint8_t *src, int len);
int lm_maccmd_valid(uint8_t mac_header, uint8_t *opts, int len);

int8_t lm_get_dr(uint8_t mod, uint32_t datarate, uint8_t bw);
int8_t lm_get_rf(uint8_t dr, uint8_t *mod, uint32_t *datarate, uint8_t *bw, uint8_t *fdev);
const char *lm_get_rf_name(uint8_t mod, uint32_t datarate, uint8_t bw, uint8_t fdev);

lm_band_t lm_get_band_type(const char *band);
const char *lm_get_band_name(lm_band_t band);

uint32_t lm_read_dw(uint8_t *buf);

/* crypto functions */
void lm_msg_mic(lm_mic_t* mic, lm_key_t *key);
void lm_join_mic(lm_mic_t* mic, lm_key_t *key);
int lm_encrypt(uint8_t *out, lm_key_t *key);
int lm_join_decrypt(uint8_t *out, lm_key_t *key);
int lm_join_encrypt(uint8_t *out, lm_key_t *key);
void lm_get_skeys(uint8_t *nwkskey, uint8_t *appskey, lm_skey_seed_t *seed);

void lm_test(void);

#endif
