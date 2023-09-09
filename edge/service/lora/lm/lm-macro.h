#ifndef __LM_MACRO_H
#define __LM_MACRO_H

#define LM_VERSION_MAJOR_R1                     (0x00)

#define LORAWAN_V11_CLASSB

enum {
    // Error code
    LM_OK                   = 0,
    LM_ERR_CMD_UNKNOWN      = -1,
    LM_ERR_PL_LEN           = -2,
    LM_ERR_MIC              = -3,
    LM_ERR_DECRYPT          = -4,
    LM_ERR_MACCMD           = -5,
    LM_ERR_MACCMD_LEN       = -6,
    LM_ERR_FOPTS_PORT0      = -7,
    LM_ERR_JOINR_LEN        = -8,
    LM_ERR_JOINA_LEN        = -9,
    LM_ERR_MALLOC           = -10,
    LM_ERR_NOT_AVALAIBLE    = -11,
    LM_ERR_BAND             = -12,
    LM_ERR_PARA             = -13,
    LM_ERR_NODE_USED_UP     = -14,
    LM_ERR_UNKOWN_FRAME     = -15,
    LM_ERR_TX_BUF_NOT_EMPTY = -16,
    LM_ERR_UNKOWN_DEVEUI    = -17,
    LM_ERR_NO_HEAP          = -18,
    LM_ERR_UNKOWN_DATA_RATE = -19,
    LM_ERR_FRAME_TOO_SHORT  = -20,
    LM_ERR_NODE_EXISTS      = -21,
    LM_ERR_PRM_SIZE         = -22,
};

typedef enum {
    LM_MTYPE_JOIN_REQUEST   = 0x00,
    LM_MTYPE_JOIN_ACCEPT    = 0x01,
    LM_MTYPE_MSG_UP         = 0x02,
    LM_MTYPE_MSG_DOWN       = 0x03,
    LM_MTYPE_CMSG_UP        = 0x04,
    LM_MTYPE_CMSG_DOWN      = 0x05,
    LM_MTYPE_RFU            = 0x06,
    LM_MTYPE_PROPRIETARY    = 0x07
} lm_mtype_t;

enum {
    LM_MHDR                 = 0x00,
};

enum {
    // Data frame format
    //LM_OFF_DAT_HDR        = 0,
    LM_DATA_OFF_DEVADDR     = 1,
    LM_DATA_OFF_FCTRL       = 5,
    LM_DATA_OFF_FCNT        = 6,
    LM_DATA_OFF_FOPTS       = 8,
};

enum {
    // Join Request frame format (offset)
    //LM_OFF_JR_HDR         = 0,
    LM_JR_OFF_APPEUI        = 1,
    LM_JR_OFF_DEVEUI        = 9,
    LM_JR_OFF_DEVNONCE      = 17,
    LM_JR_OFF_MIC           = 19,
    LM_JR_LEN               = 23
};
enum {
    // Join Accept frame format (offset)
    //LM_OFF_JA_HDR         = 0,
    LM_JA_OFF_APPNONCE      = 1,
    LM_JA_OFF_NETID         = 4,
    LM_JA_OFF_DEVADDR       = 7,
    LM_JA_OFF_DLSET         = 11,
    LM_JA_OFF_RXDLY         = 12,
    LM_JA_OFF_CFLIST        = 13,
    LM_JA_LEN               = 17,
    LM_JA_LEN_EXT           = 17+16
};

enum {
    MOTE_MAC_LINK_CHECK_REQ             = 0x02,
    MOTE_MAC_LINK_ADR_ANS               = 0x03,
    MOTE_MAC_DUTY_CYCLE_ANS             = 0x04,
    MOTE_MAC_RX_PARAM_SETUP_ANS         = 0x05,
    MOTE_MAC_DEV_STATUS_ANS             = 0x06,
    MOTE_MAC_NEW_CHANNEL_ANS            = 0x07,
    MOTE_MAC_RX_TIMING_SETUP_ANS        = 0x08,
    MOTE_MAC_TX_PARAM_SETUP_ANS         = 0x09,
    MOTE_MAC_DL_CHANNEL_ANS             = 0x0A,
    MOTE_MAC_DEVICE_TIME_REQ            = 0x0D,
    MOTE_MAC_PING_SLOT_INFO_REQ         = 0x10,
    MOTE_MAC_PING_SLOT_FREQ_ANS         = 0x11,
    MOTE_MAC_BEACON_TIMING_REQ          = 0x12,
    MOTE_MAC_BEACON_FREQ_ANS            = 0x13,
};

enum {
    SRV_MAC_LINK_CHECK_ANS           = 0x02,
    SRV_MAC_LINK_ADR_REQ             = 0x03,
    SRV_MAC_DUTY_CYCLE_REQ           = 0x04,
    SRV_MAC_RX_PARAM_SETUP_REQ       = 0x05,
    SRV_MAC_DEV_STATUS_REQ           = 0x06,
    SRV_MAC_NEW_CHANNEL_REQ          = 0x07,
    SRV_MAC_RX_TIMING_SETUP_REQ      = 0x08,
    SRV_MAC_TX_PARAM_SETUP_REQ       = 0x09,
    SRV_MAC_DL_CHANNEL_REQ           = 0x0A,
    SRV_MAC_DEVICE_TIME_ANS          = 0x0D,
    SRV_MAC_PING_SLOT_INFO_ANS       = 0x10,
    SRV_MAC_PING_SLOT_CHANNEL_REQ    = 0x11,
    SRV_MAC_BEACON_TIMING_ANS        = 0x12,
    SRV_MAC_BEACON_FREQ_REQ          = 0x13,
};

enum {
    MOTE_MAC_LEN_LINK_CHECK_REQ         = 1,
    MOTE_MAC_LEN_LINK_ADR_ANS           = 2,
    MOTE_MAC_LEN_DUTY_CYCLE_ANS         = 1,
    MOTE_MAC_LEN_RX_PARAM_SETUP_ANS     = 2,
    MOTE_MAC_LEN_DEV_STATUS_ANS         = 3,
    MOTE_MAC_LEN_NEW_CHANNEL_ANS        = 2,
    MOTE_MAC_LEN_RX_TIMING_SETUP_ANS    = 1,
    MOTE_MAC_LEN_TX_PARAM_SETUP_ANS     = 1,
    MOTE_MAC_LEN_DL_CHANNEL_ANS         = 2,
    MOTE_MAC_LEN_DEVICE_TIME_REQ        = 1,
    MOTE_MAC_LEN_PING_SLOT_INFO_REQ     = 2,
    MOTE_MAC_LEN_PING_SLOT_FREQ_ANS     = 2,
    MOTE_MAC_LEN_BEACON_TIMING_REQ      = 1,
#ifdef LORAWAN_V11_CLASSB
    MOTE_MAC_LEN_BEACON_FREQ_ANS        = 2,
#else
    MOTE_MAC_LEN_BEACON_FREQ_ANS        = 1,
#endif
};

enum {
    SRV_MAC_LEN_LINK_CHECK_ANS          = 3,
    SRV_MAC_LEN_LINK_ADR_REQ            = 5,
    SRV_MAC_LEN_DUTY_CYCLE_REQ          = 2,
    SRV_MAC_LEN_RX_PARAM_SETUP_REQ      = 5,
    SRV_MAC_LEN_DEV_STATUS_REQ          = 1,
    SRV_MAC_LEN_NEW_CHANNEL_REQ         = 6,
    SRV_MAC_LEN_RX_TIMING_SETUP_REQ     = 2,
    SRV_MAC_LEN_TX_PARAM_SETUP_REQ      = 2,
    SRV_MAC_LEN_DL_CHANNEL_REQ          = 5,
    SRV_MAC_LEN_DEVICE_TIME_ANS         = 6,
    SRV_MAC_LEN_PING_SLOT_INFO_ANS      = 1,
    SRV_MAC_LEN_PING_SLOT_CHANNEL_REQ   = 5,
    SRV_MAC_LEN_BEACON_TIMING_ANS       = 4,
    SRV_MAC_LEN_BEACON_FREQ_REQ         = 4,
};

#endif // __LM_MACRO_H
