#include "lm-log.h"
#include "lm.h"
#include "math.h"

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <time.h>
#include <linklist.h>
#include <libubox/ulog.h>

//static const char *NAME = "LMLOG";
#define LMLOG_INFO(x...)              log_print(LOG_DEBUG, x);

extern struct list mote_list;

extern const lm_region_t *lm_region;
extern const int8_t lm_max_eirp_tab[16];
extern const int8_t lm_pow_tab[16];

const char *lm_mtype_str[] = {
    "JOIN REQUEST",
    "JOIN ACCEPT",
    "UNCONFIRMED DATA UP",
    "UNCONFIRMED DATA DOWN",
    "CONFIRMED DATA UP",
    "CONFIRMED DATA DOWN",
    "RFU",
    "PROPRIETARY",
};

const char *lm_mtype_str_abbr[] = {
    "JR ->",
    "JA <-",
    "UU ->",
    "UD <-",
    "CU ->",
    "CD <-",
    "UK",
    "PM",
};

typedef struct{
    uint8_t cmd;
    char *str;
}lm_maccmd_str_t;

const lm_maccmd_str_t lm_node_maccmd_str[] = {
    { MOTE_MAC_LINK_CHECK_REQ,          "LinkCheckReq" },
    { MOTE_MAC_LINK_ADR_ANS,            "LinkADRAns" },
    { MOTE_MAC_DUTY_CYCLE_ANS,          "DutyCycleAns" },
    { MOTE_MAC_RX_PARAM_SETUP_ANS,      "RXParamSetupAns" },
    { MOTE_MAC_DEV_STATUS_ANS,          "DevStatusAns" },
    { MOTE_MAC_NEW_CHANNEL_ANS,         "NewChannelAns" },
    { MOTE_MAC_RX_TIMING_SETUP_ANS,     "RXTimingSetupAns" },
    { MOTE_MAC_TX_PARAM_SETUP_ANS,      "TxParamSetupAns" },
    { MOTE_MAC_DL_CHANNEL_ANS,          "DlChannelAns" },
    { MOTE_MAC_DEVICE_TIME_REQ,         "DeviceTimeReq" },
    { MOTE_MAC_PING_SLOT_INFO_REQ,      "PingSlotInfoReq" },
    { MOTE_MAC_PING_SLOT_FREQ_ANS,      "PingSlotFreqAns" },
    { MOTE_MAC_BEACON_TIMING_REQ,       "BeaconTimingReq" },
    { MOTE_MAC_BEACON_FREQ_ANS,         "BeaconFreqAns" },
};

const lm_maccmd_str_t lm_server_maccmd_str[] = {
    { SRV_MAC_LINK_CHECK_ANS,           "LinkCheckAns" },
    { SRV_MAC_LINK_ADR_REQ,             "LinkADRReq" },
    { SRV_MAC_DUTY_CYCLE_REQ,           "DutyCycleReq" },
    { SRV_MAC_RX_PARAM_SETUP_REQ,       "RXParamSetupReq" },
    { SRV_MAC_DEV_STATUS_REQ,           "DevStatusReq" },
    { SRV_MAC_NEW_CHANNEL_REQ,          "NewChannelReq" },
    { SRV_MAC_RX_TIMING_SETUP_REQ,      "RXTimingSetupReq" },
    { SRV_MAC_TX_PARAM_SETUP_REQ,       "TxParamSetupReq" },
    { SRV_MAC_DL_CHANNEL_REQ,           "DlChannelReq" },
    { SRV_MAC_DEVICE_TIME_ANS,          "DeviceTimeAns" },
    { SRV_MAC_PING_SLOT_INFO_ANS,       "PingSlotInfoAns" },
    { SRV_MAC_PING_SLOT_CHANNEL_REQ,    "PingSlotChannelReq" },
    { SRV_MAC_BEACON_TIMING_ANS,        "BeaconTimingAns" },
    { SRV_MAC_BEACON_FREQ_REQ,          "BeaconFreqReq" },
};

const char *lm_maccmd_str(uint8_t mtype, uint8_t cmd)
{
    int j;

    if ((mtype == LM_MTYPE_MSG_UP) || (mtype == LM_MTYPE_CMSG_UP)) {
        for (j = 0; j < (sizeof(lm_node_maccmd_str) / sizeof(lm_maccmd_str_t)); j++) {
            if (lm_node_maccmd_str[j].cmd == cmd) {
                return lm_node_maccmd_str[j].str;
            }
        }
    } else if ((mtype == LM_MTYPE_MSG_DOWN) || (mtype == LM_MTYPE_CMSG_DOWN)) {
        for (j = 0; j < (sizeof(lm_server_maccmd_str) / sizeof(lm_maccmd_str_t)); j++) {
            if (lm_server_maccmd_str[j].cmd == cmd) {
                return lm_server_maccmd_str[j].str;
            }
        }
    }
    return "Unknown";
}

void lm_unknown_pl(void)
{
    LMLOG_INFO("Unknown MAC command payload");
}

int lm_log_maccmd(uint8_t mac_header, lm_maccmd_type_t type, uint8_t *opts, int len)
{
    lm_mhdr_t mhdr;
    uint16_t ChMask;
    uint8_t dr;
    int8_t power;
    uint16_t chmaskcntl;
    uint8_t rx1drofst;
    uint8_t rx2dr;
    uint32_t freq;
    union {
        uint8_t data;
        struct {
            int8_t margin           :6;
        } bits;
    } dev_sta_margin;
    int i, ret;
    char strbuf[512];

    ret = lm_maccmd_valid(mac_header, opts, len);
    if (ret != LM_OK) {
        return ret;
    }

    mhdr.data = mac_header;

    LMLOG_INFO("MACCMD: (%s) %h", type == LM_MACCMD_FOPTS ? "FOPTS" : "PORT0", opts, len);

    i = 0;
    while (i < len) {
        strbuf[0] = '\0';
        //LMLOG_INFO("MACCMD: %02X ( %s )", opts[i], lm_maccmd_str(mhdr.bits.mtype, opts[i]));
        if ((mhdr.bits.mtype == LM_MTYPE_MSG_UP) || (mhdr.bits.mtype == LM_MTYPE_CMSG_UP)) {
            switch (opts[i]) {
                // Class A
            case MOTE_MAC_LINK_CHECK_REQ:
                LMLOG_INFO("MACCMD: %02X ( %s )", opts[i], lm_maccmd_str(mhdr.bits.mtype, opts[i]));
                i+=MOTE_MAC_LEN_LINK_CHECK_REQ;
                break;
            case MOTE_MAC_LINK_ADR_ANS:
                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "Status: 0x%02X"           ", "
                                     "Channel mask: %s"         ", "
                                     "Data rate: %s"            ", "
                                     "Power: %s",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     opts[i+1],
                                     (opts[i+1]&0x01)?"ACK":"NACK",
                                     (opts[i+1]&0x02)?"ACK":"NACK",
                                     (opts[i+1]&0x04)?"ACK":"NACK");
                i+=MOTE_MAC_LEN_LINK_ADR_ANS;
                break;
            case MOTE_MAC_DUTY_CYCLE_ANS:
                LMLOG_INFO("MACCMD: %02X ( %s )", opts[i], lm_maccmd_str(mhdr.bits.mtype, opts[i]));
                i+=MOTE_MAC_LEN_DUTY_CYCLE_ANS;
                break;
            case MOTE_MAC_RX_PARAM_SETUP_ANS:
                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "Status: 0x%02X"           ", "
                                     "Channel: %s"              ", "
                                     "RXWIN2: %s"               ", "
                                     "RX1DRoffset: %s",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     opts[i+1],
                                     (opts[i+1]&0x01)?"ACK":"NACK",
                                     (opts[i+1]&0x02)?"ACK":"NACK",
                                     (opts[i+1]&0x04)?"ACK":"NACK");
                i+=MOTE_MAC_LEN_RX_PARAM_SETUP_ANS;
                break;
            case MOTE_MAC_DEV_STATUS_ANS:
                if(opts[i+1] == 0){
                    sprintf(strbuf, "%d (External Powered)", opts[i+1]);
                }else if(opts[i+1] == 255){
                    sprintf(strbuf, "%d (Unknown)", opts[i+1]);
                }else{
                    sprintf(strbuf, "%d (%.1f%%)", opts[i+1], 100.0*opts[i+1]/255);
                }
                dev_sta_margin.data = opts[i+2];

                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "Battery: %s"              ", "
                                     "Margin: %d",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     strbuf,
                                     dev_sta_margin.bits.margin);
                i+=MOTE_MAC_LEN_DEV_STATUS_ANS;
                break;
            case MOTE_MAC_NEW_CHANNEL_ANS:
                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "Status: 0x%02X"           ", "
                                     "Channel mask: %s"         ", "
                                     "Data rate: %s",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     opts[i+1],
                                     (opts[i+1]&0x01)?"ACK":"NACK",
                                     (opts[i+1]&0x02)?"ACK":"NACK");
                i+=MOTE_MAC_LEN_NEW_CHANNEL_ANS;
                break;
            case MOTE_MAC_RX_TIMING_SETUP_ANS:
                LMLOG_INFO("MACCMD: %02X ( %s )", opts[i], lm_maccmd_str(mhdr.bits.mtype, opts[i]));
                i+=MOTE_MAC_LEN_RX_TIMING_SETUP_ANS;
                break;

            case MOTE_MAC_TX_PARAM_SETUP_ANS:
                LMLOG_INFO("MACCMD: %02X ( %s )", opts[i], lm_maccmd_str(mhdr.bits.mtype, opts[i]));
                i += MOTE_MAC_LEN_TX_PARAM_SETUP_ANS;
                break;
            case MOTE_MAC_DL_CHANNEL_ANS:
                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "Channel Frequency: %s"    ", "
                                     "Uplink Frequency: %s",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     (opts[i+1]&0x01)?"ACK":"NACK",
                                     (opts[i+1]&0x02)?"ACK":"NACK");
                i += MOTE_MAC_LEN_DL_CHANNEL_ANS;
                break;

            //Class B
            case MOTE_MAC_PING_SLOT_INFO_REQ:
#ifdef LORAWAN_V11_CLASSB
                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "Periodicity: %d" ", "
                                     "PingNb: %d" ", "
                                     "PingPeriod: %.2fs"
                                     ,
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     opts[i+1]&0x07,
                                     ( 1<<(7-(opts[i+1]&0x07))),
                                     0.03 * (1 << (5 + (opts[i+1]&0x07)))
                                     );
#else
                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "Periodicity: %ds"         ", "
                                     "DataRate: %d",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     ( 1<<((opts[i+1]>>4)&0x07) ),
                                     (opts[i+1]&0x0F));
#endif
                i+=MOTE_MAC_LEN_PING_SLOT_INFO_REQ;
                break;
            case MOTE_MAC_PING_SLOT_FREQ_ANS:
                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "Channel Frequency: %s"    ", "
                                     "Data Rate Range: %s",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     (opts[i+1]&0x01)!=0?"ACK":"NACK",
                                     (opts[i+1]&0x02)!=0?"ACK":"NACK");
                i+=MOTE_MAC_LEN_PING_SLOT_FREQ_ANS;
                break;
            case MOTE_MAC_BEACON_TIMING_REQ:
                LMLOG_INFO("MACCMD: %02X ( %s )", opts[i], lm_maccmd_str(mhdr.bits.mtype, opts[i]));
                i+=MOTE_MAC_LEN_BEACON_TIMING_REQ;
                break;
            case MOTE_MAC_BEACON_FREQ_ANS:
                LMLOG_INFO("MACCMD: %02X ( %s )", opts[i], lm_maccmd_str(mhdr.bits.mtype, opts[i]));
                i+=MOTE_MAC_LEN_BEACON_FREQ_ANS;
                break;
            case MOTE_MAC_DEVICE_TIME_REQ:
                LMLOG_INFO("MACCMD: %02X ( %s )", opts[i], lm_maccmd_str(mhdr.bits.mtype, opts[i]));
                i += MOTE_MAC_LEN_DEVICE_TIME_REQ;
                break;
            }
        }else if( (mhdr.bits.mtype == LM_MTYPE_MSG_DOWN) || (mhdr.bits.mtype == LM_MTYPE_CMSG_DOWN) ){
            switch(opts[i]){
            // Class A
            case SRV_MAC_LINK_CHECK_ANS:
                if(opts[i+1] == 255){
                    sprintf(strbuf, "%d (RFU)", opts[i+1]);
                }else{
                    sprintf(strbuf, "%ddB", opts[i+1]);
                }
                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "Margin: %s"               ", "
                                     "GwCnt: %d",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     strbuf,
                                      opts[i+2]);
                i+=SRV_MAC_LEN_LINK_CHECK_ANS;
                break;
            case SRV_MAC_LINK_ADR_REQ:
                dr = lm_region->dr_to_sfbw_tab[opts[i + 1] >> 4];
                power = lm_pow_tab[opts[i+1]&0x0F];
                chmaskcntl = (opts[i+4]>>4)&0x07;
                ChMask = opts[i+2] + (((uint16_t)opts[i+3])<<8);

                if(power == LM_POW_RFU){
                    sprintf(strbuf+strlen(strbuf), "TXPower: %d (RFU)", opts[i+1]&0x0F);
                }else{
                    sprintf(strbuf+strlen(strbuf), "TXPower: %d (%ddBm)", opts[i+1]&0x0F, power);
                }
                sprintf(strbuf+strlen(strbuf), ", ");

                if(dr == LM_DR_RFU){
                    sprintf(strbuf+strlen(strbuf), "DataRate: DR%d (RFU)", opts[i+1]>>4);
                }else if( (dr&0x0F) == FSK){
                    sprintf(strbuf+strlen(strbuf), "DataRate: DR%d (FSK)", opts[i+1]>>4);
                }else{
                    sprintf(strbuf+strlen(strbuf), "DataRate: DR%d (SF%d/BW%dKHz)", opts[i+1]>>4, dr&0x0F, (int)(125*pow(2,dr>>4)));
                }
                sprintf(strbuf+strlen(strbuf), ", ");

                sprintf(strbuf+strlen(strbuf), "ChMask: 0x%04X", ChMask);
                sprintf(strbuf+strlen(strbuf), ", ");

                sprintf(strbuf+strlen(strbuf), "NbRep: %d", opts[i+4]&0x0F);
                sprintf(strbuf+strlen(strbuf), ", ");
                switch(chmaskcntl){
                case LM_CMC_RFU:
                    sprintf(strbuf+strlen(strbuf), "ChMaskCntl: %d (RFU)", (opts[i+4]>>4)&0x07);
                    break;
                case LM_CMC_ALL_ON:
                    sprintf(strbuf+strlen(strbuf), "ChMaskCntl: %d (EU868 All on)", (opts[i+4]>>4)&0x07);
                    break;
                case LM_CMC_ALL_125KHZ_ON:
                    sprintf(strbuf+strlen(strbuf), "ChMaskCntl: %d, All 125KHz channels on, ChMask applies to 64 ~ 71", (opts[i+4]>>4)&0x07);
                    break;
                case LM_CMC_ALL_125KHZ_OFF:
                    sprintf(strbuf+strlen(strbuf), "ChMaskCntl: %d, All 125KHz channels off, ChMask applies to 64 ~ 71", (opts[i+4]>>4)&0x07);
                    break;
                default:
                    sprintf(strbuf+strlen(strbuf), "ChMaskCntl: %d, ChMask applies to %d ~ %d", (opts[i+4]>>4)&0x07, chmaskcntl*16, (chmaskcntl+1)*16-1);
                    break;
                }

                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "%s",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     strbuf);
                i+=SRV_MAC_LEN_LINK_ADR_REQ;
                break;
            case SRV_MAC_DUTY_CYCLE_REQ:
                if(opts[i+1] == 255){
                    sprintf(strbuf+strlen(strbuf), "MaxDCycle: %d(Off)", opts[i+1]);
                }else if(opts[i+1]<16){
                    sprintf(strbuf+strlen(strbuf), "MaxDCycle: %d (%.2f%%)", opts[i+1], 100.0/pow(2,opts[i+1]));
                }else{
                    sprintf(strbuf+strlen(strbuf), "MaxDCycle: %d(RFU)", opts[i+1]);
                }
                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "%s",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     strbuf);
                i+=SRV_MAC_LEN_DUTY_CYCLE_REQ;
                break;
            case SRV_MAC_RX_PARAM_SETUP_REQ:
                rx1drofst = (opts[i+1]>>4) & 0x07;
                rx2dr = lm_region->dr_to_sfbw_tab[opts[i + 1] & 0x0F];
                freq = (opts[i+2]) | ((uint32_t)opts[i+3]<<8) | ((uint32_t)opts[i+4]<<16);
                freq *= 100;

                sprintf(strbuf+strlen(strbuf), "RX1DROffset: %d", rx1drofst);
                sprintf(strbuf+strlen(strbuf), ", ");

                if(rx2dr == LM_DR_RFU){
                    sprintf(strbuf+strlen(strbuf), "RX2DataRate: DR%d (RFU)", opts[i+1] & 0x0F);
                }else if( (rx2dr&0x0F) == FSK){
                    sprintf(strbuf+strlen(strbuf), "RX2DataRate: DR%d (FSK)", opts[i+1] & 0x0F);
                }else{
                    sprintf(strbuf+strlen(strbuf), "RX2DataRate: DR%d (SF%d/BW%dKHz)", opts[i+1] & 0x0F, rx2dr&0x0F, (int)(125*pow(2,rx2dr>>4)));
                }
                sprintf(strbuf+strlen(strbuf), ", ");

                if( (freq < 100000000) && (freq != 0) ){
                    sprintf(strbuf+strlen(strbuf), "Freq: %d (RFU <100MHz)", freq);
                }else{
                    sprintf(strbuf+strlen(strbuf), "Freq: %d", freq);
                }
                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "%s",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     strbuf);
                i+=SRV_MAC_LEN_RX_PARAM_SETUP_REQ;
                break;
            case SRV_MAC_DEV_STATUS_REQ:
                LMLOG_INFO("MACCMD: %02X ( %s )",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]));
                i+=SRV_MAC_LEN_DEV_STATUS_REQ;
                break;
            case SRV_MAC_NEW_CHANNEL_REQ:
                freq = (opts[i+2]) | ((uint32_t)opts[i+3]<<8) | ((uint32_t)opts[i+4]<<16);
                freq *= 100;

                sprintf(strbuf+strlen(strbuf), "ChIndex: %d 0x%02X", opts[i+1], opts[i+1]);
                sprintf(strbuf+strlen(strbuf), ", ");

                if( (freq < 100000000) && (freq != 0) ){
                    sprintf(strbuf+strlen(strbuf), "Freq: %d (RFU <100MHz)", freq);
                }else{
                    sprintf(strbuf+strlen(strbuf), "Freq: %d", freq);
                }
                sprintf(strbuf+strlen(strbuf), ", ");

                sprintf(strbuf+strlen(strbuf), "DrRange: 0x%02X (DR%d ~ DR%d)", opts[i+5], opts[i+5]&0x0F, opts[i+5]>>4);

                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "%s",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     strbuf);
                i+=SRV_MAC_LEN_NEW_CHANNEL_REQ;
                break;
            case SRV_MAC_RX_TIMING_SETUP_REQ:
                if((opts[i+1]&0x0F) == 0){
                    sprintf(strbuf+strlen(strbuf), "Del: %ds", (opts[i+1]&0x0F)+1);
                }else{
                    sprintf(strbuf+strlen(strbuf), "Del: %ds", opts[i+1]&0x0F);
                }

                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "%s",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     strbuf);

                i+=SRV_MAC_LEN_RX_TIMING_SETUP_REQ;
                break;
            case SRV_MAC_TX_PARAM_SETUP_REQ:
                sprintf(strbuf+strlen(strbuf), "MaxEIRP: %ddBm", lm_max_eirp_tab[(opts[i+1] & 0x0F)]);
                sprintf(strbuf+strlen(strbuf), ", ");
                sprintf(strbuf+strlen(strbuf), "UplinkDwellTime: %d", (opts[i+1]&0x10)?1:0);
                sprintf(strbuf+strlen(strbuf), ", ");
                sprintf(strbuf+strlen(strbuf), "DownlinkDwellTime: %d", (opts[i+1]&0x10)?1:0);

                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "%s",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     strbuf);

                i += SRV_MAC_LEN_TX_PARAM_SETUP_REQ;
                break;
            case SRV_MAC_DL_CHANNEL_REQ:
                freq = (opts[i+2]) | ((uint32_t)opts[i+3]<<8) | ((uint32_t)opts[i+4]<<16);
                freq *= 100;

                sprintf(strbuf+strlen(strbuf), "ChIndex: %d", opts[i+1]);
                sprintf(strbuf+strlen(strbuf), ", ");

                if( (freq < 100000000) && (freq != 0) ){
                    sprintf(strbuf+strlen(strbuf), "Freq: %d (RFU <100MHz)", freq);
                }else{
                    sprintf(strbuf+strlen(strbuf), "Freq: %d", freq);
                }

                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "%s",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     strbuf);

                i += SRV_MAC_LEN_DL_CHANNEL_REQ;
                break;

            case SRV_MAC_PING_SLOT_INFO_ANS:
                LMLOG_INFO("MACCMD: %02X ( %s )",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]));
                i += SRV_MAC_LEN_PING_SLOT_INFO_ANS;
                break;
            case SRV_MAC_PING_SLOT_CHANNEL_REQ:
                freq = (opts[i+1]) | ((uint32_t)opts[i+2]<<8) | ((uint32_t)opts[i+3]<<16);
                freq *= 100;
                if( (freq < 100000000) && (freq != 0) ){
                    sprintf(strbuf+strlen(strbuf), "Freq: %d (RFU <100MHz)", freq);
                }else{
                    sprintf(strbuf+strlen(strbuf), "Freq: %d", freq);
                }
                sprintf(strbuf+strlen(strbuf), ", ");
#ifdef LORAWAN_V11_CLASSB
                sprintf(strbuf+strlen(strbuf), "Data Rate: DR%d", opts[i+4]&0x0F);
#else
                sprintf(strbuf+strlen(strbuf), "Data Rate: DR%d ~ DR%d", opts[i+4]&0x0F, (opts[i+4]>>4)&0x0F);
#endif
                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "%s",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     strbuf);

                i += SRV_MAC_LEN_PING_SLOT_CHANNEL_REQ;
                break;
            case SRV_MAC_BEACON_TIMING_ANS: {
                uint32_t delay = (opts[i+1]) | ((uint16_t)opts[i+2]<<8);
                uint8_t channel = opts[i+3];
                if(channel == 0){
                    sprintf(strbuf+strlen(strbuf), "Beacon channel fixed");
                }else{
                    sprintf(strbuf+strlen(strbuf),"Beacon channel %d", channel);
                }
                sprintf(strbuf+strlen(strbuf), ", ");

                sprintf(strbuf+strlen(strbuf),"RTime: %d ~ %dms (TX end to beacon start)", 30*delay, 30*(delay+1));

                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "%s",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     strbuf);

                i += SRV_MAC_LEN_BEACON_TIMING_ANS;
            }
                break;
            case SRV_MAC_BEACON_FREQ_REQ:
                freq = (opts[i+1]) | ((uint32_t)opts[i+2]<<8) | ((uint32_t)opts[i+3]<<16);
                freq *= 100;
                if( (freq < 100000000) && (freq != 0) ){
                    sprintf(strbuf+strlen(strbuf), "Freq: %d (RFU <100MHz)", freq);
                }else{
                    sprintf(strbuf+strlen(strbuf), "Freq: %d", freq);
                }

                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                                     "%s",
                                     opts[i],
                                     lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                                     strbuf);
                i += SRV_MAC_LEN_BEACON_FREQ_REQ;
                break;
            case SRV_MAC_DEVICE_TIME_ANS: {
                double seconds;
                time_t ult;
                struct tm  ts;
                char       buf[80];
                uint32_t sec = (opts[i+1]<<0) | (opts[i+2]<<8) | (opts[i+3]<<16) | (opts[i+4]<<24);
                seconds = (uint32_t)opts[i+5];
                seconds = seconds / (1<<8);
                printf("%u %f\n", sec, seconds);

                seconds += sec;
                ult = (time_t)seconds + 315964800 - (37-19) ;

                ts = *localtime(&ult);
                /* week: %a */
                strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S (%z)", &ts);

                LMLOG_INFO("MACCMD: %02X ( %s )"      ", "
                         "Seconds since epoch: %.3f" ", "
                         "Time: %s",
                         opts[i],
                         lm_maccmd_str(mhdr.bits.mtype, opts[i]),
                         seconds,
                         buf
                         );
                i += SRV_MAC_LEN_DEVICE_TIME_ANS;
                }
                break;
            }
        }
    }

    return LM_OK;
}

void lm_log(lm_frame_t *frame, uint8_t *msg, int len)
{
    uint8_t buf[16];
    char *fctrlbitstr;

    LMLOG_INFO("MSG: (%2s) %h", lm_mtype_str_abbr[frame->mhdr.bits.mtype], msg, len);

#if 1
    if (frame->mhdr.bits.major == LM_VERSION_MAJOR_R1) {
        LMLOG_INFO("LoRaWAN R1");
    } else {
        LMLOG_INFO("LoRaWAN version unknown");
    }

    LMLOG_INFO("%s", lm_mtype_str[frame->mhdr.bits.mtype]);
#endif

    lm_cpy(buf, frame->appeui, LM_JOIN_EUI_FIELD_SIZE);
    lm_cpy(buf+8, frame->deveui, LM_DEV_EUI_FIELD_SIZE);

    switch (frame->mhdr.bits.mtype) {
        case LM_MTYPE_JOIN_REQUEST:
            LMLOG_INFO("APPEUI: %h, DEVEUI: %h", buf, 8, buf + 8, 8);
            LMLOG_INFO("DEVNONCE: 0x%04X", frame->pld.jr.devnonce.data);
            break;
        case LM_MTYPE_JOIN_ACCEPT:
            LMLOG_INFO("APPEUI: %h, DEVEUI: %h", buf, 8, buf + 8, 8);
            LMLOG_INFO("APPNONCE: 0x%06X", frame->pld.ja.appnonce.data);
            if (frame->mote != NULL){
                LMLOG_INFO("DEVNONCE: 0x%04X", frame->mote->devnonce.data);
            }
            LMLOG_INFO("RX2DataRate: %d", frame->pld.ja.dlsettings.bits.rx2dr);
            LMLOG_INFO("RX1DRoffset: %d", frame->pld.ja.dlsettings.bits.rx1droft);
            LMLOG_INFO("NETID: 0x%06X", frame->pld.ja.netid.data);
            LMLOG_INFO("DEVADDR: %08X", frame->pld.ja.addr.data);
            LMLOG_INFO("NWKSKEY: %h", frame->pld.ja.nwkskey, 16);
            LMLOG_INFO("APPSKEY: %h", frame->pld.ja.appskey, 16);
            if (frame->pld.ja.cflist_len > 0) {
                int i, freq;
                LMLOG_INFO("CFList: %h", frame->pld.ja.cflist, frame->pld.ja.cflist_len);
                uint8_t *buf = frame->pld.ja.cflist;
                i = 0;
                while ((i+3) <= frame->pld.ja.cflist_len) {
                    freq = (buf[i+0]) | ((uint32_t)buf[i+1]<<8) | ((uint32_t)buf[i+2]<<16);
                    freq *= 100;
                    LMLOG_INFO("CHx: %d", freq);
                    i += 3;
                }
            }
            break;
        case LM_MTYPE_MSG_UP:
        case LM_MTYPE_MSG_DOWN:
        case LM_MTYPE_CMSG_UP:
        case LM_MTYPE_CMSG_DOWN:
            fctrlbitstr = "";
            if ((frame->mhdr.bits.mtype == LM_MTYPE_MSG_UP) || (frame->mhdr.bits.mtype == LM_MTYPE_CMSG_UP)) {
                if (frame->pld.mac.fhdr.fctrl.ul.classb) {
                    fctrlbitstr = ", CLASSB";
                }
            } else {
                if (frame->pld.mac.fhdr.fctrl.dl.fpending) {
                    fctrlbitstr = ", FPENDING";
                }
            }

            LMLOG_INFO("APPEUI: %h, DEVEUI: %h, DEVADDR: %08X, ADR: %d, ADRACKREQ: %d, ACK: %d, FCNT: %u [0x%08X], MIC: %h%s",
                    buf, 8,
                    buf+8, 8,
                    frame->pld.mac.fhdr.addr.data,
                    frame->pld.mac.fhdr.fctrl.ul.adr, frame->pld.mac.fhdr.fctrl.ul.adrackreq, frame->pld.mac.fhdr.fctrl.ul.ack,
                    frame->pld.mac.fhdr.fcnt, frame->pld.mac.fhdr.fcnt,
                    frame->mic.buf, 4,
                    fctrlbitstr);

            if ((frame->pld.mac.flen > 0) && (frame->pld.mac.fport > 0)) {
                LMLOG_INFO("PORT: %d, LEN: %d, DATA: %h %s",
                        frame->pld.mac.fport,
                        frame->pld.mac.flen,
                        frame->pld.mac.frmpld, frame->pld.mac.flen, frame->pld.mac.frmpld);
            } else if ((frame->pld.mac.flen > 0) && (frame->pld.mac.fport == 0)) {
                if (LM_OK != lm_log_maccmd(frame->mhdr.data, LM_MACCMD_PORT0, frame->pld.mac.frmpld, frame->pld.mac.flen)) {
                    LMLOG_INFO("MACCMD INVALID: %h (Port 0)", frame->pld.mac.frmpld, frame->pld.mac.flen);
                }
            } else {
                LMLOG_INFO("No Port and FRMPayload");
            }

            if (frame->pld.mac.fhdr.fctrl.ul.foptslen > 0) {
                if (LM_OK != lm_log_maccmd(frame->mhdr.data, LM_MACCMD_FOPTS, frame->pld.mac.fhdr.fopts, frame->pld.mac.fhdr.fctrl.ul.foptslen)) {
                    LMLOG_INFO("MACCMD INVALID: %h (FOpts)", frame->pld.mac.fhdr.fopts, frame->pld.mac.fhdr.fctrl.ul.foptslen);
                }
            }
            break;
        case LM_MTYPE_RFU:

            break;
        case LM_MTYPE_PROPRIETARY:

            break;
    }
}

void lm_log_all_node()
{
    struct list *head = &mote_list;
    struct listnode *node;
    lm_mote_t *mote;

    LMLOG_INFO("Mote List:");
    LMLOG_INFO("mode,joined,appeui,deveui,devaddr,appkey,nwkskey,appskey,ulsum,ullost,ufcnt,dfcnt");
    for (ALL_LIST_ELEMENTS_RO(head, node, mote)) {
        LMLOG_INFO("%s,%s,%h,%h,%08X,%h,%h,%h,%u,%u,%u,%u",
                mote->mode == ABP ? "ABP":"OTAA",
                mote->joined ? "YES":"NO",
                mote->appeui, 8,
                mote->deveui, 8,
                mote->addr.data,
                mote->appkey, 16,
                mote->nwkskey, 16,
                mote->appskey, 16,
                mote->ufsum,
                mote->uflost,
                mote->ufcnt,
                mote->dfcnt);
    }
}

const char *lm_log_mtype_str[] = {
    "JR",
    "JA",
    "UU",
    "UD",
    "CU",
    "CD",
    "RFU",
    "P",
};

void lm_log_frame(lm_frame_t *frame, void *pkt)
{
    uint8_t buf[16];
    uint8_t *maccmd = NULL;
    int maccmdlen = 0;
    uint8_t *macmsg;
    int macmsglen = 0;
    int port;
    lm_rxpkt_t *rxpkt = pkt;
    lm_txpkt_t *txpkt = pkt;

    lm_cpy(buf, frame->deveui, 8);
    lm_cpy(buf + 8, frame->appeui, 8);

    switch (frame->mhdr.bits.mtype) {
    case LM_MTYPE_JOIN_REQUEST:
        /*type,appeui,deveui,devnonce*/
        LMLOG_INFO("%T,%s,%d,%s,%h,%h,%04X",
                   lm_log_mtype_str[frame->mhdr.bits.mtype],
                   rxpkt->freq_hz,
                   lm_get_rf_name(rxpkt->modulation, rxpkt->datarate, rxpkt->bandwidth, 0),
                   buf,
                   8,
                   buf + 8,
                   8,
                   frame->pld.jr.devnonce.data);
        break;
    case LM_MTYPE_JOIN_ACCEPT:
        /*type,appeui,deveui,appnonce,cflist,nwkskey,appskey*/
        LMLOG_INFO("%T,%s,%d,%s,%h,%h,%08X,%06X,%06X,%d,%d,%d,%h,%h,%h",
                   lm_log_mtype_str[frame->mhdr.bits.mtype],
                   txpkt->freq_hz,
                   lm_get_rf_name(txpkt->modulation, txpkt->datarate, txpkt->bandwidth, txpkt->f_dev),
                   buf, 8,
                   buf + 8, 8,
                   frame->pld.ja.addr.data,
                   frame->pld.ja.appnonce.data,
                   frame->pld.ja.netid.data,
                   frame->pld.ja.dlsettings.bits.rx2dr,
                   frame->pld.ja.dlsettings.bits.rx1droft,
                   frame->pld.ja.rxdelay,
                   frame->pld.ja.cflist, frame->pld.ja.cflist_len,
                   frame->pld.ja.nwkskey, 16,
                   frame->pld.ja.appskey, 16
                  );
        break;
    case LM_MTYPE_MSG_UP:
    case LM_MTYPE_CMSG_UP:
        port = -1;
        if (frame->pld.mac.flen != 0) {
            port = frame->pld.mac.fport;
        }

        if (frame->pld.mac.fhdr.fctrl.ul.foptslen != 0) {
            maccmd = frame->pld.mac.fhdr.fopts;
            maccmdlen = frame->pld.mac.fhdr.fctrl.ul.foptslen;
        } else if ((frame->pld.mac.flen != 0) && (frame->pld.mac.fport == 0)) {
            maccmd = frame->pld.mac.frmpld;
            maccmdlen = frame->pld.mac.flen;
        } else {
            macmsg = frame->pld.mac.frmpld;
            maccmdlen = 0;
        }
        macmsg = frame->pld.mac.frmpld;
        macmsglen = frame->pld.mac.flen;

        /* type,appeui,deveui,devaddr,counter,port,msg,cmd */
        LMLOG_INFO("%T,%s,%d,%s,%.1f,%.1f,%h,%h,%08X,%d,%d,%h,%h",
                   lm_log_mtype_str[frame->mhdr.bits.mtype],
                   rxpkt->freq_hz,
                   lm_get_rf_name(rxpkt->modulation, rxpkt->datarate, rxpkt->bandwidth, 0),
                   rxpkt->rssi,
                   rxpkt->snr,
                   buf, 8,
                   buf + 8, 8,
                   frame->pld.mac.fhdr.addr.data,
                   frame->pld.mac.fhdr.fcnt,
                   port,
                   macmsg, macmsglen,
                   maccmd, maccmdlen);
        break;
    case LM_MTYPE_MSG_DOWN:
    case LM_MTYPE_CMSG_DOWN:
        port = -1;
        if (frame->pld.mac.flen != 0) {
            port = frame->pld.mac.fport;
        }

        if (frame->pld.mac.fhdr.fctrl.ul.foptslen != 0) {
            maccmd = frame->pld.mac.fhdr.fopts;
            maccmdlen = frame->pld.mac.fhdr.fctrl.ul.foptslen;
        } else if ((frame->pld.mac.flen != 0) && (frame->pld.mac.fport == 0)) {
            maccmd = frame->pld.mac.frmpld;
            maccmdlen = frame->pld.mac.flen;
        } else {
            macmsg = frame->pld.mac.frmpld;
            maccmdlen = 0;
        }
        macmsg = frame->pld.mac.frmpld;
        macmsglen = frame->pld.mac.flen;

        /* type,appeui,deveui,devaddr,counter,port,msg,cmd */
        LMLOG_INFO("%T,%s,%d,%s,%d,%h,%h,%08X,%d,%d,%h,%h",
                   lm_log_mtype_str[frame->mhdr.bits.mtype],
                   txpkt->freq_hz,
                   lm_get_rf_name(txpkt->modulation, txpkt->datarate, txpkt->bandwidth, txpkt->f_dev),
                   txpkt->rf_power,
                   buf, 8,
                   buf + 8, 8,
                   frame->pld.mac.fhdr.addr.data,
                   frame->pld.mac.fhdr.fcnt,
                   port,
                   macmsg, macmsglen,
                   maccmd, maccmdlen);

#if 0
        LMLOG_INFO("DEVADDR: %08X", frame->pld.mac.addr.data);
        LMLOG_INFO("ADR: %d, ADRACKREQ: %d, ACK %d", \
                   frame->pld.mac.fhdr.fctrl.ul.adr, frame->pld.mac.fhdr.fctrl.ul.adrackreq, frame->pld.mac.fhdr.fctrl.ul.ack);
        if ((frame->mhdr.bits.mtype == LM_MTYPE_MSG_UP) && (frame->mhdr.bits.mtype == LM_MTYPE_CMSG_UP)) {
            if (frame->pld.mac.fhdr.fctrl.ul.classb) {
                LMLOG_INFO("Class B");
            }
        } else {
            if (frame->pld.mac.fhdr.fctrl.dl.fpending) {
                LMLOG_INFO("FPENDING is on");
            }
        }
        LMLOG_INFO("FCNT: %d [0x%08X]", frame->pld.mac.fcnt, frame->pld.mac.fcnt);

        if ((frame->pld.mac.flen > 0) && (frame->pld.mac.fport > 0)) {
            LMLOG_INFO("PORT: %d", frame->pld.mac.fport);
            LMLOG_INFO("DATA: %H", frame->pld.mac.frmpld, frame->pld.mac.flen);
        } else {
            LMLOG_INFO("No Port and FRMPayload in message");
        }

        if (frame->pld.mac.fhdr.fctrl.ul.foptslen > 0) {
            LMLOG_INFO("FOPTS MACCMD");
            lm_log_maccmd(frame->mhdr.data, frame->pld.mac.fopts, frame->pld.mac.fhdr.fctrl.ul.foptslen);
        }

        if ((frame->pld.mac.flen > 0) && (frame->pld.mac.fport == 0)) {
            LMLOG_INFO("Port 0 MACCMD");
            lm_log_maccmd(frame->mhdr.data, frame->pld.mac.frmpld, frame->pld.mac.flen);
        }
#endif
        break;
    case LM_MTYPE_RFU:

        break;
    case LM_MTYPE_PROPRIETARY:

        break;
    }
}

/* Utilities */
uint8_t lm_get_sf(uint8_t sf)
{
    int i;
    for(i=7; i<=12; i++){
        if(sf == (1<<(i-6))){
            sf = i;
            break;
        }
    }
    return sf;
}

uint16_t lm_get_bw(uint8_t bw)
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

void lm_log_rxpkt(lm_rxpkt_t *rxpkt)
{
    if (rxpkt->status != STAT_CRC_OK) {
        return;
    }

    LMLOG_INFO("RX: %h", rxpkt->payload, rxpkt->size);

    if (rxpkt->modulation == MOD_LORA) {
        LMLOG_INFO("LORA,%08X(%u),%d,%d,SF%dBW%d,4/%d,%.1f,%.1f,%.1f,%.1f",
                        rxpkt->count_us,
                        rxpkt->count_us,
                        rxpkt->if_chain,
                        rxpkt->freq_hz,
                        lm_get_sf(rxpkt->datarate),
                        lm_get_bw(rxpkt->bandwidth),
                        rxpkt->coderate+4,
                        rxpkt->rssi,
                        rxpkt->snr,
                        rxpkt->snr_max,
                        rxpkt->snr_min
                 );
    } else if (rxpkt->modulation == MOD_FSK) {
        LMLOG_INFO("FSK,%08X(%u),%d,%d,%d,%d,%.1f",
                        rxpkt->count_us,
                        rxpkt->count_us,
                        rxpkt->if_chain,
                        rxpkt->freq_hz,
                        rxpkt->datarate,
                        rxpkt->bandwidth,
                        rxpkt->rssi
                 );
    }
}

void lm_log_txpkt(lm_txpkt_t *txpkt)
{
    LMLOG_INFO("TX: %h", txpkt->payload, txpkt->size);

    if (txpkt->modulation == MOD_LORA) {
        LMLOG_INFO("LORA,%d,%08X(%u),%d,%d,SF%dBW%d,4/%d,%d,%d,%d,%d,%d",
                        txpkt->tx_mode,
                        txpkt->count_us,
                        txpkt->count_us,
                        txpkt->rf_chain,
                        txpkt->freq_hz,
                        lm_get_sf(txpkt->datarate),
                        lm_get_bw(txpkt->bandwidth),
                        txpkt->coderate+4,
                        txpkt->rf_power,
                        txpkt->preamble,
                        txpkt->invert_pol,
                        txpkt->no_header,
                        txpkt->no_crc
                 );
    } else if (txpkt->modulation == MOD_FSK) {
        LMLOG_INFO("FSK,%d,%08X(%u),%d,%d,%d,%d,%d,%d,%d,%d",
                        txpkt->tx_mode,
                        txpkt->count_us,
                        txpkt->count_us,
                        txpkt->rf_chain,
                        txpkt->freq_hz,
                        txpkt->rf_power,
                        txpkt->preamble,
                        txpkt->datarate,
                        txpkt->f_dev,
                        txpkt->no_header,
                        txpkt->no_crc
                 );
    }
}
