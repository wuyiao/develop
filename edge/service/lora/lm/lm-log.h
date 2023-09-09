#ifndef __LM_LOG_H
#define __LM_LOG_H

#include "lm.h"

typedef enum {
    LM_MACCMD_PORT0,
    LM_MACCMD_FOPTS,
} lm_maccmd_type_t;

void lm_log_all_node();
int lm_log_maccmd(uint8_t mac_header, lm_maccmd_type_t type, uint8_t *opts, int len);
void lm_log(lm_frame_t *frame, uint8_t *msg, int len);
void lm_log_rxpkt(lm_rxpkt_t *rxpkt);
void lm_log_txpkt(lm_txpkt_t *txpkt);
void lm_log_frame(lm_frame_t *frame, void *pkt);

#endif
