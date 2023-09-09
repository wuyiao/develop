
#include <termios.h>
#include "debug.h"
#include "bluetooth.h"
#include "etu.h"
#include "gsm.h"
#include "command.h"

static const char *bt_gsm_state_str[] =
{
  "GSM_DEPENDUPON",
  "GSM_NWK",
  "GSM_CONF",
  "GSM_DELN",
  "GSM_RESET",
  "GSM_RESTART",
  "GCSM_QUERY",
};

static const char *bt_gsm_event_str[] =
{
  "GEV_UNFINISHED",
  "GEV_AKBDFAILED",
  "GEV_AKBDTIMEOUT",
  "GEV_NKDISFAILED",
  "GEV_PARAGETFAILED",
  "GEV_NONETWORKABLEDEV",
  "GEV_CMDINVALID",
  "GEV_OPBUSY",
  "GEV_PARAINVALID",
  "GCEV_NOEVT",
  "GCEV_INIT",
  "GCEV_RESET_END",
  "GCEV_NK_RECVED",
  "GCEV_AK_RECVED",
  "GCEV_NA_RECVED",
  "GCEV_PW_RECVED",
  "GCEV_MAC_RECVED",
  "GCEV_VER_RECVED",
  "GCEV_NN_RECVED",
};

int gsm_reset(struct bt_provisioner *bt)
{
    bt_reset(bt);

    return 0;
}

int gsm_reset_end(struct bt_provisioner *bt)
{
    bt_wait_respawn_start(bt);

    return GSM_DEPENDUPON;
}

int gsm_query(struct bt_provisioner *bt)
{
    bt_query_netkey(bt);

    return GCSM_QUERY;
}

int gsm_info_received(struct bt_provisioner *bt)
{
    int state = bt->state;

    switch (bt->event) {
        case GCEV_NOEVT:
        case GCEV_INIT:
            break;
        case GCEV_NK_RECVED:
            BT_GSM_QUERY_INFO(bt, OPCODE_QUERY_AK);
            break;
        case GCEV_AK_RECVED:
            BT_GSM_QUERY_INFO(bt, OPCODE_QUERY_NA);
            break;
        case GCEV_NA_RECVED:
            BT_GSM_QUERY_INFO(bt, OPCODE_QUERY_PW);
            break;
        case GCEV_PW_RECVED:
            BT_GSM_QUERY_INFO(bt, OPCODE_QUERY_MAC);
            break;
        case GCEV_MAC_RECVED:
            BT_GSM_QUERY_INFO(bt, OPCODE_QUERY_VER);
            break;
        case GCEV_VER_RECVED:
            /* judge need or not to config new NK and AK */
            if (0)
                state = GSM_CONF;
            else {
                bt_access_network(bt);
                state = GSM_NWK;
            }
            break;
    }

    return state;
}

int gsm_ak_bound_failed(struct bt_provisioner *bt)
{
    switch (bt->event) {
        case GEV_AKBDTIMEOUT:
            break;
        case GEV_AKBDFAILED:
            bt_prepare_delete_node(bt);
            break;
    }

    if (!bt_access_network_timer_start(bt))
        bt_access_network_defer_start(bt);

    return bt->state;
}

int gsm_nk_bound_failed(struct bt_provisioner *bt)
{
    switch (bt->event) {
        case GEV_AKBDTIMEOUT:
            break;
        case GEV_AKBDFAILED:
            break;
    }

    if (!bt_access_network_timer_start(bt))
        bt_access_network_defer_start(bt);

    return bt->state;
}

int gsm_na_info_received(struct bt_provisioner *bt)
{
    int state = bt->state;

    switch (bt->event) {
        case GCEV_NOEVT:
        case GCEV_INIT:
            break;
        case GCEV_NA_RECVED:
            DBG_INFO("delete node");
            bt_delete_node(bt);
            break;
    }

    if (!bt_access_network_timer_start(bt))
        bt_access_network_defer_start(bt);

    return state;
}

int gsm_nn_info_received(struct bt_provisioner *bt)
{
    int state = bt->state;

    switch (bt->event) {
        case GCEV_NOEVT:
        case GCEV_INIT:
            break;
        case GCEV_NN_RECVED:
            DBG_INFO("networking node received");
            bt_access_network(bt);
            break;
        case GEV_UNFINISHED:
        case GEV_AKBDFAILED:
        case GEV_AKBDTIMEOUT:
        case GEV_NKDISFAILED:
        case GEV_PARAGETFAILED:
        case GEV_NONETWORKABLEDEV:
        case GEV_CMDINVALID:
        case GEV_OPBUSY:
        case GEV_PARAINVALID:
            /* start timer for possible new node */
            bt_access_network_defer_start(bt);
            DBG_INFO("mesh networking finished");
            break;
    }

    return state;
}

int gsm_wait_nwk_finished(struct bt_provisioner *bt)
{
    if (!bt_access_network_timer_start(bt))
        bt_access_network_defer_start(bt);

    return bt->state;
}



/* Provisioner State Machine */
struct {
  int (*func) (struct bt_provisioner *);
  int next_state;
} GSM [BT_GSM_STATE_MAX][BT_GSM_EVENT_MAX] =
{
  {
    /* DependUpon: dummy state. */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_UNFINISHED         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_AKBDFAILED         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_AKBDTIMEOUT        */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_NKDISFAILED        */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_PARAGETFAILED      */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_NONETWORKABLEDEV   */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_CMDINVALID         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_OPBUSY             */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_PARAINVALID        */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NOEVT             */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_INIT              */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NK_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_AK_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NA_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_PW_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_MAC_RECVED        */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_VER_RECVED        */
  },
  {
    /* NWK: Networking state. */
    { gsm_wait_nwk_finished,   GSM_DEPENDUPON }, /* GEV_UNFINISHED         */
    { gsm_ak_bound_failed,     GSM_DEPENDUPON }, /* GEV_AKBDFAILED         */
    { gsm_ak_bound_failed,     GSM_DEPENDUPON }, /* GEV_AKBDTIMEOUT        */
    { gsm_nk_bound_failed,     GSM_DEPENDUPON }, /* GEV_NKDISFAILED        */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_PARAGETFAILED      */
    { gsm_nn_info_received,    GSM_DEPENDUPON }, /* GEV_NONETWORKABLEDEV   */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_CMDINVALID         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_OPBUSY             */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_PARAINVALID        */
    { gsm_nn_info_received,    GSM_DEPENDUPON }, /* GCEV_NOEVT             */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_INIT              */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_RESET_END         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NK_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_AK_RECVED         */
    { gsm_na_info_received,    GSM_DEPENDUPON }, /* GCEV_NA_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_PW_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_MAC_RECVED        */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_VER_RECVED        */
    { gsm_nn_info_received,    GSM_DEPENDUPON }, /* GCEV_NN_RECVED         */
  },
  {
    /* CONF: Config state. */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_UNFINISHED         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_AKBDFAILED         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_AKBDTIMEOUT        */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_NKDISFAILED        */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_PARAGETFAILED      */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_NONETWORKABLEDEV   */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_CMDINVALID         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_OPBUSY             */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_PARAINVALID        */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NOEVT             */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_INIT              */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NK_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_AK_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NA_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_PW_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_MAC_RECVED        */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_VER_RECVED        */
  },
  {
    /* DELN: Delete mesh node state. */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_UNFINISHED         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_AKBDFAILED         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_AKBDTIMEOUT        */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_NKDISFAILED        */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_PARAGETFAILED      */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_NONETWORKABLEDEV   */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_CMDINVALID         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_OPBUSY             */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_PARAINVALID        */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NOEVT             */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_INIT              */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NK_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_AK_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NA_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_PW_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_MAC_RECVED        */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_VER_RECVED        */
  },
  {
    /* RESET: BT reset state. */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_UNFINISHED         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_AKBDFAILED         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_AKBDTIMEOUT        */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_NKDISFAILED        */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_PARAGETFAILED      */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_NONETWORKABLEDEV   */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_CMDINVALID         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_OPBUSY             */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_PARAINVALID        */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NOEVT             */
    { gsm_reset,               GSM_RESET      }, /* GCEV_INIT              */
    { gsm_reset_end,           GSM_DEPENDUPON }, /* GCEV_RESET_END         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NK_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_AK_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NA_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_PW_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_MAC_RECVED        */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_VER_RECVED        */
  },
  {
    /* RESTART: BT restart state. */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_UNFINISHED         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_AKBDFAILED         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_AKBDTIMEOUT        */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_NKDISFAILED        */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_PARAGETFAILED      */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_NONETWORKABLEDEV   */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_CMDINVALID         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_OPBUSY             */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_PARAINVALID        */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NOEVT             */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_INIT              */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NK_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_AK_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NA_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_PW_RECVED         */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_MAC_RECVED        */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_VER_RECVED        */
  },
  {
    /* QUERY: BT info query state. */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_UNFINISHED         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_AKBDFAILED         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_AKBDTIMEOUT        */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_NKDISFAILED        */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_PARAGETFAILED      */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_NONETWORKABLEDEV   */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_CMDINVALID         */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_OPBUSY             */
    { NULL,                    GSM_DEPENDUPON }, /* GEV_PARAINVALID        */
    { NULL,                    GSM_DEPENDUPON }, /* GCEV_NOEVT             */
    { gsm_query,               GSM_DEPENDUPON }, /* GCEV_INIT              */
    { gsm_query,               GSM_DEPENDUPON }, /* GCEV_RESET_END         */
    { gsm_info_received,       GSM_DEPENDUPON }, /* GCEV_NK_RECVED         */
    { gsm_info_received,       GSM_DEPENDUPON }, /* GCEV_AK_RECVED         */
    { gsm_info_received,       GSM_DEPENDUPON }, /* GCEV_NA_RECVED         */
    { gsm_info_received,       GSM_DEPENDUPON }, /* GCEV_PW_RECVED         */
    { gsm_info_received,       GSM_DEPENDUPON }, /* GCEV_MAC_RECVED        */
    { gsm_info_received,       GSM_DEPENDUPON }, /* GCEV_VER_RECVED        */
  },
};

void bt_gsm_change_state(struct bt_provisioner *bt, int state)
{
    u_int8_t old_state;

    /* preserve old status */
    old_state = bt->state;

    /* change to new status */
    bt->state = state;

    bt->state_change++;

    if (DEBUG(gsm, GSM_EVENTS)) {
        DBG_INFO("GSM[%s:%s]: Action change state to (%s)", bt_gsm_state_str[old_state], bt_gsm_event_str[bt->event],
                bt_gsm_state_str[state]);
    }

    // switch (state) {
        // case GCSM_QUERY:
            // (*(GSM[bt->state][bt->event].func))(bt);
            // break;

    // }
}

int bt_gsm_event(struct bt_provisioner *bt)
{
    u_int8_t s, e;
    int next_state;

    s = bt->state;
    e = bt->event;

    if (DEBUG(gsm, GSM_EVENTS))
        DBG_INFO("GSM[%s:%s]: %s (%s)", bt_gsm_state_str[s], bt_gsm_event_str[e],
                " ", ecode_to_str(e));

    next_state = GSM[s][e].next_state;

    if (GSM[s][e].func != NULL) {
        int func_state = (*(GSM[s][e].func))(bt);

        if (GSM[s][e].next_state == GSM_DEPENDUPON) {
            next_state = func_state;
        }
        else if (func_state) {

            if (DEBUG(gsm, GSM_EVENTS))
                DBG_WARNING("GSM[%s:%s]: Attention: action tried to change next state to (%s)", bt_gsm_state_str[bt->state], bt_gsm_event_str[bt->event],
                        bt_gsm_state_str[func_state]);
        }
    }

    if (next_state != bt->state) {
        bt_gsm_change_state(bt, next_state);
    }

    return 0;
}


void bt_gsm_enter_state(struct bt_provisioner *bt, u_int8_t sm, u_int8_t ev)
{
    bt->state = sm;
    bt->event = ev;

    bt_gsm_event(bt);
}

void bt_gsm_init(void)
{
}


