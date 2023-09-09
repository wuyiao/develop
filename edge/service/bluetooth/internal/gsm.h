#include <sys/types.h>
/* provisioner state */
typedef enum {
    GSM_DEPENDUPON,
    GSM_NWK,
    GSM_CONF,
    GSM_DELN,
    GSM_RESET,
    GSM_RESTART,
    GSM_STATE_MAX
} BT_GSM;

#define GCSM_STATE_BASE GSM_STATE_MAX
typedef enum {
    GCSM_QUERY = GCSM_STATE_BASE,
    GCSM_STATE_MAX
} BT_GCSM;

/* provisioner event */
typedef enum {
    GEV_UNFINISHED,
    GEV_AKBDFAILED,
    GEV_AKBDTIMEOUT,
    GEV_NKDISFAILED,
    GEV_PARAGETFAILED,
    GEV_NONETWORKABLEDEV,
    GEV_CMDINVALID,
    GEV_OPBUSY,
    GEV_PARAINVALID,
    GEV_EVENT_MAX
} BT_GEV;

#define GCEV_EVENT_BASE GEV_EVENT_MAX
/* we need integrate into main sm, so */
/* provisioner custom event */
typedef enum {
    GCEV_NOEVT = GCEV_EVENT_BASE,
    GCEV_INIT,
    GCEV_RESET_END,
    GCEV_NK_RECVED,
    GCEV_AK_RECVED,
    GCEV_NA_RECVED,
    GCEV_PW_RECVED,
    GCEV_MAC_RECVED,
    GCEV_VER_RECVED,
    GCEV_NN_RECVED,
    GCEV_EVENT_MAX
} BT_GCEV;

#define BT_GSM_STATE_MAX GCSM_STATE_MAX
#define BT_GSM_EVENT_MAX GCEV_EVENT_MAX

#define BT_GSM_EVENT_SCHEDULE(bt, e) \
    bt_gsm_enter_state(bt, (bt)->state, e)

#define BT_GSM_QUERY_INFO(bt, op) \
    bt_query_commander(bt, op)

struct bt_provisioner;
int bt_gsm_event(struct bt_provisioner *bt);
void bt_gsm_enter_state(struct bt_provisioner *bt, u_int8_t sm, u_int8_t ev);
