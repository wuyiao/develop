/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
	#define _XOPEN_SOURCE 600
#else
	#define _XOPEN_SOURCE 500
#endif

#include <signal.h>		/* sigaction */
#include <errno.h>		/* error messages */

#include <sys/socket.h> /* socket specific definitions */
#include <netinet/in.h> /* INET constants and stuff */
#include <arpa/inet.h>  /* IP address conversion stuff */
#include <netdb.h>		/* gai_strerror */
#include <poll.h>

#include <edge.h>
#include <adapter.h>
#include <edev.h>
#include <edrv.h>
#include <ubuf.h>
#include <radio.h>
#include <config.h>
#include <libubox/ulog.h>
#include <libubox/runqueue.h>
#include <thread.h>
#include <softbus.h>
#include <lora.h>
#include <jhash.h>
#include <hlist.h>
#include <concentrator.h>
#include <utilities.h>
#include <cbor.h>
#include <debug.h>
#include <lm.h>
#include <lm-log.h>
#include <lm-macro.h>

/* --- PRIVATE CONSTANTS ---------------------------------------------------- */
#define LORA_DRV_NAME    "sx127x"

/* values available for the 'modulation' parameters */
/* NOTE: arbitrary values */

enum {
    ON_TX_DONE,
    ON_TX_TIMEOUT,
    ON_RX_DONE,
    ON_RX_TIMEOUT,
    ON_RX_ERROR,
    __STATE_MAX
};

struct event_desc {
    unsigned number;
    char name[32];
};

struct lora_node {
    struct hlist_node hnode;
    uint32_t uid[3];
    int status;
    int old_status;
    /*  */
    int dying_score;
    int received_packets;
    int pps;
    int keepalive;
    int print_limit;
};

uint8_t nwkskey[16] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};
uint8_t appskey[16] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};
uint8_t appkey[16] = {0x00, 0x95, 0x69, 0x00, 0x00, 0x00, 0x19, 0xCC, 0x00, 0x95, 0x69, 0x00, 0x00, 0x00, 0x19, 0xCC};

static struct lora_conf *conf;
static struct edge_driver *drv;
static RadioEvents_t radio_events;
static struct Radio_s *radio_ops;
static struct uloop_timeout lora_keepalive_timeout;

static struct ubuf *rxub;
static struct ubuf *txub;
static struct list event_list;
static struct list pending_list;
static struct hlist_head lora_nodes[BUCKETSIZE];

static struct softbus_peer *peer;
static struct lora_context lora_ctx;

struct event_desc radio_event_desc[] = {
    DESC(ON_TX_DONE),
    DESC(ON_TX_TIMEOUT),
    DESC(ON_RX_DONE),
    DESC(ON_RX_TIMEOUT),
    DESC(ON_RX_ERROR),
};


void lora_run(void);


int lora_node_add(struct lora_node *node)
{
    int index;

    index = jhash(node->uid, 8, 0) & BUCKETMASK;
    INIT_HLIST_NODE(&node->hnode);
    hlist_add_head(&node->hnode, &lora_nodes[index]);

    return 0;
}

static void send_ping_packet(void)
{
    ubuf_putc(txub, 'p');
    ubuf_putc(txub, 'i');
    ubuf_putc(txub, 'n');
    ubuf_putc(txub, 'g');

    radio_ops->Send(ubuf_data_pointer(txub), ubuf_used_len(txub));
}

static void lora_event_send(int event, void *data)
{
    lora_ctx.event = event;
    lora_ctx.data = data;
    thread_schedule(&edge_ctx.s, &lora_ctx.thread);
}

static void on_tx_done( void )
{
    radio_ops->Sleep();
    /* reset ubuf for next time */
    ubuf_reset(txub);
    lora_event_send(ON_TX_DONE, NULL);
    ULOG_DEBUG("ON TX DONE\n");
}

static void on_tx_timeout( void )
{
    radio_ops->Sleep();
    lora_event_send(ON_TX_TIMEOUT, NULL);
    ULOG_DEBUG("ON TX TIMEOUT\n");
}

static void on_rx_timeout( void )
{
    radio_ops->Sleep();
    ubuf_reset(txub);
    ubuf_reset(rxub);
    lora_event_send(ON_RX_TIMEOUT, NULL);
    ULOG_DEBUG("ON RX TIMEOUT\n");
}

static void on_rx_error( void )
{
    radio_ops->Sleep();
    lora_event_send(ON_RX_ERROR, NULL);
    ULOG_DEBUG("ON RX ERROR\n");
}

/* in theory,  */
static void on_rx_done( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    struct listnode *node;
    struct ubuf *ub = NULL;
    struct list *head = &pending_list;

    radio_ops->Sleep();

    ubuf_reset(rxub);
    if (size <= ubuf_capacity(rxub))
        memcpy(ubuf_put(rxub, size), payload, size);
    else
        ULOG_WARN("payload too large\n");

    lora_event_send(ON_RX_DONE, (void *)(uint32_t)size);
    ULOG_DEBUG("ON RX DONE\n");

    // uint32_t packet_toa = radio_ops->time_on_air(
            // lora_radio_test_paras.modem,
            // lora_radio_test_paras.bw,
            // lora_radio_test_paras.sf,
            // lora_radio_test_paras.cr,
            // LORA_PREAMBLE_LENGTH,
            // LORA_FIX_LENGTH_PAYLOAD_ON_DISABLE,
            // 32,
            // true);

    //send_ping_packet(LORA_SLAVER_DEVADDR, LORA_MASTER_DEVADDR, 32);
    //usleep(1000);

    //radio_ops->rx(0);
}


static void lora_keepalive_timeout_handler(struct uloop_timeout *timeout)
{
    //uloop_timeout_set(&lora_running_timeout, 5 * 1000);
    send_ping_packet();
}

static int _pow(int b, int ex)
{
    if (ex == 0) return 1;
    int res = b;
    while (--ex > 0) res *= b;
    return res;
}

static void _cbor_nested_describe(cbor_item_t *item, int indent) {
    //setlocale(LC_ALL, "");
    switch (cbor_typeof(item)) {
        case CBOR_TYPE_UINT:
            {
                ULOG_DEBUG("%*s[CBOR_TYPE_UINT] ", indent, " ");
                ULOG_DEBUG("Width: %dB, Value: %" PRIu64 "\n", _pow(2, cbor_int_get_width(item)), cbor_get_int(item));
                break;
            };
        case CBOR_TYPE_NEGINT:
            {
                ULOG_DEBUG("%*s[CBOR_TYPE_NEGINT] ", indent, " ");
                ULOG_DEBUG("Width: %dB, Value: -%" PRIu64 " -1\n", _pow(2, cbor_int_get_width(item)), cbor_get_int(item));
                break;
            };
        case CBOR_TYPE_BYTESTRING:
            {
                ULOG_DEBUG("%*s[CBOR_TYPE_BYTESTRING] ", indent, " ");
                if (cbor_bytestring_is_indefinite(item)) {
                    ULOG_DEBUG("Indefinite, with %zu chunks:\n",
                            cbor_bytestring_chunk_count(item));
                    for (size_t i = 0; i < cbor_bytestring_chunk_count(item); i++)
                        _cbor_nested_describe(cbor_bytestring_chunks_handle(item)[i], indent + 4);
                } else {
                    ULOG_DEBUG("Definite, length %zuB\n", cbor_bytestring_length(item));
                }
                break;
            };
        case CBOR_TYPE_STRING:
            {
                ULOG_DEBUG("%*s[CBOR_TYPE_STRING] ", indent, " ");
                if (cbor_string_is_indefinite(item)) {
                    ULOG_DEBUG("Indefinite, with %zu chunks:\n",
                            cbor_string_chunk_count(item));
                    for (size_t i = 0; i < cbor_string_chunk_count(item); i++)
                        _cbor_nested_describe(cbor_string_chunks_handle(item)[i], indent + 4);
                } else {
                    ULOG_DEBUG("Definite, length %zuB, %zu codepoints\n",
                            cbor_string_length(item), cbor_string_codepoint_count(item));
                    /* Careful - this doesn't support multibyte characters! */
                    /* Printing those is out of the scope of this demo :) */
                    /* libICU is your friend */
                    ULOG_DEBUG("%*s", indent + 4, " ");
                    /* XXX: no null at the end -> confused vprintf */
                    //fwrite(cbor_string_handle(item), (int)cbor_string_length(item), 1, out);
                    ULOG_DEBUG("\n");
                }
                break;
            };
        case CBOR_TYPE_ARRAY:
            {
                ULOG_DEBUG("%*s[CBOR_TYPE_ARRAY] ", indent, " ");
                if (cbor_array_is_definite(item)) {
                    ULOG_DEBUG("Definite, size: %zu\n", cbor_array_size(item));
                } else {
                    ULOG_DEBUG("Indefinite, size:  %zu\n", cbor_array_size(item));
                }

                for (size_t i = 0; i < cbor_array_size(item); i++)
                    _cbor_nested_describe(cbor_array_handle(item)[i], indent + 4);
                break;
            };
        case CBOR_TYPE_MAP:
            {
                ULOG_DEBUG("%*s[CBOR_TYPE_MAP] ", indent, " ");
                if (cbor_map_is_definite(item)) {
                    ULOG_DEBUG("Definite, size: %zu\n", cbor_map_size(item));
                } else {
                    ULOG_DEBUG("Indefinite, size:  %zu\n", cbor_map_size(item));
                }

                for (size_t i = 0; i < cbor_map_size(item); i++) {
                    _cbor_nested_describe(cbor_map_handle(item)[i].key, indent + 4);
                    _cbor_nested_describe(cbor_map_handle(item)[i].value, indent + 4);
                }
                break;
            };
        case CBOR_TYPE_TAG:
            {
                ULOG_DEBUG("%*s[CBOR_TYPE_TAG] ", indent, " ");
                ULOG_DEBUG("Value: %" PRIu64 "\n", cbor_tag_value(item));
                _cbor_nested_describe(cbor_move(cbor_tag_item(item)), indent + 4);
                break;
            };
        case CBOR_TYPE_FLOAT_CTRL:
            {
                ULOG_DEBUG("%*s[CBOR_TYPE_FLOAT_CTRL] ", indent, " ");
                if (cbor_float_ctrl_is_ctrl(item)) {
                    if (cbor_is_bool(item))
                        ULOG_DEBUG("Bool: %s\n", cbor_get_bool(item) ? "true" : "false");
                    else if (cbor_is_undef(item))
                        ULOG_DEBUG("Undefined\n");
                    else if (cbor_is_null(item))
                        ULOG_DEBUG("Null\n");
                    else
                        ULOG_DEBUG("Simple value %d\n", cbor_ctrl_value(item));
                } else {
                    ULOG_DEBUG("Width: %dB, value: %lf\n", _pow(2, cbor_float_get_width(item)), cbor_float_get_float(item));
                }
                break;
            };
    }
}

void cbor_ulog_describe(cbor_item_t *item) {
    _cbor_nested_describe(item, 0);
}

static void lora_events_process(struct thread *t)
{
    struct lora_context *ctx = container_of(t, struct lora_context, thread);

    switch (ctx->event) {
        case ON_TX_DONE:
            /* ready to rx */
            radio_ops->Rx(3000);
            break;
        case ON_TX_TIMEOUT:
            /* reset ? TBD */

            send_ping_packet();
            radio_ops->Rx(0);
            break;
        case ON_RX_DONE:
            {
                int ret;
                lm_frame_t frame;

                //hex_dump(ubuf_data_pointer(rxub), ubuf_used_len(rxub), 32, "RX");
                ULOG_DEBUG("%s\n", hex_dump_to_buf(ubuf_data_pointer(rxub), ubuf_used_len(rxub), 32, "RX"));
                ret = lm_parse(&frame, ubuf_data_pointer(rxub), ubuf_used_len(rxub));
                if (ret == LM_OK) {
                    lm_log(&frame, ubuf_data_pointer(rxub), ubuf_used_len(rxub));
                } else {
                    ULOG_ERR("lm parse error (%d)\n", ret);
                    ULOG_INFO("(service) Reset radio\n");
                    radio_ops->Reset(lora_run);
                    break;
                }

                struct cbor_load_result result;
                cbor_item_t* item = cbor_load(frame.pld.mac.frmpld, frame.pld.mac.flen, &result);

                if (result.error.code != CBOR_ERR_NONE) {
                    ULOG_ERR("There was an error while reading the input near byte %zu (read %zu "
                            "bytes in total): ",
                            result.error.position,
                            result.read);
                    switch (result.error.code) {
                        case CBOR_ERR_MALFORMATED:
                            {
                                ULOG_ERR("Malformed data\n");
                                break;
                            }
                        case CBOR_ERR_MEMERROR:
                            {
                                ULOG_ERR("Memory error -- perhaps the input is too large?\n");
                                break;
                            }
                        case CBOR_ERR_NODATA:
                            {
                                ULOG_ERR("The input is empty\n");
                                break;
                            }
                        case CBOR_ERR_NOTENOUGHDATA:
                            {
                                ULOG_ERR("Data seem to be missing -- is the input complete?\n");
                                break;
                            }
                        case CBOR_ERR_SYNTAXERROR:
                            {
                                ULOG_ERR("Syntactically malformed data -- see "
                                        "http://tools.ietf.org/html/rfc7049\n");
                                break;
                            }
                        case CBOR_ERR_NONE:
                            {
                                // GCC's cheap dataflow analysis gag
                                break;
                            }
                    }
                    break;
                }

                cbor_ulog_describe(item);
                //fflush(stdout);
                /* Deallocate the result */
                cbor_decref(&item);

                //softbus_push_data("sx1278", ubuf_data_pointer(rxub), ubuf_used_len(rxub) - 1);
                softbus_push_data("sx1278", frame.pld.mac.frmpld, frame.pld.mac.flen);
                /* send to peer */
                radio_ops->Rx(0);
            }
            break;
        case ON_RX_TIMEOUT:
            //send_ping_packet();
            radio_ops->Rx(0);
            break;
        case ON_RX_ERROR:
            //send_ping_packet();
            radio_ops->Rx(0);
            break;
    }

    thread_complete(t);
}

void lora_run(void)
{
    /* radio device init */
    if (radio_ops->Probe()) {
        ULOG_INFO("SX1278 lora radio detected\n");
    } else {
        ULOG_INFO("(service) Can not probe lora radio");
        return;
    }

    radio_ops->Init(&radio_events);
    radio_ops->SetChannel(conf->freq);
    radio_ops->SetPublicNetwork(false);

    if (conf->modem == MODEM_LORA) {

        radio_ops->SetTxConfig(conf->modem, conf->power, 0, conf->bw,
                conf->sf, conf->cr, conf->prlen, LORA_FIX_LENGTH_PAYLOAD_ON_DISABLE,
                true, 0, 0, conf->invertiq, 1000);

        radio_ops->SetRxConfig(conf->modem, conf->bw, conf->sf,
                conf->cr, 0, conf->prlen, LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON_DISABLE,
                0, true, 0, 0, conf->invertiq, true);

    } else {

        radio_ops->SetTxConfig(conf->modem, conf->power, FSK_FDEV, 0,
                FSK_DATARATE, 0,
                FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                true, 0, 0, 0, 3000);

        radio_ops->SetRxConfig(conf->modem, FSK_BANDWIDTH, FSK_DATARATE,
                0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                0, 0, false, true);
    }

    radio_ops->SetMaxPayloadLength(conf->modem, 255);
    radio_ops->Rx(0);

    ULOG_INFO("(service) LoRa service start\n");
}

int lora_sink(struct softbus_peer *peer, struct softbus_source *source, struct ubuf *ub)
{
    ULOG_DEBUG("lora sink");
    return 0;
}

int lora_service(void)
{
    drv = edrv_get_drv(LORA_DRV_NAME);
    conf = drv->conf;
    radio_ops = drv->ops;

    txub = ubuf_new(256);
    rxub = ubuf_new(1024);

    /* radio preinit */
    radio_events.TxDone = on_tx_done;
    radio_events.RxDone = on_rx_done;
    radio_events.TxTimeout = on_tx_timeout;
    radio_events.RxTimeout = on_rx_timeout;
    radio_events.RxError = on_rx_error;

    ULOG_DEBUG("Frequency      {freq}:   %d\n", conf->freq);
    ULOG_DEBUG("TxPower        {power}:  %d\n", conf->power);
    ULOG_DEBUG("CR             {cr}:     %d\n", conf->cr);
    ULOG_DEBUG("SF             {sf}:     %d\n", conf->sf);
    ULOG_DEBUG("BW             {bw}:     %d\n", conf->bw);
    ULOG_DEBUG("Public Network {public}: %d\n", false);
    ULOG_DEBUG("IQ Inversion   {iq}:     %d\n", conf->invertiq);

    /* LoRaMAC init */
    lm_init(CN470);
    lm_key_grp_t kgrp = {0};
    kgrp.nwkskey = nwkskey;
    kgrp.flag.bits.nwkskey = 1;
    kgrp.appskey = appskey;
    kgrp.flag.bits.appskey = 1;
    kgrp.appkey = appkey;
    kgrp.flag.bits.appkey = 1;
    lm_set_key(&kgrp);

    /* register softbus */
    peer = softbus_peer_new("lora");
    softbus_peer_add_source(peer, "sx1278", 0);
    softbus_peer_add_sink(peer, "ipc", 0, lora_sink);
    softbus_peer_register(peer);

    /* thread init */
    memset(&lora_ctx, 0, sizeof(lora_ctx));
    lora_ctx.thread.thread = lora_events_process;
    thread_init(&lora_ctx.thread);

    radio_ops->Reset(lora_run);
    //memset(&lora_keepalive_timeout, 0, sizeof(lora_keepalive_timeout));
    //lora_keepalive_timeout.cb = lora_keepalive_timeout_handler;
    //uloop_timeout_set(&lora_keepalive_timeout, 2 * 1000);

    return 0;
}
