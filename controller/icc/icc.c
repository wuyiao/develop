#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/uio.h>
#include <json-c/json.h>
#include <libubox/ulog.h>
#include <libubox/uloop.h>
#include <libubox/utils.h>
#include <libubox/ustream.h>

#include <cbor.h>
#include <tlv_box.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


#include "icc.h"
#include "net.h"
#include "utils.h"
#include "mqtt.h"


#define FIFO_NAME "/tmp/mqtt_data"


#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

static int pipe_fd;

static struct uloop_timeout benchmark_timeout;
int received_nbytes = 0;

static struct ipc_context ipc_ctx;
static struct ipc_context *ipcs;
static struct ustream *us;
static struct ustream_fd sfd;

static double transform(double value)
{
    value=(int)(1000*value+0.5);
    value=value/1000;
    return value;
}


static void benchmark(struct uloop_timeout *timeout)
{
    printf("speed: %d | %d kbps\n", received_nbytes,  received_nbytes / 1024);
    received_nbytes = 0;
    uloop_timeout_set(&benchmark_timeout, 1000);
}

static int ipc_msg_parse(char *str, int len)
{
    int raw_len = 1024;
    char raw_buf[1024] = {0};
    char buf[1024] = {0};
    char printbuf[1024] = {0};
    tlv_box_t *box;
    tlv_box_t *boxes = tlv_box_parse(str, len);
    json_object *obj;
    size_t length = 0;
    char *json_data;

    ULOG_INFO("ipc msg parse received %d bytes\n", tlv_box_get_size(boxes));

    switch (str[0]) {
        case 0xe0:

            if (tlv_box_get_object(boxes, 0xe0, &box) != 0) {
                ULOG_ERR("tlv_box_get_object failed");
                goto out;
            }

            tlv_box_get_bytes(box, 0xc0, raw_buf, &raw_len);
            short *value = raw_buf;
            ULOG_INFO("%d: %d %d %d\n", raw_len, value[0], value[1], value[2]);

            obj = json_object_new_object();
            json_object_object_add(obj, "PM1.0", json_object_new_int((int)value[0]));
            json_object_object_add(obj, "PM2.5", json_object_new_int((int)value[1]));
            json_object_object_add(obj, "PM10", json_object_new_int((int)value[2]));
            json_data = json_object_to_json_string_length(obj, 0, &length);
            if (json_data)
                memcpy(buf, json_data, length);

            buf[length] = '\r';
            buf[length + 1] = '\n';

            send_data_to_remote_platform(buf, length + 2);

            json_object_put(obj);
            tlv_box_destroy(box);
            break;
        case 0xe1:
            {
            if (tlv_box_get_object(boxes, 0xe1, &box) != 0) {
                ULOG_ERR("tlv_box_get_object failed");
                goto out;
            }

            tlv_box_get_bytes(box, 0xc0, raw_buf, &raw_len);
            char mac_str[32] = {0};
            sprintf(mac_str, MACSTR, MAC2STR(raw_buf));
            printf("mac len: %s %d\n", mac_str, raw_len);
            raw_len = 1024;
            tlv_box_get_bytes(box, 0xc1, raw_buf, &raw_len);
            char *value = raw_buf;
            printf("value len: %d\n", raw_len);

            obj = json_object_new_object();
            json_object_object_add(obj, "mac", json_object_new_string(mac_str));
            json_object_object_add(obj, "temperature", json_object_new_int((int)value[0]));
            json_object_object_add(obj, "humility", json_object_new_int((int)value[1]));
            json_object_object_add(obj, "noise", json_object_new_int((int)value[2]));
            json_object_object_add(obj, "fire", json_object_new_int((int)value[3]));

            json_object_object_add(obj, "a_phase_i", json_object_new_double(fix5_to_float((value[5] << 8) | value[4])));
            json_object_object_add(obj, "b_phase_i", json_object_new_double(fix5_to_float((value[7] << 8) | value[6])));
            json_object_object_add(obj, "c_phase_i", json_object_new_double(fix5_to_float((value[9] << 8) | value[8])));
            json_object_object_add(obj, "a_phase_v", json_object_new_double(fix5_to_float((value[11] << 8) | value[10])));
            json_object_object_add(obj, "b_phase_v", json_object_new_double(fix5_to_float((value[13] << 8) | value[12])));
            json_object_object_add(obj, "c_phase_v", json_object_new_double(fix5_to_float((value[15] << 8) | value[14])));

            union {
                float float_variable;
                char bytes_array[4];
            } my_union;

            my_union.bytes_array[0] = value[16];
            my_union.bytes_array[1] = value[17];
            my_union.bytes_array[2] = value[18];
            my_union.bytes_array[3] = value[19];

            json_object_object_add(obj, "three_phase_total_apparent_watt", json_object_new_double(my_union.float_variable));

            json_data = json_object_to_json_string_length(obj, 0, &length);
            if (json_data)
                memcpy(buf, json_data, length);

            buf[length] = '\r';
            buf[length + 1] = '\n';

            send_data_to_remote_platform(buf, length + 2);
            json_object_put(obj);
            tlv_box_destroy(box);
            }
            break;
        case 0xe2:
            if (tlv_box_get_object(boxes, 0xe2, &box) != 0) {
                ULOG_ERR("tlv_box_get_object failed");
                goto out;
            }

            tlv_box_get_bytes(box, 0xc0, raw_buf, &raw_len);
            int i, print = 0;
            for (i = 0; i < raw_len; i++)
                print += sprintf(printbuf + print, "%x ", raw_buf[i]);
            ULOG_INFO("[%d]: %s\n", raw_len, printbuf);


            short *frame = raw_buf;
            //unsigned char *byte = raw_buf;

            short dis1 = frame[2];
            //short dis1 = byte[4] + byte[5] * 256;
            //ULOG_INFO("dis1: %x, %x, %d", byte[4], byte[5], dis1);
            short dis2 = frame[3];
            //short dis2 = byte[6] + byte[7] * 256;
            short dis3 = frame[4];
            //short dis3 = byte[8] + byte[9] * 256;

            short temp = frame[5] * 0.02 - 273.15;
            //short temp = (byte[10] + byte[11] * 256) * 0.02 - 273.15;
            unsigned char bpm = raw_buf[12];

            ULOG_INFO("d1/d2/d3: %d,%d,%d temp: %d, bpm: %d\n", dis1, dis2, dis3, temp, bpm);
            obj = json_object_new_object();
            json_object *array_obj = json_object_new_array();

            json_object_array_add(array_obj, json_object_new_int((int)dis1));
            json_object_array_add(array_obj, json_object_new_int((int)dis2));
            json_object_array_add(array_obj, json_object_new_int((int)dis3));
            json_object_object_add(obj, "distance", array_obj);
            json_object_object_add(obj, "temperature", json_object_new_int((int)temp));
            json_object_object_add(obj, "BPM", json_object_new_int((int)bpm));
            json_data = json_object_to_json_string_length(obj, 0, &length);
            if (json_data)
                memcpy(buf, json_data, length);

            buf[length] = '\r';
            buf[length + 1] = '\n';

            send_data_to_remote_platform(buf, length + 2);

            json_object_put(obj);
            tlv_box_destroy(box);
            break;
    }

out:
    tlv_box_destroy(boxes);

    return 0;
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
                //ULOG_DEBUG("%*s[CBOR_TYPE_UINT] ", indent, " ");
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

char *get_pl_serial_no(int pl)
{
    static char *serial_no[] = {
        "",
        "778BP2-1",
        "779BP2-1",
        "781BP2-1",
        "782BP2-1",
        "",
        "",
        "811BP2-1",
        "812BP2-1",
        "705SPI2-1",
        "724SPI2-1"
    };

    if (0 <= pl && pl <= 10)
        return serial_no[pl];
    else
        return NULL;
}

void msg_parse(char *msg, int nbytes)
{
    int length;
    char *json_data;
    char buf[32] = {0};
    json_object *obj;
    struct cbor_load_result result;
    cbor_item_t* item = cbor_load(msg, nbytes, &result);

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
        return;
    }


    switch (cbor_typeof(item)) {
        case CBOR_TYPE_ARRAY:
            {
                cbor_item_t *elem;

                int product_line = cbor_get_int(cbor_array_handle(item)[0]);
                switch (product_line) {
                    case 0:
                        {
                            float c1 = cbor_float_get_float(cbor_array_handle(item)[1]);
                            float c2 = cbor_float_get_float(cbor_array_handle(item)[2]);
                            float c3 = cbor_float_get_float(cbor_array_handle(item)[3]);
                            float v1 = cbor_float_get_float(cbor_array_handle(item)[4]);
                            float v2 = cbor_float_get_float(cbor_array_handle(item)[5]);
                            float v3 = cbor_float_get_float(cbor_array_handle(item)[6]);
                            obj = json_object_new_object();
                            json_object_object_add(obj, "fpc", json_object_new_double(c1));
                            json_object_object_add(obj, "spc", json_object_new_double(c2));
                            json_object_object_add(obj, "tpc", json_object_new_double(c3));
                            json_object_object_add(obj, "fpv", json_object_new_double(v1));
                            json_object_object_add(obj, "spv", json_object_new_double(v2));
                            json_object_object_add(obj, "tpv", json_object_new_double(v3));
                            json_data = json_object_to_json_string_length(obj, 0, &length);

                            ULOG_DEBUG("[%d]: %s\n", product_line, json_data);
                            sprintf(buf, "power/huamaiElectric-01");
                            mqtt_publish(buf, json_data, length, 2);
                        }
                        break;
                    case 1:
                    case 2:
                    case 3:
                    case 4:
                    case 7:
                    case 8:
                        {
                            int accumulative_total = cbor_get_int(cbor_array_handle(item)[1]);
                            int len_ng = cbor_get_int(cbor_array_handle(item)[2]);
                            int vision_ng = cbor_get_int(cbor_array_handle(item)[3]);
                            int cavity_ng = cbor_get_int(cbor_array_handle(item)[4]);
                            int status = cbor_get_bool(cbor_array_handle(item)[5]);

                            obj = json_object_new_object();
                            json_object_object_add(obj, "qualified_num", json_object_new_int(accumulative_total - len_ng - vision_ng - cavity_ng));
                            json_object_object_add(obj, "unqualified_num", json_object_new_int(len_ng + vision_ng + cavity_ng));
                            json_object_object_add(obj, "status", json_object_new_boolean(status));
                            json_data = json_object_to_json_string_length(obj, 0, &length);

                            ULOG_DEBUG("[%d]: %s\n", product_line, json_data);
                            sprintf(buf, "PL/%s", get_pl_serial_no(product_line));
                            mqtt_publish(buf, json_data, length, 2);
                        }
                        break;
                    case 5:
                    case 6:
                        break;
                    case 9:
                        {
                            int accumulative_total = cbor_get_int(cbor_array_handle(item)[1]);
                            int qualified = cbor_get_int(cbor_array_handle(item)[2]);
                            int ng = cbor_get_int(cbor_array_handle(item)[3]);
                            int len_ng = cbor_get_int(cbor_array_handle(item)[4]);
                            int diameter_ng = cbor_get_int(cbor_array_handle(item)[5]);
                            int eddy_ng = cbor_get_int(cbor_array_handle(item)[6]);
                            int status = cbor_get_bool(cbor_array_handle(item)[7]);

                            obj = json_object_new_object();
                            json_object_object_add(obj, "qualified_num", json_object_new_int(qualified));
                            json_object_object_add(obj, "unqualified_num", json_object_new_int(ng));
                            json_object_object_add(obj, "status", json_object_new_boolean(status));
                            json_data = json_object_to_json_string_length(obj, 0, &length);

                            ULOG_DEBUG("[%d]: %s\n", product_line, json_data);
                            sprintf(buf, "PL/%s", get_pl_serial_no(product_line));
                            mqtt_publish(buf, json_data, length, 2);
                        }
                        break;
                    case 10:
                        {
                            int qualified = cbor_get_int(cbor_array_handle(item)[1]);
                            int ng = cbor_get_int(cbor_array_handle(item)[2]);
                            int len_ng = cbor_get_int(cbor_array_handle(item)[3]);
                            int diameter_ng = cbor_get_int(cbor_array_handle(item)[4]);
                            int eddy_ng = cbor_get_int(cbor_array_handle(item)[5]);
                            int status = cbor_get_bool(cbor_array_handle(item)[6]);

                            obj = json_object_new_object();
                            json_object_object_add(obj, "qualified_num", json_object_new_int(qualified));
                            json_object_object_add(obj, "unqualified_num", json_object_new_int(ng));
                            json_object_object_add(obj, "status", json_object_new_boolean(status));
                            json_data = json_object_to_json_string_length(obj, 0, &length);

                            ULOG_DEBUG("[%d]: %s\n", product_line, json_data);
                            sprintf(buf, "PL/%s", get_pl_serial_no(product_line));
                            mqtt_publish(buf, json_data, length, 2);
                        }

                        break;
                }
                break;
            };
    }

    cbor_ulog_describe(item);
    //fflush(stdout);
    /* Deallocate the result */
    cbor_decref(&item);
    json_object_put(obj);
}

void edge_msg_parse(char *msg, int nbytes)
{
    int length;
    char *json_data;
    char buf[32] = {0};
    json_object *obj;
    struct cbor_load_result result;
    
    if((nbytes == 92) || (nbytes == 194)){
    }else{
        ULOG_DEBUG("edge_msg_parse nbytes : %d error\n",nbytes);
        return;
    }
    cbor_item_t* item = cbor_load(msg, nbytes, &result);

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
        return;
    }

    struct mcu_chip_uid uid = {0};

    switch (cbor_typeof(item)) {
        case CBOR_TYPE_ARRAY:
            {
                cbor_item_t *elem;
                char buf[128] = {0};
                uid.uid[0] = cbor_get_uint32(cbor_array_handle(item)[0]);
                uid.uid[1] = cbor_get_uint32(cbor_array_handle(item)[1]);
                uid.uid[2] = cbor_get_uint32(cbor_array_handle(item)[2]);


                union preamble prea;
                prea.data = cbor_get_uint16(cbor_array_handle(item)[3]);

                switch (prea.bits.vp) {
                    case 1:
                        {
                            uint16_t TN = cbor_get_uint16(cbor_array_handle(item)[4]);
                            uint16_t SN = cbor_get_uint16(cbor_array_handle(item)[5]);

                            uint16_t WS_dp = cbor_get_uint16(cbor_array_handle(item)[6]);
                            uint16_t WS_ui = cbor_get_uint16(cbor_array_handle(item)[7]);
                            uint16_t WS_max = cbor_get_uint16(cbor_array_handle(item)[8]);
                            uint16_t WS_min = cbor_get_uint16(cbor_array_handle(item)[9]);
#define W_FACTOR(v) ((WS_dp > 0) ? ((v) / (float)_pow(10, WS_dp)) : (v))

                            uint16_t AS = cbor_get_uint16(cbor_array_handle(item)[10]);

                            uint16_t Dw_M = cbor_get_uint16(cbor_array_handle(item)[11]);
                            uint16_t Iw_M = cbor_get_uint16(cbor_array_handle(item)[12]);

                            uint16_t St_h = cbor_get_uint16(cbor_array_handle(item)[13]);
                            uint16_t St_m = cbor_get_uint16(cbor_array_handle(item)[14]);
                            uint16_t St_s = cbor_get_uint16(cbor_array_handle(item)[15]);

                            uint16_t Fw_status = cbor_get_uint16(cbor_array_handle(item)[16]);
                            uint16_t Fan_status = cbor_get_uint16(cbor_array_handle(item)[17]);
                            uint16_t Light_status = cbor_get_uint16(cbor_array_handle(item)[18]);
                            uint16_t Socket_status = cbor_get_uint16(cbor_array_handle(item)[19]);
                            uint16_t Sterilized_status = cbor_get_uint16(cbor_array_handle(item)[20]);

                            uint16_t Dw_set = cbor_get_uint16(cbor_array_handle(item)[21]);
                            uint16_t Iw_set = cbor_get_uint16(cbor_array_handle(item)[22]);
                            uint16_t SL_set = cbor_get_uint16(cbor_array_handle(item)[23]);

                            uint16_t fan_switch = cbor_get_uint16(cbor_array_handle(item)[24]);
                            uint16_t light_switch = cbor_get_uint16(cbor_array_handle(item)[25]);
                            uint16_t Socket_switch = cbor_get_uint16(cbor_array_handle(item)[26]);
                            uint16_t Sterilized_switch = cbor_get_uint16(cbor_array_handle(item)[27]);

                            obj = json_object_new_object();
                            sprintf(buf, "%x%x%x1-SW", uid.uid[0], uid.uid[1], uid.uid[2]);
                            json_object_object_add(obj, "ID", json_object_new_string(buf));
                            sprintf(buf, "%x", TN);
                            json_object_object_add(obj, "TN", json_object_new_string(buf));
                            sprintf(buf, "%x", SN);
                            json_object_object_add(obj, "SN", json_object_new_string(buf));

                            json_object_object_add(obj, "WS_ui", json_object_new_int(WS_ui));
                            
                            json_object_object_add(obj, "WS_max", json_object_new_double(transform(W_FACTOR(WS_max))));
                            json_object_object_add(obj, "WS_min", json_object_new_double(transform(W_FACTOR(WS_min))));

                            json_object_object_add(obj, "AS", json_object_new_int(AS));

                            json_object_object_add(obj, "Dw_M", json_object_new_double(transform(W_FACTOR(Dw_M))));
                            json_object_object_add(obj, "Iw_M", json_object_new_double(transform(W_FACTOR(Iw_M))));
                            json_object_object_add(obj, "MJ_SJ", json_object_new_int(St_h * 60 * 60 + St_m * 60 + St_s));

                            json_object_object_add(obj, "Fw_status", json_object_new_int(Fw_status));
                            json_object_object_add(obj, "Fan_status", json_object_new_int(Fan_status));
                            json_object_object_add(obj, "Light_status", json_object_new_int(Light_status));
                            json_object_object_add(obj, "Socket_status", json_object_new_int(Socket_status));
                            json_object_object_add(obj, "Sterilized_status", json_object_new_int(Sterilized_status));

                            json_object_object_add(obj, "Dw_set", json_object_new_double(transform(W_FACTOR(Dw_set))));
                            json_object_object_add(obj, "Iw_set", json_object_new_double(transform(W_FACTOR(Iw_set))));
                            json_object_object_add(obj, "SL_set", json_object_new_int(SL_set));

                            json_object_object_add(obj, "fan_switch", json_object_new_int(fan_switch));
                            json_object_object_add(obj, "light_switch", json_object_new_int(light_switch));
                            json_object_object_add(obj, "Socket_switch", json_object_new_int(Socket_switch));
                            json_object_object_add(obj, "Sterilized_switch", json_object_new_int(Sterilized_switch));


                            json_data = json_object_to_json_string_length(obj, 0, &length);

                            ULOG_DEBUG("[%x]: %s len : %d \n", prea.data, json_data,length);
                            // sprintf(buf, "kg/teste");
                            sprintf(buf, "edge3");
                            mqtt_publish(buf, json_data, length, 0);
                            // write(pipe_fd, json_data, length);

                        }
                        break;
                    case 2:
                        {
                            uint16_t TN = cbor_get_uint16(cbor_array_handle(item)[4]);
                            uint16_t SN = cbor_get_uint16(cbor_array_handle(item)[5]);
                            /* bits  */
                            union state st;
                            st.data = cbor_get_uint16(cbor_array_handle(item)[6]);

                            /* units */
                            uint16_t WD_DW = cbor_get_uint16(cbor_array_handle(item)[7]);
                            uint16_t CO2_DW = cbor_get_uint16(cbor_array_handle(item)[8]);

                            /* decimal places 6~7 */
                            uint16_t WD_dp = cbor_get_uint16(cbor_array_handle(item)[9]);
                            uint16_t CO2_dp = cbor_get_uint16(cbor_array_handle(item)[10]);
#define WD_FACTOR(v) (WD_dp > 0 ? (v) / (float)_pow(10, WD_dp) : (v))
#define CO_FACTOR(v) (CO2_dp > 0 ? (v) / (float)_pow(10, CO2_dp) : (v))

                            /* 8~11 */
                            uint16_t WD_MAX = cbor_get_uint16(cbor_array_handle(item)[11]);
                            uint16_t WD_MIN = cbor_get_uint16(cbor_array_handle(item)[12]);
                            uint16_t CO2_MAX = cbor_get_uint16(cbor_array_handle(item)[13]);
                            uint16_t CO2_MIN = cbor_get_uint16(cbor_array_handle(item)[14]);

                            //uint16_t CL_SD = cbor_get_uint16(cbor_array_handle(item)[15]);
                            uint16_t CL_WD = cbor_get_uint16(cbor_array_handle(item)[16]);
                            uint16_t SD_WD = cbor_get_uint16(cbor_array_handle(item)[17]);

                            /* 15 */
                            uint16_t CL_CO2 = cbor_get_uint16(cbor_array_handle(item)[18]);
                            uint16_t SD_CO2 = cbor_get_uint16(cbor_array_handle(item)[19]);

                            /* timer JS_T */
                            uint16_t JS_h = cbor_get_uint16(cbor_array_handle(item)[20]);
                            uint16_t JS_m = cbor_get_uint16(cbor_array_handle(item)[21]);

                            /* setting time SD_T  19~20 */
                            uint16_t SD_h = cbor_get_uint16(cbor_array_handle(item)[22]);
                            uint16_t SD_m = cbor_get_uint16(cbor_array_handle(item)[23]);

                            /* 21~24 */
                            uint16_t DQ_ZT = cbor_get_uint16(cbor_array_handle(item)[24]);
                            uint16_t DS_YX = cbor_get_uint16(cbor_array_handle(item)[25]);
                            uint16_t DS_Z = cbor_get_uint16(cbor_array_handle(item)[26]);
                            uint16_t DQ_MS = cbor_get_uint16(cbor_array_handle(item)[27]);

                            /* 25~29 YY_T */
                            uint16_t YY_Y = cbor_get_uint16(cbor_array_handle(item)[28]);
                            uint16_t YY_M = cbor_get_uint16(cbor_array_handle(item)[29]);
                            uint16_t YY_D = cbor_get_uint16(cbor_array_handle(item)[30]);
                            uint16_t YY_h = cbor_get_uint16(cbor_array_handle(item)[31]);
                            uint16_t YY_m = cbor_get_uint16(cbor_array_handle(item)[32]);

                            /* 30 */
                            uint16_t O_C = cbor_get_uint16(cbor_array_handle(item)[33]);

                            /* 31 DZ_T */
                            uint16_t DZ_T_h = cbor_get_uint16(cbor_array_handle(item)[34]);
                            uint16_t DZ_T_m = cbor_get_uint16(cbor_array_handle(item)[35]);

                            /* 33 */
                            uint16_t DZ_W = cbor_get_uint16(cbor_array_handle(item)[36]);
                            uint16_t DZ_CO2 = cbor_get_uint16(cbor_array_handle(item)[37]);

                            /* 35 */
                            uint16_t ZDS = cbor_get_uint16(cbor_array_handle(item)[38]);
                            uint16_t SD_ms = cbor_get_uint16(cbor_array_handle(item)[39]);
                            uint16_t DS1_h = cbor_get_uint16(cbor_array_handle(item)[40]);
                            uint16_t DS1_m = cbor_get_uint16(cbor_array_handle(item)[41]);
                            uint16_t DS1_W = cbor_get_uint16(cbor_array_handle(item)[42]);
                            uint16_t DS1_CO2 = cbor_get_uint16(cbor_array_handle(item)[43]);

                            /* 41 */
                            uint16_t DS2_h = cbor_get_uint16(cbor_array_handle(item)[44]);
                            uint16_t DS2_m = cbor_get_uint16(cbor_array_handle(item)[45]);
                            uint16_t DS2_W = cbor_get_uint16(cbor_array_handle(item)[46]);
                            uint16_t DS2_CO2 = cbor_get_uint16(cbor_array_handle(item)[47]);

                            /* 45 */
                            uint16_t DS3_h = cbor_get_uint16(cbor_array_handle(item)[48]);
                            uint16_t DS3_m = cbor_get_uint16(cbor_array_handle(item)[49]);
                            uint16_t DS3_W = cbor_get_uint16(cbor_array_handle(item)[50]);
                            uint16_t DS3_CO2 = cbor_get_uint16(cbor_array_handle(item)[51]);

                            /* 49 */
                            uint16_t DS4_h = cbor_get_uint16(cbor_array_handle(item)[52]);
                            uint16_t DS4_m = cbor_get_uint16(cbor_array_handle(item)[53]);
                            uint16_t DS4_W = cbor_get_uint16(cbor_array_handle(item)[54]);
                            uint16_t DS4_CO2 = cbor_get_uint16(cbor_array_handle(item)[55]);

                            /* 53 */
                            uint16_t DS5_h = cbor_get_uint16(cbor_array_handle(item)[56]);
                            uint16_t DS5_m = cbor_get_uint16(cbor_array_handle(item)[57]);
                            uint16_t DS5_W = cbor_get_uint16(cbor_array_handle(item)[58]);
                            uint16_t DS5_CO2 = cbor_get_uint16(cbor_array_handle(item)[59]);

                            /* 57 */
                            uint16_t ZM = cbor_get_uint16(cbor_array_handle(item)[60]);
                            uint16_t MJ = cbor_get_uint16(cbor_array_handle(item)[61]);

                            obj = json_object_new_object();
                            sprintf(buf, "%x%x%x2-CO", uid.uid[0], uid.uid[1], uid.uid[2]);
                            json_object_object_add(obj, "ID", json_object_new_string(buf));
                            sprintf(buf, "%x", TN);
                            json_object_object_add(obj, "TN", json_object_new_string(buf));
                            sprintf(buf, "%x", SN);
                            json_object_object_add(obj, "SN", json_object_new_string(buf));

                            // json_object_object_add(obj, "02ZW_F", json_object_new_boolean(st.bits.ZW_F));
                            // json_object_object_add(obj, "02MW_F", json_object_new_boolean(st.bits.MW_F));
                            // json_object_object_add(obj, "02ND_F", json_object_new_boolean(st.bits.ND_F));
                            // json_object_object_add(obj, "02ZW_H", json_object_new_boolean(st.bits.ZW_H));
                            // json_object_object_add(obj, "02ZW_L", json_object_new_boolean(st.bits.ZW_L));
                            // json_object_object_add(obj, "02MW_H", json_object_new_boolean(st.bits.MW_H));
                            // json_object_object_add(obj, "02ND_H", json_object_new_boolean(st.bits.ND_H));
                            // json_object_object_add(obj, "02ND_L", json_object_new_boolean(st.bits.ND_L));
                            // json_object_object_add(obj, "02M_S", json_object_new_boolean(st.bits.M_S));
                            // json_object_object_add(obj, "02SW_L", json_object_new_boolean(st.bits.SW_L));
                            // json_object_object_add(obj, "02MJ", json_object_new_boolean(st.bits.MJ));
                            // json_object_object_add(obj, "02ZM", json_object_new_boolean(st.bits.ZM));
                            json_object_object_add(obj, "02ZW_F", json_object_new_int((st.data & 0x01)));
                            json_object_object_add(obj, "02MW_F", json_object_new_int(((st.data & 0x02) >> 1)));
                            json_object_object_add(obj, "02ND_F", json_object_new_int(((st.data & 0x04)>> 2)));
                            json_object_object_add(obj, "02ZW_H", json_object_new_int(((st.data & 0x08)>> 3)));
                            json_object_object_add(obj, "02ZW_L", json_object_new_int(((st.data & 0x10)>> 4)));
                            json_object_object_add(obj, "02MW_H", json_object_new_int(((st.data & 0x20)>> 5)));
                            json_object_object_add(obj, "02ND_H", json_object_new_int(((st.data & 0x40)>> 6)));
                            json_object_object_add(obj, "02ND_L", json_object_new_int(((st.data & 0x80)>> 7)));
                            json_object_object_add(obj, "02M_S", json_object_new_int(((st.data & 0x100)>> 8)));
                            json_object_object_add(obj, "02SW_L", json_object_new_int(((st.data & 0x200)>> 9)));
                            json_object_object_add(obj, "02MJ", json_object_new_int(((st.data & 0x400)>> 10)));
                            json_object_object_add(obj, "02ZM", json_object_new_int(((st.data & 0x800)>> 11)));
                            ULOG_DEBUG("st.data : %x\n", st.data);

                            /* 4 */
                            json_object_object_add(obj, "WD_DW", json_object_new_int(WD_DW));
                            json_object_object_add(obj, "CO2_DW", json_object_new_int(CO2_DW));

                            /* 8 */
                            json_object_object_add(obj, "WD_MAX", json_object_new_double(transform(WD_FACTOR(WD_MAX))));
                            json_object_object_add(obj, "WD_MIN", json_object_new_double(transform(WD_FACTOR(WD_MIN))));
                            json_object_object_add(obj, "CO2_MAX", json_object_new_double(transform(CO_FACTOR(CO2_MAX))));
                            json_object_object_add(obj, "CO2_MIN", json_object_new_double(transform(CO_FACTOR(CO2_MIN))));

                            json_object_object_add(obj, "CL_WD", json_object_new_double(transform(WD_FACTOR(CL_WD))));
                            json_object_object_add(obj, "SD_WD", json_object_new_double(transform(WD_FACTOR(SD_WD))));

                            json_object_object_add(obj, "CL_CO2", json_object_new_double(transform(CO_FACTOR(CL_CO2))));
                            json_object_object_add(obj, "SD_CO2", json_object_new_double(transform(CO_FACTOR(SD_CO2))));

                            json_object_object_add(obj, "JS_T", json_object_new_int(JS_h * 60 + JS_m));
                            json_object_object_add(obj, "SD_T", json_object_new_int(SD_h * 60 + SD_m));

                            /* 21 */
                            json_object_object_add(obj, "DQ_ZT", json_object_new_int(DQ_ZT));
                            json_object_object_add(obj, "DS_YX", json_object_new_int(DS_YX));
                            json_object_object_add(obj, "DS_Z", json_object_new_int(DS_Z));
                            json_object_object_add(obj, "DQ_MS", json_object_new_int(DQ_MS));

                            sprintf(buf, "%d-%d-%d %d:%d", YY_Y, YY_M, YY_D, YY_h, YY_m);
                            json_object_object_add(obj, "YY_T", json_object_new_string(buf));

                            json_object_object_add(obj, "O_C", json_object_new_int(O_C));

                            json_object_object_add(obj, "DZ_T", json_object_new_double(DZ_T_h * 60 + DZ_T_m));

                            json_object_object_add(obj, "DZ_W", json_object_new_double(transform(WD_FACTOR(DZ_W))));
                            json_object_object_add(obj, "DZ_CO2", json_object_new_double(transform(CO_FACTOR(DZ_CO2))));

                            json_object_object_add(obj, "ZDS", json_object_new_int(ZDS));
                            json_object_object_add(obj, "SZ_ms", json_object_new_int(SD_ms));

                            json_object_object_add(obj, "DS1_h", json_object_new_int(DS1_h));
                            json_object_object_add(obj, "DS1_m", json_object_new_int(DS1_m));
                            json_object_object_add(obj, "DS1_W", json_object_new_double(transform(WD_FACTOR(DS1_W))));
                            json_object_object_add(obj, "DS1_CO2", json_object_new_double((CO_FACTOR(DS1_CO2))));

                            json_object_object_add(obj, "DS2_h", json_object_new_int(DS2_h));
                            json_object_object_add(obj, "DS2_m", json_object_new_int(DS2_m));
                            json_object_object_add(obj, "DS2_W", json_object_new_double(transform(WD_FACTOR(DS2_W))));
                            json_object_object_add(obj, "DS2_CO2", json_object_new_double(transform(CO_FACTOR(DS2_CO2))));

                            json_object_object_add(obj, "DS3_h", json_object_new_int(DS3_h));
                            json_object_object_add(obj, "DS3_m", json_object_new_int(DS3_m));
                            json_object_object_add(obj, "DS3_W", json_object_new_double(transform(WD_FACTOR(DS3_W))));
                            json_object_object_add(obj, "DS3_CO2", json_object_new_double(transform(CO_FACTOR(DS3_CO2))));

                            json_object_object_add(obj, "DS4_h", json_object_new_int(DS4_h));
                            json_object_object_add(obj, "DS4_m", json_object_new_int(DS4_m));
                            json_object_object_add(obj, "DS4_W", json_object_new_double(transform(WD_FACTOR(DS4_W))));
                            json_object_object_add(obj, "DS4_CO2", json_object_new_double(transform(CO_FACTOR(DS4_CO2))));

                            json_object_object_add(obj, "DS5_h", json_object_new_int(DS5_h));
                            json_object_object_add(obj, "DS5_m", json_object_new_int(DS5_m));
                            json_object_object_add(obj, "DS5_W", json_object_new_double(transform(WD_FACTOR(DS5_W))));
                            json_object_object_add(obj, "DS5_CO2", json_object_new_double(transform(CO_FACTOR(DS5_CO2))));

                            json_object_object_add(obj, "ZM", json_object_new_int(ZM));
                            json_object_object_add(obj, "MJ", json_object_new_int(MJ));

                            json_data = json_object_to_json_string_length(obj, 0, &length);

                            ULOG_DEBUG("[%x]: %s len : %d \n", prea.data, json_data,length);
                            // sprintf(buf, "kg/teste");
                            sprintf(buf, "edge3");
                            mqtt_publish(buf, json_data, length, 0);
                            // write(pipe_fd, json_data, length);
                        }
                        break;
                }
                break;
            };
    }

    cbor_ulog_describe(item);
    //fflush(stdout);
    /* Deallocate the result */
    cbor_decref(&item);
    json_object_put(obj);

}

static void ustream_read_cb(struct ustream *s, int bytes)
{
	char *str;
    int i, nbytes, print;
    //char printbuf[4096] = {0};

	do {
		str = ustream_get_read_buf(s, &nbytes);
		if (!str)
			break;

        /* seriously, this's not a good solution to accurately identifie the header of
         * a message, but it works for now. make it right later.
         */
        //ipc_msg_parse(str, nbytes);
        //msg_parse(str, nbytes);
        edge_msg_parse(str, nbytes);

        //print = 0;

        // for (i = 0; i < nbytes; i++)
            // print += sprintf(printbuf + print, "%x", *str++);
        // ULOG_INFO("[%d]: %s\n", nbytes, printbuf);

		ustream_consume(s, nbytes);
	} while(1);

	if (s->w.data_bytes > 256 && !ustream_read_blocked(s)) {
		ULOG_NOTE("Block read, bytes: %d\n", s->w.data_bytes);
		ustream_set_read_blocked(s, true);
	}
}

static void ipcs_close(struct ustream *s)
{
	ULOG_INFO("Connection closed");
	ustream_free(s);
	close(ipc_get_fd(ipcs));
}

static void ustream_write_cb(struct ustream *s, int bytes)
{
	ULOG_INFO("Wrote %d bytes, pending: %d\n", bytes, s->w.data_bytes);

	if (s->w.data_bytes < 128 && ustream_read_blocked(s)) {
		ULOG_INFO("Unblock read");
		ustream_set_read_blocked(s, false);
	}
}

static void ustream_state_cb(struct ustream *s)
{
	if (!s->eof)
		return;

	ULOG_ERR("eof!, pending: %d\n", s->w.data_bytes);
	if (!s->w.data_bytes)
		return ipcs_close(s);
}


int ipc_init(void)
{
    int fd,res;
    
    int open_mode = O_WRONLY;

    if(access(FIFO_NAME, F_OK) == -1)
    {
        res = mknod(FIFO_NAME, 0777, 0);
        if(res != 0)
        {
            ULOG_ERR(stderr, "Could not create fifo %s\n", FIFO_NAME);
        }
    }
    pipe_fd = open(FIFO_NAME, open_mode);
    if (pipe_fd == NULL) {
        ULOG_ERR("open fifo failed!\n");
        return -1;
    }

    ipcs = ipcs_create(&ipc_ctx, "controller");
    if (ipcs == NULL) {
        ULOG_ERR("ipc_create failed!\n");
        return -1;
    }

    fd = ipc_get_fd(ipcs);

    if (fd < 0) {
        ULOG_ERR("ipcs_create error: %d\n", fd);
        return -1;
    }

    us = &sfd.stream;
    us->notify_read = ustream_read_cb;
    us->notify_write = ustream_write_cb;
    us->notify_state = ustream_state_cb;
    us->string_data = false;
    ustream_fd_init(&sfd, fd);

    memset(&benchmark_timeout, 0, sizeof(benchmark_timeout));
    benchmark_timeout.cb = benchmark;
    //uloop_timeout_set(&benchmark_timeout, 1000);
    return 0;
}
