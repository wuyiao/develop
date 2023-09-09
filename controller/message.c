#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <json-c/json.h>

#include "message.h"


void control_msg_handler(void *msg, int len)
{
    json_object *msg_obj = NULL;
    json_object *current_obj = NULL;

    if (msg)
        msg_obj = json_tokener_parse(msg);

    if (!msg_obj)
        goto parse_error;

    if (!json_object_object_get_ex(msg_obj, "command", &current_obj))
        goto parse_error;


    switch (json_object_get_int(current_obj)) {
        case CC_RESET:
            break;

        case CC_REBOOT:
            break;

        default:
            break;
    }

parse_error:
    if (msg_obj)
        json_object_put(msg_obj);
}

void config_msg_handler(void *msg, int len)
{
    json_object *msg_obj = NULL;
    json_object *current_obj = NULL;

    if (msg)
        msg_obj = json_tokener_parse(msg);

    if (!msg_obj)
        goto parse_error;

    if (!json_object_object_get_ex(msg_obj, "command", &current_obj))
        goto parse_error;


    switch (json_object_get_int(current_obj)) {
        case CC_RESET:
            break;

        default:
            break;
    }

parse_error:
    if (msg_obj)
        json_object_put(msg_obj);
}
