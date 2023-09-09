#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>

#include <libubox/ulog.h>
#include <libubox/uloop.h>
#include <libubox/utils.h>
#include <mosquitto.h>
#include <mqtt_protocol.h>
#include <log.h>
#include "mqtt.h"
#include "config.h"
#include "message.h"

#define UNUSED(A) (void)(A)

bool process_messages = true;
int msg_count = 0;
static struct mosquitto *mosq = NULL;
int last_mid = 0;
static bool timed_out = false;
static int connack_result = 0;
static struct uloop_fd mosquitto_client;
static struct uloop_timeout loop_timeout;
static struct uloop_timeout reconnect_timeout;

static void mosquitto_user_reconnect(void);

static char *topics[] = {
    "/sys/${uuid}/gw/mgmt/rpc/request/+",
    "/sys/${uuid}/gw/mgmt/config/push",
};

void (* topic_handler[]) (void *msg, int len) = {
    control_msg_handler,
    config_msg_handler
};


void my_signal_handler(int signum)
{
	if (signum == SIGALRM || signum == SIGTERM || signum == SIGINT) {
		process_messages = false;
		mosquitto_disconnect_v5(mosq, MQTT_RC_DISCONNECT_WITH_WILL_MSG, NULL);
	}
	if (signum == SIGALRM) {
		timed_out = true;
	}
}

void my_message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message, const mosquitto_property *properties)
{
	int i;
	bool res;

	UNUSED(obj);
    UNUSED(properties);

	if (process_messages == false) return;

	if (!message->retain && process_messages) {
		process_messages = false;
		if (last_mid == 0){
			mosquitto_disconnect_v5(mosq, 0, NULL);
		}
		return;
	}

    for (i = 0; i < sizeof(topics); i++) {
        mosquitto_topic_matches_sub(topics[i], message->topic, &res);
        if (res)
            topic_handler[i](message->payload, message->payloadlen);
    }

    ULOG_INFO("%s: %s\n", message->topic, message->payload);
}

void my_connect_callback(struct mosquitto *mosq, void *obj, int result, int flags, const mosquitto_property *properties)
{
	UNUSED(obj);
    UNUSED(flags);
    UNUSED(properties);

	connack_result = result;
	if (!result) {
		mosquitto_subscribe_multiple(mosq, NULL, 2, topics, 0, 0, NULL);

		// for (i = 0; i < unsub_topic_count; i++){
			// mosquitto_unsubscribe_v5(mosq, NULL, cfg.unsub_topics[i], cfg.unsubscribe_props);
		// }
	} else {
		if (result) {
            if (result == MQTT_RC_UNSUPPORTED_PROTOCOL_VERSION) {
                ULOG_ERR("Connection error: %s. Try connecting to an MQTT v5 broker, or use MQTT v3.x mode.\n", mosquitto_reason_string(result));
            } else {
                ULOG_ERR("Connection error: %s\n", mosquitto_reason_string(result));
            }
		}
		mosquitto_disconnect_v5(mosq, 0, NULL);

        mosquitto_user_reconnect();
	}

    ULOG_INFO("mqtt connected\n");
}

void my_subscribe_callback(struct mosquitto *mosq, void *obj, int mid, int qos_count, const int *granted_qos)
{
	int i;
	bool some_sub_allowed = (granted_qos[0] < 128);
	UNUSED(obj);

	ULOG_INFO("Subscribed (mid: %d): %d", mid, granted_qos[0]);
	for (i = 1; i < qos_count; i++) {
		if (true)
            ULOG_INFO(", %d", granted_qos[i]);
		some_sub_allowed |= (granted_qos[i] < 128);
	}
    ULOG_INFO("\n");

	if (some_sub_allowed == false) {
		mosquitto_disconnect_v5(mosq, 0, NULL);
		ULOG_ERR("All subscription requests were denied.\n");
	}
}

void my_publish_callback(struct mosquitto *mosq, void *obj, int mid, int reason_code, const mosquitto_property *properties)
{
	char *reason_string = NULL;
	UNUSED(obj);
	UNUSED(properties);

	if (reason_code > 127) {
		ULOG_WARN("Warning: Publish %d failed: %s.\n", mid, mosquitto_reason_string(reason_code));
		mosquitto_property_read_string(properties, MQTT_PROP_REASON_STRING, &reason_string, false);
		if (reason_string) {
			ULOG_WARN("%s\n", reason_string);
			free(reason_string);
		}
	}
}

void my_log_callback(struct mosquitto *mosq, void *obj, int level, const char *str)
{
	UNUSED(mosq);
	UNUSED(obj);
	UNUSED(level);

	ULOG_INFO("%s\n", str);
}

void mosquitto_print_version(void)
{
	int major, minor, revision;

	mosquitto_lib_version(&major, &minor, &revision);
	ULOG_INFO("mosquitto running on libmosquitto %d.%d.%d.\n", major, minor, revision);
}

int get_temperature(void)
{
	sleep(1); /* Prevent a storm of messages - this pretend sensor works at 1Hz */
	return random()%100;
}

/* This function pretends to read some data from a sensor and publish it.*/
void publish_sensor_data(struct mosquitto *mosq)
{
	char payload[20];
	int temp;
	int rc;

	/* Get our pretend data */
	temp = get_temperature();
	/* Print it to a string for easy human reading - payload format is highly
	 * application dependent. */
	snprintf(payload, sizeof(payload), "%d", temp);

	/* Publish the message
	 * mosq - our client instance
	 * *mid = NULL - we don't want to know what the message id for this message is
	 * topic = "example/temperature" - the topic on which this message will be published
	 * payloadlen = strlen(payload) - the length of our payload in bytes
	 * payload - the actual payload
	 * qos = 2 - publish with QoS 2 for this example
	 * retain = false - do not use the retained message feature for this message
	 */
	rc = mosquitto_publish(mosq, NULL, "example/temperature", strlen(payload), payload, 2, false);
	if (rc != MOSQ_ERR_SUCCESS) {
		ULOG_ERR("Error publishing: %s\n", mosquitto_strerror(rc));
	}
}

static void mosquitto_user_reconnect(void)
{
    uloop_timeout_set(&reconnect_timeout, 1000);
}

int mqtt_publish(char *topic, const void *payload, int length, int qos)
{
    int rc;

    /* Publish the message
     * mosq - our client instance
     * *mid = NULL - we don't want to know what the message id for this message is
     * topic = "example/temperature" - the topic on which this message will be published
     * payloadlen = strlen(payload) - the length of our payload in bytes
     * payload - the actual payload
     * qos = 2 - publish with QoS 2 for this example
     * retain = false - do not use the retained message feature for this message
     */
    rc = mosquitto_publish(mosq, NULL, topic, length, payload, qos, false);
    switch (rc) {
        case MOSQ_ERR_SUCCESS:
            break;
        case MOSQ_ERR_NO_CONN:
            mosquitto_user_reconnect();
            ULOG_INFO("mqtt client reconnect\n");
            break;
        default:
            ULOG_ERR("publish: %s\n", mosquitto_strerror(rc));
            break;
    }

    return rc;
}


static void mosquitto_client_cb(struct uloop_fd *fd, unsigned int events)
{
    /* there is no need to loop write, mosquitto lib will do it. */
    if (events & ULOOP_READ)
        mosquitto_loop_read(mosq, 1);
    else {
        if (mosquitto_want_write(mosq))
            mosquitto_loop_write(mosq, 1);
    }
}

static void loop_misc(struct uloop_timeout *timeout)
{
    mosquitto_loop_misc(mosq);
}

static void reconnect_cb(struct uloop_timeout *timeout)
{
    int sock;

    sock = mosquitto_socket(mosq);
    if (sock < 0) {
        ULOG_ERR("mosquitto socket invalid\n");
    } else {
        ULOG_INFO("stopping mosquitto io watcher(%d)\n", sock);
    }
    uloop_fd_delete(&mosquitto_client);

    if (mosquitto_reconnect_async(mosq) == MOSQ_ERR_SUCCESS) {
        ULOG_INFO("mqtt async reconnect successed\n");
        sock = mosquitto_socket(mosq);
        if (sock < 0) {
            ULOG_ERR("mosquitto socket invalid\n");
        } else {
            ULOG_INFO("starting mosquitto io watcher(%d)\n", sock);
            mosquitto_client.fd = sock;
            uloop_fd_add(&mosquitto_client, ULOOP_READ);
        }
    } else {
        ULOG_ERR("mqtt async reconnect failed\n");
    }
}

int mqtt_init(void)
{
	int rc;
    int sock = 0;

    char ip_str[64];
    int port = 0;

	mosquitto_lib_init();
    mosquitto_print_version();
    config_get_access_point(ip_str, &port);

	mosq = mosquitto_new("huamai-device-2-1", true, NULL);
	if (!mosq) {
		switch (errno) {
			case ENOMEM:
				ULOG_ERR("Error: Out of memory.\n");
				break;
			case EINVAL:
				ULOG_ERR("Error: Invalid id and/or clean_session.\n");
				break;
		}
		goto cleanup;
	}

    //mosquitto_log_callback_set(mosq, my_log_callback);
	mosquitto_connect_v5_callback_set(mosq, my_connect_callback);
	mosquitto_subscribe_callback_set(mosq, my_subscribe_callback);
	mosquitto_message_v5_callback_set(mosq, my_message_callback);
    mosquitto_publish_v5_callback_set(mosq, my_publish_callback);

    memset(&loop_timeout, 0, sizeof(loop_timeout));
    loop_timeout.cb = loop_misc;
    uloop_timeout_set(&loop_timeout, 1000);

    memset(&reconnect_timeout, 0, sizeof(reconnect_timeout));
    reconnect_timeout.cb = reconnect_cb;

    mosquitto_client.cb = mosquitto_client_cb;

	rc = mosquitto_connect(mosq, ip_str, port, 120);
	if (rc) {
        ULOG_ERR("mosq connect: %s\n", mosquitto_strerror(rc));
		mosquitto_user_reconnect();
		goto out;
	}

    sock = mosquitto_socket(mosq);
    if (sock < 0) {
        ULOG_ERR("mosquitto socket get failed\n");
        goto out;
    }

    mosquitto_client.fd = sock;
    uloop_fd_add(&mosquitto_client, ULOOP_READ);

out:
    return 0;

cleanup:
	mosquitto_destroy(mosq);
	mosquitto_lib_cleanup();
	return -1;
}

