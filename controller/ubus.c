#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <syslog.h>
#include <sys/sysinfo.h>
#include <libubox/ulog.h>
#include <libubox/uloop.h>
#include <libubox/ustream.h>
#include <libubox/utils.h>
#include <libubox/blobmsg.h>
#include <libubox/blobmsg_json.h>
#include <libubus.h>
#include <json-c/json.h>

#include "ubus.h"
#include "utils.h"

enum {
    DEBUG_LOGLEVEL,
    _DEBUG_MAX_
};

struct ubus_context *ubus_ctx = NULL;
static struct blob_buf b;
static const char *ubus_path;

extern struct cpu_tick *cpustat[2];

static int
controller_statistic(struct ubus_context *ctx, struct ubus_object *obj,
		      struct ubus_request_data *req, const char *method,
		      struct blob_attr *msg)
{
	return 0;
}

static const struct blobmsg_policy controller_debug_policy[_DEBUG_MAX_] = {
	[DEBUG_LOGLEVEL] = { .name = "loglevel", .type = BLOBMSG_TYPE_INT32 },
};


static int
controller_debug(struct ubus_context *ctx, struct ubus_object *obj,
		      struct ubus_request_data *req, const char *method,
		      struct blob_attr *msg)
{
    int loglevel = LOG_DEBUG;
	struct blob_attr *tb[_DEBUG_MAX_];

	blobmsg_parse(controller_debug_policy, _DEBUG_MAX_, tb, blob_data(msg), blob_len(msg));

	if (tb[DEBUG_LOGLEVEL])
		loglevel = blobmsg_get_u32(tb[DEBUG_LOGLEVEL]);

    ulog_threshold(loglevel);

	blob_buf_init(&b, 0);
	blobmsg_add_u32(&b, "log level", loglevel);
	ubus_send_reply(ctx, req, b.head);

	return 0;
}

static int
controller_sysinfo(struct ubus_context *ctx, struct ubus_object *obj,
		      struct ubus_request_data *req, const char *method,
		      struct blob_attr *msg)
{
    int i;
    int ret = 0;
    int mem_used = 0;
    char buf[64] = {0};
    float mem_usage = 0;
    struct sysinfo info;
    int nprocs = 0;

    struct cpu_tick *cpustat_now;
    struct cpu_tick *cpustat_prev, *cpustat_curr;
    json_object *cpux_obj = NULL;
    json_object *cpuload_obj = NULL;

    if (cpustat[0] == NULL || cpustat[1] == NULL)
        goto out;

    cpux_obj = json_object_new_object();
    cpuload_obj = json_object_new_object();

    if (cpux_obj == NULL || cpuload_obj == NULL) {
        ULOG_ERR("Out of memory\n");
        goto out_objs_free;
    }

    nprocs =  get_nprocs();
    cpustat_now = malloc(sizeof(struct cpu_tick) * (nprocs + 1));
    if (cpustat_now == NULL) {
        ULOG_ERR("Out of memory\n");
        goto out_objs_free;
    }
    ret = sysinfo(&info);
    if (ret != 0) {
        ULOG_ERR("sysinfo: %s\n", strerror(errno));
        goto out_cpustat_now_free;
    }

    mem_used = info.totalram - info.freeram;
    mem_usage = (float)mem_used / info.totalram * 100;

    blob_buf_init(&b, 0);
    snprintf(buf, sizeof(buf), "%.1f%%", mem_usage);
    blobmsg_add_string(&b, "mem_usage", buf);

    read_cpustat(cpustat_now);
    cpustat_curr = cpustat_now;

    /* cancel cpustat timer */
    utils_cpustat_timeout_cancel();

    if (utils_cpustat_timeout_remaining() > 200)
        cpustat_prev = cpustat[utils_cpustat_curr_index()];
    else
        cpustat_prev = cpustat[utils_cpustat_curr_index() ^ 1];

    for (i = 0; i <= nprocs; i++) {
        uint64_t total = total_ticks(cpustat_curr + i) - total_ticks(cpustat_prev + i);
        uint64_t idle = idle_ticks(cpustat_curr + i) - idle_ticks(cpustat_prev + i);
        uint64_t active = total - idle;
        snprintf(buf, sizeof(buf), "%.1f%%", active * 100.f / total);
        json_object_object_add(cpux_obj, cpustat_curr[i].name, json_object_new_string(buf));
    }

    /* set cpustat monitor timer */
    utils_cpustat_timeout_set();

    blobmsg_add_json_element(&b, "cpu_usage", cpux_obj);

    snprintf(buf, sizeof(buf), "%.2f", (double)info.loads[0] / 65536);
    json_object_object_add(cpuload_obj, "1 minute", json_object_new_string(buf));
    snprintf(buf, sizeof(buf), "%.2f", (double)info.loads[1] / 65536);
    json_object_object_add(cpuload_obj, "5 minutes", json_object_new_string(buf));
    snprintf(buf, sizeof(buf), "%.2f", (double)info.loads[2] / 65536);
    json_object_object_add(cpuload_obj, "15 minutes", json_object_new_string(buf));

    blobmsg_add_json_element(&b, "cpu_loadavg", cpuload_obj);

    blobmsg_add_u32(&b, "totalram", (uint32_t)info.totalram);
    blobmsg_add_u32(&b, "freeram", (uint32_t)info.freeram);
    blobmsg_add_u32(&b, "procs", (uint32_t)info.procs);

    ubus_send_reply(ctx, req, b.head);

out_cpustat_now_free:
    if (cpustat_now)
        free(cpustat_now);
out_objs_free:
    if (cpux_obj)
        json_object_put(cpux_obj);
    if (cpuload_obj)
        json_object_put(cpuload_obj);
out:
    return 0;
}

static struct ubus_method main_object_methods[] = {
    { .name = "statistic", .handler = controller_statistic},
    UBUS_METHOD("debug", controller_debug, controller_debug_policy),
    UBUS_METHOD_NOARG("sysinfo", controller_sysinfo), 
};

static struct ubus_object_type main_object_type =
	UBUS_OBJECT_TYPE("controller", main_object_methods);

static struct ubus_object main_object = {
	.name = "controller",
	.type = &main_object_type,
	.methods = main_object_methods,
	.n_methods = ARRAY_SIZE(main_object_methods),
};

void system_fd_set_cloexec(int fd)
{
#ifdef FD_CLOEXEC
	fcntl(fd, F_SETFD, fcntl(fd, F_GETFD) | FD_CLOEXEC);
#endif
}

static void
controller_ubus_add_fd(void)
{
	ubus_add_uloop(ubus_ctx);
	system_fd_set_cloexec(ubus_ctx->sock.fd);
}


static void
controller_ubus_reconnect_timer(struct uloop_timeout *timeout)
{
	static struct uloop_timeout retry = {
		.cb = controller_ubus_reconnect_timer,
	};
	int t = 2;

	if (ubus_reconnect(ubus_ctx, ubus_path) != 0) {
		ULOG_WARN("failed to reconnect, trying again in %d seconds\n", t);
		uloop_timeout_set(&retry, t * 1000);
		return;
	}

	ULOG_INFO("reconnected to ubus, new id: %08x\n", ubus_ctx->local_id);
	controller_ubus_add_fd();
}

static void
controller_ubus_connection_lost(struct ubus_context *ctx)
{
	controller_ubus_reconnect_timer(NULL);
}

static void controller_add_object(struct ubus_object *obj)
{
	int ret = ubus_add_object(ubus_ctx, obj);

	if (ret != 0)
		ULOG_ERR("Failed to publish object '%s': %s\n", obj->name, ubus_strerror(ret));
}

int
controller_ubus_init(const char *path)
{
	uloop_init();
	ubus_path = path;

	ubus_ctx = ubus_connect(path);
	if (!ubus_ctx)
		return -EIO;

	ULOG_INFO("ubus connected as %08x\n", ubus_ctx->local_id);
	ubus_ctx->connection_lost = controller_ubus_connection_lost;
	controller_ubus_add_fd();

	controller_add_object(&main_object);

	return 0;
}

void
controller_ubus_done(void)
{
	ubus_free(ubus_ctx);
}
