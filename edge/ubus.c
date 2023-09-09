#include <edge.h>
#include <service.h>

static struct ubus_context *ubus_ctx = NULL;
static struct ubus_subscriber observer_event;
static struct blob_buf b;

#if 0
enum {
	HELLO_ID,
	HELLO_MSG,
	__HELLO_MAX
};

static const struct blobmsg_policy hello_policy[] = {
	[HELLO_ID] = { .name = "id", .type = BLOBMSG_TYPE_INT32 },
	[HELLO_MSG] = { .name = "msg", .type = BLOBMSG_TYPE_STRING },
};

struct hello_request {
	struct ubus_request_data req;
	struct uloop_timeout timeout;
	int fd;
	int idx;
	char data[];
};

static void test_hello_fd_reply(struct uloop_timeout *t)
{
	struct hello_request *req = container_of(t, struct hello_request, timeout);
	char *data;

	data = alloca(strlen(req->data) + 32);
	sprintf(data, "msg%d: %s\n", ++req->idx, req->data);
	if (write(req->fd, data, strlen(data)) < 0) {
		close(req->fd);
		free(req);
		return;
	}

	uloop_timeout_set(&req->timeout, 1000);
}

static void test_hello_reply(struct uloop_timeout *t)
{
	struct hello_request *req = container_of(t, struct hello_request, timeout);
	int fds[2];

	blob_buf_init(&b, 0);
	blobmsg_add_string(&b, "message", req->data);
	ubus_send_reply(ctx, &req->req, b.head);

	if (pipe(fds) == -1) {
		fprintf(stderr, "Failed to create pipe\n");
		return;
	}
	ubus_request_set_fd(ctx, &req->req, fds[0]);
	ubus_complete_deferred_request(ctx, &req->req, 0);
	req->fd = fds[1];

	req->timeout.cb = test_hello_fd_reply;
	test_hello_fd_reply(t);
}

static int test_hello(struct ubus_context *ctx, struct ubus_object *obj,
		      struct ubus_request_data *req, const char *method,
		      struct blob_attr *msg)
{
	struct hello_request *hreq;
	struct blob_attr *tb[__HELLO_MAX];
	const char format[] = "%s received a message: %s";
	const char *msgstr = "(unknown)";

	blobmsg_parse(hello_policy, ARRAY_SIZE(hello_policy), tb, blob_data(msg), blob_len(msg));

	if (tb[HELLO_MSG])
		msgstr = blobmsg_data(tb[HELLO_MSG]);

	size_t len = sizeof(*hreq) + sizeof(format) + strlen(obj->name) + strlen(msgstr) + 1;
	hreq = calloc(1, len);
	if (!hreq)
		return UBUS_STATUS_UNKNOWN_ERROR;

	snprintf(hreq->data, len, format, obj->name, msgstr);
	ubus_defer_request(ctx, req, &hreq->req);
	hreq->timeout.cb = test_hello_reply;
	uloop_timeout_set(&hreq->timeout, 1000);

	return 0;
}

enum {
	WATCH_ID,
	WATCH_COUNTER,
	__WATCH_MAX
};

static const struct blobmsg_policy watch_policy[__WATCH_MAX] = {
	[WATCH_ID] = { .name = "id", .type = BLOBMSG_TYPE_INT32 },
	[WATCH_COUNTER] = { .name = "counter", .type = BLOBMSG_TYPE_INT32 },
};

static void
test_handle_remove(struct ubus_context *ctx, struct ubus_subscriber *s,
                   uint32_t id)
{
	fprintf(stderr, "Object %08x went away\n", id);
}

static int
test_notify(struct ubus_context *ctx, struct ubus_object *obj,
	    struct ubus_request_data *req, const char *method,
	    struct blob_attr *msg)
{
#if 0
	char *str;

	str = blobmsg_format_json(msg, true);
	fprintf(stderr, "Received notification '%s': %s\n", method, str);
	free(str);
#endif

	return 0;
}

static int test_watch(struct ubus_context *ctx, struct ubus_object *obj,
		      struct ubus_request_data *req, const char *method,
		      struct blob_attr *msg)
{
	struct blob_attr *tb[__WATCH_MAX];
	int ret;

	blobmsg_parse(watch_policy, __WATCH_MAX, tb, blob_data(msg), blob_len(msg));
	if (!tb[WATCH_ID])
		return UBUS_STATUS_INVALID_ARGUMENT;

	test_event.remove_cb = test_handle_remove;
	test_event.cb = test_notify;
	ret = ubus_subscribe(ctx, &test_event, blobmsg_get_u32(tb[WATCH_ID]));
	fprintf(stderr, "Watching object %08x: %s\n", blobmsg_get_u32(tb[WATCH_ID]), ubus_strerror(ret));
	return ret;
}
#endif

enum {
    OBSERVE_OBJECT,
    __OBSERVE_MAX
};

static const struct blobmsg_policy observe_policy[__OBSERVE_MAX] = {
    [OBSERVE_OBJECT] = { .name = "object", .type = BLOBMSG_TYPE_STRING },
};

static int observe_info(struct ubus_context *ctx, struct ubus_object *obj,
		      struct ubus_request_data *req, const char *method,
		      struct blob_attr *msg)
{
	struct blob_attr *tb[__OBSERVE_MAX];
	char *s1, *s2;
	uint32_t num;

	blobmsg_parse(observe_policy, __OBSERVE_MAX, tb, blob_data(msg), blob_len(msg));
	if (!tb[OBSERVE_OBJECT])
		return UBUS_STATUS_INVALID_ARGUMENT;

	s1 = blobmsg_get_string(tb[OBSERVE_OBJECT]);

    lm_log_all_node();

	blob_buf_init(&b, 0);
	blobmsg_add_u32(&b, "rc", 0);
	ubus_send_reply(ctx, req, b.head);

	return 0;
}

static const struct ubus_method observer_methods[] = {
	UBUS_METHOD("observe", observe_info, observe_policy),
};

static struct ubus_object_type observer_object_type =
	UBUS_OBJECT_TYPE("observer", observer_methods);

static struct ubus_object observer_object = {
	.name = "edge.observer",
	.type = &observer_object_type,
	.methods = observer_methods,
	.n_methods = ARRAY_SIZE(observer_methods),
};

static void ubus_register(void)
{
	int ret;

	ret = ubus_add_object(ubus_ctx, &observer_object);
	if (ret)
		ULOG_ERR("Failed to add object: %s\n", ubus_strerror(ret));

	ret = ubus_register_subscriber(ubus_ctx, &observer_event);
	if (ret)
		ULOG_ERR("Failed to add watch handler: %s\n", ubus_strerror(ret));
}

void system_fd_set_cloexec(int fd)
{
#ifdef FD_CLOEXEC
	fcntl(fd, F_SETFD, fcntl(fd, F_GETFD) | FD_CLOEXEC);
#endif
}

static void
edge_ubus_add_fd(void)
{
	ubus_add_uloop(ubus_ctx);
	system_fd_set_cloexec(ubus_ctx->sock.fd);
}

static void
edge_ubus_reconnect_timer(struct uloop_timeout *timeout)
{
	static struct uloop_timeout retry = {
		.cb = edge_ubus_reconnect_timer,
	};
	int t = 2;

	if (ubus_reconnect(ubus_ctx, NULL) != 0) {
		ULOG_WARN("failed to reconnect, trying again in %d seconds\n", t);
		uloop_timeout_set(&retry, t * 1000);
		return;
	}

	ULOG_INFO("reconnected to ubus, new id: %08x\n", ubus_ctx->local_id);
	edge_ubus_add_fd();
}

static void
edge_ubus_connection_lost(struct ubus_context *ctx)
{
	edge_ubus_reconnect_timer(NULL);
}


int ubus_init()
{
    ubus_ctx = ubus_connect(NULL);
    if (!ubus_ctx)
        return -EIO;

    ubus_ctx->connection_lost = edge_ubus_connection_lost;
    edge_ubus_add_fd();

    ubus_register();

    return 0;
}
