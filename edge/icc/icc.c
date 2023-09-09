/* inter-communication channel 
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/uio.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <libubox/utils.h>
#include <libubox/uloop.h>
#include <libubox/ustream.h>

#include <edge.h>
#include <ipc.h>
#include <softbus.h>
#include <ubuf.h>
#include <debug.h>

#define IPC_NAME "controller"

static struct ustream *us;
static struct ustream_fd sfd;
static struct ipc_context ipc_ctx;
static struct ipc_context *ipcc;
static struct uloop_timeout ipcc_reconnect_timeout;

static struct softbus_peer *peer;

static void ustream_read_cb(struct ustream *s, int bytes)
{
	struct ustream_buf *buf = s->r.head;
	char *newline, *str;

	do {
		str = ustream_get_read_buf(s, NULL);
		if (!str)
			break;

		newline = strchr(buf->data, '\n');
		if (!newline)
			break;

		*newline = 0;
		ustream_printf(s, "%s\n", str);
		ustream_consume(s, newline + 1 - str);
	} while(1);

	if (s->w.data_bytes > 256 && !ustream_read_blocked(s)) {
		ULOG_NOTE("Block read, bytes: %d\n", s->w.data_bytes);
		ustream_set_read_blocked(s, true);
	}
}

static void ipcc_close(struct ustream *s)
{
    int fd = -1;

	ULOG_INFO("Connection closed\n");
	ustream_free(s);

    if (ipcc)
        fd = ipc_get_fd(ipcc);

    if (fd > 0)
        close(fd);

    us = NULL;
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
        return ipcc_close(s);
}

void ipcc_defer_reconnect()
{
    uloop_timeout_set(&ipcc_reconnect_timeout, 1000 * 10);
}

int icc_stream_write(const void *data, int len)
{
    int rc;

    if (us) {
        ULOG_DEBUG("len: %d ->\n", len);

        rc = ustream_write(us, data, len, true);
        if (rc <= 0) {
            ULOG_ERR("ipc stream connection lost, retry soon\n");
            ipcc_close(us);
            ipcc_defer_reconnect();
        }
        return rc;
    }
    else
        return -1;
}

int icc_sink(struct softbus_peer *peer, struct softbus_source *source, struct ubuf *ub)
{
    /* truncate string */
    ULOG_DEBUG("%s ->\n", source->addr);
    /* encode data base source */
    return icc_stream_write(ubuf_data_pointer(ub), ubuf_used_len(ub));
}

void ipcc_reconnect()
{
    int fd;

    /* should not block here, use unblock sock instread */
    memset(&sfd, 0, sizeof(sfd));
    ipcc = ipcc_connect(&ipc_ctx, IPC_NAME);

    if (ipcc == NULL) {
        ULOG_ERR("ipcc connects to controller failed!\n");
        ipcc_defer_reconnect();
        return;
    }

    fd = ipc_get_fd(ipcc);
    if (fd < 0) {
        ULOG_ERR("ipcc error fd: %d\n", fd);
        ipcc_defer_reconnect();
        return;
    }

    us = &sfd.stream;
    us->notify_read = ustream_read_cb;
    us->notify_write = ustream_write_cb;
    us->notify_state = ustream_state_cb;
    us->string_data = false;
    ustream_fd_init(&sfd, fd);

    ULOG_INFO("icc connected\n");
}


int icc_init(void)
{
    peer = softbus_peer_new("ipcc");
    softbus_peer_add_source(peer, "ipc", 0);
    softbus_peer_add_sink(peer, "all", 0, icc_sink);
    softbus_peer_register(peer);

    /* initialization befor using it */
    memset(&ipcc_reconnect_timeout, 0, sizeof(ipcc_reconnect_timeout));
    ipcc_reconnect_timeout.cb = ipcc_reconnect;

    ipcc_reconnect();

    return 0;
}
