#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include <uci.h>
#include <libubox/ulog.h>
#include <libubox/blobmsg_json.h>

#include "config.h"
const char *config_path = DEFAULT_CONFIG_PATH;


static struct uci_context *uci_ctx;
static struct uci_package *uci_controller;
static struct blob_buf b;

static struct uci_package *
config_init_package(const char *config)
{
	struct uci_context *ctx = uci_ctx;
	struct uci_package *p = NULL;

	if (!ctx) {
		ctx = uci_alloc_context();
		uci_ctx = ctx;

		ctx->flags &= ~UCI_FLAG_STRICT;
		if (config_path)
			uci_set_confdir(ctx, config_path);

#ifdef DUMMY_MODE
		uci_set_savedir(ctx, "./tmp");
#endif
	} else {
		p = uci_lookup_package(ctx, config);
		if (p)
			uci_unload(ctx, p);
	}

	if (uci_load(ctx, config, &p))
		return NULL;

	return p;
}

void
config_get_access_point(char *ip, int *port)
{
	struct uci_element *e;
    const char *ip_str, *port_str;

	uci_foreach_element(&uci_controller->sections, e) {
		struct uci_section *s = uci_to_section(e);

		if (strcmp(s->type, "mqtt") != 0)
			continue;

		ip_str = uci_lookup_option_string(uci_ctx, s, "ip");
		if (ip)
            strcpy(ip, ip_str);

		port_str = uci_lookup_option_string(uci_ctx, s, "port");
        if (port_str)
            *port = atoi(port_str);
	}
}

int config_init(void)
{
	int ret = 0;
	char *err;

	uci_controller = config_init_package("controller");
	if (!uci_controller) {
		uci_get_errorstr(uci_ctx, &err, NULL);
		ULOG_ERR("Failed to load controller config (%s)\n", err);
		free(err);
		return -1;
	}

	return ret;
}
