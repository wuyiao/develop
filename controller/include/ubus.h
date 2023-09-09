#ifndef CONTROLLER_UBUS_H
#define CONTROLLER_UBUS_H 
extern struct ubus_context *ubus_ctx;

int controller_ubus_init(const char *path);
void controller_ubus_done(void);
#endif

