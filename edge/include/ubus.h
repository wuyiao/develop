#ifndef EDCD_UBUS_H
#define EDCD_UBUS_H
extern struct ubus_context *ubus_ctx;

int ubus_init(void);
void ubus_done(void);
#endif

