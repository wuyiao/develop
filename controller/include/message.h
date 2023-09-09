#ifndef CONTROLLER_MESSAGE_H
#define CONTROLLER_MESSAGE_H

enum {
    CC_RESET,
    CC_REBOOT,
};

void control_msg_handler(void *msg, int len);
void config_msg_handler(void *msg, int len);
#endif


