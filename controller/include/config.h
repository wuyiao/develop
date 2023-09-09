#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H
#define DEFAULT_CONFIG_PATH	"/etc/config"

int config_init(void);

void config_get_access_point(char *ip_str, int *port);
#endif
