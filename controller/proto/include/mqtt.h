#ifndef MQTT_H
#define MQTT_H

//#define EI_LINK "47.103.143.106"
#define EI_LINK "10.1.1.10"
int mqtt_init(void);
int mqtt_publish(char *topic, const void *payload, int length, int qos);
void mqtt_done(void);
#endif
