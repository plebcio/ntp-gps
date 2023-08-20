#ifndef _MQTT_HPP_
#define _MQTT_HPP_

#include <Arduino.h>

void mqtt_mess_send(const char *data, char *topic);
void setupMQTT();
void stopMQTT();
void mqtt_check_for_mess();

#endif // _MQTT_HPP_