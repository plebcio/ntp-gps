#include <PubSubClient.h>
#include <WiFi.h>
#include <string>

#include "../include/config.h"
#include "../include/gps.h"
#include "../include/mqtt.h"
#include "../include/wifiStuff.h"

int encoded_mqtt_data = 0;

// prototypes
void callback(char *topic, byte *payload, unsigned int length);

// MQTT client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setupMQTT() {
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  // set the callback function
  mqttClient.setCallback(callback);
}

void stopMQTT() { mqttClient.disconnect(); }

void reconnect() {
  if (debugIsOn)
    SerialMon.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
    if (debugIsOn)
      SerialMon.println("Reconnecting to MQTT Broker..");
    // String clientId = "ESP32Client-";
    // clientId += "1388";
    String clientId = MQTT_CLIENT_ID;

    if (mqttClient.connect(clientId.c_str())) {
      if (debugIsOn) {
        SerialMon.println("Connected.");
        SerialMon.print("Client ID: ");
        SerialMon.println(clientId);
      }
      // subscribe to topic
      mqttClient.subscribe(TOPIC_COMMANDS_ESP32);
      if (debugIsOn)
        SerialMon.println("Subscribed to topic of sort /ntpESP32/commands");
    }
  }
}

void mqtt_check_for_mess() {
  if (!mqttClient.connected())
    reconnect();

  // mqtt lib loop, not user defined
  mqttClient.loop();

  mqttClient.disconnect();
}

void mqtt_mess_send(const char *data, char *topic) {
  if (!mqttClient.connected())
    reconnect();

  // mqtt lib loop, not user defined
  mqttClient.loop();

  // publish to topic
  mqttClient.publish(topic, data);
}

void callback(char *topic, byte *payload, unsigned int length) {
  if (debugIsOn) {
    SerialMon.print("Callback - ");
    SerialMon.print("Message:");
  }
  char mess[length];
  memcpy(mess, payload, length);

  if (debugIsOn)
    SerialMon.println(mess);

  if (length < 2)
    return;

  switch (mess[0]) {
  case 'L':
    if (mess[1] == '1') {
      if (debugIsOn)
        SerialMon.println("Location enabled");
      encoded_mqtt_data |= 1; // enable location bit
    } else if (mess[1] == '0') {
      if (debugIsOn)
        SerialMon.println("Location disabled");
      encoded_mqtt_data &= (~1); // disable location bit
    } else {
      if (debugIsOn)
        SerialMon.println("Wrong command");
    }
    break;

  default:
    break;
  }

  mqtt_mess_send(mess, TOPIC_INFO);
}
