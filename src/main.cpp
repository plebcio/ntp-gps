#include "../include/config.h"
#include "../include/gps.h"
#include "../include/mqtt.h"
#include "../include/wifiStuff.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <TinyGsmClient.h>
#include <string>

// for json
DynamicJsonDocument doc(1024);

volatile int mqttCallbackToggles = 0;

TimerHandle_t gpsDataFetchTimer, mqttCheckTimer;
TaskHandle_t ntpServerHandle;

int mqtt_counter_location = 0;
int mqtt_counter_voltage = 0;

void mqtt_check_mess_wrapper(TimerHandle_t xTimer) {
  // suspend ntp task
  // vTaskSuspend(ntpServerHandle);
  // SerialMon.println("Suspended ntp task");
  // start mqtt server

  // stopUdpServer();
  // setupMQTT();

  if (debugIsOn)
    SerialMon.println("mqtt should is live");

  mqtt_check_for_mess();

  doc["timstamp"] = getFormatedTimestamp();
  doc["counter"] = mqtt_counter_location++;
  doc["data"][0] = location_ret[0];
  doc["data"][1] = location_ret[1];
  String outStr = "";
  serializeJson(doc, outStr);
  if (debugIsOn) {
    SerialMon.println("sending loc");
    SerialMon.println(outStr);
  }
  mqtt_mess_send(outStr.c_str(), TOPIC_GPS);

  doc["timstamp"] = getFormatedTimestamp();
  doc["counter"] = mqtt_counter_voltage++;
  // doc["data"] = modem.getBattVoltage();

  outStr = "";
  serializeJson(doc, outStr);
  if (debugIsOn) {
    SerialMon.println("sending vol");
    SerialMon.println(outStr);
  }
  mqtt_mess_send(outStr.c_str(), TOPIC_VOLTAGE);

  // check for messages
  // mqtt_check_for_mess();

  // stop mqtt server
  // stopMQTT();

  // startUDPSever();

  // resume ntp task
  // vTaskResume(ntpServerHandle);
}

void setup() {

  if (debugIsOn) {
    SerialMon.begin(115200);
    SerialMon.println("Place your board outside to catch satelite signal");
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // setup wifi
  setupWifi();

  // set up gps
  setupForNtp();

  // launch server
  startUDPSever();

  // initial setup of date and time
  setDateAndTimeFromGPSInside();

  // task to be set up:
  // 1. start a task to refresh the date and time from the gps
  gpsDataFetchTimer =
      xTimerCreate("gpsDataFetchTimer", pdMS_TO_TICKS(NTP_REFRESH_TIME_MS),
                   pdTRUE, 0, setDateAndTimeFromGPS);

  SerialMon.println("Created timer for gps data fetch");

  if (gpsDataFetchTimer == NULL) {
    // panic and restart
    SerialMon.println("Could not create timer for gps data fetch");
    ESP.restart();
  }

  if (xTimerStart(gpsDataFetchTimer, pdMS_TO_TICKS(5000)) != pdPASS) {
    // panic and restart
    SerialMon.println("Could not start timer for gps data fetch");
    ESP.restart();
  }

  // 2. start a task to process ntp requests
  xTaskCreate(processNTPRequests, "processNTPRequests", 5000, NULL, 1,
              &ntpServerHandle);

  // 3. start a task to process mqtt messages
  mqttCheckTimer =
      xTimerCreate("mqttCheckTimer", pdMS_TO_TICKS(MQTT_COMMAND_CHECK_TIME_MS),
                   pdTRUE, (void *)0, mqtt_check_mess_wrapper);

  if (mqttCheckTimer == NULL) {
    // panic and restart
    SerialMon.println("Could not create timer for mqtt check");
    ESP.restart();
  }

  setupMQTT();

  // start mqtt timer
  if (xTimerStart(mqttCheckTimer, pdMS_TO_TICKS(5000)) != pdPASS) {
    // panic and restart
    SerialMon.println("Could not start timer for mqtt check");
    ESP.restart();
  }

  if (debugIsOn)
    SerialMon.println("started mqtt -- everything is on");
}

void loop() {}