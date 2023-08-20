#ifndef __CONFIG_H__
#define __CONFIG_H__

#define debugIsOn true

// LilyGO T-SIM7000G Pinout
#define UART_BAUD 115200
#define PIN_DTR 25
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4

#define LED_PIN 12

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT Serial1

// // wifi definitions
#define ssid "Wifi123123"
#define password "Pomidory2137"

#define STATIC_IP 192, 168, 1, 184

// MQTT definitions
#define MQTT_SERVER "broker.hivemq.com"
#define MQTT_CLIENT_ID "ESP32Client-1388"
#define TOPIC_INFO "/ntpESP32/info"
#define TOPIC_GPS "/ntpESP32/gps"
#define TOPIC_VOLTAGE "/ntpESP32/voltage"

#define TOPIC_COMMANDS_ESP32 "/ntpESP32/commands"
#define TOPIC_LOCATION "/ntpESP32/location" // TOOD change

#define MQTT_COMMAND_CHECK_TIME_MS 30 * 1000
#define MQTT_PORT 1883

// ntp
#define NTP_PORT 123
#define GPS_RETRIES 15

// time between ntp refreshes
#define NTP_REFRESH_TIME_MS 60 * 1000

extern float location_ret[2];
extern int encoded_mqtt_data;

/* ==================== Definiton of Mqtt commands ====================
Fist byte of message is command
np:
'L' - moduł ma publikować dane o lokalizacji do opowiedniego topicu
    drugi char to 1 enable, 0 disable
inne TODO
*/

// FreeRTOS task config
// #define configUSE_TIMERS 1
// #define configTIMER_TASK_PRIORITY 5 // arbitrary
// #define configTIMER_TASK_STACK_DEPTH 10000

#endif // __CONFIG_H__