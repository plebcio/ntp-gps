#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>

#include "../include/config.h"
#include "../include/wifiStuff.h"

#include <string>

// global message from gps
char *message = NULL;

// Set your Static IP address
// IPAddress local_IP(STATIC_IP);
// Set your Gateway IP address
// IPAddress gateway(STATIC_IP);

// IPAddress subnet(255, 255, 0, 0);
// IPAddress primaryDNS(8, 8, 8, 8);   // optional
// IPAddress secondaryDNS(8, 8, 4, 4); // optional

void setupWifi(void) {

  // WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  SerialMon.println("");
  SerialMon.print("Connected to ");
  SerialMon.println(ssid);
  SerialMon.print("IP address: ");
  SerialMon.println(WiFi.localIP());
}