#ifndef __GPS_H__
#define __GPS_H__

#include <Arduino.h>
#include <TinyGsmClient.h>

extern TinyGsm modem;

void setDateAndTimeFromGPS(TimerHandle_t xTimer);
void setDateAndTimeFromGPSInside();

// needs to be running to process NTP requests
void processNTPRequests(void *pvParameters);
void setupForNtp();
void turnOffGPS();
void startUDPSever();
void stopUdpServer();
unsigned long getTimeEpooche();
String getFormatedTimestamp();

#endif // __GPS_H__