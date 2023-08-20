// Rob Latour, 2023
// License: MIT
// https://github.com/roblatour
//
// This program's key setting can be viewed/updated in the file:
//      ESP32TimeServerKeySettings.h
//
// https://time.is may be used a reference point to confirm your computer's
// date/time accuracy
//

// board: Olimex ESP32 wrover kit
#include <TimeLib.h>
#include <WiFi.h>

#include "../include/config.h "
#include "../include/gps.h"
#include "../include/mqtt.h"
#include "../include/wifiStuff.h"

#include <ESP32Time.h>
#include <Timezone.h>

// ESP32Time real time clock
ESP32Time rtc(0);

// Ethernet
bool eth_connected = false;
bool eth_got_IP = false;
String ip = "";

// GPS
// volatile bool ppsFlag; // GPS one-pulse-per-second flag
// !!!!!!!!!!!!! TODO set interupts on this
TinyGsm modem(SerialAT);

// TimeZone
TimeChangeRule usEDT = {"EDT", Second, Sun, Mar, 2, -240};
TimeChangeRule usEST = {"EST", First, Sun, Nov, 2, -300};
Timezone usEastern(usEDT, usEST);
TimeChangeRule *tcr;

// NTP port and packet buffer
#define NTP_PACKET_SIZE 48
byte packetBuffer[NTP_PACKET_SIZE];
WiFiUDP Udp;

// Constants and global variables
const unsigned long oneSecond_inMilliseconds =
    1000; // one second in milliseconds
const unsigned long oneMinute_inMilliseconds =
    60 * oneSecond_inMilliseconds; // one minute in milliseconds
const unsigned long thirtyMinutes_inMilliseconds =
    30 * oneMinute_inMilliseconds; // 30 minutes in milliseconds
const long oneSecond_inMicroseconds_L =
    1000000; // one second in microseconds (signed long)
const double oneSecond_inMicroseconds_D =
    1000000.0; // one second in microseconds (double)
               //
const unsigned long periodicTimeRefreshPeriod =
    thirtyMinutes_inMilliseconds; // how often the system's real time clock is
                                  // refreshed with GPS data
const time_t safeguardThresholdInSeconds =
    1; // used to ensure a GPS time refresh is only performed if the difference
       // between the old and new times is this many seconds or less
volatile bool SafeGuardTripped =
    false; // used to ensure the time isn't changed beyond that which would
           // reasonably be expected within the periodicTimeRefreshPeriod
           //
volatile bool theTimeSettingProcessIsUnderway; // signifies when the time is
                                               // being set / refreshed
                                               //
SemaphoreHandle_t mutex; // used to ensure an NTP request results are not
                         // impacted by the process that refreshes the time
                         //

// ----------- GLOBALS for gps data
float lat_data = 0;
float lon_data = 0;
float speed_data = 0;
float alt_data = 0;
int vsat_data = 0;
int usat_data = 0;
float accuracy_data = 0;
int year_data = 0;
int month_data = 0;
int day_data = 0;
int hour_data = 0;
int min_data = 0;
int sec_data = 0;

float location_ret[2] = {0, 0};

//**************************************************************************************************************************

void GetAdjustedDateAndTimeStrings(time_t UTC_Time, String &dateString,
                                   String &timeString) {

  // adjust utc time to local time
  time_t now_Local_Time = usEastern.toLocal(UTC_Time, &tcr);

  // format dateLine

  dateString = String(year(now_Local_Time));

  dateString.concat("-");

  if (month(now_Local_Time) < 10)
    dateString.concat("0");

  dateString.concat(String(month(now_Local_Time)));

  dateString.concat("-");

  if (day(now_Local_Time) < 10)
    dateString.concat("0");

  dateString.concat(String(day(now_Local_Time)));

  // format timeLine

  timeString = String(hourFormat12(now_Local_Time));

  timeString.concat(":");

  if (minute(now_Local_Time) < 10)
    timeString.concat("0");

  timeString.concat(String(minute(now_Local_Time)));

  timeString.concat(":");

  if (second(now_Local_Time) < 10)
    timeString.concat("0");

  timeString.concat(String(second(now_Local_Time)));

  if (isAM(now_Local_Time))
    timeString.concat(" AM");
  else
    timeString.concat(" PM");
};

String GetUpTime() {

  unsigned long ms = millis();

  const int oneSecond = 1000;
  const int oneMinute = oneSecond * 60;
  const int oneHour = oneMinute * 60;
  const int oneDay = oneHour * 24;

  int numberOfDays = ms / oneDay;
  ms = ms - numberOfDays * oneDay;

  int numberOfHours = ms / oneHour;
  ms = ms - numberOfHours * oneHour;

  int numberOfMinutes = ms / oneMinute;
  ms = ms - numberOfMinutes * oneMinute;

  int numberOfSeconds = ms / oneSecond;

  String returnValue = "";

  char buffer[21];

  sprintf(buffer, "%d %02d:%02d:%02d", numberOfDays, numberOfHours,
          numberOfMinutes, numberOfSeconds);

  returnValue = String(buffer);
  return returnValue;
}

unsigned long getTimeEpooche() { return rtc.getEpoch(); }

String getFormatedTimestamp() {
  String outStr = rtc.getTime("%Y-%m-%d %H:%M:%S");
  outStr += "." + String(rtc.getMicros());
  return outStr;
}

// TODO
// !!!!!!!!!!!!!! change this function
void setDateAndTimeFromGPS(TimerHandle_t xTimer) {
  setDateAndTimeFromGPSInside();
}

void setDateAndTimeFromGPSInside() {

  static bool thisIsTheFirstTimeSetBeingMadeAtStartup = true;

  /// used below to ensure a GPS time refresh if is only performed if the
  /// difference between the old and new times is reasonable for the
  /// periodicTimeRefreshPeriod
  const time_t safeguardThresholdHigh = safeguardThresholdInSeconds;
  const time_t safeguardThresholdLow = -1 * safeguardThresholdInSeconds;

  time_t candidateDateAndTime;

  if (debugIsOn)
    SerialMon.println("Start setDateAndTimeFromGPS task");

  // mqtt_mess_send(TOPIC_INFO, "getting time from GPS");

  if (debugIsOn)
    SerialMon.println("Sent mqtt mess");

  int retry_num;
  for (retry_num = 0; retry_num < GPS_RETRIES; retry_num++) {

    theTimeSettingProcessIsUnderway = true;

    // modem propbly handles this - nie ważen ??
    // wait for the ppsFlag to be raised at the start of the 1st second
    // ppsFlag = false;
    // while (!ppsFlag)
    //   ;

    if (!modem.getGPS(&lat_data, &lon_data, &speed_data, &alt_data, &vsat_data,
                      &usat_data, &accuracy_data, &year_data, &month_data,
                      &day_data, &hour_data, &min_data, &sec_data)) {
      // couldnt get data, retry
      if (debugIsOn)
        SerialMon.println("GPS data not available, retrying in 5 seconds");
      SerialMon.println(modem.getGPSraw());
      // mqtt_mess_send(TOPIC_INFO,
      //                "GPS data not available, retrying in 5 seconds");
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    struct tm tmp_time_holder;
    tmp_time_holder.tm_year = year_data;
    tmp_time_holder.tm_mon = month_data;
    tmp_time_holder.tm_mday = day_data;
    tmp_time_holder.tm_hour = hour_data;
    tmp_time_holder.tm_min = min_data;
    tmp_time_holder.tm_sec = sec_data;

    // make sure the values are within reason
    //  if values are nonesenical, retry
    if (!((tmp_time_holder.tm_year > 2022) && (tmp_time_holder.tm_mon > 0) &&
          (tmp_time_holder.tm_mon < 13) && (tmp_time_holder.tm_mday > 0) &&
          (tmp_time_holder.tm_mday < 32) && (tmp_time_holder.tm_hour < 24) &&
          (tmp_time_holder.tm_min < 60) && (tmp_time_holder.tm_sec < 61))) {
      if (debugIsOn)
        Serial.println("GPS time values are nonsensical, retrying");
      continue;
    }

    // set candidate time according the gps (this will be effective when
    // the PPS flag is next raised)
    tmp_time_holder.tm_year -= 1900; // adjust year (see you again in 2036)
    tmp_time_holder.tm_mon -= 1;     // adjust month (January is month 0)
    // not sure why the + 1 but it is !!!!!!!!! dont think so - one sec ahead of
    // time
    candidateDateAndTime = mktime(&tmp_time_holder) + 1;

    if (debugIsOn)
      Serial.println("Candidate date and time " +
                     String(tmp_time_holder.tm_year) + " " +
                     String(tmp_time_holder.tm_mon) + " " +
                     String(tmp_time_holder.tm_mday) + " " +
                     String(tmp_time_holder.tm_hour) + " " +
                     String(tmp_time_holder.tm_min) + " " +
                     String(tmp_time_holder.tm_sec));

    time_t wt_t = candidateDateAndTime;
    time_t candiateDateAndTime_t = time(&wt_t);

    // ============ nie ważne  ???? ============ // TODO
    // // give some time to ensure the PPS pin is reset
    // // wait for the PPS flag to be raised (signifying the true start of
    // // the candidate time)
    // ppsFlag = false;
    // while (!ppsFlag)
    //   ;

    // wait for next pulse GPS - how ???
    // opcja 1
    vTaskDelay(pdMS_TO_TICKS(500));
    while (!modem.getGPSTime(&year_data, &month_data, &day_data, &hour_data,
                             &min_data, &sec_data)) {
      SerialMon.println("Waiting for GPS time");
      ;
    }
    // moze to czeka a nie zwraca wartosc cachowana

    unsigned long pegProcessingAdjustmentStartTime = micros();

    // at this point:
    // apply a sanity check; the current rtc time and the candidate time
    // just taken from the gps readings which will be used to refresh the
    // current rtc should be within a second of each other
    // (safeguardThresholdInSeconds) if the sanity check fails, do not set
    // the time and raise a Safeguard flag which be used to update the
    // display to show the user the latest time refresh failed if the
    // sanity check passes, proceed with refreshing the time and if the
    // Safeguard flag been previously been raised then lower it

    bool SanityCheckPassed;
    time_t updateDelta;

    if (thisIsTheFirstTimeSetBeingMadeAtStartup) {
      SanityCheckPassed = true;
    } else {
      time_t currentRTC_t = rtc.getEpoch();
      time_t currentRTCDateAndTime_t = time(&currentRTC_t);
      updateDelta = currentRTCDateAndTime_t - candiateDateAndTime_t;
      bool SanityCheckPassed = (((updateDelta >= safeguardThresholdLow) &&
                                 (updateDelta <= safeguardThresholdHigh)));
    }

    if (!SanityCheckPassed) {
      if (debugIsOn) {
        Serial.println("This date and time refresh failed its sanity "
                       "check with a time delta of " +
                       String(updateDelta) + " seconds");
        Serial.println("The time was not refreshed.");
        Serial.print("Date and time are ");
        String ws = rtc.getDateTime(true);
        ws.trim();
        Serial.println(ws + " (UTC)");
        Serial.println("Will try again");
      }

      SafeGuardTripped = true;
      continue;
    }
    // place a hold on (the date and time) so if an NTP request is
    // underway in the fraction of a second this code will take, the
    // time and date values don't change mid way through that request.
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {

      // set the date and time
      unsigned long pegProcessingAdjustmentEndTime = micros();
      unsigned long ProcessingAdjustment =
          pegProcessingAdjustmentEndTime - pegProcessingAdjustmentStartTime;

      // set the real time clock
      rtc.setTime((unsigned long)candidateDateAndTime,
                  (int)ProcessingAdjustment);

      // release the hold
      xSemaphoreGive(mutex);

      if (debugIsOn) {
        Serial.print("Date and time set to ");
        String ws = rtc.getDateTime(true);
        ws.trim();
        Serial.println(ws + " (UTC)");
      };

      SafeGuardTripped = false;
      theTimeSettingProcessIsUnderway = false;
      thisIsTheFirstTimeSetBeingMadeAtStartup = false;

      // whew that was hard work but fun, lets take a break and then do
      // it all again
      // vTaskDelay(periodicTimeRefreshPeriod / portTICK_PERIOD_MS);
      // just return all done
      return;

    } else {
      if (debugIsOn) {
        Serial.println(
            "Could not refresh the time as a NTP request was underway");
        Serial.println("Will try again");
      }
    } // end of if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
  }   // end of for loop

  if (retry_num == GPS_RETRIES) {
    if (debugIsOn)
      SerialMon.println("GPS failed to get a fix");
    // send info over mqtt
    // mqtt_mess_send("ERROR: GPS failed to get a fix", TOPIC_INFO);
  } else {
    // set gps data
    location_ret[0] = lat_data;
    location_ret[1] = lon_data;
    // send info over mqtt
    // mqtt_mess_send("GPS data set", TOPIC_INFO);
  }

  // todo change so params can be sent
  // if ((int)parameter & 0x01) {
  //   // send location data
  //   if (debugIsOn)
  //     SerialMon.println("Sending location data");
  //   mqtt_mess_send("Sending location data", TOPIC_INFO);
  //   mqtt_mess_send(
  //       String(String(lat_data, 6) + ", " + String(lon_data, 6)).c_str(),
  //       TOPIC_LOCATION);
  // }
}

// !!!!!!!!!!!!!!!!!!!!!!!!!! importnent !!!!!!!!!!!
void startUDPSever() {
  SerialMon.println("udp Udp");
  Udp.begin(WiFi.localIP(), NTP_PORT);
}

void stopUdpServer() { Udp.stop(); }

uint64_t getCurrentTimeInNTP64BitFormat() {

  const uint64_t numberOfSecondsBetween1900and1970 = 2208988800;

  uint64_t clockSecondsSinceEpoch =
      numberOfSecondsBetween1900and1970 + (uint64_t)rtc.getEpoch();
  long clockMicroSeconds = (long)rtc.getMicros();

  // as one might infer clockMicroSeconds is in microseconds (i.e. 1 second =
  // 1,000,000 microseconds)
  //
  // accordingly, if the clockMicroSeconds is greater than one million ...
  //   for every million that is over:
  //     add 1 (second) to clockSecondsSinceEpoch, and
  //     reduce the clockMicroSeconds by one million (microseconds)
  //
  // likewise ...
  //
  // if the clockMicroSeconds is less than zero:
  //   for every million that is under zero:
  //     subtract (second) from clockSecondsSinceEpoch, and
  //     increase the clockMicroSeconds by one million (microseconds)

  while (clockMicroSeconds > oneSecond_inMicroseconds_L) {
    clockSecondsSinceEpoch++;
    clockMicroSeconds -= oneSecond_inMicroseconds_L;
  };

  while (clockMicroSeconds < 0L) {
    clockSecondsSinceEpoch--;
    clockMicroSeconds += oneSecond_inMicroseconds_L;
  };

  // for the next two lines to be clear, please see:
  // https://tickelton.gitlab.io/articles/ntp-timestamps/

  double clockMicroSeconds_D =
      (double)clockMicroSeconds * (double)(4294.967296);
  uint64_t ntpts = ((uint64_t)clockSecondsSinceEpoch << 32) |
                   (uint64_t)(clockMicroSeconds_D);

  return ntpts;
}

// send NTP reply
void sendNTPpacket(IPAddress remoteIP, int remotePort) {

  // set the receive time to the current time
  uint64_t receiveTime_uint64_t = getCurrentTimeInNTP64BitFormat();

  // Initialize values needed to form NTP request

  // LI: 0, Version: 4, Mode: 4 (server)
  // packetBuffer[0] = 0b00100100;
  // LI: 0, Version: 3, Mode: 4 (server)
  packetBuffer[0] = 0b00011100;

  // Stratum, or type of clock
  packetBuffer[1] = 0b00000001;

  // Polling Interval
  packetBuffer[2] = 4;

  // Peer Clock Precision
  // log2(sec)
  // 0xF6 <--> -10 <--> 0.0009765625 s
  // 0xF7 <--> -9 <--> 0.001953125 s
  // 0xF8 <--> -8 <--> 0.00390625 s
  // 0xF9 <--> -7 <--> 0.0078125 s
  // 0xFA <--> -6 <--> 0.0156250 s
  // 0xFB <--> -5 <--> 0.0312500 s
  packetBuffer[3] = 0xF7;

  // 8 bytes for Root Delay & Root Dispersion
  // root delay
  packetBuffer[4] = 0;
  packetBuffer[5] = 0;
  packetBuffer[6] = 0;
  packetBuffer[7] = 0;

  // root dispersion
  packetBuffer[8] = 0;
  packetBuffer[9] = 0;
  packetBuffer[10] = 0;
  packetBuffer[11] = 0x50;

  // time source (namestring)
  packetBuffer[12] = 71; // G
  packetBuffer[13] = 80; // P
  packetBuffer[14] = 83; // S
  packetBuffer[15] = 0;

  // get the current time and write it out as the reference time to bytes 16
  // to 23 of the response packet
  uint64_t referenceTime_uint64_t = getCurrentTimeInNTP64BitFormat();

  packetBuffer[16] = (int)((referenceTime_uint64_t >> 56) & 0xFF);
  packetBuffer[17] = (int)((referenceTime_uint64_t >> 48) & 0xFF);
  packetBuffer[18] = (int)((referenceTime_uint64_t >> 40) & 0xFF);
  packetBuffer[19] = (int)((referenceTime_uint64_t >> 32) & 0xFF);
  packetBuffer[20] = (int)((referenceTime_uint64_t >> 24) & 0xFF);
  packetBuffer[21] = (int)((referenceTime_uint64_t >> 16) & 0xFF);
  packetBuffer[22] = (int)((referenceTime_uint64_t >> 8) & 0xFF);
  packetBuffer[23] = (int)(referenceTime_uint64_t & 0xFF);

  // copy transmit time from the NTP original request to bytes 24 to 31 of the
  // response packet
  packetBuffer[24] = packetBuffer[40];
  packetBuffer[25] = packetBuffer[41];
  packetBuffer[26] = packetBuffer[42];
  packetBuffer[27] = packetBuffer[43];
  packetBuffer[28] = packetBuffer[44];
  packetBuffer[29] = packetBuffer[45];
  packetBuffer[30] = packetBuffer[46];
  packetBuffer[31] = packetBuffer[47];

  // write out the receive time (it was set above) to bytes 32 to 39 of the
  // response packet
  packetBuffer[32] = (int)((receiveTime_uint64_t >> 56) & 0xFF);
  packetBuffer[33] = (int)((receiveTime_uint64_t >> 48) & 0xFF);
  packetBuffer[34] = (int)((receiveTime_uint64_t >> 40) & 0xFF);
  packetBuffer[35] = (int)((receiveTime_uint64_t >> 32) & 0xFF);
  packetBuffer[36] = (int)((receiveTime_uint64_t >> 24) & 0xFF);
  packetBuffer[37] = (int)((receiveTime_uint64_t >> 16) & 0xFF);
  packetBuffer[38] = (int)((receiveTime_uint64_t >> 8) & 0xFF);
  packetBuffer[39] = (int)(receiveTime_uint64_t & 0xFF);

  // get the current time and write it out as the transmit time to bytes 40 to
  // 47 of the response packet
  uint64_t transmitTime_uint64_t = getCurrentTimeInNTP64BitFormat();

  packetBuffer[40] = (int)((transmitTime_uint64_t >> 56) & 0xFF);
  packetBuffer[41] = (int)((transmitTime_uint64_t >> 48) & 0xFF);
  packetBuffer[42] = (int)((transmitTime_uint64_t >> 40) & 0xFF);
  packetBuffer[43] = (int)((transmitTime_uint64_t >> 32) & 0xFF);
  packetBuffer[44] = (int)((transmitTime_uint64_t >> 24) & 0xFF);
  packetBuffer[45] = (int)((transmitTime_uint64_t >> 16) & 0xFF);
  packetBuffer[46] = (int)((transmitTime_uint64_t >> 8) & 0xFF);
  packetBuffer[47] = (int)(transmitTime_uint64_t & 0xFF);

  // send the reply
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void processNTPRequests(void *pvParameters) {
  int ntpRequestCount = 0;

  while (true) {
    ntpRequestCount++;
    if (ntpRequestCount % 100000 == 0)
      SerialMon.println("                     waiting for ntp packet");
    unsigned long replyStartTime = micros();
    int packetSize = Udp.parsePacket();

    if (packetSize != NTP_PACKET_SIZE) {
      if (packetSize > 0) {
        SerialMon.print("received packet of size ");
        SerialMon.println(packetSize);
        Udp.flush(); // not sure what this incoming packet is, but it is not
                     // an ntp request so get rid of it
      };
      continue;
    }
    SerialMon.println("got ntp packet");

    // store sender ip for later use
    IPAddress remoteIP = Udp.remoteIP();

    // read the data from the packet into the buffer for later use
    Udp.read(packetBuffer, NTP_PACKET_SIZE);

    // hold here if and while the date and time are being refreshed
    // when ok to proceed place a hold on using the mutex to prevent the date
    // and time from being refreshed while the reply packet is being built
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
      // send NTP reply
      sendNTPpacket(remoteIP, Udp.remotePort());
      xSemaphoreGive(mutex);
    }
  }
}

void setupForNtp() {

  if (debugIsOn)
    Serial.println("ESP32 Time Server starting setup ...");

  //  ------------ setup gps ---------------- //
  // Turn on the modem
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(300);
  digitalWrite(PWR_PIN, LOW);

  // Set module baud rate and UART pins
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  if (debugIsOn)
    SerialMon.println("Initializing modem...");

  if (!modem.init()) {
    if (debugIsOn)
      Serial.println(
          "Failed to init modem, attempting to continue without restarting");
  }

  if (debugIsOn) {
    // Print modem info
    String modemName = modem.getModemName();
    delay(500);
    SerialMon.println("Modem Name: " + modemName);

    String modemInfo = modem.getModemInfo();
    delay(500);
    SerialMon.println("Modem Info: " + modemInfo);
  }

  // power up the modem (?)
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    SerialMon.println(" SGPIO=0,4,1,1 false ");
  }
  modem.enableGPS();
  //  ------------ end of gps setup ---------------- //

  // create a mutex to be used to ensure an NTP request results are not
  // impacted by the process that refreshes the time
  mutex = xSemaphoreCreateMutex();

  // ================ to chyba nie prztdatne =================== //
  // setup for the use of the pulse-per-second pin
  //   pinMode(PPSPin, INPUT_PULLUP); // change to freeRTOS interupt TODO
  //   attachInterrupt(digitalPinToInterrupt(PPSPin), ppsHandlerRising,
  //   RISING);

  // IMPORTANT - still need to call
  // processNTPRequests() in the loop() function

  // createing the FreeRTOS tasks - done in main
}

void turnOffGPS() {
  modem.disableGPS();

  // Set SIM7000G GPIO4 LOW ,turn off GPS power
  // CMD:AT+SGPIO=0,4,1,0
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+SGPIO=0,4,1,0");
  if (modem.waitResponse(10000L) != 1) {
    SerialMon.println(" SGPIO=0,4,1,0 false ");
  }
}