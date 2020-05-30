
#ifndef _GLOBALS_H
#define _GLOBALS_H

#define HAS_DISPLAY U8G2_SSD1306_128X64_NONAME_F_HW_I2C

#define USE_WIFI 1
#define USE_BME280 0
#define USE_CAYENNE 0
#define USE_MQTT 0

#define HAS_PMU 0
#define HAS_GPS 0
#define USE_INA 1

#define display_refresh 10      // every second

#define ESP_SLEEP 0            // Main switch
#define TIME_TO_SLEEP 1         // sleep for 1 minute
#define TIME_TO_NEXT_SLEEP  5      // sleep after n minutes or
#define USE_OTA 1

#include <Arduino.h>
#include <SPI.h>
#include <Ticker.h>
#include "esp_sleep.h"
#include <Wire.h>
#include "WiFi.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "esp_log.h"
#include <Preferences.h>
#include <ESP32Servo.h>
#include <driver/gpio.h>

#include "driver/pcnt.h"

//--------------------------------------------------------
// Time Server
//--------------------------------------------------------
#include <time.h>
#define ntpServer "pool.ntp.org"
#define gmtOffset_sec 3600
#define daylightOffset_sec 3600

#include <SimpleButton.h> 
using namespace simplebutton;

typedef struct {
  float iaq;             // IAQ signal
  uint8_t iaq_accuracy;  // accuracy of IAQ signal
  float temperature;     // temperature signal
  float humidity;        // humidity signal
  float pressure;        // pressure signal
  float raw_temperature; // raw temperature signal
  float raw_humidity;    // raw humidity signal
  float gas;             // raw gas sensor signal
  uint8_t aliveCounter;   // aliveCounter  
  uint8_t sleepCounter;   // aliveCounter 
  uint8_t txCounter;   // aliveCounter    
  uint8_t bytesReceived;   
  uint16_t bat_voltage = 0;
  double  sun_azimuth;
  double  sun_elevation;
  float busvoltage1 = 0;
  float current_1 = 0;
  float current_2 = 0;
  float current_3 = 0;
  tm timeinfo;
} deviceStatus_t;


extern int runmode;

#include "jsutilities.h"
#include "display.h"
#include "gps.h"
#include "Helios.h"


#if (USE_MQTT)
#include <PubSubClient.h>
#endif

#endif