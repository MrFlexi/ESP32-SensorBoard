
#ifndef _GLOBALS_H
#define _GLOBALS_H

#define USE_WIFI 1
#define USE_BME280 1
#define USE_CAYENNE 0
#define USE_MQTT 1

#define USE_MOTOR 0
#define USE_PULS_COUNTER 0
#define USE_SERVO 0

#define USE_FAN_PWM 1
#define FAN_PWM_PIN GPIO_NUM_27

#define HAS_PMU 0
#define HAS_GPS 0
#define USE_INA 1

#define display_refresh 10      // every second

#define ESP_SLEEP 0           // Main switch
#define TIME_TO_SLEEP 5        // sleep for n minute
#define TIME_TO_NEXT_SLEEP  5      // sleep after n minutes or


// Modor Shield 3 Ampere
#define MotorALeft_pin    GPIO_NUM_25
#define MotorARight_pin   GPIO_NUM_26
#define SERVO_PIN         GPIO_NUM_27

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
#include <driver/gpio.h>
#include "driver/pcnt.h"
#include "ESP32MotorControl.h"

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
extern bool I2C_lock;

#include "jsutilities.h"
#include "display.h"
#include "gps.h"
//#include "Helios.h"
#include "ServoEasing.h"
#include "motor.h"


#if (USE_MQTT)
#include <PubSubClient.h>
#endif


#if (USE_FAN_PWM)
#include <fanPWM.h>
#endif



#endif