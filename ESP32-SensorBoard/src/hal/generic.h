// clang-format off
// upload_speed 115200
// board esp32dev

#ifndef _GENERIC_H
#define _GENERIC_H

#include <stdint.h>

// Hardware related definitions for generic ESP32 boards
// generic.h is kitchensink with all available options

#define SPI_MOSI GPIO_NUM_23
#define SPI_MISO GPIO_NUM_19
#define SPI_SCLK GPIO_NUM_18
#define SPI_CS   GPIO_NUM_5

// enable only if device has these sensors, otherwise comment these lines
// tutorial to connect BME sensor see here:
// https://sbamueller.wordpress.com/2019/02/26/paxcounter-mit-umweltsensor/
//
// in platformio.ini append
// lib_deps = <...> ${common.lib_deps_sensors}
// for loading necessary libraries

// BME680 sensor on I2C bus
#define HAS_BME 1 // Enable BME sensors in general
//#define HAS_BME680 GPIO_NUM_21, GPIO_NUM_22 // SDA, SCL
//#define BME680_ADDR BME680_I2C_ADDR_PRIMARY // connect SDIO of BME680 to GND

// BME280 sensor on I2C bus
#define HAS_BME 1 // Enable BME sensors in general
#define HAS_BME280 SDA, SCL // SDA, SCL
#define BME280_ADDR 0x76 // change to 0x77 depending on your wiring

// user defined sensors
//#define HAS_SENSORS 1 // comment out if device has user defined sensors

#define LED_BUILTIN 2

//#define BOARD_HAS_PSRAM // use if board has external PSRAM
//#define DISABLE_BROWNOUT 1 // comment out if you want to keep brownout feature

//#define HAS_DISPLAY U8X8_SSD1306_128X64_NONAME_HW_I2C
//#define DISPLAY_FLIP  1 // use if display is rotated
//#define BAT_MEASURE_ADC ADC1_GPIO35_CHANNEL // battery probe GPIO pin -> ADC1_CHANNEL_7
//#define BAT_VOLTAGE_DIVIDER 2 // voltage divider 100k/100k on board

//#define HAS_LED (21) // on board  LED
//#define HAS_BUTTON (39) // on board button
//#define HAS_RGB_LED SmartLed rgb_led(LED_WS2812, 1, GPIO_NUM_0) // WS2812B RGB LED on GPIO0

// GPS settings
#define HAS_GPS 0 // use on board GPS
#define GPS_RX GPIO_NUM_16
#define GPS_TX GPIO_NUM_15
#define GPS_INT GPIO_NUM_13 // 30ns accurary timepulse, to be external wired on pcb: NEO 6M Pin#3 -> GPIO13

// Pins for I2C interface of OLED Display
#define MY_OLED_SDA (4)
#define MY_OLED_SCL (15)
#define MY_OLED_RST (16)

// Settings for on board DS3231 RTC chip
#define HAS_RTC MY_OLED_SDA, MY_OLED_SCL // SDA, SCL
#define RTC_INT GPIO_NUM_34 // timepulse with accuracy +/- 2*e-6 [microseconds] = 0,1728sec / day

// Settings for IF482 interface
//#define HAS_IF482 9600, SERIAL_7E1, GPIO_NUM_12, GPIO_NUM_14 // IF482 serial port parameters

// Settings for DCF77 interface
//#define HAS_DCF77 GPIO_NUM_1
//#define DCF77_ACTIVE_LOW 1


#endif