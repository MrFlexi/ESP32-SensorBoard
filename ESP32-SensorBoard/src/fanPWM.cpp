#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-ledc.h>
#include "globals.h"

int pwmValue = 0;

const int pwmFreq              = 25000;
const int pwmChannel           = 0;
const int pwmResolution        = 8;

// https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
void setup_PWMfan(void){
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(FAN_PWM_PIN, pwmChannel);

  pwmValue = 0;

  setFanSpeed(255);

}

void setFanSpeed(uint32_t pwmValue ){
  ledcWrite(pwmChannel, pwmValue);
}

