#include <Arduino.h>
#include "motor.h"

 ESP32MotorControl MotorControl = ESP32MotorControl();

void setup_motor()
{
  //pinMode(LED, OUTPUT);
  //pinMode(MotorALeft_pin, OUTPUT);
  //pinMode(MotorARight_pin, OUTPUT);

  //digitalWrite(MotorARight_pin, 0);
  //digitalWrite(MotorALeft_pin, 0);

MotorControl.attachMotor(MotorALeft_pin, MotorARight_pin);
MotorControl.motorsStop();
}

void setSpeedLeft(uint16_t iv_speed)
{
  MotorControl.motorForward(0, iv_speed);
}

void setSpeedRight(uint16_t iv_speed)
{
  MotorControl.motorReverse(0, iv_speed);
}

void setSpeedOff()
{
 MotorControl.motorsStop();
}

void motorA_fade()
{
  // fade in from min to max in increments of 5 points:
  for (int fadeValue = 0; fadeValue <= 1023; fadeValue += 10)
  {
    // sets the value (range from 0 to 255):
    analogWrite(MotorALeft_pin, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(20);
  }

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 1023; fadeValue >= 0; fadeValue -= 10)
  {
    // sets the value (range from 0 to 255):
    analogWrite(MotorALeft_pin, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(20);
  }
}



