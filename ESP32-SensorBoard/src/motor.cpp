#include <Arduino.h>
#include "motor.h"

void setup_motor()
{
  //pinMode(LED, OUTPUT);
  pinMode(MotorALeft_pin, OUTPUT);
  pinMode(MotorARight_pin, OUTPUT);

  digitalWrite(MotorARight_pin, 0);
  digitalWrite(MotorALeft_pin, 0);
  //analogWriteFreq(8000); // 8 KHz PWM Frequency
}

void setSpeedLeft(double iv_speed)
{
  analogWrite(MotorALeft_pin, (int)iv_speed);
  analogWrite(MotorARight_pin, 0);
}

void setSpeedRight(double iv_speed)
{
  analogWrite(MotorARight_pin, (int)iv_speed);
  analogWrite(MotorALeft_pin, 0);
}

void setSpeedOff()
{
  analogWrite(MotorARight_pin, 0);
  analogWrite(MotorALeft_pin, 0);
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



