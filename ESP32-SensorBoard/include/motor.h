//--------------------------------------------------------------------------
// U8G2 Display Setup  Definition
//--------------------------------------------------------------------------

#pragma once
#include <Arduino.h>
#include "globals.h"

void setup_motor();
void setSpeedLeft(uint16_t iv_speed);
void setSpeedRight(uint16_t iv_speed);
void setSpeedOff();
void motorA_fade();

