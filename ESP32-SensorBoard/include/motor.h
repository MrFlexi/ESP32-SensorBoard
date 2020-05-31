//--------------------------------------------------------------------------
// U8G2 Display Setup  Definition
//--------------------------------------------------------------------------

#pragma once
#include <Arduino.h>
#include "globals.h"






void setup_motor();
void setSpeedLeft(double iv_speed);
void setSpeedRight(double iv_speed);
void setSpeedOff();
void motorA_fade();

