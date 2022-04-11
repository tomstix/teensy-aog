#pragma once

#include "main.h"

#define CURVE_COMMAND_INTERVAL 50
#define ADDRESS_CLAIM_INTERVAL 1000

#define vbusScaleLeft 897
#define vbusScaleRight 1039
#define WHEELBASE 2.783

void setupCAN();
void sendGoEnd();