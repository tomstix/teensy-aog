#pragma once

#include "main.h"

#define CMPSAddress 0x60
#define ADS_ADDRESS 0x48

uint8_t setupSensors();
int16_t adsWorker();