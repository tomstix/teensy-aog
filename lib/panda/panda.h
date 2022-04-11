#pragma once

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>


void makePANDAfromPVT(UBX_NAV_PVT_data_t *pvt, char *panda, float yaw, float pitch, float roll);