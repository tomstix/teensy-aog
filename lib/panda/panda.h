#pragma once

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>


void makePANDAfromPVT(UBX_NAV_PVT_data_t *pvt, int8_t latHp, int8_t lonHp, char *panda, float yaw, float roll, float pitch);